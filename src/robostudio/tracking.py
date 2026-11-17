"""Recorded target-to-motion experiments, with bounded inputs and exact replay."""

import math
import platform
import re
from dataclasses import asdict
from importlib.metadata import version
from pathlib import Path
from typing import Any, cast

from . import __version__
from .backends.mujoco_backend import MuJoCoBackend
from .core import ErrorCode, Pose, RoboStudioError, RobotReference, RunResult, RunStatus
from .core.contracts import finite, vector
from .core.errors import require
from .core.motion import IKProblem, IKSolver, JointController, JointTrajectory
from .demo import _save, source_hash
from .jacobian import verify_jacobian
from .kinematics import LIMITS, forward_kinematics, verify_model
from .motion import (
    HOME,
    KD,
    KP,
    DampedLeastSquares,
    R6Kinematics,
    joint_pd,
    pose_at,
    pose_error,
    quaternion_from_matrix,
    quintic,
    trajectory_duration,
)
from .registry import default_registry
from .registry.manifests import record, sequence
from .serialization import canonical, digest, read_json

PRESETS = (
    ("reach", "前方定位", "平稳伸向工作区前方", (25, -38, 68, 12, -45, 18)),
    ("sweep", "侧向转移", "观察底座与肩肘协同运动", (-45, -58, 88, -25, -45, 35)),
    ("lift", "抬升定位", "将末端抬高并调整工具朝向", (10, -90, 75, 28, 30, -30)),
)
# Explicit trusted implementations; manifests cannot import or execute modules.
SOLVER: IKSolver = DampedLeastSquares()
TRAJECTORY: JointTrajectory = quintic
CONTROLLER: JointController = joint_pd
SCENARIO = {
    "scenario_id": "r6_pose_tracking_v1",
    "physics_dt_s": 0.002,
    "control_dt_s": 0.002,
    "hold_s": 0.8,
    "velocity_limit_rad_s": 1.2,
    "acceleration_limit_rad_s2": 2.5,
    "kp": KP,
    "kd": KD,
    "position_tolerance_m": 0.002,
    "orientation_tolerance_rad": 0.01,
    "scope": "original R6; joint path; torque control; contacts disabled; no collision guarantee",
}


def preset_config(preset_id: str) -> dict[str, Any]:
    preset = next((p for p in PRESETS if p[0] == preset_id), None)
    require(preset is not None, ErrorCode.NOT_FOUND, "unknown preset")
    assert preset is not None
    target = pose_at(tuple(math.radians(v) for v in preset[3]))
    return {
        "target": {
            "position_m": list(target.position_m),
            "quaternion_wxyz": list(target.quaternion_wxyz),
            "frame_id": "world",
        },
        "seed_q_rad": list(HOME),
        "duration_s": 2.5,
        "controller": "pd_bias",
        "seed": 42,
    }


def catalog() -> dict[str, Any]:
    return {
        "presets": [
            {"id": p[0], "name": p[1], "description": p[2], "config": preset_config(p[0])}
            for p in PRESETS
        ],
        "algorithms": [
            {
                "id": p.plugin_id,
                "provides": p.provides_capability,
                "requires": sorted(p.requires_capabilities.values),
            }
            for p in default_registry().plugins
        ],
        "home_q_rad": HOME,
    }


def validate_config(value: object) -> tuple[dict[str, Any], IKProblem]:
    data = record(value, "tracking", {"target", "seed_q_rad", "duration_s", "controller", "seed"})
    target_data = record(data["target"], "target", {"position_m", "quaternion_wxyz", "frame_id"})
    target = Pose(
        tuple(cast(list[float], sequence(target_data["position_m"], "position_m"))),
        tuple(cast(list[float], sequence(target_data["quaternion_wxyz"], "quaternion_wxyz"))),
        cast(str, target_data["frame_id"]),
    )
    require(target.frame_id == "world", ErrorCode.FRAME_MISMATCH, "expected world target")
    require(
        all(abs(v) <= 5 for v in target.position_m),
        ErrorCode.INVALID_VALUE,
        "target coordinates must be within +/-5 m",
    )
    q = vector(tuple(cast(list[float], sequence(data["seed_q_rad"], "seed_q_rad"))), "seed", 6)
    require(
        all(lo <= v <= hi for v, (lo, hi) in zip(q, LIMITS, strict=True)),
        ErrorCode.INVALID_VALUE,
        "seed exceeds joint limits",
    )
    duration = cast(float, data["duration_s"])
    finite(duration, "duration_s")
    require(1 <= duration <= 8, ErrorCode.INVALID_VALUE, "duration must be 1..8 s")
    require(data["controller"] in ("pd_bias", "pd"), ErrorCode.INVALID_VALUE, "unknown controller")
    registry = default_registry()
    robot = registry.robot("r6_compact")
    for plugin, parameters in (
        ("dls_pose_v1", {"max_iterations": 160, "attempts": 6}),
        ("quintic_v1", {"duration_s": duration}),
        ("joint_pd_v1", {}),
    ):
        registry.require_plugin(plugin, robot.robot_id).validate_parameters(parameters)
    problem = IKProblem(
        target, q, robot.robot_hash, robot.model.sha256, seed=cast(int, data["seed"])
    )
    # Canonical normalized inputs: seed and duration are not coerced from strings/bools.
    normalized = {
        "target": asdict(target),
        "seed_q_rad": q,
        "duration_s": duration,
        "controller": data["controller"],
        "seed": problem.seed,
    }
    return normalized, problem


def solve_target(value: object) -> dict[str, Any]:
    config, problem = validate_config(value)
    model = R6Kinematics(default_registry().robot("r6_compact"))
    result = SOLVER.solve(model, problem)
    return {
        "solution": asdict(result),
        "config": config,
        "robot_hash": problem.robot_hash,
        "model_hash": problem.model_hash,
        "source_hash": source_hash(),
        "pose": asdict(model.pose(result.q_rad)),
        "frame_id": "world",
    }


def _build(value: object) -> tuple[RunResult, dict[str, Any], list[dict[str, Any]]]:
    config, problem = validate_config(value)
    robot = default_registry().robot("r6_compact")
    environment = {
        "python": platform.python_version(),
        "platform": platform.system(),
        "machine": platform.machine(),
        "robostudio": __version__,
    }
    samples: list[dict[str, Any]] = []
    verification: dict[str, Any] = {}
    solution: dict[str, Any] = {}
    metrics: dict[str, float] = {"steps": 0, "saturated_commands": 0, "joint_limit_violations": 0}
    failure: RoboStudioError | None = None
    backend = MuJoCoBackend()
    try:
        backend.load(robot)
        environment.update(mujoco=version("mujoco-core"), numpy=version("numpy"))
        backend.reset(problem.seed)
        verification = {"fk": verify_model(backend), "jacobian": verify_jacobian(backend)}
        ik = SOLVER.solve(R6Kinematics(robot), problem)
        solution = asdict(ik)
        require(
            ik.status == "CONVERGED",
            ErrorCode.CONVERGENCE_FAILED,
            "No pose solution within the bounded search; adjust target or seed",
        )
        duration = trajectory_duration(problem.seed_q_rad, ik.q_rad, config["duration_s"])
        metrics["trajectory_duration_s"] = duration
        backend.set_configuration(problem.seed_q_rad)
        bounds = backend.effort_bounds()
        steps = round((duration + 0.8) / 0.002)
        last_command: dict[str, Any] | None = None
        for step in range(steps + 1):
            state = backend.observe()
            desired_q, desired_v = TRAJECTORY(problem.seed_q_rad, ik.q_rad, state.time_s, duration)
            reference = RobotReference(
                robot_id=robot.robot_id,
                robot_hash=robot.robot_hash,
                model_hash=robot.model.sha256,
                frame_id=robot.frame_id,
                time_s=state.time_s,
                qpos_si=desired_q,
                qvel_si=desired_v,
            )
            frame = backend.scene()
            desired_pose = pose_at(desired_q)
            frame.update(
                reference=asdict(reference),
                command=last_command,
                reference_tcp=list(desired_pose.position_m),
                tracking_error_m=math.dist(frame["tcp"], desired_pose.position_m),
            )
            samples.append(frame)
            metrics["joint_limit_violations"] += sum(
                int(q < lo - 1e-6 or q > hi + 1e-6)
                for q, (lo, hi) in zip(state.qpos_si, LIMITS, strict=True)
            )
            if step == steps:
                break
            command, saturated = CONTROLLER(
                robot,
                state,
                reference,
                backend.bias_effort() if config["controller"] == "pd_bias" else (0.0,) * 6,
                bounds,
            )
            metrics["saturated_commands"] += saturated
            backend.apply(command)
            last_command = asdict(command)
            backend.step(0.002)
        final_pose = Pose(
            tuple(samples[-1]["tcp"]), quaternion_from_matrix(samples[-1]["rotation"]), "world"
        )
        residual = pose_error(problem.target, final_pose)
        metrics.update(
            final_position_error_m=math.sqrt(sum(v * v for v in residual[:3])),
            final_orientation_error_rad=math.sqrt(sum(v * v for v in residual[3:])),
        )
        require(
            metrics["final_position_error_m"] <= 0.002
            and metrics["final_orientation_error_rad"] <= 0.01
            and metrics["joint_limit_violations"] == 0,
            ErrorCode.TRACKING_FAILED,
            "Tracking finished outside the acceptance tolerance",
        )
    except RoboStudioError as error:
        failure = error
    finally:
        backend.close()
    metrics["steps"] = max(0, len(samples) - 1)
    if samples:
        metrics.update(
            sim_time_s=samples[-1]["time_s"],
            tcp_tracking_rms_m=math.sqrt(
                sum(s["tracking_error_m"] ** 2 for s in samples) / len(samples)
            ),
            joint_tracking_rms_rad=math.sqrt(
                sum(
                    (q - ref) ** 2
                    for s in samples
                    for q, ref in zip(s["qpos_si"], s["reference"]["qpos_si"], strict=True)
                )
                / (len(samples) * 6)
            ),
            peak_effort_nm=max(abs(v) for s in samples for v in s["effort_si"]),
            max_playback_fk_error_m=max(
                math.dist(s["tcp"], forward_kinematics(tuple(s["qpos_si"]))["tcp"]) for s in samples
            ),
        )
    identity = {
        "config_hash": digest(config),
        "scenario_hash": digest(SCENARIO),
        "source_hash": source_hash(),
        "robot_hash": robot.robot_hash,
        "model_hash": robot.model.sha256,
    }
    run_id = "track_" + digest({**identity, "environment": environment})[:24]
    result = RunResult(
        run_id=run_id,
        scenario_id="r6_pose_tracking_v1",
        **identity,
        seed=problem.seed,
        status=RunStatus.FAILED if failure else RunStatus.SUCCEEDED,
        metrics=metrics,
        error_code=failure.code if failure else None,
    )
    document = {
        "schema_version": "tracking-run-v1",
        "result": result.as_dict(),
        "config": config,
        "scenario": SCENARIO,
        "environment": environment,
        "verification": verification,
        "solution": solution,
        "observations_hash": digest(samples),
        "error": failure.as_dict() if failure else None,
        "replay_command": "robostudio experiment replay run.json --output ..",
        "replay_working_directory": "directory containing this run.json",
    }
    return result, document, samples


def playback(document: dict[str, Any], samples: list[dict[str, Any]]) -> dict[str, Any]:
    frames = samples[::10]
    if samples and (len(samples) - 1) % 10:
        frames.append(samples[-1])
    return {**document, "frames": frames}


def run_tracking(value: object, output_dir: Path = Path("artifacts")) -> dict[str, Any]:
    result, document, samples = _build(value)
    _save(output_dir, result.run_id, document, samples)
    return playback(document, samples)


def replay_tracking(path: Path, output_dir: Path) -> RunResult:
    document = record(read_json(path), "run")
    result, expected, samples = _build(document.get("config"))
    require(
        canonical(document) == canonical(expected)
        and canonical(read_json(path.parent / "observations.json")) == canonical(samples),
        ErrorCode.INTEGRITY_ERROR,
        "replay integrity mismatch: source, inputs or observations",
    )
    _save(output_dir, result.run_id, expected, samples)
    return result


def load_record(output_dir: Path, run_id: str, full: bool = False) -> dict[str, Any]:
    require(
        re.fullmatch(r"track_[a-f0-9]{24}", run_id) is not None,
        ErrorCode.NOT_FOUND,
        "unknown tracking record",
    )
    folder = (output_dir / run_id).resolve()
    require(folder.is_relative_to(output_dir.resolve()), ErrorCode.NOT_FOUND, "record path invalid")
    for name in ("run.json", "observations.json"):
        path = folder / name
        require(
            path.resolve().is_relative_to(output_dir.resolve())
            and path.is_file()
            and path.stat().st_size < 32_000_000,
            ErrorCode.NOT_FOUND,
            "record unavailable",
        )
    document = cast(dict[str, Any], record(read_json(folder / "run.json"), "run"))
    samples = cast(
        list[dict[str, Any]], sequence(read_json(folder / "observations.json"), "frames")
    )
    result = record(document.get("result"), "result")
    require(
        document.get("schema_version") == "tracking-run-v1"
        and result.get("run_id") == run_id
        and digest(samples) == document.get("observations_hash")
        and digest(document.get("config")) == result.get("config_hash")
        and digest(document.get("scenario")) == result.get("scenario_hash"),
        ErrorCode.INTEGRITY_ERROR,
        "saved observations integrity mismatch",
    )
    response = playback(document, samples)
    if full:
        response["observations"] = samples
    return response


def list_records(output_dir: Path) -> list[dict[str, Any]]:
    if not output_dir.is_dir():
        return []
    entries = sorted(
        output_dir.glob("track_*/run.json"), key=lambda p: p.stat().st_mtime, reverse=True
    )[:50]
    result = []
    for path in entries:
        try:
            if (
                not path.resolve().is_relative_to(output_dir.resolve())
                or path.stat().st_size > 100_000
            ):
                continue
            d = record(read_json(path), "run")
            if d.get("schema_version") == "tracking-run-v1":
                result.append({"result": d["result"], "config": d["config"]})
        except (RoboStudioError, OSError, KeyError):
            continue
    return result
