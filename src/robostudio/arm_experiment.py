"""Bounded M1a gravity release, independent FK check and exact-environment replay."""

import platform
import random
from importlib.metadata import version
from math import radians, sqrt
from pathlib import Path
from typing import Any, cast

from . import __version__
from .backends.mujoco_backend import MuJoCoBackend
from .core import ErrorCode, RoboStudioError, RobotCommand, RunResult, RunStatus
from .core.errors import require
from .demo import _save, source_hash
from .jacobian import verify_jacobian
from .kinematics import forward_kinematics, verify_model
from .registry import default_registry
from .registry.manifests import record
from .serialization import canonical, digest, read_json

SCENARIO = {
    "scenario_id": "r6_gravity_release_v1",
    "robot_id": "r6_compact",
    "physics_dt_s": 0.002,
    "control_dt_s": 0.02,
    "steps": 1000,
    "initial_q_deg": [0, -60, 60, 0, -28, 0],
    "jitter_rad": 0.02,
    "effort_si": [0.0] * 6,
    "gravity_m_s2": [0, 0, -9.81],
    "scope": "original idealized model; passive gravity; contacts disabled; no servo/IK",
}


def _build(seed: int) -> tuple[RunResult, dict[str, Any], list[dict[str, Any]]]:
    require(
        type(seed) is int and 0 <= seed < 2**32,
        ErrorCode.INVALID_VALUE,
        "seed must be a uint32",
        "seed",
    )
    robot = default_registry().robot("r6_compact")
    config = {"seed": seed}
    environment = {
        "python": platform.python_version(),
        "platform": platform.system(),
        "machine": platform.machine(),
        "robostudio": __version__,
    }
    backend = MuJoCoBackend()
    samples: list[dict[str, Any]] = []
    verification: dict[str, Any] = {}
    failure: RoboStudioError | None = None
    try:
        backend.load(robot)
        environment.update(mujoco=version("mujoco-core"), numpy=version("numpy"))
        backend.reset(seed)
        verification = verify_model(backend)
        verification["jacobian"] = verify_jacobian(backend)
        rng = random.Random(seed)
        q = tuple(radians(v) + (rng.random() - 0.5) * 0.02 for v in (0, -60, 60, 0, -28, 0))
        backend.set_configuration(q)
        samples.append(backend.scene())
        for step in range(1000):
            if step % 10 == 0:
                state = backend.observe()
                backend.apply(
                    RobotCommand(
                        robot_id=robot.robot_id,
                        robot_hash=robot.robot_hash,
                        model_hash=robot.model.sha256,
                        frame_id="world",
                        time_s=state.time_s,
                        effort_si=(0.0,) * 6,
                    )
                )
            backend.step(0.002)
            samples.append(backend.scene())
    except RoboStudioError as error:
        failure = error
    finally:
        backend.close()
    identity = {
        "config_hash": digest(config),
        "scenario_hash": digest(SCENARIO),
        "source_hash": source_hash(),
        "robot_hash": robot.robot_hash,
        "model_hash": robot.model.sha256,
    }
    run_id = "arm_" + digest({**identity, "environment": environment})[:24]
    metrics: dict[str, float] = {"steps": max(0, len(samples) - 1)}
    if samples:
        metrics.update(
            sim_time_s=samples[-1]["time_s"],
            final_speed_norm_rad_s=sqrt(sum(v * v for v in samples[-1]["qvel_si"])),
            max_playback_fk_error_m=max(
                sqrt(
                    sum(
                        (a - b) ** 2
                        for a, b in zip(
                            sample["tcp"],
                            forward_kinematics(tuple(sample["qpos_si"]))["tcp"],
                            strict=True,
                        )
                    )
                )
                for sample in samples
            ),
        )
    result = RunResult(
        run_id=run_id,
        scenario_id=str(SCENARIO["scenario_id"]),
        **identity,
        seed=seed,
        status=RunStatus.FAILED if failure else RunStatus.SUCCEEDED,
        metrics=metrics,
        error_code=failure.code if failure else None,
    )
    document = {
        "schema_version": "arm-run-v1",
        "result": result.as_dict(),
        "config": config,
        "scenario": SCENARIO,
        "environment": environment,
        "verification": verification,
        "observations_hash": digest(samples),
        "error": failure.as_dict() if failure else None,
        "replay_command": "robostudio experiment replay run.json --output ..",
        "replay_working_directory": "directory containing this run.json",
    }
    return result, document, samples


def run_arm(seed: int = 42, output_dir: Path = Path("artifacts")) -> dict[str, Any]:
    result, document, samples = _build(seed)
    _save(output_dir, result.run_id, document, samples)
    frames = samples[::10]
    if samples and (len(samples) - 1) % 10:
        frames.append(samples[-1])
    return {
        "result": result.as_dict(),
        "verification": document["verification"],
        "environment": document["environment"],
        "frames": frames,
        "error": document["error"],
    }


def replay_arm(path: Path, output_dir: Path) -> RunResult:
    document = record(read_json(path), "run")
    config = record(document.get("config"), "config", {"seed"})
    result, expected, samples = _build(cast(int, config["seed"]))
    require(
        canonical(document) == canonical(expected)
        and canonical(read_json(path.parent / "observations.json")) == canonical(samples),
        ErrorCode.INTEGRITY_ERROR,
        "replay integrity mismatch: source, engine/environment, inputs or observations changed",
    )
    _save(output_dir, result.run_id, expected, samples)
    return result
