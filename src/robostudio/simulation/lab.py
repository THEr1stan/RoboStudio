"""Bounded, reproducible algorithm experiments. Private cells never advance live sessions."""

import copy
import math
from typing import Any

from .. import __version__
from ..cell_ik import SOLVERS, solve
from ..cell_plugins import require_algorithm
from ..core import ErrorCode
from ..core.errors import require
from ..demo import source_hash
from ..serialization import digest
from .planning import CollisionProbe, plan_path
from .scene import StudioCell, scene_vector, validate_scene
from .trajectory import PROFILES, time_profile

FAMILIES = {
    "ik": tuple(SOLVERS),
    "planner": ("direct", "rrt", "rrt_connect", "prm"),
    "trajectory": PROFILES,
    "controller": ("computed", "pd_bias"),
}


def pose_frame(cell: StudioCell, q: list[float]) -> dict[str, Any]:
    """Only called on a disposable visualization cell, never a Session's cell."""
    cell.data.qpos[cell.q_indices] = q
    cell.data.qvel[:] = 0
    cell.mj.mj_forward(cell.model, cell.data)
    frame = cell.frame("algorithm_preview")
    # Kinematic previews have no meaningful dynamics/contact observation.
    for key in ("state", "reference", "command", "scene_state", "contact_sensor", "qvel"):
        frame.pop(key, None)
    return frame


def visualize(cell: StudioCell, family: str, result: dict[str, Any]) -> dict[str, Any]:
    if family == "ik":
        frames = [pose_frame(cell, row["q"]) for row in result["trace"]]
        return {"frames": frames, "result": result}
    probe = cell.probe()

    def point(q: list[float]) -> list[float]:
        return list(probe.evaluate(tuple(q))[0].position_m)

    workspace = [
        {
            "from_m": point(row["from_q"]),
            "to_m": point(row["q"]),
            "sample_m": point(row["sample_q"]),
            "collision_m": point(row["collision_q"]) if row["collision_q"] else None,
        }
        for row in result.get("search_trace", [])
    ]
    path = result["path"]
    frames = []
    if len(path) > 1:
        lengths = [math.dist(a, b) for a, b in zip(path, path[1:], strict=False)]
        total = sum(lengths)
        samples = min(200, 20 * (len(path) - 1))
        segment, offset = 0, 0.0
        for i in range(samples + 1):
            distance = total * i / samples
            while segment < len(lengths) - 1 and distance > offset + lengths[segment]:
                offset += lengths[segment]
                segment += 1
            u = min(1.0, max(0.0, (distance - offset) / max(1e-12, lengths[segment])))
            a, b = path[segment], path[segment + 1]
            q = [x + (y - x) * u for x, y in zip(a, b, strict=True)]
            frames.append(pose_frame(cell, q))
    if not frames:
        frames = [pose_frame(cell, cell.data.qpos[cell.q_indices].tolist())]
    return {
        "frames": frames,
        "result": result,
        "workspace_trace": workspace,
        "path_m": [point(q) for q in path],
        "raw_path_m": [point(q) for q in result.get("raw_path", [])],
    }


def run_lab(config: Any, family: Any, method: Any, target_m: Any) -> dict[str, Any]:
    require(
        isinstance(family, str) and family in FAMILIES,
        ErrorCode.INVALID_VALUE,
        "unknown algorithm family",
    )
    require(
        isinstance(method, str) and method in FAMILIES[family],
        ErrorCode.INVALID_VALUE,
        "algorithm does not belong to family",
    )
    target = list(scene_vector(target_m, "target_m", 3))
    require(all(abs(v) <= 2 for v in target), ErrorCode.INVALID_VALUE, "target exceeds 2 m")
    cfg = validate_scene(config)
    cfg[
        {
            "ik": "solver",
            "planner": "planner",
            "trajectory": "trajectory",
            "controller": "controller",
        }[family]
    ] = method
    cell = StudioCell(
        {**cfg, "controller": method if family == "controller" else cfg["controller"]}
    )
    start = cell.data.qpos[cell.q_indices].tolist()
    base = pose_frame(cell, start)
    result: dict[str, Any]
    payload: dict[str, Any]
    if family == "ik":
        result = solve(cell, target, method, damping=cfg["damping"])
        payload = visualize(cell, family, result)
    elif family == "planner":
        # A specified transfer query; not a silently changed live robot pose.
        initial = solve(cell, [0.42, -0.16, 0.23])
        goal = solve(cell, target, initial=initial["q"])
        require(
            initial["status"] == "CONVERGED" and goal["status"] == "CONVERGED",
            ErrorCode.INVALID_VALUE,
            "planning endpoints: IK did not converge",
        )
        base = pose_frame(cell, initial["q"])
        result = plan_path(
            initial["q"],
            goal["q"],
            cell.limits,
            CollisionProbe(cell),
            method=method,
            seed=cfg["seed"],
        )
        result.update(start_m=initial["target_m"], target_m=target)
        payload = visualize(cell, family, result)
    else:
        goal = solve(cell, target)
        require(goal["status"] == "CONVERGED", ErrorCode.INVALID_VALUE, "target IK failed")
        path = plan_path(start, goal["q"], cell.limits, CollisionProbe(cell), method="direct")
        require(
            path["status"] == "FOUND",
            ErrorCode.INVALID_VALUE,
            "time/control experiment requires a collision-free direct path",
        )
        duration = (
            math.ceil(
                max(2.5, max(abs(y - x) for x, y in zip(start, goal["q"], strict=True)) * 2) / 0.002
            )
            * 0.002
        )
        delta = cell.np.array(goal["q"]) - start
        profile = method if family == "trajectory" else cfg["trajectory"]
        require_algorithm(profile, frozenset({"joint_path"}))
        frames, trace = [], []
        squared, peak, contacts = 0.0, 0.0, 0
        steps = round((duration + 0.5) / 0.002) if family == "controller" else 101
        for i in range(steps):
            t = (i + 1) * 0.002 if family == "controller" else i * duration / 100
            u = min(1.0, t / duration)
            s, ds, dds = time_profile(profile, u)
            ref = cell.np.array(start) + delta * s
            velocity, acceleration = delta * ds / duration, delta * dds / duration**2
            if t > duration:
                velocity[:], acceleration[:] = 0, 0
            if family == "controller":
                cell.advance(ref, velocity, acceleration, False)
                error = float(cell.np.linalg.norm(cell.data.qpos[cell.q_indices] - ref))
                squared += error**2
                peak = max(peak, error)
                contacts += int(bool(cell.contact_reading()["unintended"]))
                if i % 10 != 0 and i != steps - 1:
                    continue
                frame = cell.frame("controller_experiment")
            else:
                error = 0.0
                frame = pose_frame(cell, ref.tolist())
            frames.append(frame)
            trace.append(
                {
                    "time_s": t,
                    "q": frame["q"],
                    "reference_q": ref.tolist(),
                    "velocity": velocity.tolist(),
                    "acceleration": acceleration.tolist(),
                    "s": s,
                    "ds": ds / duration,
                    "dds": dds / duration**2,
                    "error_si": error,
                    "effort": cell.data.ctrl[cell.ctrl_indices].tolist(),
                }
            )
        result = {
            "status": "COMPLETED",
            "algorithm": method,
            "trace": trace,
            "duration_s": duration,
            "physics_steps": steps if family == "controller" else 0,
            "tracking_rms_si": math.sqrt(squared / steps),
            "peak_error_si": peak,
            "collision_steps": contacts,
            "target_m": target,
            "profile": profile,
        }
        payload = {"frames": frames, "result": result}
        if family == "trajectory":
            for key in ("tracking_rms_si", "peak_error_si", "collision_steps"):
                result.pop(key)
            for row in trace:
                row.pop("effort")
                row.pop("error_si")
    body = {
        "schema_version": "algorithm-lab-v1",
        "family": family,
        "method": method,
        "config": cfg,
        "config_hash": digest(cfg),
        "task_hash": digest({"family": family, "start_q": base["q"], "target_m": target}),
        "seed": cfg["seed"],
        "target_m": target,
        "source_hash": source_hash(),
        "model_hash": cell.model_hash,
        "robot_hash": cell.robot_hash,
        "version": __version__,
        "physics_version": cell.mj.__version__,
        "numpy_version": cell.np.__version__,
        "frame_id": "world",
        "joint_unit": "rad" if cfg["robot"] == "r6" else "m",
        "mode": "physics" if family == "controller" else "kinematic_preview",
        "replay_command": ".venv/Scripts/python.exe scripts/replay_lab.py --record <export.json>",
        "base_frame": base,
        **payload,
    }
    body["replay_request"] = {"config": cfg, "family": family, "method": method, "target_m": target}
    return {**body, "lab_id": "lab_" + digest(body)[:24]}


def inspect_task(
    config: dict[str, Any], family: str, records: list[dict[str, Any]], index: int
) -> dict[str, Any]:
    require(family in ("ik", "planner"), ErrorCode.INVALID_VALUE, "inspect IK or planner")
    require(
        type(index) is int and 0 <= index < len(records),
        ErrorCode.INVALID_VALUE,
        "record index out of bounds",
    )
    result = copy.deepcopy(records[index])
    cell = StudioCell(config, live="scene_mocap_pos" in result)
    if "scene_qpos" in result:
        cell.data.qpos[:] = result["scene_qpos"]
        if "scene_mocap_pos" in result and cell.model.nmocap:
            cell.data.mocap_pos[:] = result["scene_mocap_pos"]
            cell.data.mocap_quat[:] = result["scene_mocap_quat"]
        cell.mj.mj_forward(cell.model, cell.data)
    base = pose_frame(cell, cell.data.qpos[cell.q_indices].tolist())
    return {
        "family": family,
        "method": result["algorithm"],
        "config": config,
        "mode": "kinematic_preview",
        "origin": "actual_task_record",
        "frame_id": "world",
        "source_hash": source_hash(),
        "model_hash": cell.model_hash,
        "base_frame": base,
        "joint_unit": "rad" if config["robot"] == "r6" else "m",
        **visualize(cell, family, result),
    }
