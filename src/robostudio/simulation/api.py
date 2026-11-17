"""Local session ownership, bounded actions, immutable experiment artifacts."""

import json
import re
import secrets
from pathlib import Path
from typing import Any, cast

from ..core import ErrorCode
from ..core.errors import require
from ..serialization import canonical, digest, read_json
from .documents import scene_document
from .lab import FAMILIES, inspect_task, run_lab
from .presets import industrial_scenes
from .runtime import PHASE_NAMES, Session, run_experiment
from .scene import COLORS, default_scene
from .world import WorldSession, replay_world


def save_report(output: Path, report: dict[str, Any]) -> dict[str, Any]:
    path = output / report["run_id"] / "run.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        require(
            canonical(read_json(path)) == canonical(report),
            ErrorCode.INTEGRITY_ERROR,
            "record differs",
        )
    else:
        temporary = path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(report, ensure_ascii=False, allow_nan=False), encoding="utf-8"
        )
        temporary.replace(path)
    return report


def load_report(output: Path, identifier: Any) -> dict[str, Any]:
    require(
        isinstance(identifier, str)
        and re.fullmatch(r"studio_[0-9a-f]{24}", identifier) is not None,
        ErrorCode.INVALID_VALUE,
        "invalid studio record ID",
    )
    path = (output / identifier / "run.json").resolve()
    require(
        path.is_relative_to(output.resolve()) and path.is_file(),
        ErrorCode.NOT_FOUND,
        "record missing",
    )
    require(path.stat().st_size <= 100_000_000, ErrorCode.INVALID_VALUE, "record too large")
    result = cast(dict[str, Any], read_json(path))
    body = {key: value for key, value in result.items() if key != "run_id"}
    require(
        "studio_" + digest(body)[:24] == identifier,
        ErrorCode.INTEGRITY_ERROR,
        "record hash mismatch",
    )
    return result


class StudioAPI:
    def __init__(self, output: Path):
        self.output = output
        self.sessions: dict[str, Session] = {}
        self.views: dict[str, dict[str, tuple[int, Any]]] = {}
        self.view_sequence = 0

    def live_view(self, identifier: str, data: dict[str, Any], known: dict[str, int]) -> Any:
        """Omit only acknowledged immutable panels; keep full state and reports intact."""
        cached = self.views.setdefault(identifier, {})
        data["sensor"] = self.sessions[identifier].image
        versions = {}
        for key in ("plan", "solution", "grasp", "sensor", "trace", "events"):
            value = data[key]
            previous = cached.get(key)
            same = previous is not None and (
                previous[1] == value if key in ("trace", "events") else previous[1] is value
            )
            if not same:
                self.view_sequence += 1
                cached[key] = (self.view_sequence, value)
            versions[key] = cached[key][0]
            if known.get(key) == versions[key]:
                del data[key]
        data["view"] = versions
        return data

    def get(self, path: str) -> Any:
        if path == "catalog":
            return {
                "config": default_scene(),
                "industrial_scenes": industrial_scenes(),
                "colors": COLORS,
                "phases": PHASE_NAMES,
                "robots": [
                    {"id": "r6", "name": "R6 Compact", "dof": 6, "kind": "六轴关节臂"},
                    {"id": "gantry", "name": "XYZ Gantry", "dof": 3, "kind": "三轴直角坐标"},
                ],
                "algorithms": [method for methods in FAMILIES.values() for method in methods],
                "camera": "128 × 112 ray-cast RGB + depth · 10 Hz",
                "algorithm_families": FAMILIES,
                "capabilities": [
                    "rigid_body_scene",
                    "physical_teleop",
                    "contact_grasp",
                    "sorting",
                    "stacking",
                    "rgbd",
                    "replay",
                ],
            }
        if path == "records":
            records = []
            for filename in sorted(
                self.output.glob("studio_*/run.json"), key=lambda p: p.stat().st_mtime, reverse=True
            )[:30]:
                try:
                    result = load_report(self.output, filename.parent.name)
                    records.append(
                        {
                            key: result[key]
                            for key in ("run_id", "status", "error", "metrics", "config")
                        }
                    )
                except (ValueError, OSError):
                    continue
            return records
        if path.startswith("records/"):
            return load_report(self.output, path.removeprefix("records/"))
        require(False, ErrorCode.NOT_FOUND, "unknown studio endpoint")

    def post(self, action: str, request: Any) -> Any:
        require(isinstance(request, dict), ErrorCode.INVALID_VALUE, "JSON object required")
        if action == "scene_validate":
            require(set(request) == {"document"}, ErrorCode.INVALID_VALUE, "document required")
            return scene_document(request["document"])
        if action == "lab":
            require(
                set(request) == {"config", "family", "method", "target_m"},
                ErrorCode.INVALID_VALUE,
                "invalid lab request fields",
            )
            return run_lab(**request)
        if action in ("create", "create_world"):
            require(set(request) == {"config"}, ErrorCode.INVALID_VALUE, "config required")
            require(
                len(self.sessions) < 8,
                ErrorCode.INVALID_VALUE,
                "close an existing studio tab first",
            )
            sim = (
                WorldSession(request["config"])
                if action == "create_world"
                else Session(request["config"])
            )
            identifier = secrets.token_hex(16)
            self.sessions[identifier] = sim
            return {"session": identifier, "config": sim.config, **sim.snapshot(sensor=True)}
        if action == "experiment":
            require(set(request) == {"config"}, ErrorCode.INVALID_VALUE, "config required")
            return save_report(self.output, run_experiment(request["config"]))
        if action == "replay":
            require(set(request) == {"run_id"}, ErrorCode.INVALID_VALUE, "run_id required")
            old = load_report(self.output, request["run_id"])
            new = (
                replay_world(old)
                if old["schema"] == "studio-world-v1"
                else run_experiment(old["config"], old["actions"], old["metrics"]["steps"])
            )
            require(
                canonical(old) == canonical(new),
                ErrorCode.INTEGRITY_ERROR,
                "replay differs: source, environment or recorded values changed",
            )
            return {
                "status": "IDENTICAL",
                "run_id": old["run_id"],
                "steps": old["metrics"]["steps"],
            }
        fields = {
            "step": {"session", "steps"},
            "command": {"session", "command"},
            "snapshot": {"session"},
            "save": {"session"},
            "close": {"session"},
            "reset": {"session", "config"},
            "algorithm_records": {"session"},
            "inspect": {"session", "family", "index"},
        }
        view = request.get("view", {})
        has_view = "view" in request and action in ("step", "command", "snapshot")
        require(
            action in fields and set(request) == fields[action] | ({"view"} if has_view else set()),
            ErrorCode.INVALID_VALUE,
            "invalid request fields",
        )
        require(
            isinstance(view, dict)
            and set(view) <= {"plan", "solution", "grasp", "sensor", "trace", "events"}
            and all(type(v) is int and 0 <= v <= 2**53 - 1 for v in view.values()),
            ErrorCode.INVALID_VALUE,
            "invalid live view cursor",
        )
        identifier = request["session"]
        require(
            isinstance(identifier, str) and identifier in self.sessions,
            ErrorCode.NOT_FOUND,
            "session expired",
        )
        sim = self.sessions[identifier]
        if action == "algorithm_records":
            return {
                family: [
                    {
                        "index": i,
                        "phase": row["phase"],
                        "step": row["step"],
                        "algorithm": row["algorithm"],
                        "status": row["status"],
                        "iterations": row["iterations"],
                    }
                    for i, row in enumerate(records)
                ]
                for family, records in (("ik", sim.solutions), ("planner", sim.plans))
            }
        if action == "inspect":
            require(
                request["family"] in ("ik", "planner"),
                ErrorCode.INVALID_VALUE,
                "inspect IK or planner",
            )
            return inspect_task(
                sim.initial_config if isinstance(sim, WorldSession) else sim.config,
                request["family"],
                sim.solutions if request["family"] == "ik" else sim.plans,
                request["index"],
            )
        if action in ("step", "snapshot", "command"):
            if action == "step":
                data = sim.advance(request["steps"])
                data["sensor"] = sim.image
            elif action == "command":
                data = sim.command(request["command"])
            else:
                data = sim.snapshot(sensor=True)
            return self.live_view(identifier, data, view) if has_view else data
        if action == "save":
            return save_report(self.output, sim.report())
        if action == "reset":
            self.sessions[identifier] = sim = type(sim)(request["config"])
            self.views.pop(identifier, None)
            return {"config": sim.config, **sim.snapshot(sensor=True)}
        del self.sessions[identifier]
        self.views.pop(identifier, None)
        return {"status": "CLOSED"}
