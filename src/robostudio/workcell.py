"""Contact-driven desktop pick/place. Scene and robot coordinates are explicitly mapped.

No weld constraints, object pose writes during execution, or generated-code loading.
"""

import hashlib
import importlib
import json
import math
import platform
import random
import re
import xml.etree.ElementTree as ET
from dataclasses import asdict
from importlib.resources import files
from pathlib import Path
from typing import Any, cast

from . import __version__
from .backends.mujoco_backend import engine
from .cell_ik import SOLVERS, solve
from .cell_plugins import require_algorithm
from .core import ErrorCode, Pose, RobotCommand, RobotReference, RobotState
from .core.contracts import finite, vector
from .core.errors import require
from .demo import source_hash
from .motion import quaternion_from_matrix
from .registry.manifests import record
from .serialization import canonical, digest, read_json

ROBOT_NAMES = {"r6": "R6 · 六轴关节臂", "gantry": "XYZ · 三轴龙门"}
PHASES = ["approach", "descend", "close", "lift", "transfer", "lower", "release", "retreat"]


def default_config(robot: str = "r6") -> dict[str, Any]:
    return {
        "robot": robot,
        "seed": 42,
        "mass_kg": 0.045,
        "friction": 1.2,
        "object_xy": [0.42, -0.14],
        "destination_xy": [0.42, 0.19],
        "solver": "dls",
        "controller": "computed",
        "damping": 0.035,
    }


def validate_config(value: Any) -> dict[str, Any]:
    require(
        isinstance(value, dict) and set(value) == set(default_config()),
        ErrorCode.INVALID_VALUE,
        "invalid workcell configuration fields",
    )
    config = dict(value)
    require(
        isinstance(config["robot"], str) and config["robot"] in ROBOT_NAMES,
        ErrorCode.INVALID_VALUE,
        "unknown robot",
    )
    require(
        isinstance(config["solver"], str) and config["solver"] in SOLVERS,
        ErrorCode.INVALID_VALUE,
        "unknown solver",
    )
    require(
        config["controller"] in ("computed", "pd_bias"),
        ErrorCode.INVALID_VALUE,
        "unknown controller",
    )
    require(
        type(config["seed"]) is int and 0 <= config["seed"] < 2**32,
        ErrorCode.INVALID_VALUE,
        "invalid seed",
    )
    for key, lo, hi in (("mass_kg", 0.01, 0.3), ("friction", 0.05, 2.0), ("damping", 0.001, 0.5)):
        finite(config[key], key)
        require(lo <= config[key] <= hi, ErrorCode.INVALID_VALUE, f"{key} out of range")
    for key in ("object_xy", "destination_xy"):
        require(isinstance(config[key], list), ErrorCode.INVALID_VALUE, "expected XY array")
        xy = vector(tuple(config[key]), key, 2)
        require(
            0.30 <= xy[0] <= 0.53 and -0.28 <= xy[1] <= 0.28,
            ErrorCode.INVALID_VALUE,
            "XY outside tabletop task region",
        )
        config[key] = list(xy)
    require(
        math.dist(config["object_xy"], config["destination_xy"]) >= 0.13,
        ErrorCode.INVALID_VALUE,
        "pick and place must be at least 13 cm apart",
    )
    return config


def scene_xml(config: dict[str, Any]) -> str:
    if config["robot"] == "r6":
        root = ET.fromstring(files("robostudio").joinpath("data", "r6_compact.xml").read_text())
        world = root.find("worldbody")
        actuator = root.find("actuator")
        tool = root.find(".//body[@name='link6']")
        assert world is not None and actuator is not None and tool is not None
        for geom in world.iter("geom"):
            geom.set("contype", "1")
            geom.set("conaffinity", "6")
        # Base is anchored to the floor; exclude its visual capsule from contacts.
        base = world.find("./body/geom")
        assert base is not None
        base.set("contype", "0")
        base.set("conaffinity", "0")
        origin = 0.1
    else:
        root = ET.fromstring("""<mujoco model="RoboStudio XYZ gantry">
        <compiler angle="radian"/><option timestep=".002" integrator="implicitfast"/>
        <worldbody><body name="rail_x" pos="0 0 .5">
        <joint name="joint1" type="slide" axis="1 0 0" range=".2 .65" damping="2"/>
        <geom type="box" size=".03 .03 .03" mass="1" contype="0" conaffinity="0"/>
        <body name="rail_y"><joint name="joint2" type="slide" axis="0 1 0"
        range="-.35 .35" damping="2"/><geom type="box" size=".025 .025 .03"
        mass=".6" contype="0" conaffinity="0"/>
        <body name="rail_z"><joint name="joint3" type="slide" axis="0 0 1"
        range="-.44 -.12" damping="2"/><geom type="box" size=".02 .02 .05"
        mass=".3" contype="0" conaffinity="0"/>
        <body name="tool" quat=".7071067811865476 0 .7071067811865476 0">
        <site name="tcp" size=".005"/></body></body></body></body></worldbody>
        <actuator><motor name="motor2" joint="joint2" gear="1" ctrlrange="-100 100"/>
        <motor name="motor1" joint="joint1" gear="1" ctrlrange="-100 100"/>
        <motor name="motor3" joint="joint3" gear="1" ctrlrange="-100 100"/>
        </actuator></mujoco>""")
        world, actuator, tool = (
            root.find("worldbody"),
            root.find("actuator"),
            root.find(".//body[@name='tool']"),
        )
        assert world is not None and actuator is not None and tool is not None
        origin = 0.0
    for name, side in (("left", 1), ("right", -1)):
        finger = ET.SubElement(tool, "body", name=f"finger_{name}", pos=f"{origin} {side * 0.05} 0")
        ET.SubElement(
            finger,
            "joint",
            name=f"finger_{name}",
            type="slide",
            axis=f"0 {-side} 0",
            range="0 .043",
            damping="3",
            armature=".001",
        )
        ET.SubElement(
            finger,
            "geom",
            name=f"pad_{name}",
            type="box",
            pos=".033 0 0",
            size=".033 .008 .016",
            mass=".04",
            contype="1",
            conaffinity="6",
            friction=f"{config['friction']} .02 .001",
            condim="4",
            solref=".006 1",
        )
        ET.SubElement(
            actuator,
            "position",
            name=f"grip_{name}",
            joint=f"finger_{name}",
            kp="700",
            kv="8",
            ctrlrange="0 .043",
            forcerange="-8 8",
        )
    ET.SubElement(
        world,
        "geom",
        name="table",
        type="box",
        pos=".35 0 -.025",
        size=".48 .43 .025",
        contype="4",
        conaffinity="3",
        friction="1 .01 .001",
    )
    rng = random.Random(config["seed"])
    positions = [
        config["object_xy"],
        [0.30 + rng.uniform(-0.015, 0.015), -0.24],
        [0.52 + rng.uniform(-0.015, 0.015), -0.24],
    ]
    for i, xy in enumerate(positions):
        body = ET.SubElement(world, "body", name=f"block{i}", pos=f"{xy[0]} {xy[1]} .023")
        ET.SubElement(body, "freejoint", name=f"free{i}")
        ET.SubElement(
            body,
            "geom",
            name=f"cube{i}",
            type="box",
            size=".02 .02 .02",
            mass=str(config["mass_kg"]),
            contype="2",
            conaffinity="7",
            friction=f"{config['friction']} .02 .001",
            condim="4",
            solref=".006 1",
        )
    return ET.tostring(root, encoding="unicode")


class Cell:
    """Private physics instance per request; mapping also supports unequal nq/nv."""

    capabilities = frozenset({"cartesian_ik", "joint_effort", "joint_trajectory", "contact_grasp"})

    def probe(self) -> "CellProbe":
        return CellProbe(self)

    def __init__(self, config: Any, *, xml_override: str | None = None):
        self.config = validate_config(config)
        require_algorithm(self.config["controller"], self.capabilities)
        self.mj, self.np = engine(), importlib.import_module("numpy")
        self.xml = scene_xml(self.config) if xml_override is None else xml_override
        self.model_hash = hashlib.sha256(self.xml.encode()).hexdigest()
        self.model = self.mj.MjModel.from_xml_string(self.xml)
        self.data = self.mj.MjData(self.model)
        count = 6 if self.config["robot"] == "r6" else 3
        self.joint_ids = [self.model.joint(f"joint{i + 1}").id for i in range(count)]
        self.q_indices = [int(self.model.jnt_qposadr[j]) for j in self.joint_ids]
        self.v_indices = [int(self.model.jnt_dofadr[j]) for j in self.joint_ids]
        self.ctrl_indices = [self.model.actuator(f"motor{i + 1}").id for i in range(count)]
        self.limits = [tuple(float(v) for v in self.model.jnt_range[j]) for j in self.joint_ids]
        self.grip_ctrl = [self.model.actuator(f"grip_{name}").id for name in ("left", "right")]
        self.pad_ids = [self.model.geom(f"pad_{name}").id for name in ("left", "right")]
        self.tcp = self.model.site("tcp").id
        home = (
            [math.radians(v) for v in (0, -60, 90, 0, 60, 0)] if count == 6 else [0.4, 0.0, -0.22]
        )
        self.data.qpos[self.q_indices] = home
        self.mj.mj_forward(self.model, self.data)
        self.mass_matrix = self.np.zeros((self.model.nv, self.model.nv))
        self.robot_hash = digest(
            {
                "robot": self.config["robot"],
                "limits": self.limits,
                "qpos": self.q_indices,
                "qvel": self.v_indices,
                "ctrl": self.ctrl_indices,
            }
        )
        self.last_reference: RobotReference | None = None
        self.last_command: RobotCommand | None = None

    def identity(self) -> dict[str, Any]:
        return {
            "robot_id": "cell_" + self.config["robot"],
            "robot_hash": self.robot_hash,
            "model_hash": self.model_hash,
            "frame_id": "world",
            "time_s": float(self.data.time),
        }

    def frame(self, phase: str) -> dict[str, Any]:
        d, m = self.data, self.model
        objects = [
            {
                "position": d.xpos[m.body(f"block{i}").id].tolist(),
                "rotation": d.xmat[m.body(f"block{i}").id].reshape(3, 3).tolist(),
                "half_size": [0.02] * 3,
                "color": [[241, 172, 85], [123, 143, 238], [95, 198, 179]][i],
            }
            for i in range(3)
        ]
        pads = [
            {
                "position": d.geom_xpos[i].tolist(),
                "rotation": d.geom_xmat[i].reshape(3, 3).tolist(),
                "half_size": m.geom_size[i].tolist(),
                "color": [172, 193, 210],
            }
            for i in self.pad_ids
        ]
        tcp = d.site_xpos[self.tcp].tolist()
        state = RobotState(
            **self.identity(),
            qpos_si=tuple(float(q) for q in d.qpos[self.q_indices]),
            qvel_si=tuple(float(v) for v in d.qvel[self.v_indices]),
        )
        return {
            "state": asdict(state),
            "scene_state": {
                "qpos": d.qpos.tolist(),
                "qvel": d.qvel.tolist(),
                "ctrl": d.ctrl.tolist(),
            },
            "reference": asdict(self.last_reference) if self.last_reference else None,
            "command": asdict(self.last_command) if self.last_command else None,
            "time_s": float(d.time),
            "phase": phase,
            "robot": self.config["robot"],
            "q": d.qpos[self.q_indices].tolist(),
            "qvel": d.qvel[self.v_indices].tolist(),
            "effort": d.ctrl[self.ctrl_indices].tolist(),
            "tcp": tcp,
            "rotation": d.site_xmat[self.tcp].reshape(3, 3).tolist(),
            "points": [d.xanchor[j].tolist() for j in self.joint_ids] + [tcp],
            "jointAxes": [d.xaxis[j].tolist() for j in self.joint_ids],
            "objects": objects,
            "pads": pads,
            "contacts": int(d.ncon),
        }

    def advance(self, reference: Any, velocity: Any, acceleration: Any, close: bool) -> None:
        np, m, d = self.np, self.model, self.data
        q, v = d.qpos[self.q_indices], d.qvel[self.v_indices]
        if self.config["controller"] == "computed":
            self.mj.mj_fullM(m, d, self.mass_matrix)
            desired_acc = acceleration + 180 * (reference - q) + 28 * (velocity - v)
            torque = self.mass_matrix[np.ix_(self.v_indices, self.v_indices)] @ desired_acc
            torque += d.qfrc_bias[self.v_indices] - d.qfrc_passive[self.v_indices]
        else:
            torque = 500 * (reference - q) + 42 * (velocity - v) + d.qfrc_bias[self.v_indices]
        bounds = m.actuator_ctrlrange[self.ctrl_indices]
        d.ctrl[self.ctrl_indices] = np.clip(torque, bounds[:, 0], bounds[:, 1])
        d.ctrl[self.grip_ctrl] = 0.043 if close else 0.0
        self.last_reference = RobotReference(
            **self.identity(),
            qpos_si=tuple(float(x) for x in reference),
            qvel_si=tuple(float(x) for x in velocity),
        )
        self.last_command = RobotCommand(
            **self.identity(), effort_si=tuple(float(d.ctrl[i]) for i in sorted(self.ctrl_indices))
        )
        self.mj.mj_step(m, d)
        self.mj.mj_forward(m, d)


class CellProbe:
    """Implements the engine-free kinematic protocol using isolated MuJoCo data."""

    def __init__(self, cell: Cell):
        self.cell = cell
        self.limits = cell.limits
        self.configuration = tuple(float(q) for q in cell.data.qpos[cell.q_indices])
        self.data = cell.mj.MjData(cell.model)
        self.data.qpos[:] = cell.data.qpos
        self.jp = cell.np.zeros((3, cell.model.nv))
        self.jr = cell.np.zeros((3, cell.model.nv))

    def evaluate(self, q: tuple[float, ...]) -> tuple[Pose, tuple[tuple[float, ...], ...]]:
        c, d = self.cell, self.data
        vector(q, "q_si", len(c.q_indices))
        d.qpos[c.q_indices] = q
        c.mj.mj_forward(c.model, d)
        c.mj.mj_jacSite(c.model, d, self.jp, self.jr, c.tcp)
        pose = Pose(
            tuple(float(x) for x in d.site_xpos[c.tcp]),
            quaternion_from_matrix(d.site_xmat[c.tcp].reshape(3, 3).tolist()),
            "world",
        )
        matrix = c.np.vstack((self.jp[:, c.v_indices], self.jr[:, c.v_indices]))
        return pose, tuple(tuple(float(x) for x in row) for row in matrix)


def preview(config: Any) -> dict[str, Any]:
    cell = Cell(config)
    return {
        "frame": cell.frame("ready"),
        "config": cell.config,
        "limits": cell.limits,
        "q_indices": cell.q_indices,
        "v_indices": cell.v_indices,
        "ctrl_indices": cell.ctrl_indices,
        "nq": cell.model.nq,
        "nv": cell.model.nv,
    }


def run_cell(config: Any, output_dir: Path | None = None) -> dict[str, Any]:
    cell = Cell(config)
    cfg, np = cell.config, cell.np
    x, y = cfg["object_xy"]
    dx, dy = cfg["destination_xy"]
    stages = [
        ("approach", [x, y, 0.20], False, 1.6),
        ("descend", [x, y, 0.073], False, 1.4),
        ("close", [x, y, 0.073], True, 0.8),
        ("lift", [x, y, 0.24], True, 1.6),
        ("transfer", [dx, dy, 0.24], True, 2.0),
        ("lower", [dx, dy, 0.078], True, 1.6),
        ("release", [dx, dy, 0.078], False, 0.8),
        ("retreat", [dx, dy, 0.25], False, 1.6),
    ]
    frames = [cell.frame("ready")]
    solutions, events = [], []
    max_z, bilateral, error_sum, samples = 0.02, 0, 0.0, 0
    block_id = cell.model.body("block0").id
    cube_id = cell.model.geom("cube0").id
    failure: str | None = None
    for phase, target, close, duration in stages:
        solution = solve(cell, target, cfg["solver"], damping=cfg["damping"])
        solutions.append({"phase": phase, **solution})
        events.append({"phase": phase, "time_s": float(cell.data.time), "target_m": target})
        if solution["status"] != "CONVERGED":
            failure = f"IK_FAILED:{phase}"
            break
        start = cell.data.qpos[cell.q_indices].copy()
        delta = np.array(solution["q"]) - start
        duration = math.ceil(max(duration, float(np.max(np.abs(delta))) * 2.0) / 0.002) * 0.002
        steps = round(duration / 0.002)
        for k in range(steps + 150):
            u = min(1.0, (k + 1) / steps)
            s = u**3 * (10 - 15 * u + 6 * u * u)
            ds = 30 * u * u * (1 - u) ** 2 / duration
            dds = 60 * u * (1 - 3 * u + 2 * u * u) / duration**2
            cell.advance(start + delta * s, delta * ds, delta * dds, close)
            max_z = max(max_z, float(cell.data.xpos[block_id][2]))
            touching = set()
            for contact in cell.data.contact:
                pair = {int(contact.geom1), int(contact.geom2)}
                if cube_id in pair:
                    touching.update(pair.intersection(cell.pad_ids))
            bilateral += int(len(touching) == 2)
            error_sum += float(np.sum((cell.data.qpos[cell.q_indices] - (start + delta * s)) ** 2))
            samples += 1
            if samples % 15 == 0:
                frame = cell.frame(phase)
                frame["reference_q"] = (start + delta * s).tolist()
                frame["target_m"] = target
                frames.append(frame)
        if phase in ("lift", "transfer") and float(cell.data.xpos[block_id][2]) < 0.10:
            failure = f"GRASP_LOST:{phase}"
            events.append({"phase": "grasp_lost", "time_s": float(cell.data.time)})
            break
    position = cell.data.xpos[block_id].tolist()
    landing = math.dist(position[:2], [dx, dy])
    success = (
        failure is None
        and max_z > 0.10
        and bilateral > 20
        and landing < 0.045
        and position[2] < 0.04
    )
    frames.append(cell.frame("finished" if success else "failed"))
    metrics = {
        "lift_m": max_z - 0.02,
        "landing_error_m": landing,
        "bilateral_contact_steps": bilateral,
        "joint_tracking_rms": math.sqrt(error_sum / max(1, samples)),
        "simulation_time_s": float(cell.data.time),
        "steps": samples,
    }
    document = {
        "schema": "workcell-v1",
        "status": "SUCCEEDED" if success else "FAILED",
        "error": failure or (None if success else "GRASP_OR_PLACEMENT_FAILED"),
        "config": cfg,
        "metrics": metrics,
        "frames": frames,
        "solutions": solutions,
        "events": events,
        "seed": cfg["seed"],
        "frame_id": "world",
        "dt_s": 0.002,
        "source_hash": source_hash(),
        "model_hash": cell.model_hash,
        "robot_hash": cell.robot_hash,
        "engine_version": str(cell.mj.__version__),
        "application_version": __version__,
        "python_version": platform.python_version(),
        "platform": platform.system(),
        "numpy_version": str(np.__version__),
        "mapping": {
            "qpos": cell.q_indices,
            "qvel": cell.v_indices,
            "ctrl": cell.ctrl_indices,
            "nq": cell.model.nq,
            "nv": cell.model.nv,
        },
        "task": "contact_pick_place_v1",
        "attachment_constraints": 0,
    }
    document["config_hash"] = digest(cfg)
    document["task_hash"] = digest(stages)
    document["run_id"] = "cell_" + digest(document)[:24]
    document["replay_command"] = f"robostudio experiment replay {document['run_id']}"
    if output_dir is not None:
        directory = (output_dir / document["run_id"]).resolve()
        require(
            directory.is_relative_to(output_dir.resolve()), ErrorCode.INVALID_VALUE, "record path"
        )
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / "run.json"
        if path.exists():
            require(
                canonical(read_json(path)) == canonical(document),
                ErrorCode.INTEGRITY_ERROR,
                "existing record differs",
            )
        else:
            temporary = directory / "run.tmp"
            temporary.write_text(json.dumps(document, ensure_ascii=False), encoding="utf-8")
            temporary.replace(path)
    return document


def cell_catalog() -> dict[str, Any]:
    return {
        "robots": ROBOT_NAMES,
        "algorithms": SOLVERS,
        "config": default_config(),
        "phases": PHASES,
    }


def load_cell_record(output: Path, identifier: str) -> dict[str, Any]:
    require(
        bool(re.fullmatch(r"cell_[a-f0-9]{24}", identifier)),
        ErrorCode.INVALID_VALUE,
        "invalid record ID",
    )
    path = (output / identifier / "run.json").resolve()
    require(
        path.is_relative_to(output.resolve()) and path.is_file(),
        ErrorCode.NOT_FOUND,
        "record missing",
    )
    require(path.stat().st_size < 16_000_000, ErrorCode.INVALID_VALUE, "record too large")
    data = read_json(path)
    require(isinstance(data, dict), ErrorCode.INTEGRITY_ERROR, "invalid record")
    result = dict(cast(dict[str, Any], data))
    identity = result.pop("run_id", None)
    replay_command = result.pop("replay_command", None)
    require(
        identity == identifier == "cell_" + digest(result)[:24],
        ErrorCode.INTEGRITY_ERROR,
        "record hash mismatch",
    )
    result["run_id"], result["replay_command"] = identity, replay_command
    return result


def records(output: Path) -> list[dict[str, Any]]:
    results = []
    for path in sorted(
        output.glob("cell_*/run.json"), key=lambda p: p.stat().st_mtime, reverse=True
    )[:40]:
        try:
            doc = load_cell_record(output, path.parent.name)
            results.append({key: doc[key] for key in ("run_id", "status", "config", "metrics")})
        except (ValueError, OSError):
            continue
    return results


def handle_request(action: str, payload: Any, output: Path) -> Any:
    if action == "preview":
        return preview(payload)
    if action == "run":
        return run_cell(payload, output)
    if action == "replay":
        request = record(payload, "replay", {"run_id"})
        require(isinstance(request["run_id"], str), ErrorCode.INVALID_VALUE, "expected run ID")
        old = load_cell_record(output, str(request["run_id"]))
        new = run_cell(old["config"])
        require(
            canonical(old) == canonical(new),
            ErrorCode.INTEGRITY_ERROR,
            "replay differs; source or environment changed",
        )
        return {"status": "IDENTICAL", "run_id": old["run_id"]}
    if action in ("ik", "compare"):
        request = record(payload, action, {"config", "target_m", "initial_q"})
        cell = Cell(request["config"])
        target, initial = request["target_m"], request["initial_q"]
        require(
            isinstance(target, list) and (initial is None or isinstance(initial, list)),
            ErrorCode.INVALID_VALUE,
            "target and initial must be arrays",
        )
        if action == "compare":
            return {
                "results": [
                    solve(cell, target, algorithm, initial, cell.config["damping"])  # type: ignore[arg-type]
                    for algorithm in ("dls", "pinv", "transpose")
                ],
                "config": cell.config,
            }
        solution = solve(cell, target, cell.config["solver"], initial, cell.config["damping"])  # type: ignore[arg-type]
        # Preview only: the real runtime never writes joint poses after reset.
        cell.data.qpos[cell.q_indices] = solution["q"]
        cell.mj.mj_forward(cell.model, cell.data)
        return {"solution": solution, "frame": cell.frame("ik_preview")}
    if action == "spec":
        from .paper_spec import draft_spec

        return draft_spec(payload)
    if action == "candidate":
        from .paper_spec import generate_candidate

        return generate_candidate(payload, output)
    require(False, ErrorCode.NOT_FOUND, "unknown workcell action")
