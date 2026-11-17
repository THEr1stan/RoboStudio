"""Validated, project-authored scene adapter. All lengths in metres, angles in radians."""

import copy
import math
import xml.etree.ElementTree as ET
from dataclasses import asdict
from typing import Any

from ..cell_plugins import require_algorithm
from ..core import ErrorCode, RobotState
from ..core.contracts import finite, vector
from ..core.errors import require
from ..workcell import Cell, default_config, scene_xml, validate_config

COLORS = {"orange": [0.95, 0.40, 0.12], "blue": [0.16, 0.42, 0.86], "mint": [0.12, 0.66, 0.49]}


def default_scene(robot: str = "r6") -> dict[str, Any]:
    return {
        "robot": robot,
        "seed": 42,
        "controller": "computed",
        "solver": "dls",
        "damping": 0.035,
        "planner": "rrt_connect",
        "trajectory": "quintic",
        "task": "single",
        "observation": "camera",
        "gravity": 9.81,
        "noise_mm": 0.0,
        "dropout": 0.0,
        "latency_ms": 0,
        "retries": 1,
        "destination": [0.42, 0.20],
        "objects": [
            {
                "color": color,
                "shape": "box",
                "xy": xy,
                "size": [0.02, 0.02, 0.02],
                "yaw": 0.0,
                "mass": 0.045,
                "friction": 1.2,
            }
            for color, xy in zip(COLORS, [[0.42, -0.16], [0.32, -0.23], [0.53, -0.23]], strict=True)
        ],
        "obstacles": [],
    }


def bounded(value: Any, low: float, high: float, name: str) -> float:
    finite(value, name)
    require(low <= value <= high, ErrorCode.INVALID_VALUE, f"{name}: {low}..{high}")
    return float(value)


def scene_vector(value: Any, name: str, size: int) -> tuple[float, ...]:
    require(isinstance(value, list), ErrorCode.INVALID_VALUE, f"{name} must be an array")
    return vector(tuple(value), name, size)


def validate_scene(value: Any) -> dict[str, Any]:
    if isinstance(value, dict) and "trajectory" not in value:
        value = {**value, "trajectory": "quintic"}
    require(
        isinstance(value, dict) and set(value) == set(default_scene()),
        ErrorCode.INVALID_VALUE,
        "invalid scene fields",
    )
    cfg: dict[str, Any] = copy.deepcopy(value)
    baseline = default_config()
    for key in ("robot", "seed", "controller", "solver", "damping"):
        baseline[key] = cfg[key]
    validate_config(baseline)
    for key, choices in (
        ("planner", ("direct", "rrt", "rrt_connect", "prm")),
        ("trajectory", ("quintic", "cubic", "trapezoidal")),
        ("task", ("single", "sort", "stack")),
        ("observation", ("camera", "truth")),
    ):
        require(
            isinstance(cfg[key], str) and cfg[key] in choices,
            ErrorCode.INVALID_VALUE,
            f"invalid {key}",
        )
    for key, lo, hi in (("gravity", 1, 20), ("noise_mm", 0, 8), ("dropout", 0, 0.4)):
        cfg[key] = bounded(cfg[key], lo, hi, key)
    require_algorithm(cfg["trajectory"], frozenset({"joint_path"}))
    for key, lo, hi in (("latency_ms", 0, 500), ("retries", 0, 2)):
        require(
            type(cfg[key]) is int and lo <= cfg[key] <= hi,
            ErrorCode.INVALID_VALUE,
            f"invalid {key}",
        )
    xy = scene_vector(cfg["destination"], "destination", 2)
    require(
        0.28 <= xy[0] <= 0.55 and 0.08 <= xy[1] <= 0.30,
        ErrorCode.INVALID_VALUE,
        "destination outside receiving area",
    )
    require(
        isinstance(cfg["objects"], list) and 1 <= len(cfg["objects"]) <= 3,
        ErrorCode.INVALID_VALUE,
        "scene needs 1..3 objects",
    )
    colors: set[str] = set()
    for obj in cfg["objects"]:
        require(
            isinstance(obj, dict)
            and set(obj) == {"color", "shape", "xy", "size", "yaw", "mass", "friction"},
            ErrorCode.INVALID_VALUE,
            "invalid object",
        )
        require(
            isinstance(obj["color"], str) and obj["color"] in COLORS and obj["color"] not in colors,
            ErrorCode.INVALID_VALUE,
            "unique supported color required",
        )
        colors.add(obj["color"])
        require(obj["shape"] in ("box", "cylinder"), ErrorCode.INVALID_VALUE, "invalid shape")
        x, y = scene_vector(obj["xy"], "object XY", 2)
        require(
            0.26 <= x <= 0.57 and -0.30 <= y <= 0.30,
            ErrorCode.INVALID_VALUE,
            "object outside table",
        )
        size = scene_vector(obj["size"], "half size", 3)
        require(
            all(0.012 <= v <= 0.03 for v in size), ErrorCode.INVALID_VALUE, "half size .012..03"
        )
        for key, low, high in (
            ("yaw", -math.pi, math.pi),
            ("mass", 0.01, 0.3),
            ("friction", 0.05, 2),
        ):
            obj[key] = bounded(obj[key], low, high, key)
    require(
        isinstance(cfg["obstacles"], list) and len(cfg["obstacles"]) <= 4,
        ErrorCode.INVALID_VALUE,
        "maximum 4 obstacles",
    )
    for obstacle in cfg["obstacles"]:
        require(
            isinstance(obstacle, dict) and set(obstacle) == {"xy", "size", "yaw"},
            ErrorCode.INVALID_VALUE,
            "invalid obstacle",
        )
        x, y = scene_vector(obstacle["xy"], "obstacle XY", 2)
        require(
            0.15 <= x <= 0.65 and abs(y) <= 0.32, ErrorCode.INVALID_VALUE, "obstacle outside table"
        )
        size = scene_vector(obstacle["size"], "obstacle half size", 3)
        require(
            all(0.01 <= v <= 0.16 for v in size), ErrorCode.INVALID_VALUE, "obstacle size .01..16"
        )
        bounded(obstacle["yaw"], -math.pi, math.pi, "obstacle yaw")
    return cfg


def build_scene(cfg: dict[str, Any], *, live: bool = False) -> tuple[dict[str, Any], str]:
    baseline = default_config(cfg["robot"])
    for key in ("seed", "controller", "solver", "damping"):
        baseline[key] = cfg[key]
    root = ET.fromstring(scene_xml(baseline))
    world = root.find("worldbody")
    option = root.find("option")
    assert world is not None and option is not None
    option.set("gravity", f"0 0 {-cfg['gravity']}")
    ET.SubElement(
        world,
        "geom",
        name="floor",
        type="box",
        pos=".3 0 -.09",
        size="2 2 .015",
        rgba=".96 .96 .97 1",
        contype="4",
        conaffinity="3",
    )
    tool = (
        world.find(".//body[@name='link6']")
        if cfg["robot"] == "r6"
        else world.find(".//body[@name='tool']")
    )
    assert tool is not None
    ET.SubElement(
        tool,
        "geom",
        name="gripper_palm",
        type="box",
        pos=".091 0 0" if cfg["robot"] == "r6" else "-.009 0 0",
        size=".014 .065 .024",
        mass=".08",
        contype="1",
        conaffinity="7",
        rgba=".29 .32 .38 1",
    )
    for i in range(3):
        body = world.find(f"body[@name='block{i}']")
        assert body is not None
        world.remove(body)
    for i, obj in enumerate(cfg["objects"]):
        x, y = obj["xy"]
        sx, sy, sz = obj["size"]
        body = ET.SubElement(
            world, "body", name=f"block{i}", pos=f"{x} {y} {sz + 0.003}", euler=f"0 0 {obj['yaw']}"
        )
        ET.SubElement(body, "freejoint", name=f"free{i}")
        ET.SubElement(
            body,
            "geom",
            name=f"cube{i}",
            type=obj["shape"],
            size=f"{sx} {sy} {sz}" if obj["shape"] == "box" else f"{sx} {sz}",
            mass=str(obj["mass"]),
            friction=f"{obj['friction']} .02 .001",
            rgba=" ".join(map(str, [*COLORS[obj["color"]], 1])),
            group="1",
            contype="2",
            conaffinity="7",
            condim="4",
            solref=".006 1",
        )
    for i, obs in enumerate(cfg["obstacles"]):
        x, y = obs["xy"]
        parent = world
        if live:
            parent = ET.SubElement(
                world,
                "body",
                name=f"obstacle_body{i}",
                mocap="true",
                pos=f"{x} {y} {obs['size'][2]}",
            )
        ET.SubElement(
            parent,
            "geom",
            name=f"obstacle{i}",
            type="box",
            pos="0 0 0" if live else f"{x} {y} {obs['size'][2]}",
            euler=f"0 0 {obs['yaw']}",
            size=" ".join(map(str, obs["size"])),
            rgba=".37 .39 .43 1",
            group="1",
            contype="4",
            conaffinity="3",
        )
    for geom in world.iter("geom"):
        if geom.get("name") == "table":
            geom.set("rgba", ".86 .87 .89 1")
            geom.set("friction", ".05 .01 .001")
        elif geom.get("name", "").startswith("pad_"):
            geom.set("rgba", ".16 .18 .22 1")
            # MuJoCo combines friction with max(); keep pads below the object's setting.
            geom.set("friction", ".05 .02 .001")
        elif geom.get("rgba") is None:
            geom.set("rgba", ".82 .85 .89 1")
    carriage = world.find(".//body[@name='rail_z']/geom")
    if carriage is not None:
        carriage.set("pos", "0 0 .10")
        carriage.set("size", ".018 .018 .10")
    # Exclude mechanically adjacent links, but enable distal self-collision.
    parents: dict[str, str] = {}

    def visit(parent: ET.Element, name: str) -> None:
        for body in parent.findall("body"):
            child = body.get("name", "")
            if child.startswith("block"):
                continue
            parents[child] = name
            visit(body, child)

    visit(world, "world")
    contact = ET.SubElement(root, "contact")
    for a in parents:
        near = {a, parents[a], parents.get(parents[a], "world")}
        for b in parents:
            if a >= b:
                continue
            if (
                b in near
                or a in {parents[b], parents.get(parents[b], "world")}
                or parents[a] == parents[b]
            ):
                ET.SubElement(contact, "exclude", body1=a, body2=b)
    for geom in world.iter("geom"):
        if geom.get("contype") == "1":
            geom.set("conaffinity", "7")
    return baseline, ET.tostring(root, encoding="unicode")


class StudioCell(Cell):
    def __init__(self, scene: Any, *, live: bool = False):
        self.live = live
        self.scene = validate_scene(scene)
        baseline, xml = build_scene(self.scene, live=live)
        super().__init__(baseline, xml_override=xml)
        self.object_bodies = [
            self.model.body(f"block{i}").id for i in range(len(self.scene["objects"]))
        ]
        self.object_geoms = [
            self.model.geom(f"cube{i}").id for i in range(len(self.scene["objects"]))
        ]
        self.obstacle_geoms = [
            self.model.geom(f"obstacle{i}").id for i in range(len(self.scene["obstacles"]))
        ]
        self.robot_geoms = [
            i
            for i in range(self.model.ngeom)
            if self.model.geom_bodyid[i] != 0
            and i not in self.object_geoms
            and i not in self.obstacle_geoms
            and self.model.geom_contype[i] != 0
        ]
        # A raised reset pose leaves room for tabletop obstacles and the overhead camera.
        self.data.qpos[self.q_indices] = (
            [math.radians(v) for v in (0, -85, 80, 0, 95, 0)]
            if len(self.joint_ids) == 6
            else [0.28, -0.05, -0.14]
        )
        self.mj.mj_forward(self.model, self.data)
        self.self_pairs = []
        for a in self.robot_geoms:
            if self.model.geom_contype[a] == 0:
                continue
            ba = int(self.model.geom_bodyid[a])
            for b in self.robot_geoms:
                if b <= a or self.model.geom_contype[b] == 0:
                    continue
                bb = int(self.model.geom_bodyid[b])
                pa, pb = int(self.model.body_parentid[ba]), int(self.model.body_parentid[bb])
                if (
                    ba == bb
                    or ba in (pb, int(self.model.body_parentid[pb]))
                    or bb in (pa, int(self.model.body_parentid[pa]))
                    or pa == pb
                ):
                    continue
                self.self_pairs.append((a, b))

    def contact_reading(self) -> dict[str, Any]:
        forces = [0.0, 0.0]
        touching: dict[int, set[int]] = {}
        unintended = []
        force = self.np.zeros(6)
        for index, contact in enumerate(self.data.contact):
            a, b = int(contact.geom1), int(contact.geom2)
            self.mj.mj_contactForce(self.model, self.data, index, force)
            magnitude = float(self.np.linalg.norm(force[:3]))
            if magnitude < 0.01:
                continue
            for pad, pad_id in enumerate(self.pad_ids):
                if pad_id in (a, b):
                    forces[pad] += magnitude
                    other = b if a == pad_id else a
                    if other in self.object_geoms:
                        touching.setdefault(self.object_geoms.index(other), set()).add(pad)
            if (a in self.robot_geoms or b in self.robot_geoms) and not (
                (a in self.pad_ids and b in self.object_geoms)
                or (b in self.pad_ids and a in self.object_geoms)
            ):
                unintended.append([a, b, magnitude])
        return {
            "finger_force_n": forces,
            "bilateral_objects": [i for i, pads in touching.items() if len(pads) == 2],
            "unintended": unintended,
            "contacts": int(self.data.ncon),
        }

    def frame(self, phase: str) -> dict[str, Any]:
        m, d = self.model, self.data
        geoms = [
            {
                "id": i,
                **(
                    {
                        "body_id": int(m.geom_bodyid[i]),
                        "pick_kind": "object"
                        if i in self.object_geoms
                        else "obstacle"
                        if i in self.obstacle_geoms
                        else "body"
                        if i in self.robot_geoms and m.body_dofnum[m.geom_bodyid[i]] > 0
                        else "fixed",
                    }
                    if self.live
                    else {}
                ),
                "name": m.geom(i).name,
                "type": int(m.geom_type[i]),
                "size": m.geom_size[i].tolist(),
                "position": d.geom_xpos[i].tolist(),
                "rotation": d.geom_xmat[i].reshape(3, 3).tolist(),
                "color": m.geom_rgba[i].tolist(),
            }
            for i in range(m.ngeom)
        ]
        state = RobotState(
            **self.identity(),
            qpos_si=tuple(float(x) for x in d.qpos[self.q_indices]),
            qvel_si=tuple(float(x) for x in d.qvel[self.v_indices]),
        )
        return {
            "state": asdict(state),
            "reference": asdict(self.last_reference) if self.last_reference else None,
            "command": asdict(self.last_command) if self.last_command else None,
            "scene_state": {
                "qpos": d.qpos.tolist(),
                "qvel": d.qvel.tolist(),
                "ctrl": d.ctrl.tolist(),
                **(
                    {"mocap_pos": d.mocap_pos.tolist(), "mocap_quat": d.mocap_quat.tolist()}
                    if self.live
                    else {}
                ),
            },
            "time_s": float(d.time),
            "phase": phase,
            "robot": self.scene["robot"],
            "q": d.qpos[self.q_indices].tolist(),
            "qvel": d.qvel[self.v_indices].tolist(),
            "tcp": d.site_xpos[self.tcp].tolist(),
            "rotation": d.site_xmat[self.tcp].reshape(3, 3).tolist(),
            "points": [d.xanchor[j].tolist() for j in self.joint_ids]
            + [d.site_xpos[self.tcp].tolist()],
            "jointAxes": [d.xaxis[j].tolist() for j in self.joint_ids],
            "geoms": geoms,
            "objects": [geoms[i] for i in self.object_geoms],
            **(
                {
                    "obstacles": [geoms[i] for i in self.obstacle_geoms],
                    "destination": self.scene["destination"],
                }
                if self.live
                else {}
            ),
            "contact_sensor": self.contact_reading(),
        }
