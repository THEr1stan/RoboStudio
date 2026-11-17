"""SI contracts only; no backend, algorithm, registry or UI dependencies."""

from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from math import isfinite, sqrt
from types import MappingProxyType
from typing import Protocol, runtime_checkable

from .errors import ErrorCode as E
from .errors import require


def finite(value: float, path: str) -> None:
    require(type(value) in (int, float), E.INVALID_VALUE, "expected a real number", path)
    try:
        valid = isfinite(value)
    except OverflowError:
        valid = False
    require(valid, E.INVALID_VALUE, "expected a finite number", path)


def vector(values: tuple[float, ...], path: str, size: int | None = None) -> tuple[float, ...]:
    result = tuple(values)
    if size is not None:
        require(len(result) == size, E.INVALID_DIMENSION, f"expected {size} values", path)
    for index, value in enumerate(result):
        finite(value, f"{path}[{index}]")
    return result


def identifier(value: str, path: str) -> None:
    require(
        isinstance(value, str) and bool(value.strip()),
        E.INVALID_VALUE,
        "expected a nonempty identifier",
        path,
    )


def sha256(value: str, path: str) -> None:
    require(
        isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value),
        E.INVALID_VALUE,
        "expected a lowercase SHA256",
        path,
    )


class Capability(StrEnum):
    JOINT_EFFORT = "joint_effort"
    JOINT_TRAJECTORY = "joint_trajectory"
    CARTESIAN_IK = "cartesian_ik"
    BASE_VELOCITY = "base_velocity"
    NAVIGATE_2D = "navigate_2d"
    FLOATING_BASE_CONTROL = "floating_base_control"
    COLLISION_CHECK = "collision_check"
    JOINT_PATH = "joint_path"
    RGBD_OBSERVATION = "rgbd_observation"
    OBJECT_POSE_OBSERVATION = "object_pose_observation"


@dataclass(frozen=True)
class CapabilitySet:
    values: frozenset[Capability]

    def __post_init__(self) -> None:
        values = frozenset(self.values)
        require(
            all(isinstance(c, Capability) for c in values),
            E.UNKNOWN_CAPABILITY,
            "use declared Capability values",
            "capabilities",
        )
        object.__setattr__(self, "values", values)

    def supports(self, required: "CapabilitySet") -> bool:
        return required.values <= self.values


@dataclass(frozen=True)
class Pose:
    position_m: tuple[float, ...]
    quaternion_wxyz: tuple[float, ...]
    frame_id: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_m", vector(self.position_m, "position_m", 3))
        q = vector(self.quaternion_wxyz, "quaternion_wxyz", 4)
        require(
            abs(sqrt(sum(x * x for x in q)) - 1.0) <= 1e-9,
            E.INVALID_VALUE,
            "quaternion must be normalized (wxyz)",
            "quaternion_wxyz",
        )
        object.__setattr__(self, "quaternion_wxyz", q)
        identifier(self.frame_id, "frame_id")


@dataclass(frozen=True)
class JointMapping:
    name: str
    kind: str
    qpos_indices: tuple[int, ...]
    qvel_indices: tuple[int, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "qpos_indices", tuple(self.qpos_indices))
        object.__setattr__(self, "qvel_indices", tuple(self.qvel_indices))


@dataclass(frozen=True)
class JointGroup:
    group_id: str
    joint_names: tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "joint_names", tuple(self.joint_names))


@dataclass(frozen=True)
class ActuatorMapping:
    name: str
    joint_name: str
    ctrl_index: int
    effort_unit: str


@dataclass(frozen=True)
class ModelReference:
    path: str
    format: str
    sha256: str


@dataclass(frozen=True)
class RobotSpec:
    api_version: str
    robot_id: str
    family: str
    frame_id: str
    robot_hash: str
    model: ModelReference
    capabilities: CapabilitySet
    qpos_size: int
    qvel_size: int
    ctrl_size: int
    joints: tuple[JointMapping, ...]
    joint_groups: tuple[JointGroup, ...]
    actuator_mapping: tuple[ActuatorMapping, ...]

    def __post_init__(self) -> None:
        for field in ("joints", "joint_groups", "actuator_mapping"):
            object.__setattr__(self, field, tuple(getattr(self, field)))
        require(
            self.api_version == "v1alpha1",
            E.UNSUPPORTED_VERSION,
            "expected v1alpha1",
            "api_version",
        )
        for field in ("robot_id", "family", "frame_id"):
            identifier(getattr(self, field), field)
        sha256(self.robot_hash, "robot_hash")
        sha256(self.model.sha256, "model.sha256")
        for field in ("qpos_size", "qvel_size", "ctrl_size"):
            value = getattr(self, field)
            require(
                type(value) is int and value >= 0,
                E.INVALID_DIMENSION,
                "expected a nonnegative integer",
                field,
            )
        require(bool(self.joints), E.INVALID_MAPPING, "at least one joint is required", "joints")
        names = [joint.name for joint in self.joints]
        require(len(set(names)) == len(names), E.DUPLICATE_ID, "duplicate joint names", "joints")
        widths = {"revolute": (1, 1), "prismatic": (1, 1), "free": (7, 6)}
        for joint in self.joints:
            identifier(joint.name, "joints.name")
            require(joint.kind in widths, E.INVALID_MAPPING, "unsupported joint kind", joint.name)
            require(
                (len(joint.qpos_indices), len(joint.qvel_indices)) == widths[joint.kind],
                E.INVALID_MAPPING,
                "incorrect qpos/qvel width",
                joint.name,
            )
        for attr, size in (("qpos_indices", self.qpos_size), ("qvel_indices", self.qvel_size)):
            indices = [i for joint in self.joints for i in getattr(joint, attr)]
            require(
                len(indices) == size
                and all(type(i) is int for i in indices)
                and sorted(indices) == list(range(size)),
                E.INVALID_MAPPING,
                "indices must cover the dimension exactly once",
                attr,
            )
        group_ids = [group.group_id for group in self.joint_groups]
        require(
            len(set(group_ids)) == len(group_ids),
            E.DUPLICATE_ID,
            "duplicate joint group IDs",
            "joint_groups",
        )
        require(
            bool(group_ids), E.INVALID_MAPPING, "at least one group is required", "joint_groups"
        )
        for group in self.joint_groups:
            identifier(group.group_id, "joint_groups.group_id")
            require(
                bool(group.joint_names)
                and len(set(group.joint_names)) == len(group.joint_names)
                and set(group.joint_names) <= set(names),
                E.INVALID_MAPPING,
                "group must contain unique known joints",
                group.group_id,
            )
        actuator_names = [a.name for a in self.actuator_mapping]
        require(
            len(set(actuator_names)) == len(actuator_names),
            E.DUPLICATE_ID,
            "duplicate actuator names",
            "actuator_mapping",
        )
        indices = [a.ctrl_index for a in self.actuator_mapping]
        require(
            len(indices) == self.ctrl_size
            and all(type(i) is int for i in indices)
            and sorted(indices) == list(range(self.ctrl_size)),
            E.INVALID_MAPPING,
            "ctrl indices must cover the dimension exactly once",
            "actuator_mapping",
        )
        joint_by_name = {j.name: j for j in self.joints}
        for actuator in self.actuator_mapping:
            identifier(actuator.name, "actuator_mapping.name")
            require(
                actuator.joint_name in joint_by_name,
                E.INVALID_MAPPING,
                "actuator references an unknown joint",
                "actuator_mapping",
            )
            kind = joint_by_name[actuator.joint_name].kind
            require(
                kind in ("revolute", "prismatic"),
                E.INVALID_MAPPING,
                "v1alpha1 supports scalar effort actuators only",
                "actuator_mapping",
            )
            unit = "N*m" if kind == "revolute" else "N"
            require(
                actuator.effort_unit == unit,
                E.INVALID_MAPPING,
                f"expected effort unit {unit}",
                "actuator_mapping.effort_unit",
            )


@dataclass(frozen=True, kw_only=True)
class _RobotSample:
    robot_id: str
    robot_hash: str
    model_hash: str
    frame_id: str
    time_s: float

    def __post_init__(self) -> None:
        identifier(self.robot_id, "robot_id")
        identifier(self.frame_id, "frame_id")
        sha256(self.robot_hash, "robot_hash")
        sha256(self.model_hash, "model_hash")
        finite(self.time_s, "time_s")
        require(self.time_s >= 0, E.INVALID_VALUE, "time must be nonnegative", "time_s")

    def validate_for(self, robot: RobotSpec) -> None:
        require(
            self.robot_id == robot.robot_id
            and self.robot_hash == robot.robot_hash
            and self.model_hash == robot.model.sha256,
            E.MODEL_MISMATCH,
            "robot identity or model hash differs",
            "robot_id/robot_hash/model_hash",
        )
        require(
            self.frame_id == robot.frame_id,
            E.FRAME_MISMATCH,
            f"expected frame {robot.frame_id}",
            "frame_id",
        )


@dataclass(frozen=True, kw_only=True)
class RobotState(_RobotSample):
    """qpos: rad/m; free = xyz(m), wxyz. qvel: rad/s, m/s; free = linear, angular."""

    qpos_si: tuple[float, ...]
    qvel_si: tuple[float, ...]

    def __post_init__(self) -> None:
        super().__post_init__()
        object.__setattr__(self, "qpos_si", vector(self.qpos_si, "qpos_si"))
        object.__setattr__(self, "qvel_si", vector(self.qvel_si, "qvel_si"))

    def validate_for(self, robot: RobotSpec) -> None:
        super().validate_for(robot)
        vector(self.qpos_si, "qpos_si", robot.qpos_size)
        vector(self.qvel_si, "qvel_si", robot.qvel_size)
        for joint in robot.joints:
            if joint.kind == "free":
                q = tuple(self.qpos_si[i] for i in joint.qpos_indices[3:])
                Pose((0.0, 0.0, 0.0), q, self.frame_id)


@dataclass(frozen=True, kw_only=True)
class RobotReference(_RobotSample):
    """Desired generalized positions and velocities, with the same SI layout as RobotState."""

    qpos_si: tuple[float, ...]
    qvel_si: tuple[float, ...]

    def __post_init__(self) -> None:
        super().__post_init__()
        object.__setattr__(self, "qpos_si", vector(self.qpos_si, "qpos_si"))
        object.__setattr__(self, "qvel_si", vector(self.qvel_si, "qvel_si"))

    def validate_for(self, robot: RobotSpec) -> None:
        super().validate_for(robot)
        RobotState(**{k: getattr(self, k) for k in self.__dataclass_fields__}).validate_for(robot)


@dataclass(frozen=True, kw_only=True)
class RobotCommand(_RobotSample):
    """Efforts in ctrl order; units are N or N*m per actuator_mapping."""

    effort_si: tuple[float, ...]

    def __post_init__(self) -> None:
        super().__post_init__()
        object.__setattr__(self, "effort_si", vector(self.effort_si, "effort_si"))

    def validate_for(self, robot: RobotSpec) -> None:
        super().validate_for(robot)
        vector(self.effort_si, "effort_si", robot.ctrl_size)


class RunStatus(StrEnum):
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class RunResult:
    run_id: str
    scenario_id: str
    config_hash: str
    scenario_hash: str
    source_hash: str
    robot_hash: str
    model_hash: str
    seed: int
    status: RunStatus
    metrics: Mapping[str, float]
    error_code: E | None = None

    def __post_init__(self) -> None:
        for key in ("config_hash", "scenario_hash", "source_hash", "robot_hash", "model_hash"):
            sha256(getattr(self, key), key)
        require(
            type(self.seed) is int and 0 <= self.seed < 2**32,
            E.INVALID_VALUE,
            "seed must be a uint32",
            "seed",
        )
        require(isinstance(self.status, RunStatus), E.INVALID_VALUE, "invalid run status", "status")
        require(
            (self.status == RunStatus.FAILED) == (self.error_code is not None),
            E.INVALID_VALUE,
            "failed results require an error code",
            "error_code",
        )
        for key, value in self.metrics.items():
            identifier(key, "metrics")
            finite(value, key)
        object.__setattr__(self, "metrics", MappingProxyType(dict(self.metrics)))

    def as_dict(self) -> dict[str, object]:
        return {
            key: dict(value) if key == "metrics" else value
            for key in self.__dataclass_fields__
            for value in (getattr(self, key),)
        }


@runtime_checkable
class SimulationBackend(Protocol):
    def load(self, robot: RobotSpec) -> None: ...
    def reset(self, seed: int) -> None: ...
    def observe(self) -> RobotState: ...
    def apply(self, command: RobotCommand) -> None: ...
    def step(self, dt_s: float) -> None: ...
    def close(self) -> None: ...
