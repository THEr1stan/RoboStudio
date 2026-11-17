"""Strict v1alpha1 schemas; validation never imports code from a manifest."""

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import cast

from robostudio.core import (
    ActuatorMapping,
    Capability,
    CapabilitySet,
    ErrorCode,
    JointGroup,
    JointMapping,
    ModelReference,
    RoboStudioError,
    RobotSpec,
)
from robostudio.core.contracts import finite
from robostudio.core.errors import require
from robostudio.serialization import digest


def record(value: object, path: str, keys: set[str] | None = None) -> dict[str, object]:
    require(isinstance(value, dict), ErrorCode.INVALID_MANIFEST, "expected an object", path)
    result = cast(dict[str, object], value)
    require(
        all(isinstance(k, str) for k in result),
        ErrorCode.INVALID_MANIFEST,
        "keys must be strings",
        path,
    )
    if keys is not None:
        missing, extra = keys - result.keys(), result.keys() - keys
        require(
            not missing and not extra,
            ErrorCode.INVALID_MANIFEST,
            f"missing fields: {sorted(missing)}; unknown fields: {sorted(extra)}",
            path,
        )
    return result


def sequence(value: object, path: str) -> list[object]:
    require(isinstance(value, list), ErrorCode.INVALID_MANIFEST, "expected an array", path)
    return cast(list[object], value)


def string(value: object, path: str) -> str:
    require(
        isinstance(value, str) and bool(value.strip()),
        ErrorCode.INVALID_MANIFEST,
        "expected a nonempty string",
        path,
    )
    return cast(str, value)


def integer(value: object, path: str) -> int:
    require(
        type(value) is int and value >= 0,
        ErrorCode.INVALID_MANIFEST,
        "expected a nonnegative integer",
        path,
    )
    return cast(int, value)


def version(value: object) -> str:
    require(
        value == "v1alpha1",
        ErrorCode.UNSUPPORTED_VERSION,
        "supported api_version: v1alpha1",
        "api_version",
    )
    return "v1alpha1"


def capability(value: object, path: str) -> Capability:
    try:
        return Capability(string(value, path))
    except ValueError as error:
        raise RoboStudioError(
            ErrorCode.UNKNOWN_CAPABILITY, f"unknown capability: {value}", path
        ) from error


def capabilities(value: object, path: str) -> CapabilitySet:
    items = [capability(v, path) for v in sequence(value, path)]
    require(len(set(items)) == len(items), ErrorCode.DUPLICATE_ID, "duplicate capabilities", path)
    return CapabilitySet(frozenset(items))


def parse_robot(value: object) -> RobotSpec:
    d = record(
        value,
        "robot",
        {
            "api_version",
            "robot_id",
            "family",
            "frame_id",
            "model",
            "capabilities",
            "qpos_size",
            "qvel_size",
            "ctrl_size",
            "joints",
            "joint_groups",
            "actuator_mapping",
        },
    )
    api = version(d["api_version"])
    model = record(d["model"], "model", {"path", "format", "sha256"})
    joints: list[JointMapping] = []
    for i, value in enumerate(sequence(d["joints"], "joints")):
        path = f"joints[{i}]"
        j = record(value, path, {"name", "kind", "qpos_indices", "qvel_indices"})
        joints.append(
            JointMapping(
                string(j["name"], path + ".name"),
                string(j["kind"], path + ".kind"),
                tuple(integer(v, path) for v in sequence(j["qpos_indices"], path)),
                tuple(integer(v, path) for v in sequence(j["qvel_indices"], path)),
            )
        )
    groups: list[JointGroup] = []
    for value in sequence(d["joint_groups"], "joint_groups"):
        g = record(value, "joint_groups", {"group_id", "joint_names"})
        groups.append(
            JointGroup(
                string(g["group_id"], "group_id"),
                tuple(string(v, "joint_names") for v in sequence(g["joint_names"], "joint_names")),
            )
        )
    actuators: list[ActuatorMapping] = []
    for value in sequence(d["actuator_mapping"], "actuator_mapping"):
        a = record(value, "actuator_mapping", {"name", "joint_name", "ctrl_index", "effort_unit"})
        actuators.append(
            ActuatorMapping(
                string(a["name"], "actuator_mapping.name"),
                string(a["joint_name"], "joint_name"),
                integer(a["ctrl_index"], "ctrl_index"),
                string(a["effort_unit"], "effort_unit"),
            )
        )
    return RobotSpec(
        api_version=api,
        robot_id=string(d["robot_id"], "robot_id"),
        family=string(d["family"], "family"),
        frame_id=string(d["frame_id"], "frame_id"),
        robot_hash=digest(d),
        model=ModelReference(
            string(model["path"], "model.path"),
            string(model["format"], "model.format"),
            string(model["sha256"], "model.sha256"),
        ),
        capabilities=capabilities(d["capabilities"], "capabilities"),
        qpos_size=integer(d["qpos_size"], "qpos_size"),
        qvel_size=integer(d["qvel_size"], "qvel_size"),
        ctrl_size=integer(d["ctrl_size"], "ctrl_size"),
        joints=tuple(joints),
        joint_groups=tuple(groups),
        actuator_mapping=tuple(actuators),
    )


@dataclass(frozen=True)
class ParameterSpec:
    kind: str
    minimum: float
    maximum: float


@dataclass(frozen=True)
class PluginManifest:
    api_version: str
    plugin_id: str
    requires_capabilities: CapabilitySet
    provides_capability: Capability
    parameters: Mapping[str, ParameterSpec]

    def __post_init__(self) -> None:
        object.__setattr__(self, "parameters", MappingProxyType(dict(self.parameters)))

    def validate_parameters(self, value: object) -> dict[str, float]:
        data = record(value, "parameters", set(self.parameters))
        result: dict[str, float] = {}
        for key, spec in self.parameters.items():
            v = data[key]
            types = (int,) if spec.kind == "integer" else (int, float)
            require(type(v) in types, ErrorCode.INVALID_VALUE, f"expected {spec.kind}", key)
            n = cast(float, v)
            finite(n, key)
            require(
                spec.minimum <= n <= spec.maximum,
                ErrorCode.INVALID_VALUE,
                f"expected [{spec.minimum}, {spec.maximum}]",
                key,
            )
            result[key] = n
        return result


def parse_plugin(value: object) -> PluginManifest:
    d = record(
        value,
        "plugin",
        {"api_version", "plugin_id", "requires_capabilities", "provides_capability", "parameters"},
    )
    api = version(d["api_version"])
    parameters: dict[str, ParameterSpec] = {}
    for name, value in record(d["parameters"], "parameters").items():
        string(name, "parameter name")
        p = record(value, name, {"type", "minimum", "maximum"})
        require(
            p["type"] in ("number", "integer"),
            ErrorCode.INVALID_MANIFEST,
            "supported parameter types: number, integer",
            name,
        )
        for key in ("minimum", "maximum"):
            finite(cast(float, p[key]), f"{name}.{key}")
        lower, upper = cast(float, p["minimum"]), cast(float, p["maximum"])
        require(lower <= upper, ErrorCode.INVALID_MANIFEST, "invalid bounds", name)
        parameters[name] = ParameterSpec(cast(str, p["type"]), lower, upper)
    return PluginManifest(
        api,
        string(d["plugin_id"], "plugin_id"),
        capabilities(d["requires_capabilities"], "requires_capabilities"),
        capability(d["provides_capability"], "provides_capability"),
        parameters,
    )
