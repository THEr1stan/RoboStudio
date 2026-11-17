from copy import deepcopy

import pytest

from robostudio.core import Capability, ErrorCode, RoboStudioError
from robostudio.registry import Registry, parse_plugin, parse_robot


@pytest.mark.parametrize(
    "key,value,code",
    [
        ("robot_id", "", ErrorCode.INVALID_MANIFEST),
        ("api_version", "future", ErrorCode.UNSUPPORTED_VERSION),
        ("capabilities", ["magic"], ErrorCode.UNKNOWN_CAPABILITY),
        ("qpos_size", True, ErrorCode.INVALID_MANIFEST),
        ("surprise", 123, ErrorCode.INVALID_MANIFEST),
    ],
)
def test_invalid_fields(manifest, key, value, code):
    manifest[key] = value
    with pytest.raises(RoboStudioError) as error:
        parse_robot(manifest)
    assert error.value.code == code
    assert error.value.path


def test_missing_id(manifest):
    del manifest["robot_id"]
    with pytest.raises(RoboStudioError, match="robot_id"):
        parse_robot(manifest)


@pytest.mark.parametrize("mutation", ["index", "duplicate", "unknown_joint", "unit"])
def test_invalid_mapping(manifest, mutation):
    if mutation == "index":
        manifest["joints"][0]["qpos_indices"] = [10]
    elif mutation == "duplicate":
        manifest["joints"][1]["qpos_indices"] = [0]
    elif mutation == "unknown_joint":
        manifest["actuator_mapping"][0]["joint_name"] = "absent"
    else:
        manifest["actuator_mapping"][0]["effort_unit"] = "degrees"
    with pytest.raises(RoboStudioError) as error:
        parse_robot(manifest)
    assert error.value.code == ErrorCode.INVALID_MAPPING


def test_floating_base_has_unequal_position_velocity_sizes(manifest):
    manifest["joints"].append(
        {
            "name": "base",
            "kind": "free",
            "qpos_indices": list(range(2, 9)),
            "qvel_indices": list(range(2, 8)),
        }
    )
    manifest["qpos_size"] = 9
    manifest["qvel_size"] = 8
    spec = parse_robot(manifest)
    assert spec.qpos_size == 9 and spec.qvel_size == 8


def test_duplicate_robot_and_plugin_registration(robot):
    registry = Registry()
    registry.add_robot(robot)
    with pytest.raises(RoboStudioError) as error:
        registry.add_robot(robot)
    assert error.value.code == ErrorCode.DUPLICATE_ID
    plugin = parse_plugin(
        {
            "api_version": "v1alpha1",
            "plugin_id": "test.dummy",
            "requires_capabilities": ["joint_effort"],
            "provides_capability": "joint_effort",
            "parameters": {"gain": {"type": "number", "minimum": 0.0, "maximum": 2.0}},
        }
    )
    registry.add_plugin(plugin)
    assert registry.compatible_plugins(robot.robot_id, Capability.JOINT_EFFORT) == (plugin,)
    assert plugin.validate_parameters({"gain": 1.0}) == {"gain": 1.0}
    with pytest.raises(RoboStudioError):
        plugin.validate_parameters({"gain": 3.0})
    with pytest.raises(RoboStudioError):
        registry.add_plugin(plugin)
    changed = deepcopy(
        {
            "api_version": "v1alpha1",
            "plugin_id": "test.ik",
            "requires_capabilities": ["cartesian_ik"],
            "provides_capability": "cartesian_ik",
            "parameters": {},
        }
    )
    registry.add_plugin(parse_plugin(changed))
    with pytest.raises(RoboStudioError) as error:
        registry.require_plugin("test.ik", robot.robot_id)
    assert error.value.code == ErrorCode.UNSUPPORTED_CAPABILITY
