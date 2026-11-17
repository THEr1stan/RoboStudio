import ast
import dataclasses
import math
from pathlib import Path

import pytest

import robostudio.core
from robostudio.core import (
    ErrorCode,
    Pose,
    RoboStudioError,
    RobotCommand,
    RobotReference,
    RobotState,
)


def test_pose_uses_immutable_si_snapshot():
    position = [1.0, 2.0, 3.0]
    pose = Pose(position, [1.0, 0.0, 0.0, 0.0], "world")
    position[0] = 99.0
    assert pose.position_m == (1.0, 2.0, 3.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        pose.frame_id = "other"


@pytest.mark.parametrize(
    "position,quaternion,frame",
    [
        ([0, 0], [1, 0, 0, 0], "world"),
        ([0, 0, 0], [2, 0, 0, 0], "world"),
        ([math.nan, 0, 0], [1, 0, 0, 0], "world"),
        ([0, 0, 0], [1, 0, 0, 0], ""),
    ],
)
def test_invalid_pose_is_rejected(position, quaternion, frame):
    with pytest.raises(RoboStudioError):
        Pose(position, quaternion, frame)


def test_state_reference_command_are_distinct(robot):
    identity = dict(
        robot_id=robot.robot_id,
        robot_hash=robot.robot_hash,
        model_hash=robot.model.sha256,
        frame_id=robot.frame_id,
        time_s=0.0,
    )
    state = RobotState(**identity, qpos_si=(0.0, 0.0), qvel_si=(0.0, 0.0))
    reference = RobotReference(**identity, qpos_si=(1.0, 1.0), qvel_si=(0.0, 0.0))
    command = RobotCommand(**identity, effort_si=(1.0, 2.0))
    for item in (state, reference, command):
        item.validate_for(robot)
    assert type(state) is not type(reference)
    assert type(command) is not type(reference)
    with pytest.raises(RoboStudioError) as error:
        dataclasses.replace(state, qpos_si=(0.0,)).validate_for(robot)
    assert error.value.code == ErrorCode.INVALID_DIMENSION
    with pytest.raises(RoboStudioError) as error:
        dataclasses.replace(command, frame_id="tool").validate_for(robot)
    assert error.value.code == ErrorCode.FRAME_MISMATCH
    with pytest.raises(RoboStudioError) as error:
        dataclasses.replace(state, model_hash="0" * 64).validate_for(robot)
    assert error.value.code == ErrorCode.MODEL_MISMATCH


def test_core_only_imports_standard_library_or_core():
    import sys

    root = Path(robostudio.core.__file__).parent
    for path in root.glob("*.py"):
        for node in ast.walk(ast.parse(path.read_text("utf-8"))):
            if isinstance(node, ast.Import):
                names = [item.name for item in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0:
                names = [node.module or ""]
            else:
                continue
            for name in names:
                assert name.split(".")[0] in sys.stdlib_module_names or name.startswith(
                    "robostudio.core"
                ), (path, name)
