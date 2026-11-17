import hashlib
from dataclasses import replace
from importlib.resources import files

import pytest

pytest.importorskip("mujoco", reason="optional physics extra is not installed")

from robostudio.backends.mujoco_backend import MuJoCoBackend
from robostudio.core import RoboStudioError, RobotCommand
from robostudio.kinematics import forward_kinematics, verify_model
from robostudio.registry import default_registry


@pytest.fixture
def arm():
    robot = default_registry().robot("r6_compact")
    backend = MuJoCoBackend()
    backend.load(robot)
    backend.reset(42)
    yield robot, backend
    backend.close()


def test_independent_fk_matches_all_links_and_tcp(arm):
    robot, backend = arm
    backend.set_configuration((0.0,) * 6)
    assert backend.scene()["tcp"] == pytest.approx([0.84, 0, 0.22], abs=1e-12)
    report = verify_model(backend, seed=42, samples=100)
    assert report["configurations"] == 101
    assert report["max_position_error_m"] < 1e-12
    assert report["max_rotation_error"] < 1e-12
    assert backend.observe().model_hash == robot.model.sha256


@pytest.mark.parametrize(
    "old,new",
    [
        (b'axis="0 1 0"', b'axis="0 -1 0"'),
        (b'name="tcp" pos="0.1 0 0"', b'name="tcp" pos="0.12 0 0"'),
    ],
)
def test_fk_detects_wrong_axis_or_tcp(old, new):
    robot = default_registry().robot("r6_compact")
    xml = files("robostudio").joinpath("data/r6_compact.xml").read_bytes()
    changed = xml.replace(old, new, 1)
    assert changed != xml
    robot = replace(robot, model=replace(robot.model, sha256=hashlib.sha256(changed).hexdigest()))
    backend = MuJoCoBackend(changed)
    backend.load(robot)
    backend.reset(42)
    with pytest.raises(RoboStudioError, match="MODEL_MISMATCH"):
        verify_model(backend, seed=42, samples=10)
    backend.close()


def test_lifecycle_mapping_hash_and_unsupported_frame(arm):
    robot, backend = arm
    with pytest.raises(RoboStudioError, match="INVALID_LIFECYCLE"):
        MuJoCoBackend().observe()
    with pytest.raises(RoboStudioError, match="MODEL_MISMATCH"):
        MuJoCoBackend().load(replace(robot, model=replace(robot.model, sha256="0" * 64)))
    joints = list(robot.joints)
    joints[0] = replace(joints[0], qpos_indices=(1,))
    joints[1] = replace(joints[1], qpos_indices=(0,))
    with pytest.raises(RoboStudioError, match="INVALID_MAPPING"):
        MuJoCoBackend().load(replace(robot, joints=tuple(joints)))
    with pytest.raises(RoboStudioError, match="FRAME_MISMATCH"):
        MuJoCoBackend().load(replace(robot, frame_id="base"))
    backend.close()
    with pytest.raises(RoboStudioError, match="INVALID_LIFECYCLE"):
        backend.step(0.002)


def test_torque_command_timestamp_bounds_and_state_copy(arm):
    robot, backend = arm
    before = backend.observe()
    command = RobotCommand(
        robot_id=robot.robot_id,
        robot_hash=robot.robot_hash,
        model_hash=robot.model.sha256,
        frame_id="world",
        time_s=0,
        effort_si=(1, 0, 0, 0, 0, 0),
    )
    backend.apply(command)
    backend.step(0.002)
    assert before.time_s == 0 and before.qvel_si == (0,) * 6
    assert backend.observe().time_s == pytest.approx(0.002)
    assert backend.observe().qvel_si != before.qvel_si
    with pytest.raises(RoboStudioError, match="timestamp"):
        backend.apply(command)
    with pytest.raises(RoboStudioError, match="effort"):
        backend.apply(replace(command, time_s=0.002, effort_si=(1e9,) * 6))
    with pytest.raises(RoboStudioError, match="dt_s"):
        backend.step(0.003)


@pytest.mark.parametrize("q", [(0,) * 5, (float("nan"),) * 6, (100,) * 6])
def test_invalid_configuration_does_not_change_state(arm, q):
    _, backend = arm
    before = backend.observe()
    with pytest.raises(RoboStudioError):
        backend.set_configuration(q)
    assert backend.observe() == before


def test_fk_base_rotation_and_input_validation():
    from math import pi

    assert forward_kinematics((pi / 2, 0, 0, 0, 0, 0))["tcp"] == pytest.approx(
        [0, 0.84, 0.22], abs=1e-12
    )
    with pytest.raises(RoboStudioError):
        forward_kinematics((0,) * 7)


def test_explicit_motor_order_and_mapping_rejection(arm):
    robot, backend = arm
    from robostudio.backends.mujoco_backend import engine

    mj = engine()
    raw_model = mj.MjModel.from_xml_string(
        files("robostudio").joinpath("data/r6_compact.xml").read_text("utf-8")
    )
    raw = mj.MjData(raw_model)
    raw.ctrl[0] = 3  # motor2, not joint1
    command = RobotCommand(
        robot_id=robot.robot_id,
        robot_hash=robot.robot_hash,
        model_hash=robot.model.sha256,
        frame_id="world",
        time_s=0,
        effort_si=(3, 0, 0, 0, 0, 0),
    )
    backend.apply(command)
    backend.step(0.002)
    mj.mj_step(raw_model, raw)
    assert backend.observe().qvel_si == pytest.approx(raw.qvel, abs=1e-12)
    assert raw.actuator_force.tolist() == pytest.approx([3, 0, 0, 0, 0, 0])
    mapping = list(robot.actuator_mapping)
    mapping[0] = replace(mapping[0], ctrl_index=1)
    mapping[1] = replace(mapping[1], ctrl_index=0)
    with pytest.raises(RoboStudioError, match="INVALID_MAPPING"):
        MuJoCoBackend().load(replace(robot, actuator_mapping=tuple(mapping)))


def test_rejects_free_base_before_reading_engine(arm):
    from robostudio.core import JointMapping

    robot, _ = arm
    joints = (JointMapping("base", "free", tuple(range(7)), tuple(range(6))),) + tuple(
        replace(j, qpos_indices=(i + 7,), qvel_indices=(i + 6,)) for i, j in enumerate(robot.joints)
    )
    floating = replace(robot, qpos_size=13, qvel_size=12, joints=joints)
    with pytest.raises(RoboStudioError, match="UNSUPPORTED_CAPABILITY"):
        MuJoCoBackend().load(floating)
