from dataclasses import replace
from math import pi, sqrt

import pytest

from robostudio.core import RoboStudioError, RobotReference, RobotState
from robostudio.core.motion import IKProblem
from robostudio.motion import (
    HOME,
    DampedLeastSquares,
    R6Kinematics,
    joint_pd,
    pose_at,
    pose_error,
    quintic,
    trajectory_duration,
)
from robostudio.registry import default_registry


def test_rotation_error_handles_pi_and_world_frame():
    from robostudio.core import Pose

    a = Pose((0, 0, 0), (1, 0, 0, 0), "world")
    b = Pose((0.1, 0, 0), (0, 1, 0, 0), "world")
    assert pose_error(b, a) == pytest.approx((0.1, 0, 0, pi, 0, 0))
    assert pose_error(a, a) == pytest.approx((0,) * 6)
    with pytest.raises(RoboStudioError):
        pose_error(replace(b, frame_id="tool"), a)


@pytest.mark.parametrize(
    "goal",
    [(0.4, -0.7, 1.1, 0.2, -0.4, 0.3), (-0.6, -1.0, 0.7, -0.3, 0.5, -0.4), (0, 0, 0, 0, 0, 0)],
)
def test_ik_satisfies_independent_pose_residual_and_limits(goal):
    pytest.importorskip("numpy")
    robot = default_registry().robot("r6_compact")
    model = R6Kinematics(robot)
    problem = IKProblem(pose_at(goal), HOME, robot.robot_hash, robot.model.sha256)
    result = DampedLeastSquares().solve(model, problem)
    assert result.status == "CONVERGED"
    error = pose_error(problem.target, pose_at(result.q_rad))
    assert sqrt(sum(e * e for e in error[:3])) <= 1e-4
    assert sqrt(sum(e * e for e in error[3:])) <= 1e-3
    assert all(lo <= q <= hi for q, (lo, hi) in zip(result.q_rad, model.limits, strict=True))
    assert DampedLeastSquares().solve(model, problem) == result


def test_ik_rejects_wrong_identity_and_reports_unreachable():
    from robostudio.core import Pose

    pytest.importorskip("numpy")
    robot = default_registry().robot("r6_compact")
    model = R6Kinematics(robot)
    problem = IKProblem(
        Pose((4, 0, 0), (1, 0, 0, 0), "world"), HOME, robot.robot_hash, robot.model.sha256
    )
    result = DampedLeastSquares().solve(model, problem)
    assert result.status != "CONVERGED"
    assert result.iterations <= problem.max_iterations * problem.attempts
    for bad in (
        replace(problem, robot_hash="0" * 64),
        replace(problem, target=replace(problem.target, frame_id="tool")),
        replace(problem, seed_q_rad=(9,) * 6),
    ):
        with pytest.raises(RoboStudioError):
            DampedLeastSquares().solve(model, bad)


def test_quintic_endpoints_and_derivative_limits():
    start, end = (0,) * 6, (2, -1, 0.5, 0, 1, -2)
    duration = trajectory_duration(start, end)
    assert quintic(start, end, 0, duration) == (start, (0,) * 6)
    assert quintic(start, end, duration, duration) == (end, (0,) * 6)
    previous = (0,) * 6
    dt = duration / 2000
    for i in range(2001):
        q, velocity = quintic(start, end, i * dt, duration)
        assert max(abs(v) for v in velocity) <= 1.2 + 1e-10
        assert max(abs(v - p) / dt for v, p in zip(velocity, previous, strict=True)) <= 2.5 + 1e-6
        assert all(min(a, b) <= v <= max(a, b) for a, b, v in zip(start, end, q, strict=True))
        previous = velocity


def test_controller_uses_actuator_mapping_and_validates_reference():
    robot = default_registry().robot("r6_compact")
    identity = dict(
        robot_id=robot.robot_id,
        robot_hash=robot.robot_hash,
        model_hash=robot.model.sha256,
        frame_id="world",
        time_s=0.0,
    )
    state = RobotState(**identity, qpos_si=(0,) * 6, qvel_si=(0,) * 6)
    reference = RobotReference(**identity, qpos_si=(0.01, 0, 0, 0, 0, 0), qvel_si=(0,) * 6)
    command, saturated = joint_pd(robot, state, reference, (0,) * 6, ((-100, 100),) * 6)
    assert command.effort_si[0] == 0
    assert command.effort_si[1] > 0
    assert saturated == 0
    with pytest.raises(RoboStudioError):
        joint_pd(robot, state, replace(reference, time_s=1), (0,) * 6, ((-100, 100),) * 6)


@pytest.mark.parametrize("axis", [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1 / sqrt(3),) * 3])
def test_rotation_log_at_pi_is_sign_invariant_and_has_correct_magnitude(axis):
    from robostudio.core import Pose

    identity = Pose((0, 0, 0), (1, 0, 0, 0), "world")
    target = Pose((0, 0, 0), (0, *axis), "world")
    error = pose_error(target, identity)
    assert sqrt(sum(v * v for v in error[3:])) == pytest.approx(pi)
    assert pose_error(
        target, replace(target, quaternion_wxyz=tuple(-v for v in target.quaternion_wxyz))
    ) == pytest.approx((0,) * 6)


def test_bias_effort_is_copied_and_uses_ctrl_mapping():
    pytest.importorskip("mujoco")
    from robostudio.backends.mujoco_backend import MuJoCoBackend

    backend = MuJoCoBackend()
    backend.load(default_registry().robot("r6_compact"))
    backend.reset(42)
    backend.set_configuration(HOME)
    before = backend.observe()
    bias = backend.bias_effort()
    assert backend.observe() == before
    assert abs(bias[0]) > 1  # J2 lifts the arm, ctrl[0] is shoulder
    assert bias[1] == pytest.approx(0, abs=1e-12)  # J1 rotates about gravity
    backend.set_configuration((0,) * 6)
    assert bias != backend.bias_effort()
    assert isinstance(bias, tuple)
    backend.close()
