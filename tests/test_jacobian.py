import math
import random

import pytest

from robostudio.core import RoboStudioError
from robostudio.jacobian import finite_difference_jacobian, geometric_jacobian, predict_twist
from robostudio.kinematics import LIMITS, forward_kinematics


def difference(a, b):
    return max(abs(x - y) for ra, rb in zip(a, b, strict=True) for x, y in zip(ra, rb, strict=True))


def test_known_zero_pose_columns_include_tcp_offset_and_world_angular_velocity():
    j = geometric_jacobian((0.0,) * 6)
    expected = (
        (0, 0, 0, 0, 0, 0),
        (0.84, 0, 0, 0, 0, 0),
        (0, -0.84, -0.52, 0, -0.17, 0),
        (0, 0, 0, 1, 0, 1),
        (0, 1, 1, 0, 1, 0),
        (1, 0, 0, 0, 0, 0),
    )
    assert isinstance(j, tuple) and all(isinstance(row, tuple) for row in j)
    assert difference(j, expected) < 1e-12


@pytest.mark.parametrize("h", [1e-4, 1e-6])
def test_central_differences_regular_singular_and_limit_configurations(h):
    rng = random.Random(24)
    poses = [
        (0.0,) * 6,
        (0.4, -0.6, 0.8, 0.3, 1e-9, -0.5),
        tuple(lo for lo, _ in LIMITS),
        tuple(hi for _, hi in LIMITS),
    ]
    poses += [tuple(lo + rng.random() * (hi - lo) for lo, hi in LIMITS) for _ in range(100)]
    for q in poses:
        assert difference(geometric_jacobian(q), finite_difference_jacobian(q, h)) < 1e-8


def test_world_frame_rotates_both_linear_and_angular_rows():
    q = (0.3, -0.8, 0.6, 0.4, -0.5, 0.7)
    turned = (q[0] + math.pi / 2,) + q[1:]
    a, b = geometric_jacobian(q), geometric_jacobian(turned)
    for offset in (0, 3):
        assert b[offset] == pytest.approx([-v for v in a[offset + 1]], abs=1e-12)
        assert b[offset + 1] == pytest.approx(a[offset], abs=1e-12)
        assert b[offset + 2] == pytest.approx(a[offset + 2], abs=1e-12)


def test_directional_velocity_matches_pose_motion_and_zero_rate():
    q = (0.5, -0.6, 0.9, 0.3, -0.4, 0.7)
    rate = (0.3, -0.2, 0.1, 0.4, -0.3, 0.2)
    twist = predict_twist(geometric_jacobian(q), rate)
    h = 1e-6
    plus = forward_kinematics(tuple(a + h * b for a, b in zip(q, rate, strict=True)))
    minus = forward_kinematics(tuple(a - h * b for a, b in zip(q, rate, strict=True)))
    center = forward_kinematics(q)
    linear = [(a - b) / (2 * h) for a, b in zip(plus["tcp"], minus["tcp"], strict=True)]
    d_r = [
        [(plus["rotation"][i][j] - minus["rotation"][i][j]) / (2 * h) for j in range(3)]
        for i in range(3)
    ]
    spatial = [
        [sum(d_r[i][k] * center["rotation"][j][k] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]
    assert twist == pytest.approx(linear + [spatial[2][1], spatial[0][2], spatial[1][0]], abs=1e-8)
    assert predict_twist(geometric_jacobian(q), (0,) * 6) == (0,) * 6


@pytest.mark.parametrize("q", [(0,) * 5, (True,) * 6, (float("nan"),) * 6, (float("inf"),) * 6])
def test_rejects_bad_joint_input(q):
    with pytest.raises(RoboStudioError):
        geometric_jacobian(q)
    with pytest.raises(RoboStudioError):
        finite_difference_jacobian(q)


@pytest.mark.parametrize("h", [0, -1e-6, True, 1e-12, 1.0, float("nan")])
def test_rejects_unreliable_difference_step(h):
    with pytest.raises(RoboStudioError):
        finite_difference_jacobian((0,) * 6, h)


def test_twist_rejects_invalid_matrix_or_rate():
    with pytest.raises(RoboStudioError):
        predict_twist(((0,) * 6,) * 5, (0,) * 6)
    with pytest.raises(RoboStudioError):
        predict_twist(((float("nan"),) * 6,) * 6, (0,) * 6)
    with pytest.raises(RoboStudioError):
        predict_twist(geometric_jacobian((0,) * 6), (0,) * 7)
