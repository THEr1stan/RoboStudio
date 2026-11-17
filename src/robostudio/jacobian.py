"""Original R6 Compact TCP geometric Jacobian, expressed in the world frame.

Rows: linear XYZ (m/rad), angular XYZ (rad/rad). Columns: J1..J6.
J @ qdot returns spatial [v (m/s), omega (rad/s)], not Euler-angle rates.
"""

import math
import random
from typing import Any, Protocol

from .core import ErrorCode
from .core.contracts import finite, vector
from .core.errors import require
from .kinematics import LIMITS, KinematicProbe, forward_kinematics

Matrix = tuple[tuple[float, ...], ...]
FD_STEPS = (1e-4, 1e-6)
FD_TOLERANCE = 1e-8
ENGINE_TOLERANCE = 1e-10


def _matrix(values: Matrix) -> Matrix:
    require(len(values) == 6, ErrorCode.INVALID_DIMENSION, "expected 6 Jacobian rows")
    return tuple(vector(row, f"jacobian[{i}]", 6) for i, row in enumerate(values))


def geometric_jacobian(q: tuple[float, ...]) -> Matrix:
    state = forward_kinematics(q)
    columns = []
    for origin, axis in zip(state["points"][:6], state["jointAxes"], strict=True):
        r = [a - b for a, b in zip(state["tcp"], origin, strict=True)]
        # z_i cross (p_tcp - o_i), with both vectors in world coordinates.
        linear = (
            axis[1] * r[2] - axis[2] * r[1],
            axis[2] * r[0] - axis[0] * r[2],
            axis[0] * r[1] - axis[1] * r[0],
        )
        columns.append((*linear, *axis))
    return _matrix(tuple(tuple(column[row] for column in columns) for row in range(6)))


def finite_difference_jacobian(q: tuple[float, ...], h: float = 1e-6) -> Matrix:
    """Central differential of FK. Limits do not clip the mathematical derivative."""
    q = vector(q, "q_rad", 6)
    finite(h, "h_rad")
    require(1e-7 <= h <= 1e-3, ErrorCode.INVALID_VALUE, "h_rad must be in [1e-7, 1e-3]", "h_rad")
    center = forward_kinematics(q)["rotation"]
    columns = []
    for column in range(6):
        plus, minus = list(q), list(q)
        plus[column] += h
        minus[column] -= h
        require(
            plus[column] != q[column] and minus[column] != q[column],
            ErrorCode.INVALID_VALUE,
            "h_rad cannot resolve this joint magnitude",
        )
        a, b = forward_kinematics(tuple(plus)), forward_kinematics(tuple(minus))
        linear = tuple((x - y) / (2 * h) for x, y in zip(a["tcp"], b["tcp"], strict=True))
        derivative = [
            [(a["rotation"][i][j] - b["rotation"][i][j]) / (2 * h) for j in range(3)]
            for i in range(3)
        ]
        # Spatial angular velocity: omega^ = R_dot R^T (world, not body frame).
        spin = [
            [sum(derivative[i][k] * center[j][k] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]
        angular = (
            (spin[2][1] - spin[1][2]) / 2,
            (spin[0][2] - spin[2][0]) / 2,
            (spin[1][0] - spin[0][1]) / 2,
        )
        columns.append((*linear, *angular))
    return _matrix(tuple(tuple(column[row] for column in columns) for row in range(6)))


def predict_twist(jacobian: Matrix, qvel: tuple[float, ...]) -> tuple[float, ...]:
    matrix = _matrix(jacobian)
    qvel = vector(qvel, "qvel_rad_s", 6)
    return vector(
        tuple(sum(a * b for a, b in zip(row, qvel, strict=True)) for row in matrix), "twist_si", 6
    )


def _errors(a: Matrix, b: Matrix) -> tuple[float, float]:
    a, b = _matrix(a), _matrix(b)
    errors = [
        max(abs(x - y) for x, y in zip(ra, rb, strict=True)) for ra, rb in zip(a, b, strict=True)
    ]
    return max(errors[:3]), max(errors[3:])


def compare_jacobian(q: tuple[float, ...], engine_matrix: Matrix) -> dict[str, Any]:
    analytical = geometric_jacobian(q)
    engine_linear, engine_angular = _errors(analytical, engine_matrix)
    differences = []
    for h in FD_STEPS:
        linear, angular = _errors(analytical, finite_difference_jacobian(q, h))
        differences.append(
            {"h_rad": h, "linear_error_m_per_rad": linear, "angular_error_rad_per_rad": angular}
        )
    fd_linear = max(row["linear_error_m_per_rad"] for row in differences)
    fd_angular = max(row["angular_error_rad_per_rad"] for row in differences)
    require(
        max(engine_linear, engine_angular) <= ENGINE_TOLERANCE
        and max(fd_linear, fd_angular) <= FD_TOLERANCE,
        ErrorCode.MODEL_MISMATCH,
        "Jacobian differs from central finite difference or MuJoCo",
    )
    return {
        "max_engine_error": max(engine_linear, engine_angular),
        "max_engine_linear_error_m_per_rad": engine_linear,
        "max_engine_angular_error_rad_per_rad": engine_angular,
        "max_fd_error": max(fd_linear, fd_angular),
        "max_fd_linear_error_m_per_rad": fd_linear,
        "max_fd_angular_error_rad_per_rad": fd_angular,
        "fd_tolerance": FD_TOLERANCE,
        "engine_tolerance": ENGINE_TOLERANCE,
        "finite_differences": differences,
    }


class JacobianProbe(KinematicProbe, Protocol):
    def tcp_jacobian(self) -> Matrix: ...


def verify_jacobian(backend: JacobianProbe, seed: int = 42, samples: int = 100) -> dict[str, Any]:
    require(type(seed) is int and 0 <= seed < 2**32, ErrorCode.INVALID_VALUE, "invalid seed")
    require(
        type(samples) is int and 1 <= samples <= 1000,
        ErrorCode.INVALID_VALUE,
        "samples must be in [1, 1000]",
    )
    rng = random.Random(seed)
    poses = [
        (0.0,) * 6,
        (0.3, -0.8, 0.6, 0.4, -0.5, 0.7),
        (0.4, -0.6, 0.8, 0.3, 1e-9, -0.5),
        tuple(lo for lo, _ in LIMITS),
        tuple(hi for _, hi in LIMITS),
    ]
    poses += [tuple(lo + rng.random() * (hi - lo) for lo, hi in LIMITS) for _ in range(samples)]
    reports = []
    for q in poses:
        backend.set_configuration(q)
        reports.append(compare_jacobian(q, backend.tcp_jacobian()))
    summary = {
        key: max(report[key] for report in reports) for key in reports[0] if key.startswith("max_")
    }
    return {
        **summary,
        "configurations": len(poses),
        "seed": seed,
        "frame_id": "world",
        "special_cases": ["zero", "generic", "near_wrist_singular", "lower_limits", "upper_limits"],
        "fd_steps_rad": list(FD_STEPS),
        "fd_tolerance": FD_TOLERANCE,
        "engine_tolerance": ENGINE_TOLERANCE,
        "row_order": ["vx", "vy", "vz", "wx", "wy", "wz"],
        "column_order": [f"joint{i + 1}" for i in range(6)],
    }


def analyze_pose(q: tuple[float, ...], qvel: tuple[float, ...]) -> dict[str, Any]:
    """One bounded read-only pose analysis, independent of an ongoing experiment."""
    from . import __version__
    from .backends.mujoco_backend import MuJoCoBackend, engine
    from .demo import source_hash
    from .registry import default_registry

    q, qvel = vector(q, "q_rad", 6), vector(qvel, "qvel_rad_s", 6)
    require(
        all(abs(v) <= math.pi for v in qvel),
        ErrorCode.INVALID_VALUE,
        "analysis rate must be within +/- pi rad/s",
        "qvel_rad_s",
    )
    robot = default_registry().robot("r6_compact")
    backend = MuJoCoBackend()
    try:
        backend.load(robot)
        backend.reset(0)
        backend.set_configuration(q)
        matrix = geometric_jacobian(q)
        verification = compare_jacobian(q, backend.tcp_jacobian())
        return {
            "schema_version": "jacobian-analysis-v1",
            "robot_id": robot.robot_id,
            "robot_hash": robot.robot_hash,
            "model_hash": robot.model.sha256,
            "source_hash": source_hash(),
            "frame_id": "world",
            "q_rad": q,
            "qvel_rad_s": qvel,
            "tcp_position_m": backend.scene()["tcp"],
            "jacobian": matrix,
            "twist_si": predict_twist(matrix, qvel),
            "verification": verification,
            "row_order": ["vx", "vy", "vz", "wx", "wy", "wz"],
            "column_order": [f"joint{i + 1}" for i in range(6)],
            "environment": {"robostudio": __version__, "mujoco": str(engine().__version__)},
            "scope": "instantaneous world velocity prediction; no motion command or integration",
        }
    finally:
        backend.close()
