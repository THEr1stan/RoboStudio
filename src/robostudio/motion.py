"""Trusted bounded IK and joint control; no physics state is overwritten to move."""

import importlib
import math
import random
from typing import cast

from .core import (
    Capability,
    ErrorCode,
    Pose,
    RoboStudioError,
    RobotCommand,
    RobotReference,
    RobotSpec,
    RobotState,
)
from .core.contracts import finite, vector
from .core.errors import require
from .core.motion import IKProblem, IKResult, Kinematics
from .jacobian import geometric_jacobian
from .kinematics import LIMITS, forward_kinematics

HOME = tuple(math.radians(v) for v in (0, -60, 60, 0, -28, 0))
KP = (240.0, 280.0, 220.0, 90.0, 80.0, 60.0)
KD = (35.0, 40.0, 30.0, 12.0, 11.0, 9.0)
MODEL_HASH = "9842007ccad37dc2d87db0bd1f6c2815f61a0324ab31932fbdb0dbd2ddf314b1"


def quaternion_from_matrix(r: list[list[float]]) -> tuple[float, float, float, float]:
    """Largest-component extraction remains well-conditioned at angle pi."""
    trace = sum(r[i][i] for i in range(3))
    if trace > 0:
        s = 2 * math.sqrt(1 + trace)
        q = (s / 4, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s)
    else:
        i = max(range(3), key=lambda k: r[k][k])
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(max(0, 1 + r[i][i] - r[j][j] - r[k][k]))
        v = [0.0] * 4
        v[0], v[i + 1] = (r[k][j] - r[j][k]) / s, s / 4
        v[j + 1], v[k + 1] = (r[i][j] + r[j][i]) / s, (r[i][k] + r[k][i]) / s
        q = cast(tuple[float, float, float, float], tuple(v))
    norm = math.sqrt(sum(v * v for v in q))
    return cast(tuple[float, float, float, float], tuple(v / norm for v in q))


def pose_at(q: tuple[float, ...]) -> Pose:
    result = forward_kinematics(q)
    return Pose(tuple(result["tcp"]), quaternion_from_matrix(result["rotation"]), "world")


def pose_error(target: Pose, actual: Pose) -> tuple[float, ...]:
    """World spatial error: p_target-p_actual and log(R_target R_actual^T)."""
    require(target.frame_id == actual.frame_id, ErrorCode.FRAME_MISMATCH, "pose frames differ")
    a, b, c, d = target.quaternion_wxyz
    w, x, y, z = actual.quaternion_wxyz
    # Target * conjugate(actual), scalar-first Hamilton product.
    q = (
        a * w + b * x + c * y + d * z,
        -a * x + b * w - c * z + d * y,
        -a * y + b * z + c * w - d * x,
        -a * z - b * y + c * x + d * w,
    )
    if q[0] < 0:
        q = (-q[0], -q[1], -q[2], -q[3])
    norm = math.sqrt(sum(v * v for v in q[1:]))
    factor = 2 * math.atan2(norm, max(0, q[0])) / norm if norm > 1e-12 else 2.0
    return tuple(a - b for a, b in zip(target.position_m, actual.position_m, strict=True)) + (
        q[1] * factor,
        q[2] * factor,
        q[3] * factor,
    )


class R6Kinematics:
    """Independent original-model adapter, selected by verified model identity."""

    def __init__(self, robot: RobotSpec):
        require(
            Capability.CARTESIAN_IK in robot.capabilities.values,
            ErrorCode.UNSUPPORTED_CAPABILITY,
            "requires cartesian_ik",
        )
        require(
            robot.model.sha256 == MODEL_HASH and robot.qpos_size == 6,
            ErrorCode.MODEL_MISMATCH,
            "no verified kinematic adapter for this model",
        )
        self.robot = robot
        self.limits = LIMITS

    def pose(self, q: tuple[float, ...]) -> Pose:
        return pose_at(q)

    def jacobian(self, q: tuple[float, ...]) -> tuple[tuple[float, ...], ...]:
        return geometric_jacobian(q)


class DampedLeastSquares:
    def solve(self, model: Kinematics, problem: IKProblem) -> IKResult:
        require(
            problem.robot_hash == model.robot.robot_hash
            and problem.model_hash == model.robot.model.sha256,
            ErrorCode.MODEL_MISMATCH,
            "IK identity mismatch",
        )
        require(
            problem.target.frame_id == model.robot.frame_id,
            ErrorCode.FRAME_MISMATCH,
            "IK target frame mismatch",
        )
        require(
            all(
                lo <= q <= hi for q, (lo, hi) in zip(problem.seed_q_rad, model.limits, strict=True)
            ),
            ErrorCode.INVALID_VALUE,
            "IK seed exceeds joint limits",
        )
        try:
            np = importlib.import_module("numpy")
        except ImportError as import_error:
            raise RoboStudioError(
                ErrorCode.UNSUPPORTED_CAPABILITY, "IK requires the approved physics extra"
            ) from import_error
        rng = random.Random(problem.seed)
        seeds = [problem.seed_q_rad] + [
            tuple(rng.uniform(lo * 0.85, hi * 0.85) for lo, hi in model.limits)
            for _ in range(problem.attempts - 1)
        ]
        weights = np.array([1, 1, 1, 0.25, 0.25, 0.25])
        low, high = np.array(model.limits).T
        best_q, best_cost, count, attempted = problem.seed_q_rad, math.inf, 0, 0
        for attempted, start in enumerate(seeds, 1):
            q = np.array(start, dtype=float)
            damping = 0.02
            for _ in range(problem.max_iterations):
                count += 1
                qt = tuple(float(v) for v in q)
                e = np.array(pose_error(problem.target, model.pose(qt)))
                weighted = e * weights
                cost = float(weighted @ weighted)
                if cost < best_cost:
                    best_q, best_cost = qt, cost
                if float(np.linalg.norm(e[:3])) <= 1e-4 and float(np.linalg.norm(e[3:])) <= 1e-3:
                    return IKResult(
                        "CONVERGED",
                        qt,
                        float(np.linalg.norm(e[:3])),
                        float(np.linalg.norm(e[3:])),
                        count,
                        attempted,
                    )
                j = np.array(model.jacobian(qt)) * weights[:, None]
                step = j.T @ np.linalg.solve(j @ j.T + damping**2 * np.eye(6), weighted)
                step *= min(1.0, 0.25 / max(1e-12, float(np.linalg.norm(step))))
                accepted = False
                for scale in (1.0, 0.5, 0.25, 0.125):
                    candidate = np.clip(q + scale * step, low, high)
                    residual = (
                        np.array(
                            pose_error(
                                problem.target, model.pose(tuple(float(v) for v in candidate))
                            )
                        )
                        * weights
                    )
                    if float(residual @ residual) < cost - 1e-16:
                        q, accepted = candidate, True
                        damping = max(0.001, damping * 0.7)
                        break
                if not accepted:
                    damping = min(1.0, damping * 3)
                    if damping >= 1.0:
                        break
        error = pose_error(problem.target, model.pose(best_q))
        return IKResult(
            "NOT_CONVERGED",
            best_q,
            math.sqrt(sum(v * v for v in error[:3])),
            math.sqrt(sum(v * v for v in error[3:])),
            count,
            attempted,
        )


def trajectory_duration(
    start: tuple[float, ...], end: tuple[float, ...], requested: float = 2.5
) -> float:
    vector(start, "start", 6)
    vector(end, "end", 6)
    finite(requested, "duration_s")
    require(1 <= requested <= 8, ErrorCode.INVALID_VALUE, "duration must be 1..8 s")
    delta = max(abs(a - b) for a, b in zip(start, end, strict=True))
    # Exact extrema of the normalized quintic first and second derivative.
    duration = max(requested, 1.875 * delta / 1.2, math.sqrt(5.773503 * delta / 2.5))
    return math.ceil(duration / 0.002) * 0.002


def quintic(
    start: tuple[float, ...], end: tuple[float, ...], time_s: float, duration_s: float
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    vector(start, "start", 6)
    vector(end, "end", 6)
    finite(time_s, "time_s")
    finite(duration_s, "duration_s")
    require(duration_s > 0 and time_s >= 0, ErrorCode.INVALID_VALUE, "invalid trajectory time")
    u = min(1.0, time_s / duration_s)
    s = u**3 * (10 - 15 * u + 6 * u * u)
    velocity = 30 * u * u * (1 - u) ** 2 / duration_s
    return (
        tuple(a + (b - a) * s for a, b in zip(start, end, strict=True)),
        tuple((b - a) * velocity for a, b in zip(start, end, strict=True)),
    )


def joint_pd(
    robot: RobotSpec,
    state: RobotState,
    reference: RobotReference,
    bias_ctrl: tuple[float, ...],
    bounds: tuple[tuple[float, float], ...],
) -> tuple[RobotCommand, int]:
    state.validate_for(robot)
    reference.validate_for(robot)
    require(state.time_s == reference.time_s, ErrorCode.INVALID_VALUE, "reference time mismatch")
    require(
        robot.qpos_size == robot.qvel_size == robot.ctrl_size == 6
        and all(j.kind == "revolute" for j in robot.joints),
        ErrorCode.UNSUPPORTED_CAPABILITY,
        "PD requires six scalar revolute joints",
    )
    vector(bias_ctrl, "bias_ctrl", robot.ctrl_size)
    require(len(bounds) == robot.ctrl_size, ErrorCode.INVALID_DIMENSION, "effort bounds mismatch")
    joints = {j.name: j for j in robot.joints}
    effort, saturated = [0.0] * robot.ctrl_size, 0
    for actuator in robot.actuator_mapping:
        joint = joints[actuator.joint_name]
        p, v, c = joint.qpos_indices[0], joint.qvel_indices[0], actuator.ctrl_index
        low, high = bounds[c]
        finite(low, "effort_lower")
        finite(high, "effort_upper")
        require(low < high, ErrorCode.INVALID_VALUE, "invalid effort bounds")
        raw = (
            KP[p] * (reference.qpos_si[p] - state.qpos_si[p])
            + (KD[v] * (reference.qvel_si[v] - state.qvel_si[v]))
            + bias_ctrl[c]
        )
        effort[c] = min(high, max(low, raw))
        saturated += int(raw != effort[c])
    return RobotCommand(
        robot_id=robot.robot_id,
        robot_hash=robot.robot_hash,
        model_hash=robot.model.sha256,
        frame_id=robot.frame_id,
        time_s=state.time_s,
        effort_si=tuple(effort),
    ), saturated
