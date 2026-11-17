"""Traced, bounded Jacobian solvers on a private kinematic probe (world / SI)."""

import importlib
import math
from typing import Any

from .cell_plugins import require_algorithm
from .core import ErrorCode, Pose
from .core.contracts import vector
from .core.errors import require
from .core.manipulation import ManipulationKinematics
from .motion import pose_error

SOLVERS = {
    "dls": "阻尼最小二乘",
    "pinv": "SVD 伪逆",
    "transpose": "雅可比转置",
    "adaptive_dls": "自适应阻尼最小二乘",
}


def solve(
    cell: ManipulationKinematics,
    target: list[float],
    algorithm: str = "dls",
    initial: list[float] | None = None,
    damping: float = 0.035,
    quaternion_wxyz: list[float] | None = None,
) -> dict[str, Any]:
    """Same seed, 140 steps, bounds and tolerances for all algorithms. No live writes."""
    require(
        "cartesian_ik" in cell.capabilities,
        ErrorCode.UNSUPPORTED_CAPABILITY,
        "model does not provide Cartesian IK",
    )
    np = importlib.import_module("numpy")
    require_algorithm(algorithm, cell.capabilities, damping)
    vector(tuple(target), "target_m", 3)
    require(all(abs(v) <= 2 for v in target), ErrorCode.INVALID_VALUE, "target exceeds 2 m")
    require(algorithm in SOLVERS, ErrorCode.INVALID_VALUE, "unknown IK algorithm")
    require(0.001 <= damping <= 0.5, ErrorCode.INVALID_VALUE, "damping must be .001..0.5")
    probe = cell.probe()
    if initial is not None:
        vector(tuple(initial), "initial_q", len(probe.configuration))
    q = np.array(initial if initial is not None else probe.configuration, dtype=float)
    vector(tuple(float(x) for x in q), "initial_q", len(probe.configuration))
    low, high = np.array(probe.limits).T
    require(bool(np.all((q >= low) & (q <= high))), ErrorCode.INVALID_VALUE, "joint limits")
    orientation = (
        tuple(vector(tuple(quaternion_wxyz), "quaternion_wxyz", 4))
        if quaternion_wxyz is not None
        else (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)
    )
    goal = Pose(tuple(target), orientation, "world")
    weights = np.array([1.0, 1.0, 1.0, 0.25, 0.25, 0.25])
    trace: list[dict[str, Any]] = []

    def evaluate(values: Any) -> tuple[Any, Pose, Any]:
        pose, jacobian = probe.evaluate(tuple(float(x) for x in values))
        return np.array(pose_error(goal, pose)), pose, np.array(jacobian)

    for iteration in range(141):
        error, pose, jac = evaluate(q)
        # Quaternion to rotation, kept independent of the physics engine.
        w, x, y, z = pose.quaternion_wxyz
        rotation = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        j, e = jac * weights[:, None], error * weights
        singular = np.linalg.svd(j, compute_uv=False)
        # Smooth singularity-dependent regularization; not an LM trust-region method.
        actual_damping = damping
        if algorithm == "adaptive_dls":
            actual_damping = damping + 0.08 * max(0.0, 1 - float(singular[-1]) / 0.08) ** 2
        position_error = float(np.linalg.norm(error[:3]))
        orientation_error = float(np.linalg.norm(error[3:]))
        row = {
            "iteration": iteration,
            "q": q.tolist(),
            "tcp": list(pose.position_m),
            "rotation": rotation,
            "error": error.tolist(),
            "jacobian": jac.tolist(),
            "position_error_m": position_error,
            "orientation_error_rad": orientation_error,
            "sigma_min": float(singular[-1]),
            "damping": actual_damping if algorithm in ("dls", "adaptive_dls") else 0.0,
            "delta_q": [0.0] * len(q),
            "step_scale": 0.0,
        }
        trace.append(row)
        if position_error < 0.0005 and orientation_error < 0.002:
            status = "CONVERGED"
            break
        if iteration == 140:
            status = "NOT_CONVERGED"
            break
        if algorithm in ("dls", "adaptive_dls"):
            step = j.T @ np.linalg.solve(j @ j.T + actual_damping**2 * np.eye(6), e)
        elif algorithm == "pinv":
            step = np.linalg.pinv(j, rcond=1e-5) @ e
        else:
            je = j @ j.T @ e
            alpha = float(e @ je) / max(1e-16, float(je @ je))
            step = alpha * j.T @ e
        step *= min(1.0, 0.18 / max(1e-12, float(np.linalg.norm(step))))
        cost = float(e @ e)
        for scale in (1.0, 0.5, 0.25, 0.125):
            candidate = np.clip(q + step * scale, low, high)
            residual, _, _ = evaluate(candidate)
            if float((residual * weights) @ (residual * weights)) < cost:
                row["delta_q"] = (candidate - q).tolist()
                row["step_scale"] = scale
                q = candidate
                break
    return {
        "algorithm": algorithm,
        "status": status,
        "q": q.tolist(),
        "target_m": target,
        "target_quaternion_wxyz": list(orientation),
        "position_error_m": position_error,
        "orientation_error_rad": orientation_error,
        "iterations": iteration,
        "trace": trace,
        "budget": 140,
        "frame_id": "world",
    }
