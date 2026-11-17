"""Engine-independent motion protocols. All angles are radians, positions meters."""

from dataclasses import dataclass
from typing import Protocol

from .contracts import Pose, RobotCommand, RobotReference, RobotSpec, RobotState, sha256, vector
from .errors import ErrorCode, require


@dataclass(frozen=True)
class IKProblem:
    target: Pose
    seed_q_rad: tuple[float, ...]
    robot_hash: str
    model_hash: str
    max_iterations: int = 160
    attempts: int = 6
    seed: int = 42

    def __post_init__(self) -> None:
        require(isinstance(self.target, Pose), ErrorCode.INVALID_VALUE, "expected Pose")
        object.__setattr__(self, "seed_q_rad", vector(self.seed_q_rad, "seed_q_rad", 6))
        sha256(self.robot_hash, "robot_hash")
        sha256(self.model_hash, "model_hash")
        for name, lo, hi in (
            ("max_iterations", 1, 200),
            ("attempts", 1, 8),
            ("seed", 0, 2**32 - 1),
        ):
            value = getattr(self, name)
            require(
                type(value) is int and lo <= value <= hi,
                ErrorCode.INVALID_VALUE,
                f"expected integer in [{lo}, {hi}]",
                name,
            )


@dataclass(frozen=True)
class IKResult:
    status: str
    q_rad: tuple[float, ...]
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    attempts: int


class Kinematics(Protocol):
    @property
    def robot(self) -> RobotSpec: ...
    @property
    def limits(self) -> tuple[tuple[float, float], ...]: ...
    def pose(self, q: tuple[float, ...]) -> Pose: ...
    def jacobian(self, q: tuple[float, ...]) -> tuple[tuple[float, ...], ...]: ...


class IKSolver(Protocol):
    def solve(self, model: Kinematics, problem: IKProblem) -> IKResult: ...


class JointTrajectory(Protocol):
    def __call__(
        self, start: tuple[float, ...], end: tuple[float, ...], time_s: float, duration_s: float
    ) -> tuple[tuple[float, ...], tuple[float, ...]]: ...


class JointController(Protocol):
    def __call__(
        self,
        robot: RobotSpec,
        state: RobotState,
        reference: RobotReference,
        bias_ctrl: tuple[float, ...],
        bounds: tuple[tuple[float, float], ...],
    ) -> tuple[RobotCommand, int]: ...
