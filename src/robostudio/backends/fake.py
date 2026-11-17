"""Deterministic scalar-joint test double. Not a robotics physics engine."""

import hashlib
import random
from importlib.resources import files

from robostudio.core import (
    Capability,
    ErrorCode,
    RobotCommand,
    RobotSpec,
    RobotState,
)
from robostudio.core.contracts import finite
from robostudio.core.errors import require


class FakeBackend:
    def __init__(self) -> None:
        self._phase = "new"
        self._robot: RobotSpec | None = None
        self._state: RobotState | None = None
        self._effort: tuple[float, ...] = ()

    def _require_phase(self, *phases: str) -> None:
        require(
            self._phase in phases,
            ErrorCode.INVALID_LIFECYCLE,
            f"expected {phases}; current phase is {self._phase}",
        )

    def load(self, robot: RobotSpec) -> None:
        self._require_phase("new")
        require(
            Capability.JOINT_EFFORT in robot.capabilities.values,
            ErrorCode.UNSUPPORTED_CAPABILITY,
            "fake requires joint_effort",
        )
        expected_hash = hashlib.sha256(
            files("robostudio").joinpath("data/fake_model.json").read_bytes()
        ).hexdigest()
        require(
            robot.model.format == "fake-v1" and robot.model.sha256 == expected_hash,
            ErrorCode.MODEL_MISMATCH,
            "fake supports only the bundled test dynamics model",
        )
        require(
            all(j.kind == "revolute" for j in robot.joints),
            ErrorCode.UNSUPPORTED_CAPABILITY,
            "fake supports scalar revolute joints only",
        )
        self._robot = robot
        self._phase = "loaded"

    def reset(self, seed: int) -> None:
        self._require_phase("loaded", "ready")
        require(
            type(seed) is int and 0 <= seed < 2**32,
            ErrorCode.INVALID_VALUE,
            "seed must be a uint32",
            "seed",
        )
        assert self._robot is not None
        rng = random.Random(seed)
        # Use only Random.random(), whose seeded sequence is stable across Python versions.
        q = tuple((rng.random() - 0.5) * 0.2 for _ in range(self._robot.qpos_size))
        self._state = self._sample(0.0, q, (0.0,) * self._robot.qvel_size)
        self._effort = (0.0,) * self._robot.ctrl_size
        self._phase = "ready"

    def _sample(self, time_s: float, q: tuple[float, ...], v: tuple[float, ...]) -> RobotState:
        assert self._robot is not None
        robot = self._robot
        state = RobotState(
            robot_id=robot.robot_id,
            robot_hash=robot.robot_hash,
            model_hash=robot.model.sha256,
            frame_id=robot.frame_id,
            time_s=time_s,
            qpos_si=q,
            qvel_si=v,
        )
        state.validate_for(robot)
        return state

    def observe(self) -> RobotState:
        self._require_phase("ready")
        assert self._state is not None
        return self._state

    def apply(self, command: RobotCommand) -> None:
        self._require_phase("ready")
        assert self._robot is not None
        require(
            isinstance(command, RobotCommand),
            ErrorCode.INVALID_VALUE,
            "apply requires RobotCommand, not state or reference",
        )
        command.validate_for(self._robot)
        require(
            command.time_s == self.observe().time_s,
            ErrorCode.INVALID_VALUE,
            "command timestamp must match current simulation time",
            "time_s",
        )
        self._effort = command.effort_si

    def step(self, dt_s: float) -> None:
        self._require_phase("ready")
        finite(dt_s, "dt_s")
        require(0 < dt_s <= 0.1, ErrorCode.INVALID_VALUE, "expected 0 < dt_s <= 0.1", "dt_s")
        assert self._robot is not None
        state = self.observe()
        q, v = list(state.qpos_si), list(state.qvel_si)
        efforts = {j.name: 0.0 for j in self._robot.joints}
        for actuator in self._robot.actuator_mapping:
            efforts[actuator.joint_name] += self._effort[actuator.ctrl_index]
        # Unit inertia (kg*m^2), damping 0.1 N*m*s/rad, semi-implicit Euler.
        for joint in self._robot.joints:
            qi, vi = joint.qpos_indices[0], joint.qvel_indices[0]
            v[vi] += (efforts[joint.name] - 0.1 * v[vi]) * dt_s
            q[qi] += v[vi] * dt_s
        # Validate before committing, preserving the prior state on numerical failure.
        self._state = self._sample(state.time_s + dt_s, tuple(q), tuple(v))

    def close(self) -> None:
        self._state = None
        self._robot = None
        self._effort = ()
        self._phase = "closed"
