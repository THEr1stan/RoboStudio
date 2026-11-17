"""Optional headless MuJoCo adapter. Fixed scalar hinge joints only in M1a."""

import hashlib
import importlib
from importlib.resources import files
from math import isclose
from typing import Any

from robostudio.core import (
    Capability,
    ErrorCode,
    RoboStudioError,
    RobotCommand,
    RobotSpec,
    RobotState,
)
from robostudio.core.contracts import finite, vector
from robostudio.core.errors import require


def engine() -> Any:
    try:
        return importlib.import_module("mujoco")
    except (ImportError, OSError) as error:
        raise RoboStudioError(
            ErrorCode.UNSUPPORTED_CAPABILITY,
            "MuJoCo unavailable; install robostudio[physics]",
            "physics",
        ) from error


class MuJoCoBackend:
    def __init__(self, model_xml: bytes | None = None) -> None:
        # Bytes are trusted application assets, never HTTP/user-provided model code.
        self._xml = model_xml
        self._phase = "new"
        self._robot: RobotSpec | None = None
        self._mj: Any = None
        self._model: Any = None
        self._data: Any = None
        self._joint_ids: list[int] = []

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
            "requires joint_effort",
        )
        require(
            all(j.kind == "revolute" for j in robot.joints),
            ErrorCode.UNSUPPORTED_CAPABILITY,
            "only fixed-base revolute joints supported",
        )
        require(robot.frame_id == "world", ErrorCode.FRAME_MISMATCH, "expected world frame")
        xml = (
            self._xml
            if self._xml is not None
            else files("robostudio").joinpath("data/r6_compact.xml").read_bytes()
        )
        require(
            robot.model.format == "mjcf" and hashlib.sha256(xml).hexdigest() == robot.model.sha256,
            ErrorCode.MODEL_MISMATCH,
            "MJCF hash mismatch",
        )
        mj = engine()
        try:
            model = mj.MjModel.from_xml_string(xml.decode("utf-8"))
        except (ValueError, UnicodeError) as error:
            raise RoboStudioError(ErrorCode.INVALID_MANIFEST, str(error), "MJCF") from error
        require(
            (model.nq, model.nv, model.nu, model.njnt)
            == (robot.qpos_size, robot.qvel_size, robot.ctrl_size, len(robot.joints)),
            ErrorCode.INVALID_MAPPING,
            "engine dimensions differ from manifest",
        )
        joint_ids = []
        for joint in robot.joints:
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint.name)
            require(jid >= 0, ErrorCode.INVALID_MAPPING, "missing joint", joint.name)
            require(
                model.jnt_type[jid] == mj.mjtJoint.mjJNT_HINGE
                and (int(model.jnt_qposadr[jid]),) == joint.qpos_indices
                and (int(model.jnt_dofadr[jid]),) == joint.qvel_indices,
                ErrorCode.INVALID_MAPPING,
                "joint index/type mismatch",
                joint.name,
            )
            require(
                bool(model.jnt_limited[jid]),
                ErrorCode.INVALID_MAPPING,
                "bounded hinge required",
                joint.name,
            )
            joint_ids.append(jid)
        for actuator in robot.actuator_mapping:
            aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, actuator.name)
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, actuator.joint_name)
            require(
                aid == actuator.ctrl_index
                and actuator.effort_unit == "N*m"
                and model.actuator_trntype[aid] == mj.mjtTrn.mjTRN_JOINT
                and model.actuator_trnid[aid, 0] == jid
                and list(model.actuator_gear[aid]) == [1, 0, 0, 0, 0, 0]
                and model.actuator_dyntype[aid] == mj.mjtDyn.mjDYN_NONE
                and model.actuator_gaintype[aid] == mj.mjtGain.mjGAIN_FIXED
                and model.actuator_gainprm[aid, 0] == 1
                and model.actuator_biastype[aid] == mj.mjtBias.mjBIAS_NONE
                and bool(model.actuator_ctrllimited[aid]),
                ErrorCode.INVALID_MAPPING,
                "requires bounded unit-gear torque motor",
                actuator.name,
            )
        require(
            mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "tcp") >= 0,
            ErrorCode.INVALID_MAPPING,
            "missing tcp site",
        )
        self._mj, self._model, self._data = mj, model, mj.MjData(model)
        self._robot, self._joint_ids, self._phase = robot, joint_ids, "loaded"

    def reset(self, seed: int) -> None:
        self._require_phase("loaded", "ready")
        require(
            type(seed) is int and 0 <= seed < 2**32,
            ErrorCode.INVALID_VALUE,
            "seed must be a uint32",
            "seed",
        )
        self._mj.mj_resetData(self._model, self._data)
        self._mj.mj_forward(self._model, self._data)
        self._phase = "ready"

    def set_configuration(self, q: tuple[float, ...]) -> None:
        """Kinematic setup at t=0, not a servo or physical command."""
        self._require_phase("ready")
        assert self._robot is not None
        vector(q, "q_rad", self._robot.qpos_size)
        for joint, jid in zip(self._robot.joints, self._joint_ids, strict=True):
            low, high = self._model.jnt_range[jid]
            require(
                low <= q[joint.qpos_indices[0]] <= high,
                ErrorCode.INVALID_VALUE,
                "configuration exceeds joint limit",
                joint.name,
            )
        self.reset(0)
        self._data.qpos[:] = q
        self._mj.mj_forward(self._model, self._data)

    def observe(self) -> RobotState:
        self._require_phase("ready")
        assert self._robot is not None
        robot = self._robot
        state = RobotState(
            robot_id=robot.robot_id,
            robot_hash=robot.robot_hash,
            model_hash=robot.model.sha256,
            frame_id=robot.frame_id,
            time_s=float(self._data.time),
            qpos_si=tuple(float(v) for v in self._data.qpos),
            qvel_si=tuple(float(v) for v in self._data.qvel),
        )
        state.validate_for(robot)
        return state

    def apply(self, command: RobotCommand) -> None:
        self._require_phase("ready")
        assert self._robot is not None
        require(isinstance(command, RobotCommand), ErrorCode.INVALID_VALUE, "expected RobotCommand")
        command.validate_for(self._robot)
        require(
            command.time_s == self.observe().time_s,
            ErrorCode.INVALID_VALUE,
            "command timestamp must match simulation time",
        )
        for index, effort in enumerate(command.effort_si):
            low, high = self._model.actuator_ctrlrange[index]
            require(low <= effort <= high, ErrorCode.INVALID_VALUE, "effort outside motor limit")
        self._data.ctrl[:] = command.effort_si

    def step(self, dt_s: float) -> None:
        self._require_phase("ready")
        finite(dt_s, "dt_s")
        step = float(self._model.opt.timestep)
        require(
            step <= dt_s <= 0.02 and isclose(dt_s / step, round(dt_s / step), abs_tol=1e-10),
            ErrorCode.INVALID_VALUE,
            "dt_s must be a timestep multiple <= 0.02",
            "dt_s",
        )
        before = self.observe().time_s
        self._mj.mj_step(self._model, self._data, nstep=round(dt_s / step))
        self._mj.mj_forward(self._model, self._data)
        state = self.observe()
        require(
            isclose(state.time_s, before + dt_s, rel_tol=0, abs_tol=1e-10),
            ErrorCode.INVALID_VALUE,
            "engine reset after unstable step",
        )

    def effort_bounds(self) -> tuple[tuple[float, float], ...]:
        self._require_phase("ready")
        return tuple((float(lo), float(hi)) for lo, hi in self._model.actuator_ctrlrange)

    def bias_effort(self) -> tuple[float, ...]:
        """Gravity + Coriolis/centrifugal bias, copied in explicit control order."""
        self._require_phase("ready")
        assert self._robot is not None
        joints = {joint.name: joint for joint in self._robot.joints}
        values = [0.0] * self._robot.ctrl_size
        for actuator in self._robot.actuator_mapping:
            dof = joints[actuator.joint_name].qvel_indices[0]
            values[actuator.ctrl_index] = float(self._data.qfrc_bias[dof])
        return tuple(values)

    def tcp_jacobian(self) -> tuple[tuple[float, ...], ...]:
        """Read-only site Jacobian: world [linear; angular], raw qvel column order."""
        self._require_phase("ready")
        np = importlib.import_module("numpy")
        position = np.zeros((3, self._model.nv))
        rotation = np.zeros((3, self._model.nv))
        site_id = self._mj.mj_name2id(self._model, self._mj.mjtObj.mjOBJ_SITE, "tcp")
        self._mj.mj_jacSite(self._model, self._data, position, rotation, site_id)
        return tuple(tuple(float(v) for v in row) for row in (*position, *rotation))

    def scene(self) -> dict[str, Any]:
        """Copied world-space engine coordinates, suitable for browser rendering."""
        self._require_phase("ready")
        from dataclasses import asdict

        tcp = self._data.site("tcp")
        points = [self._data.xanchor[j].tolist() for j in self._joint_ids]
        return {
            **asdict(self.observe()),
            "points": points + [tcp.xpos.tolist()],
            "jointAxes": [self._data.xaxis[j].tolist() for j in self._joint_ids],
            "tcp": tcp.xpos.tolist(),
            "rotation": tcp.xmat.reshape(3, 3).tolist(),
            "effort_si": self._data.ctrl.tolist(),
        }

    def close(self) -> None:
        self._data = self._model = self._robot = None
        self._phase = "closed"
