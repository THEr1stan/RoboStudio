"""Incremental manipulation state machine, measured physics and replayable user actions."""

import copy
import math
import platform
from typing import Any

from .. import __version__
from ..cell_ik import solve
from ..core import ErrorCode
from ..core.errors import require
from ..demo import source_hash
from ..serialization import digest
from .planning import CollisionProbe, plan_path
from .scene import StudioCell, scene_vector
from .sensor import capture, detect
from .trajectory import time_profile

PHASE_NAMES = {
    "idle": "准备就绪",
    "observe": "观察物体",
    "approach": "规划接近",
    "descend": "下探",
    "close": "闭合夹爪",
    "lift": "抬升检查",
    "transfer": "避障搬运",
    "lower": "下放",
    "release": "释放",
    "retreat": "退回观察",
    "viewpoint": "移开相机遮挡",
    "verify": "检查放置",
    "retry": "重新观察",
    "recovery": "接触恢复",
    "teleop": "手动控制",
    "teach": "示教回放",
    "finished": "任务完成",
    "failed": "任务未完成",
}


def down_quaternion(yaw: float) -> list[float]:
    c, s, k = math.cos(yaw / 2), math.sin(yaw / 2), math.sqrt(0.5)
    return [c * k, -s * k, c * k, s * k]


class Session:
    """A session advances only when requested. Internal step is exactly 2 ms."""

    def __init__(self, config: Any, *, live: bool = False):
        self.cell = StudioCell(config, live=live)
        self.step_limit: int | None = 65000
        self.config = self.cell.scene
        self.phase, self.status = "idle", "READY"
        self.steps, self.tick_in_segment = 0, 0
        self.reference = self.cell.data.qpos[self.cell.q_indices].copy()
        self.close = False
        self.segment: dict[str, Any] | None = None
        self.segments: list[dict[str, Any]] = []
        self.stage_queue: list[dict[str, Any]] = []
        self.targets: list[int] = []
        self.current: int | None = None
        self.attempts: dict[int, int] = {}
        self.completed: list[int] = []
        self.failures: list[dict[str, Any]] = []
        self.placements: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []
        self.actions: list[dict[str, Any]] = []
        self.plans: list[dict[str, Any]] = []
        self.solutions: list[dict[str, Any]] = []
        self.grasps: list[dict[str, Any]] = []
        self.frames: list[dict[str, Any]] = []
        self.sensor_records: list[dict[str, Any]] = []
        self.sensor_history: list[dict[str, Any]] = []
        self.image = capture(self.cell, 0)
        self.sensor_history.append(self.image)
        self.detections = detect(self.image)
        self.sensor_records.append(self.image)
        self.trace: list[dict[str, Any]] = []
        self.teaching: list[dict[str, Any]] = []
        self.bilateral, self.collision_steps, self.squared_error = 0, 0, 0.0
        self.effort_integral, self.max_lift = 0.0, 0.0
        self.sensor_error_sum, self.sensor_error_count = 0.0, 0
        self.push_until = 0
        self.error: str | None = None
        self.desired_destination: list[float] | None = None
        self.selected_yaw = 0.0
        self._record_frame()

    def event(self, kind: str, **details: Any) -> None:
        self.events.append(
            {"step": self.steps, "time_s": float(self.cell.data.time), "kind": kind, **details}
        )

    def _sense(self) -> None:
        fresh = capture(self.cell, self.steps // 50)
        self.sensor_history.append(fresh)
        self.sensor_history = self.sensor_history[-8:]
        timestamp = self.cell.data.time - self.config["latency_ms"] / 1000
        available = [item for item in self.sensor_history if item["time_s"] <= timestamp + 1e-9]
        self.image = available[-1] if available else self.sensor_history[0]
        self.detections = detect(self.image)
        if self.steps % 500 == 0 and self.sensor_records[-1]["sequence"] != self.image["sequence"]:
            self.sensor_records.append(self.image)

    def _observation(self, index: int) -> dict[str, Any] | None:
        c = self.cell
        if self.config["observation"] == "truth":
            point = c.data.xpos[c.object_bodies[index]].copy()
            rotation = c.data.xmat[c.object_bodies[index]].reshape(3, 3)
            obj = self.config["objects"][index]
            if obj["shape"] == "box":
                point[2] += float(c.np.abs(rotation[2]) @ c.np.array(obj["size"]))
            else:
                point[2] += obj["size"][2] * abs(float(rotation[2, 2])) + obj["size"][
                    0
                ] * math.hypot(float(rotation[2, 0]), float(rotation[2, 1]))
            return {
                "position_m": point.tolist(),
                "yaw_rad": math.atan2(float(rotation[1, 0]), float(rotation[0, 0])),
                "source": "simulator_ground_truth",
                "confidence": 1,
                "time_s": float(c.data.time),
            }
        color = self.config["objects"][index]["color"]
        found = next((item for item in self.detections if item["color"] == color), None)
        if found is not None:
            # Truth is an evaluator here; it is never used to correct the estimated target.
            error = math.dist(found["position_m"][:2], c.data.xpos[c.object_bodies[index]][:2])
            self.sensor_error_sum += error * error
            self.sensor_error_count += 1
        return found

    def _record_frame(self) -> None:
        frame = self.cell.frame(self.phase)
        frame["reference_q"] = self.reference.tolist()
        self.frames.append(frame)
        reading = frame["contact_sensor"]
        self.trace.append(
            {
                "time_s": frame["time_s"],
                "phase": self.phase,
                "q": frame["q"],
                "reference_q": frame["reference_q"],
                "effort": self.cell.data.ctrl[self.cell.ctrl_indices].tolist(),
                "height_m": [obj["position"][2] for obj in frame["objects"]],
                "finger_force_n": reading["finger_force_n"],
                "detections": [
                    {k: v for k, v in obs.items() if k != "points_m"} for obs in self.detections
                ],
                "sensor_time_s": self.image["time_s"],
            }
        )

    def _wait(self, phase: str, duration: float) -> None:
        self.phase = phase
        self.segment = {
            "start": self.cell.data.qpos[self.cell.q_indices].copy(),
            "end": self.reference.copy(),
            "duration": duration,
            "hold": True,
        }
        self.tick_in_segment = 0
        self.event("phase", phase=phase)

    def _move(self, path: list[list[float]], phase: str, minimum: float = 0.8) -> None:
        np = self.cell.np
        self.phase = phase
        for a, b in zip(path, path[1:], strict=False):
            delta = np.array(b) - np.array(a)
            duration = math.ceil(max(minimum, float(np.max(np.abs(delta))) * 2.0) / 0.002) * 0.002
            self.segments.append(
                {"start": np.array(a), "end": np.array(b), "duration": duration, "hold": False}
            )
        self.segment = self.segments.pop(0) if self.segments else None
        self.tick_in_segment = 0
        self.event("phase", phase=phase)

    def _plan(
        self,
        target: list[float],
        quaternion: list[float],
        *,
        held: bool = False,
        allow_touch: bool = False,
        phase: str,
        minimum: float = 0.9,
    ) -> bool:
        c = self.cell
        solved = solve(
            c,
            target,
            self.config["solver"],
            damping=self.config["damping"],
            quaternion_wxyz=quaternion,
        )
        self.solutions.append(
            {"phase": phase, "step": self.steps, "scene_qpos": c.data.qpos.tolist(), **solved}
        )
        if solved["status"] != "CONVERGED":
            self.event("planning_failed", reason="IK_FAILED", phase=phase)
            return False
        oracle = CollisionProbe(c, self.current, held, allow_touch, self.close)
        plan = plan_path(
            c.data.qpos[c.q_indices].tolist(),
            solved["q"],
            c.limits,
            oracle,
            method=self.config["planner"],
            seed=self.config["seed"] + len(self.plans),
        )
        plan.update(
            phase=phase,
            step=self.steps,
            target_m=target,
            last_collision_pair=oracle.last_pair,
            scene_qpos=c.data.qpos.tolist(),
        )
        probe = c.probe()
        plan["tcp_path_m"] = [list(probe.evaluate(tuple(q))[0].position_m) for q in plan["path"]]
        self.plans.append(plan)
        if plan["status"] != "FOUND":
            self.event("planning_failed", reason="COLLISION_BLOCKED", phase=phase)
            return False
        self._move(plan["path"], phase, minimum)
        return True

    def _choose_grasp(self, observation: dict[str, Any]) -> bool:
        c = self.cell
        x, y, z = observation["position_m"]
        grasp_z = max(0.06, z + 0.033)
        approach_z = max(0.23, grasp_z + 0.12)
        yaws = (
            [0.0]
            if len(c.joint_ids) == 3
            else [observation["yaw_rad"], observation["yaw_rad"] + math.pi / 2]
        )
        candidates = []
        for yaw in yaws:
            quat = down_quaternion(yaw)
            approach = solve(
                c,
                [x, y, approach_z],
                self.config["solver"],
                damping=self.config["damping"],
                quaternion_wxyz=quat,
            )
            down = solve(
                c,
                [x, y, grasp_z],
                self.config["solver"],
                initial=approach["q"],
                damping=self.config["damping"],
                quaternion_wxyz=quat,
            )
            oracle = CollisionProbe(c, self.current, allow_touch=True)
            valid = (
                approach["status"] == down["status"] == "CONVERGED"
                and oracle(down["q"])
                and oracle(approach["q"])
            )
            candidates.append(
                {
                    "yaw_rad": yaw,
                    "feasible": bool(valid),
                    "cost": math.dist(c.data.qpos[c.q_indices], approach["q"]),
                    "position_error_m": down["position_error_m"],
                    "pair": oracle.last_pair,
                }
            )
        self.grasps.append(
            {
                "step": self.steps,
                "object": self.current,
                "observation": observation,
                "candidates": candidates,
            }
        )
        feasible = sorted(
            (item for item in candidates if item["feasible"]), key=lambda item: item["cost"]
        )
        if not feasible:
            return False
        selected = feasible[self.attempts.get(self.current or 0, 0) % len(feasible)]
        self.selected_yaw = selected["yaw_rad"]
        self.grasps[-1]["selected_yaw_rad"] = self.selected_yaw
        dx, dy = self._placement_xy()
        stack_z = (
            sum(2 * self.config["objects"][i]["size"][2] for i in self.completed)
            if self.config["task"] == "stack"
            else 0.0
        )
        self.desired_destination = [
            dx,
            dy,
            stack_z + self.config["objects"][self.current]["size"][2],
        ]
        transfer_z = max(0.27, stack_z + 0.20)
        self.stage_queue = [
            {"phase": "approach", "target": [x, y, approach_z]},
            {"phase": "descend", "target": [x, y, grasp_z]},
            {"phase": "close"},
            {"phase": "lift", "target": [x, y, transfer_z]},
            {"phase": "transfer", "target": [dx, dy, transfer_z]},
            {"phase": "lower", "target": [dx, dy, stack_z + grasp_z + 0.003]},
            {"phase": "release"},
            {"phase": "retreat", "target": [dx, dy, transfer_z]},
            {"phase": "viewpoint", "target": [0.34, 0.0, 0.36]},
            {"phase": "verify"},
        ]
        return True

    def _placement_xy(self) -> tuple[float, float]:
        dx, dy = self.config["destination"]
        if self.config["task"] == "sort":
            dx += (len(self.completed) - (len(self.config["objects"]) - 1) / 2) * 0.08
        return float(dx), float(dy)

    def _failure(self, reason: str) -> None:
        self.event("failure", reason=reason, object=self.current)
        self.failures.append({"reason": reason, "object": self.current, "step": self.steps})
        self.stage_queue, self.segments, self.segment = [], [], None
        if self.current is not None and self.attempts.get(self.current, 0) < self.config["retries"]:
            self.attempts[self.current] = self.attempts.get(self.current, 0) + 1
            self.close = False
            point = [0.34, 0.0, 0.36]
            if self._plan(
                point, down_quaternion(self.selected_yaw), allow_touch=True, phase="retry"
            ):
                self.event("retry", reason=reason, attempt=self.attempts[self.current])
                return
        self.status, self.phase, self.error = "FAILED", "failed", reason

    def _next(self) -> None:
        c = self.cell
        if self.phase == "close" and self.current not in c.contact_reading()["bilateral_objects"]:
            self._failure("NO_BILATERAL_CONTACT")
            return
        if self.phase in ("lift", "transfer") and self.current is not None:
            pos = c.data.xpos[c.object_bodies[self.current]]
            if pos[2] < 0.09 or math.dist(pos[:2], c.data.site_xpos[c.tcp][:2]) > 0.07:
                self._failure("GRASP_LOST")
                return
        if self.phase == "retry":
            self._wait("observe", 0.65)
            return
        if self.phase == "observe":
            if self.current is None:
                self.current = self.targets.pop(0)
            self.sensor_records.append(self.image)
            obs = self._observation(self.current)
            if obs is None:
                self._failure("OBJECT_NOT_VISIBLE")
                return
            if not self._choose_grasp(obs):
                self._failure("NO_FEASIBLE_GRASP")
                return
        if self.phase == "verify":
            assert self.current is not None and self.desired_destination is not None
            self.sensor_records.append(self.image)
            observation = self._observation(self.current)
            actual = c.data.xpos[c.object_bodies[self.current]].tolist()
            error = math.dist(actual[:2], self.desired_destination[:2])
            verified = (
                observation is not None
                and math.dist(observation["position_m"][:2], self.desired_destination[:2]) < 0.038
            )
            # Outcome also requires physical settled height; camera alone cannot certify dynamics.
            success = (
                verified and error < 0.04 and abs(actual[2] - self.desired_destination[2]) < 0.018
            )
            self.placements.append(
                {
                    "object": self.current,
                    "success": success,
                    "position_m": actual,
                    "goal_m": self.desired_destination,
                    "error_m": error,
                    "camera_verified": verified,
                    "step": self.steps,
                }
            )
            if not success:
                self._failure("PLACEMENT_NOT_VERIFIED")
                return
            self.completed.append(self.current)
            self.event("placed", object=self.current, error_m=error)
            self.current = None
            if self.targets:
                self._wait("observe", 0.6)
            else:
                self.phase, self.status = "finished", "SUCCEEDED"
            return
        if not self.stage_queue:
            return
        stage = self.stage_queue.pop(0)
        phase = stage["phase"]
        if phase in ("close", "release"):
            self.close = phase == "close"
            self._wait(phase, 0.85)
        elif phase == "verify":
            self._wait(phase, 0.7)
        elif not self._plan(
            stage["target"],
            down_quaternion(self.selected_yaw),
            held=phase in ("lift", "transfer", "lower"),
            allow_touch=phase in ("descend", "retreat"),
            phase=phase,
            minimum=1.3 if phase in ("descend", "lower", "lift") else 1.6,
        ):
            self._failure("PATH_OR_IK_BLOCKED")

    def _before_step(self) -> None:
        """Optional live-world input; batch experiments leave this hook empty."""

    def _after_step(self) -> None:
        """Optional live-world supervision after one deterministic physics step."""

    def _after_physics(self) -> None:
        """Live manual constraints before measurement; empty in batch experiments."""

    def advance(self, steps: int = 40) -> dict[str, Any]:
        require(type(steps) is int and 1 <= steps <= 500, ErrorCode.INVALID_VALUE, "steps 1..500")
        c, np = self.cell, self.cell.np
        zero = np.zeros(len(c.joint_ids))
        for _ in range(steps):
            self._before_step()
            if self.status in ("SUCCEEDED", "FAILED", "STOPPED"):
                break
            velocity, acceleration = zero, zero
            segment = self.segment
            if segment is not None:
                if "grip" in segment:
                    self.close = segment["grip"]
                self.tick_in_segment += 1
                duration = segment["duration"]
                u = min(1.0, self.tick_in_segment * 0.002 / duration)
                delta = segment["end"] - segment["start"]
                s, ds, dds = time_profile(segment.get("trajectory", self.config["trajectory"]), u)
                self.reference = segment["start"] + delta * s
                velocity = delta * (ds / duration)
                acceleration = delta * (dds / duration**2)
                if self.tick_in_segment * 0.002 > duration:
                    velocity, acceleration = zero, zero
            if self.steps >= self.push_until:
                c.data.xfrc_applied[:] = 0
            c.advance(self.reference, velocity, acceleration, self.close)
            self._after_physics()
            self.steps += 1
            reading = c.contact_reading()
            self.bilateral += int(bool(reading["bilateral_objects"]))
            self.collision_steps += int(bool(reading["unintended"]))
            self.squared_error += float(np.sum((c.data.qpos[c.q_indices] - self.reference) ** 2))
            self.effort_integral += float(np.sum(c.data.ctrl[c.ctrl_indices] ** 2)) * 0.002
            for i, body in enumerate(c.object_bodies):
                self.max_lift = max(
                    self.max_lift,
                    float(c.data.xpos[body][2]) - self.config["objects"][i]["size"][2],
                )
            if self.steps % 50 == 0:
                self._sense()
            if self.steps % 20 == 0:
                self._record_frame()
            if segment is not None and self.tick_in_segment * 0.002 >= segment["duration"] + 0.22:
                self.segment = self.segments.pop(0) if self.segments else None
                self.tick_in_segment = 0
                if self.segment is None:
                    if self.status == "RUNNING":
                        self._next()
                    elif self.phase in ("teleop", "teach"):
                        self.phase = "idle"
            if self.step_limit is not None and self.steps >= self.step_limit:
                self.status, self.phase, self.error = "FAILED", "failed", "TIME_BUDGET"
            self._after_step()
        return self.snapshot()

    def command(self, command: Any, *, log: bool = True) -> dict[str, Any]:
        require(
            isinstance(command, dict) and isinstance(command.get("action"), str),
            ErrorCode.INVALID_VALUE,
            "action required",
        )
        action = command["action"]
        allowed = {
            "start": {"action"},
            "stop": {"action"},
            "gripper": {"action", "close"},
            "target": {"action", "position_m", "quaternion_wxyz"},
            "joints": {"action", "q"},
            "push": {"action", "object", "force_n"},
            "teach_add": {"action"},
            "teach_play": {"action"},
        }
        require(
            action in allowed and set(command) == allowed[action],
            ErrorCode.INVALID_VALUE,
            "invalid action fields",
        )
        c = self.cell
        if action == "start":
            require(
                self.status == "READY",
                ErrorCode.INVALID_LIFECYCLE,
                "reset before starting a new task",
            )
            self.targets = list(
                range(1 if self.config["task"] == "single" else len(c.object_bodies))
            )
            self.reference = c.data.qpos[c.q_indices].copy()
            self.segments = []
            self.status = "RUNNING"
            self._wait("observe", 0.65)
        elif action == "stop":
            self.status, self.phase = "STOPPED", "idle"
            self.segments, self.stage_queue, self.segment = [], [], None
        elif action == "push":
            index = command["object"]
            require(
                type(index) is int and 0 <= index < len(c.object_bodies),
                ErrorCode.INVALID_VALUE,
                "invalid object",
            )
            force = scene_vector(command["force_n"], "force_n", 3)
            require(
                all(abs(f) <= 3 for f in force),
                ErrorCode.INVALID_VALUE,
                "force limited to 3 N per axis",
            )
            c.data.xfrc_applied[c.object_bodies[index], :3] = force
            self.push_until = self.steps + 50
            self.event("external_force", object=index, force_n=list(force), duration_s=0.1)
        else:
            require(
                self.status == "READY",
                ErrorCode.INVALID_LIFECYCLE,
                "manual commands require a ready scene",
            )
            if action == "gripper":
                require(
                    type(command["close"]) is bool, ErrorCode.INVALID_VALUE, "close must be boolean"
                )
                self.close = command["close"]
            elif action == "target":
                p = scene_vector(command["position_m"], "position_m", 3)
                require(
                    0.12 <= p[0] <= 0.70 and abs(p[1]) <= 0.4 and 0.065 <= p[2] <= 0.65,
                    ErrorCode.INVALID_VALUE,
                    "target outside work volume",
                )
                quat = list(scene_vector(command["quaternion_wxyz"], "quaternion", 4))
                require(
                    abs(math.sqrt(sum(v * v for v in quat)) - 1) <= 1e-9,
                    ErrorCode.INVALID_VALUE,
                    "unit quaternion required",
                )
                self.segment, self.segments = None, []
                self.reference = c.data.qpos[c.q_indices].copy()
                if not self._plan(list(p), quat, phase="teleop"):
                    self.event("manual_target_rejected", target_m=list(p))
            elif action == "joints":
                q = list(scene_vector(command["q"], "q", len(c.joint_ids)))
                require(
                    all(lo <= x <= hi for x, (lo, hi) in zip(q, c.limits, strict=True)),
                    ErrorCode.INVALID_VALUE,
                    "joint limits",
                )
                plan = plan_path(
                    c.data.qpos[c.q_indices].tolist(),
                    q,
                    c.limits,
                    CollisionProbe(c),
                    seed=self.config["seed"],
                )
                self.plans.append(plan)
                require(plan["status"] == "FOUND", ErrorCode.INVALID_VALUE, "joint motion blocked")
                self.segment, self.segments = None, []
                self._move(plan["path"], "teleop")
            elif action == "teach_add":
                require(
                    len(self.teaching) < 12, ErrorCode.INVALID_VALUE, "maximum 12 teaching points"
                )
                self.teaching.append({"q": c.data.qpos[c.q_indices].tolist(), "close": self.close})
            elif action == "teach_play":
                require(bool(self.teaching), ErrorCode.INVALID_VALUE, "no teaching points")
                path = [c.data.qpos[c.q_indices].tolist()]
                prepared = []
                for item in self.teaching:
                    plan = plan_path(
                        path[-1], item["q"], c.limits, CollisionProbe(c), seed=self.config["seed"]
                    )
                    require(
                        plan["status"] == "FOUND", ErrorCode.INVALID_VALUE, "teaching path blocked"
                    )
                    path += plan["path"][1:]
                    for a, b in zip(plan["path"], plan["path"][1:], strict=False):
                        delta = c.np.array(b) - c.np.array(a)
                        duration = (
                            math.ceil(max(1.5, float(c.np.max(c.np.abs(delta))) * 2) / 0.002)
                            * 0.002
                        )
                        prepared.append(
                            {"start": c.np.array(a), "end": c.np.array(b), "duration": duration}
                        )
                    prepared.append(
                        {
                            "start": c.np.array(item["q"]),
                            "end": c.np.array(item["q"]),
                            "duration": 0.5,
                            "grip": item["close"],
                        }
                    )
                self.segment, self.segments = None, []
                self.segments = prepared
                self.segment = self.segments.pop(0)
                self.tick_in_segment, self.phase = 0, "teach"
        if log:
            self.actions.append({"step": self.steps, "command": copy.deepcopy(command)})
        return self.snapshot()

    def metrics(self) -> dict[str, Any]:
        return {
            "placed": len(self.completed),
            "requested": 1 if self.config["task"] == "single" else len(self.config["objects"]),
            "lift_m": self.max_lift,
            "bilateral_steps": self.bilateral,
            "collision_steps": self.collision_steps,
            "joint_tracking_rms": math.sqrt(self.squared_error / max(1, self.steps)),
            "effort_squared_integral": self.effort_integral,
            "landing_error_m": self.placements[-1]["error_m"] if self.placements else None,
            "perception_xy_rms_m": math.sqrt(self.sensor_error_sum / self.sensor_error_count)
            if self.sensor_error_count
            else None,
            "simulation_time_s": float(self.cell.data.time),
            "steps": self.steps,
            "retries": sum(self.attempts.values()),
            "planning_checks": sum(p["collision_checks"] for p in self.plans),
            "joint_path_length": sum(p["length_joint_space"] for p in self.plans),
        }

    def snapshot(self, *, sensor: bool = False) -> dict[str, Any]:
        data = {
            "status": self.status,
            "phase": self.phase,
            "phase_label": PHASE_NAMES[self.phase],
            "error": self.error,
            "step": self.steps,
            "frame": self.cell.frame(self.phase),
            "metrics": self.metrics(),
            "events": self.events[-20:],
            "detections": self.detections,
            "plan": self.plans[-1] if self.plans else None,
            "solution": self.solutions[-1] if self.solutions else None,
            "grasp": self.grasps[-1] if self.grasps else None,
            "trace": self.trace[-400:],
            "teaching": self.teaching,
            "limits": self.cell.limits,
            "sensor_time_s": self.image["time_s"],
        }
        if sensor:
            data["sensor"] = self.image
        return data

    def report(self) -> dict[str, Any]:
        frames = list(self.frames)
        if frames[-1]["time_s"] != float(self.cell.data.time):
            final_frame = self.cell.frame(self.phase)
            final_frame["reference_q"] = self.reference.tolist()
            frames.append(final_frame)
        result = {
            "schema": "studio-v1",
            "config": self.config,
            "status": self.status,
            "error": self.error,
            "metrics": self.metrics(),
            "frames": frames,
            "trace": self.trace,
            "sensors": self.sensor_records,
            "events": self.events,
            "actions": self.actions,
            "solutions": self.solutions,
            "plans": self.plans,
            "grasps": self.grasps,
            "placements": self.placements,
            "failures": self.failures,
            "seed": self.config["seed"],
            "source_hash": source_hash(),
            "model_hash": self.cell.model_hash,
            "robot_hash": self.cell.robot_hash,
            "config_hash": digest(self.config),
            "task_hash": digest(
                {
                    "task": self.config["task"],
                    "destination": self.config["destination"],
                    "version": 1,
                }
            ),
            "dt_s": 0.002,
            "frame_sample_dt_s": 0.04,
            "sensor_sample_dt_s": 0.1,
            "sensor_image_record_dt_s": 1.0,
            "frame_id": "world",
            "engine_version": str(self.cell.mj.__version__),
            "numpy_version": str(self.cell.np.__version__),
            "python_version": platform.python_version(),
            "application_version": __version__,
            "attachment_constraints": int(self.cell.model.neq),
            "mapping": {
                "qpos": self.cell.q_indices,
                "qvel": self.cell.v_indices,
                "ctrl": self.cell.ctrl_indices,
                "nq": self.cell.model.nq,
                "nv": self.cell.model.nv,
            },
        }
        result["run_id"] = "studio_" + digest(result)[:24]
        return result


def run_experiment(
    config: Any, actions: list[dict[str, Any]] | None = None, total_steps: int | None = None
) -> dict[str, Any]:
    session = Session(config)
    if actions is None:
        session.command({"action": "start"})
        while session.status == "RUNNING":
            session.advance(500)
    else:
        for item in actions:
            remaining = item["step"] - session.steps
            while remaining > 0:
                batch = min(500, remaining)
                session.advance(batch)
                remaining -= batch
            session.command(item["command"])
        remaining = (total_steps or session.steps) - session.steps
        while remaining > 0:
            batch = min(500, remaining)
            session.advance(batch)
            remaining -= batch
    return session.report()
