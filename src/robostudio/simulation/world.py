"""Persistent goals and bounded physical pointer interaction, separate from batch runs."""

import copy
import math
from typing import Any

from ..core import ErrorCode
from ..core.errors import require
from ..serialization import digest
from .objects import ObjectHandle
from .recovery import contact_retreat
from .runtime import Session, down_quaternion
from .scene import scene_vector, validate_scene


class WorldSession(Session):
    """Fixed 2 ms simulation; commands are recorded at exact step boundaries."""

    def __init__(self, config: Any):
        super().__init__(config, live=True)
        self.initial_config = copy.deepcopy(self.config)
        self.step_limit = None
        self.autonomy = False
        self.goal_state = "manual"
        self.next_observe = 0
        self.interaction: dict[str, Any] | None = None
        self.force_n = [0.0, 0.0, 0.0]
        self.force_point_m: list[float] | None = None
        self.interaction_note = ""
        self.obstacle_target: tuple[int, list[float]] | None = None
        self.tcp_dirty = False
        self.cycles = 0
        self.discarded_checks = 0
        self.discarded_length = 0.0
        self.recovery_pending = False
        self.recovery_running = False
        self.recoveries: list[dict[str, Any]] = []
        self.selected_object = 0
        self.object_handle: ObjectHandle | None = None

    def _clear_pointer(self) -> None:
        if self.object_handle is not None:
            if self.object_handle.blocking is not None:
                self.event("drag_cleared", reason="interaction_ended")
            self.object_handle.enforce()
            self.object_handle = None
        self.interaction = None
        self.tcp_dirty = False
        self.obstacle_target = None
        self.force_n = [0.0, 0.0, 0.0]
        self.force_point_m = None
        self.cell.data.qfrc_applied[:] = 0
        self.cell.data.xfrc_applied[:] = 0
        self.push_until = self.steps

    def _recover_contact(self) -> bool:
        result = contact_retreat(self.cell, self.config["solver"], self.config["damping"])
        if result["status"] == "CLEAR":
            self.recovery_pending = False
            return False
        solution = result.pop("solution", None)
        if solution is not None:
            self.solutions.append(
                {
                    **solution,
                    "phase": "recovery",
                    "step": self.steps,
                    "scene_qpos": self.cell.data.qpos.tolist(),
                    "scene_mocap_pos": self.cell.data.mocap_pos.tolist(),
                    "scene_mocap_quat": self.cell.data.mocap_quat.tolist(),
                }
            )
        self.recoveries.append({**result, "step": self.steps})
        if result["status"] == "FOUND":
            self.recovery_pending, self.recovery_running = False, True
            self.goal_state, self.error = "recovering", None
            self._move(result["path"], "recovery", minimum=1.0)
            self.event("contact_recovery", reason=result["reason"])
        else:
            self.goal_state, self.error = "blocked", result["reason"]
            self.next_observe = self.steps + 2500
            self.event("recovery_blocked", reason=result["reason"])
        return True

    def _hold(self) -> None:
        self.recovery_running = False
        self.segment, self.segments, self.stage_queue = None, [], []
        self.reference = self.cell.data.qpos[self.cell.q_indices].copy()
        self.status, self.phase, self.error = "READY", "idle", None
        self.current = None
        self.next_observe = self.steps + 500

    def _object_destination(self, index: int) -> tuple[float, float]:
        x, y = self.config["destination"]
        if self.config["task"] == "sort":
            x += (index - (len(self.cell.object_bodies) - 1) / 2) * 0.08
        elif self.config["task"] == "single":
            # Separate receiving slots let another color be selected after placement.
            x += (0, -0.07, 0.07)[index]
        return float(x), float(y)

    def _placement_xy(self) -> tuple[float, float]:
        assert self.current is not None
        return self._object_destination(self.current)

    def _select_object(self, index: int) -> None:
        if index == self.selected_object:
            return
        self.selected_object = index
        if self.config["task"] == "single" and not self.close:
            self._hold()
            self.next_observe = self.steps
            self.goal_state = "working" if self.autonomy else "manual"
        elif self.config["task"] == "sort" and index in self.targets:
            self.targets.remove(index)
            self.targets.insert(0, index)
        self.event("object_selected", object=index)

    def _goal_check(self) -> None:
        indices = (
            [self.selected_object]
            if self.config["task"] == "single"
            else list(range(len(self.cell.object_bodies)))
        )
        retained = []
        for index in indices:
            x, y = self._object_destination(index)
            obs = self._observation(index)
            height = (
                sum(2 * self.config["objects"][i]["size"][2] for i in range(index))
                if self.config["task"] == "stack"
                else 0
            )
            # RGBD measures the top surface, not the object's centre.
            top = height + 2 * self.config["objects"][index]["size"][2]
            valid = (
                obs is not None
                and math.dist(obs["position_m"][:2], [x, y]) < 0.038
                and abs(obs["position_m"][2] - top) < 0.025
            )
            if not valid:
                if self.config["task"] == "stack":
                    break
                continue
            retained.append(index)
        self.completed = retained
        self.next_observe = self.steps + 500
        if len(retained) == len(indices):
            self.goal_state, self.phase = "satisfied", "finished"
            return
        self.goal_state, self.status = "working", "RUNNING"
        self.targets = [i for i in indices if i not in retained]
        if self.config["task"] == "sort" and self.selected_object in self.targets:
            self.targets.remove(self.selected_object)
            self.targets.insert(0, self.selected_object)
        self.current, self.attempts, self.close, self.error = None, {}, False, None
        self.cycles += 1
        self.event("goal_reobserve", cycle=self.cycles, objects=self.targets.copy())
        self._wait("observe", 0.65)

    def _before_step(self) -> None:
        c, np = self.cell, self.cell.np
        c.data.qfrc_applied[:] = 0
        self.force_n = [0.0, 0.0, 0.0]
        self.force_point_m = None
        if self.obstacle_target is not None:
            index, target = self.obstacle_target
            body = c.model.body(f"obstacle_body{index}").id
            mocap = c.model.body_mocapid[body]
            old = c.data.mocap_pos[mocap].copy()
            delta = np.array(target) - old
            c.data.mocap_pos[mocap] += delta * min(
                1.0, 0.0004 / max(1e-9, float(np.linalg.norm(delta)))
            )
            c.mj.mj_forward(c.model, c.data)
            geom = c.obstacle_geoms[index]
            occupied = any(
                c.mj.mj_geomDistance(c.model, c.data, geom, other, 0.003, None) < 0.002
                for other in [
                    *c.robot_geoms,
                    *c.object_geoms,
                    *(g for g in c.obstacle_geoms if g != geom),
                ]
            )
            if occupied:
                c.data.mocap_pos[mocap] = old
                c.mj.mj_forward(c.model, c.data)
                self.interaction_note = "障碍移动受阻：与机械臂或物体距离不足 2 mm"
                self.obstacle_target = None
            elif float(np.linalg.norm(delta)) < 0.0005:
                self.obstacle_target = None
            # Planning and exported scene fields use the actual kinematic obstacle pose.
            self.config["obstacles"][index]["xy"] = c.data.mocap_pos[mocap, :2].tolist()
        grab = self.interaction
        if grab and grab["kind"] == "body":
            body = grab["body"]
            rotation = c.data.xmat[body].reshape(3, 3)
            point = c.data.xpos[body] + rotation @ np.array(grab["local_m"])
            jac = np.zeros((3, c.model.nv))
            c.mj.mj_jac(c.model, c.data, jac, None, point, body)
            velocity = jac @ c.data.qvel
            force = 250 * (np.array(grab["position_m"]) - point) - 8 * velocity
            force *= min(1.0, 30 / max(1e-9, float(np.linalg.norm(force))))
            c.mj.mj_applyFT(c.model, c.data, force, np.zeros(3), point, body, c.data.qfrc_applied)
            self.force_n = force.tolist()
            self.force_point_m = point.tolist()
        if grab and grab["kind"] == "tcp" and self.tcp_dirty and self.steps % 50 == 0:
            self.tcp_dirty = False
            found = self._plan(
                grab["position_m"],
                down_quaternion(0),
                allow_touch=True,
                phase="teleop",
                minimum=0.25,
            )
            self.interaction_note = (
                "逆解与碰撞检查通过" if found else "目标不可达或路径受阻，保持当前安全参考"
            )
        if (
            self.autonomy
            and (not grab or (grab.get("continues_task") and not self.recovery_pending))
            and self.obstacle_target is None
            and self.segment is None
            and self.status == "READY"
            and self.steps >= self.next_observe
        ):
            if not (self.recovery_pending and self._recover_contact()):
                self._goal_check()

    def _after_physics(self) -> None:
        if self.object_handle is not None:
            self.object_handle.enforce()

    def _after_step(self) -> None:
        if self.recovery_running and self.segment is None:
            self.recovery_running = False
            self.goal_state = "working"
            self.next_observe = self.steps + 250
            self.event("contact_recovery_complete")
        if self.status == "SUCCEEDED":
            self.status, self.goal_state = "READY", "satisfied"
            self.next_observe = self.steps + 500
            if self.config["task"] == "single" and self.selected_object not in self.completed:
                self.goal_state, self.next_observe = "working", self.steps
        elif self.status == "FAILED":
            self.status, self.goal_state = "READY", "blocked"
            self.segment, self.segments, self.stage_queue = None, [], []
            self.reference = self.cell.data.qpos[self.cell.q_indices].copy()
            self.next_observe = self.steps + 2500
            self.recovery_pending = True
            self.close = False
        # A rolling visual history bounds memory; initial config + all commands retain replay.
        if self.steps % 500 == 0:
            for name, limit in (
                ("frames", 1000),
                ("trace", 1000),
                ("sensor_records", 40),
                ("plans", 80),
                ("solutions", 80),
                ("grasps", 80),
                ("events", 2000),
                ("placements", 500),
                ("failures", 500),
                ("recoveries", 80),
            ):
                rows = getattr(self, name)
                if len(rows) > limit:
                    if name == "plans":
                        self.discarded_checks += sum(p["collision_checks"] for p in rows[:-limit])
                        self.discarded_length += sum(p["length_joint_space"] for p in rows[:-limit])
                    del rows[:-limit]

    def _plan(self, *args: Any, **kwargs: Any) -> bool:
        result = super()._plan(*args, **kwargs)
        for rows in (self.plans, self.solutions):
            if rows and rows[-1].get("step") == self.steps:
                rows[-1]["scene_mocap_pos"] = self.cell.data.mocap_pos.tolist()
                rows[-1]["scene_mocap_quat"] = self.cell.data.mocap_quat.tolist()
        return result

    def _move(self, *args: Any, **kwargs: Any) -> None:
        super()._move(*args, **kwargs)
        for segment in ([self.segment] if self.segment else []) + self.segments:
            segment["trajectory"] = self.config["trajectory"]

    def command(self, command: Any, *, log: bool = True) -> dict[str, Any]:
        require(isinstance(command, dict), ErrorCode.INVALID_VALUE, "command object required")
        action = command.get("action")
        require(isinstance(action, str), ErrorCode.INVALID_VALUE, "action string required")
        fields = {
            "select_object": {"action", "index"},
            "patch": {"action", "config"},
            "grab": {"action", "kind", "index", "point_m"},
            "drag": {"action", "position_m"},
            "release": {"action"},
            "start": {"action"},
            "stop": {"action"},
        }
        require(
            len(self.actions) < 50000 or action in ("release", "stop"),
            ErrorCode.INVALID_VALUE,
            "save and reset after 50000 inputs",
        )
        if action not in fields:
            # Legacy numeric teleoperation stays available during a goal, with validation first.
            if action in ("target", "joints", "gripper", "teach_add", "teach_play"):
                status = self.status
                self.status = "READY"
                try:
                    super().command(command, log=log)
                except Exception:
                    self.status = status
                    raise
                if action == "teach_add":
                    self.status = status
                    return self.snapshot()
                if action == "gripper":
                    self._hold()
                self.stage_queue, self.current = [], None
                self.next_observe = self.steps + 1500
                return self.snapshot()
            return super().command(command, log=log)
        require(set(command) == fields[action], ErrorCode.INVALID_VALUE, "invalid world fields")
        c = self.cell
        if action == "select_object":
            index = command["index"]
            require(
                type(index) is int and 0 <= index < len(c.object_bodies),
                ErrorCode.INVALID_VALUE,
                "invalid object",
            )
            self._select_object(index)
        elif action == "patch":
            patch = command["config"]
            allowed = {
                "solver",
                "planner",
                "controller",
                "trajectory",
                "damping",
                "task",
                "destination",
                "observation",
                "noise_mm",
                "dropout",
                "latency_ms",
            }
            require(
                isinstance(patch, dict) and set(patch) <= allowed,
                ErrorCode.INVALID_VALUE,
                "only runtime settings can be patched",
            )
            cfg = validate_scene({**copy.deepcopy(self.config), **patch})
            self.config.update(cfg)
            c.config["controller"] = cfg["controller"]
            if "task" in patch or "destination" in patch:
                self._hold()
                self.completed = []
                self.autonomy = True
                self.goal_state = "working"
            # Preserve current segment; new IK/planner/trajectory settings used by next segment.
            self.event("settings_changed", config=copy.deepcopy(patch))
        elif action == "start":
            if self.autonomy and self.status == "RUNNING":
                # Starting an already active goal must not release a held object.
                self._clear_pointer()
            else:
                settle_gripper = self.close
                if self.interaction:
                    self._hold()
                self._clear_pointer()
                self.recovery_pending = True
                self.close = False
                self.autonomy, self.goal_state = True, "working"
                self.next_observe = self.steps + (250 if settle_gripper else 0)
        elif action == "stop":
            self._clear_pointer()
            self.recovery_pending = False
            self._hold()
            self.autonomy, self.goal_state = False, "manual"
        elif action == "grab":
            kind, index = command["kind"], command["index"]
            point = list(scene_vector(command["point_m"], "world point_m", 3))
            require(
                kind in ("tcp", "object", "body", "obstacle", "destination") and type(index) is int,
                ErrorCode.INVALID_VALUE,
                "invalid selection",
            )
            body = None
            if kind == "object":
                require(
                    0 <= index < len(c.object_bodies), ErrorCode.INVALID_VALUE, "invalid object"
                )
                body = c.object_bodies[index]
            elif kind == "body":
                require(
                    index in {int(c.model.geom_bodyid[g]) for g in c.robot_geoms}
                    and int(c.model.body_dofnum[index]) > 0,
                    ErrorCode.INVALID_VALUE,
                    "select a movable robot body",
                )
                body = index
            elif kind == "obstacle":
                require(
                    0 <= index < len(c.obstacle_geoms), ErrorCode.INVALID_VALUE, "invalid obstacle"
                )
            else:
                require(index == 0, ErrorCode.INVALID_VALUE, "invalid handle")
            require(
                all(abs(v) <= 2 for v in point), ErrorCode.INVALID_VALUE, "point outside workspace"
            )
            if body is not None:
                require(
                    math.dist(point, c.data.xpos[body]) < 0.65,
                    ErrorCode.INVALID_VALUE,
                    "point too far from body",
                )
            continues_task = kind == "body" and self.autonomy
            self._clear_pointer()
            if not continues_task:
                self.recovery_pending = False
                self._hold()
            self.interaction = {
                "kind": kind,
                "index": index,
                "position_m": point,
                "continues_task": continues_task,
            }
            if body is not None:
                self.interaction.update(
                    body=body,
                    local_m=(
                        c.data.xmat[body].reshape(3, 3).T @ (c.np.array(point) - c.data.xpos[body])
                    ).tolist(),
                )
            if kind == "object":
                self._select_object(index)
                self.close = False
                self.object_handle = ObjectHandle(c, index, point)
                self.interaction["mode"] = "direct_position"
                self.event("object_hold", object=index, mode="direct_position")
            if continues_task:
                if self.goal_state == "interacting":
                    self.goal_state = "working"
                self.interaction_note = "外力干预中 · 控制器继续执行当前任务"
                self.event("force_started", phase=self.phase, object=self.current)
            else:
                self.goal_state = "interacting"
                self.interaction_note = (
                    "按住移动 · 滚轮 / Shift 调整高度 · 松手恢复物理"
                    if kind == "object"
                    else "手动接管中 · 物理持续运行"
                )
        elif action == "drag":
            require(self.interaction is not None, ErrorCode.INVALID_LIFECYCLE, "grab first")
            point = list(scene_vector(command["position_m"], "world position_m", 3))
            require(
                -0.15 <= point[0] <= 0.85 and abs(point[1]) <= 0.55 and 0.005 <= point[2] <= 0.8,
                ErrorCode.INVALID_VALUE,
                "pointer outside work volume",
            )
            assert self.interaction is not None
            kind, index = self.interaction["kind"], self.interaction["index"]
            if kind == "obstacle":
                require(
                    0.15 <= point[0] <= 0.65 and abs(point[1]) <= 0.32,
                    ErrorCode.INVALID_VALUE,
                    "obstacle outside table",
                )
                point[2] = self.config["obstacles"][index]["size"][2]
                self.obstacle_target = index, point
            elif kind == "destination":
                require(
                    0.28 <= point[0] <= 0.55 and 0.08 <= point[1] <= 0.30,
                    ErrorCode.INVALID_VALUE,
                    "destination outside receiving zone",
                )
                self.config["destination"] = point[:2]
                self.completed = []
            self.interaction["position_m"] = point
            if kind == "object":
                assert self.object_handle is not None
                previous = self.object_handle.blocking
                reached = self.object_handle.move(point)
                blocking = self.object_handle.blocking
                if blocking is not None:
                    blocking["step"] = self.steps
                    if previous is None or previous["geom_ids"] != blocking["geom_ids"]:
                        self.event("drag_blocked", blocking=self.object_handle.feedback())
                elif previous is not None:
                    self.event("drag_cleared", reason="moved_clear")
                self.interaction_note = (
                    "按住移动 · 滚轮 / Shift 调整高度 · 松手恢复物理"
                    if reached
                    else f"移动受阻：{blocking['label'] if blocking else '障碍'} · 抬高或绕开后继续"
                )
            self.tcp_dirty = kind == "tcp"
        elif action == "release":
            had_interaction = self.interaction is not None
            continued_task = bool(self.interaction and self.interaction.get("continues_task"))
            if self.interaction and self.interaction["kind"] == "tcp" and self.tcp_dirty:
                self._plan(
                    self.interaction["position_m"],
                    down_quaternion(0),
                    allow_touch=True,
                    phase="teleop",
                    minimum=0.25,
                )
            # An obstacle already moving finishes its bounded motion after mouse-up.
            obstacle_target = self.obstacle_target
            self._clear_pointer()
            self.obstacle_target = obstacle_target
            if continued_task:
                self.interaction_note = "外力已释放 · 保留当前任务进度"
                self.event("force_released", phase=self.phase, object=self.current)
            elif had_interaction:
                if self.autonomy:
                    self.close = False
                    self.recovery_pending = True
                self.next_observe = self.steps + (250 if self.recovery_pending else 750)
                self.goal_state = "working" if self.autonomy else "manual"
                self.interaction_note = (
                    "已松手 · 将根据当前观测继续任务" if self.autonomy else "自由操控"
                )
        if log:
            self.actions.append({"step": self.steps, "command": copy.deepcopy(command)})
        return self.snapshot()

    def metrics(self) -> dict[str, Any]:
        values = super().metrics()
        values["planning_checks"] += self.discarded_checks
        values["joint_path_length"] += self.discarded_length
        return values

    def snapshot(self, *, sensor: bool = False) -> dict[str, Any]:
        data = super().snapshot(sensor=sensor)
        data["world"] = {
            "autonomy": self.autonomy,
            "goal_state": self.goal_state,
            "interaction": copy.deepcopy(self.interaction),
            "force_n": self.force_n,
            "force_point_m": self.force_point_m,
            "note": self.interaction_note,
            "blocking": self.object_handle.feedback() if self.object_handle is not None else None,
            "cycles": self.cycles,
            "active_object": self.current,
            "selected_object": self.selected_object,
            "selected_destination_m": list(self._object_destination(self.selected_object)),
            "gripper_closed": self.close,
            "recovery": copy.deepcopy(self.recoveries[-1]) if self.recoveries else None,
            "config": copy.deepcopy(self.config),
            "history_window_s": 40,
        }
        if self.interaction:
            data["phase_label"] = (
                data["phase_label"] + " · 外力干预中"
                if self.interaction.get("continues_task")
                else "自由操控 · 物理运行中"
            )
        elif self.goal_state == "satisfied":
            data["phase_label"] = "目标已满足 · 持续观察"
        elif self.goal_state == "blocked":
            data["phase_label"] = "任务受阻 · 等待重试或调整场景"
        elif self.goal_state == "recovering":
            data["phase_label"] = "接触恢复 · 抬起夹爪后重新观察"
        elif not self.autonomy:
            data["phase_label"] = "自由工作环境"
        return data

    def report(self) -> dict[str, Any]:
        result = super().report()
        result.pop("run_id")
        result.update(
            schema="studio-world-v1",
            config=self.initial_config,
            effective_config=copy.deepcopy(self.config),
            config_hash=digest(self.initial_config),
            world=self.snapshot()["world"],
            recoveries=copy.deepcopy(self.recoveries),
            history_window_s=40,
            replay_command=".venv/Scripts/python.exe scripts/replay_world.py --record <run.json>",
        )
        result["run_id"] = "studio_" + digest(result)[:24]
        return result


def replay_world(report: dict[str, Any]) -> dict[str, Any]:
    sim = WorldSession(report["config"])
    for item in report["actions"]:
        while sim.steps < item["step"]:
            sim.advance(min(500, item["step"] - sim.steps))
        sim.command(item["command"])
    while sim.steps < report["metrics"]["steps"]:
        sim.advance(min(500, report["metrics"]["steps"] - sim.steps))
    return sim.report()
