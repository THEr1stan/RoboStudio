"""Live environment contracts: no reset, physical interaction, persistent goals, replay."""

import copy

import numpy as np
import pytest

from robostudio.core import RoboStudioError
from robostudio.serialization import canonical
from robostudio.studio_scene import default_scene
from robostudio.studio_world import WorldSession, replay_world


def test_world_adapter_preserves_batch_dynamics_without_intervention():
    from robostudio.studio_runtime import Session

    batch, live = Session(default_scene()), WorldSession(default_scene())
    for _ in range(3):
        batch.advance(100)
        live.advance(100)
    assert np.array_equal(batch.cell.data.qpos, live.cell.data.qpos)
    assert np.array_equal(batch.cell.data.qvel, live.cell.data.qvel)
    assert batch.cell.model_hash == live.cell.model_hash


@pytest.mark.parametrize(
    "bad",
    [
        {"action": []},
        {"action": "grab", "kind": "body", "index": -1, "point_m": [0, 0, 0.2]},
        {"action": "patch", "config": {"robot": "gantry"}},
        {"action": "drag", "position_m": [0, 0, float("nan")]},
    ],
)
def test_rejected_world_inputs_do_not_mutate_physics(bad):
    sim = WorldSession(default_scene())
    before = canonical(sim.snapshot())
    with pytest.raises(RoboStudioError):
        sim.command(bad)
    assert canonical(sim.snapshot()) == before
    assert not sim.actions


@pytest.mark.parametrize("with_obstacle", [False, True])
def test_live_task_inspection_restores_mocap_state_without_advancing_world(tmp_path, with_obstacle):
    from robostudio.studio_api import StudioAPI

    api = StudioAPI(tmp_path)
    cfg = default_scene()
    if with_obstacle:
        cfg["obstacles"] = [{"xy": [0.60, 0.26], "size": [0.025, 0.025, 0.07], "yaw": 0}]
    created = api.post("create_world", {"config": cfg})
    sim = api.sessions[created["session"]]
    if with_obstacle:
        sim.command(
            {"action": "grab", "kind": "obstacle", "index": 0, "point_m": [0.6, 0.26, 0.07]}
        )
        sim.command({"action": "drag", "position_m": [0.52, 0.26, 0.07]})
        sim.advance(300)
        sim.command({"action": "release"})
    sim.command(
        {
            "action": "target",
            "position_m": [0.42, -0.14, 0.23],
            "quaternion_wxyz": [0.7071067811865476, 0, 0.7071067811865476, 0],
        }
    )
    state = sim.cell.data.qpos.copy()
    record = api.post("inspect", {"session": created["session"], "family": "ik", "index": 0})
    assert record["origin"] == "actual_task_record"
    assert record["model_hash"] == sim.cell.model_hash
    assert np.array_equal(state, sim.cell.data.qpos)


def test_stacked_goal_continues_observing_without_dismantling():
    cfg = default_scene("gantry")
    cfg["task"] = "stack"
    sim = WorldSession(cfg)
    sim.command({"action": "start"})
    for _ in range(150):
        sim.advance(500)
        if sim.goal_state == "satisfied":
            break
    assert sim.goal_state == "satisfied"
    cycles = sim.cycles
    for _ in range(5):
        sim.advance(500)
    assert sim.goal_state == "satisfied" and sim.cycles == cycles


def test_hot_algorithm_changes_preserve_physics_and_validate_atomically():
    sim = WorldSession(default_scene())
    sim.command({"action": "start"})
    sim.advance(80)
    before = sim.cell.data.qpos.copy(), sim.cell.data.qvel.copy(), sim.steps
    sim.command({"action": "patch", "config": {"solver": "adaptive_dls", "planner": "prm"}})
    assert sim.config["solver"] == "adaptive_dls"
    assert sim.config["planner"] == "prm"
    assert np.array_equal(before[0], sim.cell.data.qpos)
    assert np.array_equal(before[1], sim.cell.data.qvel)
    assert before[2] == sim.steps
    cfg, actions = copy.deepcopy(sim.config), len(sim.actions)
    with pytest.raises(RoboStudioError):
        sim.command({"action": "patch", "config": {"solver": "fake", "controller": "computed"}})
    assert sim.config == cfg and len(sim.actions) == actions


@pytest.mark.parametrize("kind", ["body"])
def test_pointer_spring_changes_motion_without_teleport_and_releases(kind):
    sim = WorldSession(default_scene())
    sim.advance(50)
    c = sim.cell
    undisturbed = WorldSession(default_scene())
    undisturbed.advance(100)
    index = 0 if kind == "object" else c.model.geom_bodyid[c.robot_geoms[-1]].item()
    body = c.object_bodies[0] if kind == "object" else index
    point = c.data.xpos[body].tolist()
    before = c.data.qpos.copy()
    sim.command({"action": "grab", "kind": kind, "index": index, "point_m": point})
    sim.command({"action": "drag", "position_m": [point[0] + 0.06, point[1], point[2] + 0.04]})
    assert np.array_equal(before, c.data.qpos)
    sim.advance(50)
    assert np.linalg.norm(c.data.qpos - before) > 1e-5
    if kind == "body":
        assert (
            np.linalg.norm(c.data.qvel[c.v_indices] - undisturbed.cell.data.qvel[c.v_indices])
            > 1e-4
        )
    assert 0 < np.linalg.norm(sim.force_n) <= 30.00001
    sim.command({"action": "release"})
    sim.advance(1)
    assert not c.data.qfrc_applied.any()


def test_task_completion_keeps_physics_and_displaced_object_is_requested_again():
    sim = WorldSession(default_scene("gantry"))
    sim.command({"action": "start"})
    for _ in range(150):
        sim.advance(500)
        if sim.goal_state == "satisfied":
            break
    assert sim.goal_state == "satisfied"
    end = sim.steps
    sim.advance(250)
    assert sim.steps == end + 250
    assert sim.goal_state == "satisfied"
    p = sim.cell.data.xpos[sim.cell.object_bodies[0]].tolist()
    sim.command({"action": "grab", "kind": "object", "index": 0, "point_m": p})
    sim.command({"action": "drag", "position_m": [0.42, -0.14, 0.06]})
    for _ in range(5):
        sim.advance(250)
    sim.command({"action": "release"})
    for _ in range(4):
        sim.advance(250)
    assert sim.goal_state == "working"
    assert sim.status == "RUNNING"


def test_movable_obstacle_probes_and_world_replay():
    cfg = default_scene()
    cfg["obstacles"] = [{"xy": [0.60, 0.26], "size": [0.025, 0.025, 0.07], "yaw": 0}]
    sim = WorldSession(cfg)
    sim.command({"action": "grab", "kind": "obstacle", "index": 0, "point_m": [0.60, 0.26, 0.07]})
    sim.command({"action": "drag", "position_m": [0.52, 0.26, 0.07]})
    sim.advance(300)
    sim.command({"action": "release"})
    from robostudio.studio_planning import CollisionProbe

    probe = CollisionProbe(sim.cell)
    assert np.array_equal(probe.data.mocap_pos, sim.cell.data.mocap_pos)
    assert sim.cell.data.mocap_pos[0, 0] < 0.59
    sim.command({"action": "patch", "config": {"solver": "pinv"}})
    sim.advance(100)
    report = sim.report()
    assert report["config"] == cfg
    assert report["effective_config"]["solver"] == "pinv"
    assert canonical(report) == canonical(replay_world(report))


def test_tcp_interaction_interrupts_task_without_reset_and_solves():
    sim = WorldSession(default_scene())
    sim.command({"action": "start"})
    sim.advance(40)
    point = sim.cell.data.site_xpos[sim.cell.tcp].tolist()
    sim.command({"action": "grab", "kind": "tcp", "index": 0, "point_m": point})
    sim.command({"action": "drag", "position_m": [0.42, -0.12, 0.25]})
    sim.advance(100)
    assert sim.steps == 140
    assert sim.solutions and sim.solutions[-1]["algorithm"] == "dls"
    assert sim.snapshot()["world"]["interaction"]["kind"] == "tcp"


def press_robot_toward_table(sim, *, manual=False):
    sim.command({"action": "start"})
    while sim.phase != "close" and sim.steps < 15000:
        sim.advance(25)
    assert sim.phase == "close"
    if manual:
        sim.command({"action": "stop"})
    c = sim.cell
    movable = {int(c.model.jnt_bodyid[j]) for j in c.joint_ids}
    selectable = {
        int(c.model.geom_bodyid[g])
        for g in c.robot_geoms
        if c.model.body_dofnum[c.model.geom_bodyid[g]]
    }
    selectable = (selectable & movable) or selectable
    body = min(selectable, key=lambda b: np.linalg.norm(c.data.xpos[b] - c.data.site_xpos[c.tcp]))
    for _ in range(5):
        point = c.data.site_xpos[c.tcp].tolist()
        sim.command({"action": "grab", "kind": "body", "index": body, "point_m": point})
        sim.command({"action": "drag", "position_m": [point[0], point[1], 0.005]})
        sim.advance(500)
        state = c.data.qpos.copy()
        sim.command({"action": "release"})
        assert np.array_equal(state, c.data.qpos), "release must not teleport the scene"
        sim.advance(20)
    if manual:
        sim.command({"action": "start"})


@pytest.mark.parametrize("robot", ["r6", "gantry"])
def test_repeated_force_contact_recovers_and_finishes_without_reset(robot):
    sim = WorldSession(default_scene(robot))
    press_robot_toward_table(sim, manual=True)
    released = sim.steps
    for _ in range(50):
        sim.advance(500)
        if sim.goal_state == "satisfied":
            break
    assert sim.goal_state == "satisfied", sim.failures[-3:]
    assert sim.steps > released and sim.placements[-1]["success"]
    assert not sim.cell.data.qfrc_applied.any()
    retreats = [r for r in sim.recoveries if r["status"] == "FOUND"]
    if robot == "r6":  # This frozen case ends in pad/table contact.
        assert retreats and retreats[0]["collision_checks"] > 80
        assert retreats[0]["final_clearance_m"] >= 0.001
    assert canonical(sim.report()) == canonical(replay_world(sim.report()))


def test_stop_clears_pointer_and_external_force_before_restart():
    sim = WorldSession(default_scene())
    c = sim.cell
    point = c.data.xpos[c.object_bodies[0]].tolist()
    sim.command({"action": "grab", "kind": "object", "index": 0, "point_m": point})
    sim.command({"action": "drag", "position_m": [point[0], point[1], 0.1]})
    sim.advance(100)
    sim.command({"action": "push", "object": 1, "force_n": [1, 0, 0]})
    before = c.data.qpos.copy()
    sim.command({"action": "stop"})
    assert sim.interaction is None
    assert not c.data.qfrc_applied.any() and not c.data.xfrc_applied.any()
    assert np.array_equal(before, c.data.qpos)
    sim.command({"action": "start"})
    sim.advance(1)
    assert sim.status == "RUNNING"


def test_contact_retreat_uses_private_geometry_and_rejects_obstacle_contact():
    from robostudio.studio_recovery import contact_retreat

    cfg = default_scene()
    cfg["obstacles"] = [{"xy": [0.60, 0.26], "size": [0.025, 0.025, 0.07], "yaw": 0}]
    sim = WorldSession(cfg)
    press_robot_toward_table(sim, manual=True)
    sim.close = False
    sim.advance(200)  # Settle the open fingers before the scheduled recovery.
    c = sim.cell
    state, velocity, time = c.data.qpos.copy(), c.data.qvel.copy(), c.data.time
    result = contact_retreat(c, "dls", 0.035)
    assert result["status"] == "FOUND"
    assert np.array_equal(state, c.data.qpos) and np.array_equal(velocity, c.data.qvel)
    assert c.data.time == time
    # Fixture: a blocking obstacle intersects the retreat region. Never bypass it.
    c.data.mocap_pos[0] = [*c.data.site_xpos[c.tcp][:2], 0.10]
    c.mj.mj_forward(c.model, c.data)
    assert contact_retreat(c, "dls", 0.035)["status"] == "BLOCKED"
    assert np.array_equal(state, c.data.qpos)


def movable_body_at_tcp(cell):
    movable = {int(cell.model.jnt_bodyid[j]) for j in cell.joint_ids}
    selectable = {
        int(cell.model.geom_bodyid[g])
        for g in cell.robot_geoms
        if cell.model.body_dofnum[cell.model.geom_bodyid[g]]
    }
    return min(
        (selectable & movable) or selectable,
        key=lambda b: np.linalg.norm(cell.data.xpos[b] - cell.data.site_xpos[cell.tcp]),
    )


@pytest.mark.parametrize("robot", ["r6", "gantry"])
@pytest.mark.parametrize("phase", ["approach", "close", "lift", "transfer", "lower", "retreat"])
def test_force_preserves_running_stage_and_grip_instead_of_restarting(robot, phase):
    sim = WorldSession(default_scene(robot))
    sim.command({"action": "start"})
    while sim.phase != phase and sim.steps < 15000:
        sim.advance(25)
    assert sim.phase == phase
    c = sim.cell
    point = c.data.site_xpos[c.tcp].tolist()
    context = (sim.phase, sim.current, sim.close, sim.tick_in_segment)
    segment = sim.segment
    queue = copy.deepcopy(sim.stage_queue)
    q, v = c.data.qpos.copy(), c.data.qvel.copy()
    sim.command(
        {"action": "grab", "kind": "body", "index": movable_body_at_tcp(c), "point_m": point}
    )
    assert (sim.phase, sim.current, sim.close, sim.tick_in_segment) == context
    assert sim.segment is segment and sim.stage_queue == queue
    assert np.array_equal(q, c.data.qpos) and np.array_equal(v, c.data.qvel)
    sim.command({"action": "drag", "position_m": [point[0] + 0.008, point[1], point[2] + 0.004]})
    step = sim.steps
    sim.advance(80)
    assert sim.steps == step + 80 and np.linalg.norm(sim.force_n) > 0
    assert sim.status == "RUNNING"
    context = (sim.phase, sim.current, sim.close, sim.tick_in_segment)
    sim.command({"action": "release"})
    assert (sim.phase, sim.current, sim.close, sim.tick_in_segment) == context
    assert not c.data.qfrc_applied.any()
    for _ in range(50):
        sim.advance(500)
        if sim.goal_state == "satisfied":
            break
    assert sim.goal_state == "satisfied", sim.failures
    assert sim.cycles == 1 and not sim.failures
    assert len(sim.placements) == 1 and sim.placements[0]["success"]


def test_force_during_sort_keeps_completed_objects_and_current_target():
    cfg = default_scene("gantry")
    cfg["task"] = "sort"
    sim = WorldSession(cfg)
    sim.command({"action": "start"})
    while not (sim.current == 1 and sim.phase == "transfer") and sim.steps < 30000:
        sim.advance(100)
    assert sim.completed == [0] and sim.current == 1
    c = sim.cell
    point = c.data.site_xpos[c.tcp].tolist()
    sim.command(
        {"action": "grab", "kind": "body", "index": movable_body_at_tcp(c), "point_m": point}
    )
    sim.command({"action": "drag", "position_m": [point[0] + 0.008, point[1], point[2] + 0.004]})
    sim.advance(80)
    sim.command({"action": "release"})
    assert sim.completed == [0] and sim.current == 1 and sim.close
    for _ in range(60):
        sim.advance(500)
        if sim.goal_state == "satisfied":
            break
    assert sim.completed == [0, 1, 2] and sim.cycles == 1
    assert [p["object"] for p in sim.placements] == [0, 1, 2]


def test_repeated_start_and_release_do_not_open_active_grip_or_reset_task():
    sim = WorldSession(default_scene())
    sim.command({"action": "start"})
    while sim.phase != "transfer":
        sim.advance(100)
    context = (sim.current, sim.phase, sim.close, sim.tick_in_segment)
    segment = sim.segment
    sim.command({"action": "start"})
    sim.command({"action": "release"})
    assert (sim.current, sim.phase, sim.close, sim.tick_in_segment) == context
    assert sim.segment is segment and sim.close
