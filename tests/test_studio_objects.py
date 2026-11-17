"""Live selection, direct manipulation, release physics and autonomous regrasp."""

import numpy as np
import pytest

from robostudio.core import RoboStudioError
from robostudio.serialization import canonical
from robostudio.studio_scene import default_scene
from robostudio.studio_world import WorldSession, replay_world


def finish(sim):
    for _ in range(100):
        sim.advance(500)
        if sim.goal_state == "satisfied":
            return
    pytest.fail(str(sim.failures[-3:]))


@pytest.mark.parametrize("robot", ["r6", "gantry"])
@pytest.mark.parametrize("index", [1, 2])
def test_selected_color_is_physically_grasped_and_placed(robot, index):
    sim = WorldSession(default_scene(robot))
    sim.command({"action": "select_object", "index": index})
    assert sim.snapshot()["world"]["selected_object"] == index
    sim.command({"action": "start"})
    finish(sim)
    assert sim.completed == [index]
    assert sim.placements[-1]["object"] == index
    assert sim.placements[-1]["success"]
    assert sim.bilateral > 0
    assert all(g["object"] == index for g in sim.grasps)
    assert canonical(sim.report()) == canonical(replay_world(sim.report()))


def test_selection_switch_preserves_scene_and_same_selection_preserves_task():
    sim = WorldSession(default_scene())
    sim.command({"action": "start"})
    sim.advance(500)
    before = sim.cell.data.qpos.copy(), sim.steps
    sim.command({"action": "select_object", "index": 2})
    assert np.array_equal(before[0], sim.cell.data.qpos)
    assert sim.steps == before[1] and sim.autonomy
    sim.advance(500)
    phase, segment, cycles = sim.phase, sim.segment, sim.cycles
    sim.command({"action": "select_object", "index": 2})
    assert (sim.phase, sim.cycles) == (phase, cycles) and sim.segment is segment


@pytest.mark.parametrize("index", [-1, 3, True, 1.5, "1"])
def test_invalid_selection_is_atomic(index):
    sim = WorldSession(default_scene())
    before = canonical(sim.snapshot())
    with pytest.raises(RoboStudioError):
        sim.command({"action": "select_object", "index": index})
    assert canonical(sim.snapshot()) == before and not sim.actions


@pytest.mark.parametrize("shape", ["box", "cylinder"])
def test_object_hold_preserves_offset_orientation_and_releases_to_gravity(shape):
    cfg = default_scene()
    cfg["objects"][1].update(shape=shape, mass=0.28, yaw=0.4)
    sim = WorldSession(cfg)
    sim.advance(50)
    c, index = sim.cell, 1
    body = c.object_bodies[index]
    centre = c.data.xpos[body].copy()
    point = centre + [0.005, -0.003, 0.02]
    rotation = c.data.xmat[body].copy()
    robot = c.data.qpos[c.q_indices].copy()
    sim.command({"action": "grab", "kind": "object", "index": index, "point_m": point.tolist()})
    target = point + [0.04, 0.0, 0.16]
    result = sim.command({"action": "drag", "position_m": target.tolist()})
    expected = centre + target - point
    assert np.allclose(c.data.xpos[body], expected, atol=1e-10)
    assert np.array_equal(robot, c.data.qpos[c.q_indices])
    assert result["world"]["selected_object"] == index
    assert result["world"]["interaction"]["mode"] == "direct_position"
    sim.advance(150)
    assert np.allclose(c.data.xpos[body], expected, atol=1e-10)
    assert np.allclose(c.data.xmat[body], rotation, atol=1e-10)
    assert sim.force_n == [0, 0, 0]
    sim.command({"action": "release"})
    sim.advance(100)
    assert c.data.xpos[body, 2] < expected[2] - 0.05
    assert canonical(sim.report()) == canonical(replay_world(sim.report()))


def test_direct_object_drag_stops_at_obstacle_and_can_lift_over_it():
    cfg = default_scene()
    cfg["obstacles"] = [{"xy": [0.42, -0.05], "size": [0.05, 0.02, 0.08], "yaw": 0}]
    sim = WorldSession(cfg)
    c = sim.cell
    point = c.data.xpos[c.object_bodies[0]].tolist()
    sim.command({"action": "grab", "kind": "object", "index": 0, "point_m": point})
    sim.command({"action": "drag", "position_m": [0.42, 0.04, point[2]]})
    assert c.data.xpos[c.object_bodies[0], 1] < -0.09
    assert "受阻" in sim.interaction_note
    sim.command({"action": "drag", "position_m": [0.42, -0.16, 0.23]})
    sim.command({"action": "drag", "position_m": [0.42, 0.04, 0.23]})
    assert np.allclose(c.data.xpos[c.object_bodies[0]], [0.42, 0.04, 0.23])


def test_placed_blue_object_can_be_relocated_and_grasped_again():
    sim = WorldSession(default_scene("gantry"))
    sim.command({"action": "select_object", "index": 1})
    sim.command({"action": "start"})
    finish(sim)
    point = sim.cell.data.xpos[sim.cell.object_bodies[1]].tolist()
    sim.command({"action": "grab", "kind": "object", "index": 1, "point_m": point})
    sim.command({"action": "drag", "position_m": [point[0], point[1], 0.18]})
    sim.command({"action": "drag", "position_m": [0.38, -0.13, 0.18]})
    sim.advance(100)
    assert sim.goal_state == "interacting"
    sim.command({"action": "release"})
    finish(sim)
    assert [p["object"] for p in sim.placements] == [1, 1]
    assert all(p["success"] for p in sim.placements)


def test_selecting_next_color_during_carry_finishes_held_object_first():
    sim = WorldSession(default_scene("gantry"))
    sim.command({"action": "start"})
    while sim.phase != "transfer" and sim.steps < 15000:
        sim.advance(20)
    assert sim.close and sim.current == 0
    segment = sim.segment
    sim.command({"action": "select_object", "index": 1})
    assert sim.close and sim.current == 0 and sim.segment is segment
    finish(sim)
    assert [p["object"] for p in sim.placements] == [0, 1]
    assert all(p["success"] for p in sim.placements)
    assert np.linalg.norm(sim.cell.data.xpos[sim.cell.object_bodies[0]][:2] - [0.42, 0.2]) < 0.01


def test_sort_selection_preserves_finished_objects_and_fixed_receiving_slots():
    cfg = default_scene("gantry")
    cfg["task"] = "sort"
    sim = WorldSession(cfg)
    sim.command({"action": "select_object", "index": 2})
    sim.command({"action": "start"})
    finish(sim)
    assert [p["object"] for p in sim.placements] == [2, 0, 1]
    point = sim.cell.data.xpos[sim.cell.object_bodies[0]].tolist()
    sim.command({"action": "grab", "kind": "object", "index": 0, "point_m": point})
    sim.command({"action": "drag", "position_m": [point[0], point[1], 0.18]})
    sim.command({"action": "drag", "position_m": [0.38, -0.13, 0.18]})
    sim.command({"action": "release"})
    finish(sim)
    assert [p["object"] for p in sim.placements] == [2, 0, 1, 0]
    assert set(sim.completed) == {0, 1, 2}


@pytest.mark.parametrize("action", ["release", "stop", "start"])
def test_any_end_of_hold_restores_free_body_physics(action):
    sim = WorldSession(default_scene())
    c = sim.cell
    point = c.data.xpos[c.object_bodies[0]].tolist()
    sim.command({"action": "grab", "kind": "object", "index": 0, "point_m": point})
    sim.command({"action": "drag", "position_m": [point[0], point[1], 0.18]})
    sim.command({"action": action})
    sim.advance(100)
    assert sim.object_handle is None and sim.interaction is None
    assert c.data.xpos[c.object_bodies[0], 2] < 0.13
