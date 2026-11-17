"""Observable, measured obstruction feedback without changing motion decisions."""

import json
from pathlib import Path

import numpy as np
import pytest

from robostudio.core import RoboStudioError
from robostudio.serialization import canonical
from robostudio.studio_scene import default_scene
from robostudio.studio_world import WorldSession, replay_world


def obstacle_world():
    cfg = default_scene()
    cfg["obstacles"] = [{"xy": [0.42, -0.05], "size": [0.05, 0.02, 0.08], "yaw": 0}]
    return WorldSession(cfg)


def grab(sim):
    p = sim.cell.data.xpos[sim.cell.object_bodies[0]].tolist()
    sim.command({"action": "grab", "kind": "object", "index": 0, "point_m": p})
    return p


def block(sim):
    p = grab(sim)
    return sim.command({"action": "drag", "position_m": [p[0], 0.04, p[2]]})


def test_obstacle_identity_and_current_gap_come_from_actual_geometry():
    sim = obstacle_world()
    result = block(sim)
    b = result["world"]["blocking"]
    assert b["kind"] == "obstacle" and b["index"] == 0 and b["label"] == "障碍 1"
    c = sim.cell
    assert b["geom_ids"] == [c.object_geoms[0], c.obstacle_geoms[0]]
    assert b["frame"] == "world" and b["source"] == "manual_translation_guard"
    assert b["rejected_clearance_m"] < b["limit_m"]
    assert b["clearance_m"] >= b["limit_m"]
    assert np.isclose(b["clearance_m"], -0.07 - (c.data.xpos[c.object_bodies[0], 1] + 0.02))
    points = np.array(b["points_m"])
    assert points.shape == (2, 3)
    assert np.isclose(np.linalg.norm(points[0] - points[1]), b["clearance_m"])
    assert np.allclose(points[1, 1], -0.07)
    assert b["accepted_position_m"] == c.data.xpos[c.object_bodies[0]].tolist()
    assert "障碍 1" in result["world"]["note"]


def test_snapshot_is_read_only_and_blocking_replays_identically():
    sim = obstacle_world()
    block(sim)
    sim.advance(100)
    c = sim.cell
    before = c.data.qpos.copy(), c.data.qvel.copy(), c.data.qacc_warmstart.copy()
    first = canonical(sim.snapshot())
    for _ in range(3):
        assert canonical(sim.snapshot()) == first
    for saved, actual in zip(
        before, (c.data.qpos, c.data.qvel, c.data.qacc_warmstart), strict=True
    ):
        assert np.array_equal(saved, actual)
    report = sim.report()
    assert report["world"]["blocking"] is not None
    assert canonical(report) == canonical(replay_world(report))


def test_block_feedback_clears_on_valid_move_and_does_not_flood_events():
    sim = obstacle_world()
    block(sim)
    for _ in range(8):
        sim.command({"action": "drag", "position_m": [0.42, 0.04, 0.023]})
    assert len([e for e in sim.events if e["kind"] == "drag_blocked"]) == 1
    result = sim.command({"action": "drag", "position_m": [0.42, -0.16, 0.23]})
    assert result["world"]["blocking"] is None
    assert sim.events[-1]["kind"] == "drag_cleared"
    assert "受阻" not in result["world"]["note"]
    result = sim.command({"action": "drag", "position_m": [0.42, 0.04, 0.23]})
    assert result["world"]["blocking"] is None
    assert np.allclose(sim.cell.data.xpos[sim.cell.object_bodies[0]], [0.42, 0.04, 0.23])


@pytest.mark.parametrize("action", ["release", "stop", "start"])
def test_feedback_clears_when_interaction_ends(action):
    sim = obstacle_world()
    block(sim)
    result = sim.command({"action": action})
    assert result["world"]["blocking"] is None
    assert result["world"]["interaction"] is None


def test_invalid_move_preserves_last_valid_feedback():
    sim = obstacle_world()
    block(sim)
    before = canonical(sim.snapshot())
    with pytest.raises(RoboStudioError):
        sim.command({"action": "drag", "position_m": [float("nan"), 0, 0.2]})
    assert canonical(sim.snapshot()) == before


def test_other_block_is_identified_by_its_actual_color():
    cfg = default_scene()
    cfg["objects"][1]["xy"] = [0.42, -0.06]
    sim = WorldSession(cfg)
    result = block(sim)
    b = result["world"]["blocking"]
    assert b["kind"] == "object" and b["index"] == 1
    assert b["label"] == "蓝色积木"
    assert b["geom_ids"][1] == sim.cell.object_geoms[1]


def test_table_is_identified_when_trying_to_move_below_surface():
    sim = WorldSession(default_scene())
    p = grab(sim)
    result = sim.command({"action": "drag", "position_m": [p[0], p[1], 0.005]})
    b = result["world"]["blocking"]
    assert b["kind"] == "surface" and b["label"] == "桌面"
    assert b["geom_ids"][1] == sim.cell.model.geom("table").id
    assert sim.cell.data.xpos[sim.cell.object_bodies[0], 2] >= 0.02


def test_robot_finger_blocker_uses_actual_geometry_id():
    sim = WorldSession(default_scene())
    c = sim.cell
    p = grab(sim)
    finger = c.model.geom("pad_right").id
    target = c.data.geom_xpos[finger].tolist()
    sim.command({"action": "drag", "position_m": [p[0], p[1], target[2]]})
    result = sim.command({"action": "drag", "position_m": target})
    b = result["world"]["blocking"]
    assert b["kind"] == "body" and b["label"] == "机械臂夹指"
    assert b["geom_ids"][1] == finger


def test_snapshot_feedback_does_not_alias_internal_record():
    sim = obstacle_world()
    result = block(sim)
    result["world"]["blocking"]["geom_ids"][1] = -1
    assert sim.snapshot()["world"]["blocking"]["geom_ids"][1] == sim.cell.obstacle_geoms[0]


@pytest.mark.parametrize(
    "case",
    json.loads(
        (Path(__file__).parent / "fixtures/object_drag_v073.json").read_text(encoding="utf-8")
    )["cases"],
    ids=lambda case: case["kind"],
)
def test_feedback_preserves_frozen_v073_physics(case):
    sim = WorldSession(case["config"])
    for command in case["commands"]:
        sim.command(command)
    for expected in case["states"]:
        if expected["label"] == "held":
            sim.advance(100)
        elif expected["label"] == "released":
            sim.command({"action": "release"})
            sim.advance(100)
        for key in ("qpos", "qvel", "ctrl"):
            assert np.array_equal(getattr(sim.cell.data, key), expected[key])
        assert sim.cell.data.time == expected["time_s"]
