"""Behavioral contracts for the interactive physics studio, written before implementation."""

import copy

import pytest

from robostudio.core import RoboStudioError


def test_legacy_adapter_is_identical_without_scene_changes():
    from robostudio.workcell import Cell, default_config, scene_xml

    cfg = default_config()
    old, adapted = Cell(cfg), Cell(cfg, xml_override=scene_xml(cfg))
    assert old.model_hash == adapted.model_hash
    assert old.frame("ready") == adapted.frame("ready")


def test_scene_validation_and_mapping():
    from robostudio.studio_scene import StudioCell, default_scene, validate_scene

    for robot in ("r6", "gantry"):
        cfg = default_scene(robot)
        cell = StudioCell(cfg)
        assert cell.model.nq > cell.model.nv
        assert cell.model.neq == 0
        assert len(cell.frame("ready")["geoms"]) == cell.model.ngeom
        for patch in ({"gravity": -1}, {"noise_mm": float("nan")}, {"planner": "exec"}):
            with pytest.raises(RoboStudioError):
                validate_scene({**cfg, **patch})
    cfg = default_scene()
    cfg["objects"][0]["size"][0] = -1
    with pytest.raises(RoboStudioError):
        validate_scene(cfg)


def test_depth_camera_projects_geometry_and_estimates_from_pixels():
    import numpy as np

    from robostudio.studio_scene import StudioCell, default_scene
    from robostudio.studio_sensor import capture, detect

    cell = StudioCell(default_scene())
    image = capture(cell, 0)
    found = detect(image)
    item = next(row for row in found if row["color"] == "orange")
    actual = cell.data.xpos[cell.model.body("block0").id]
    assert np.linalg.norm(np.array(item["position_m"][:2]) - actual[:2]) < 0.009
    assert image["width"] * image["height"] == len(image["depth_m"])
    assert "geom_ids" not in image  # perception cannot use instance identity shortcuts
    erased = copy.deepcopy(image)
    erased["rgb"] = [100] * len(image["rgb"])
    assert detect(erased) == []


def test_planner_checks_obstacle_and_does_not_change_live_state():
    from robostudio.studio_planning import plan_path

    def clear(q):
        return not (0.35 < q[0] < 0.65 and abs(q[1]) < 0.25)

    args = ([0.0, 0.0], [1.0, 0.0], [(-0.1, 1.1), (-1.0, 1.0)], clear)
    straight = plan_path(*args, method="direct", seed=42)
    planned = plan_path(*args, method="rrt_connect", seed=42)
    assert straight["status"] == "BLOCKED"
    assert planned["status"] == "FOUND"
    assert len(planned["path"]) > 2
    assert planned == plan_path(*args, method="rrt_connect", seed=42)


def test_live_pause_step_and_force_are_actual_physics():
    from robostudio.studio_runtime import Session
    from robostudio.studio_scene import default_scene

    sim = Session(default_scene())
    before = sim.snapshot()["frame"]["time_s"]
    sim.advance(10)
    assert sim.snapshot()["frame"]["time_s"] == pytest.approx(before + 0.02)
    velocity = sim.cell.data.qvel.copy()
    sim.command({"action": "push", "object": 0, "force_n": [1, 0, 0]})
    sim.advance(10)
    assert not (sim.cell.data.qvel == velocity).all()
    assert sim.actions[-1]["command"]["action"] == "push"


def test_live_sorting_from_camera_and_repeatable_report():
    from robostudio.studio_runtime import run_experiment
    from robostudio.studio_scene import default_scene

    cfg = default_scene()
    a, b = run_experiment(cfg), run_experiment(cfg)
    assert a["status"] == "SUCCEEDED"
    assert a["metrics"]["placed"] == 1
    assert a["metrics"]["bilateral_steps"] > 20
    assert a["metrics"]["lift_m"] > 0.08
    assert a["sensors"][0]["source"] == "mujoco_ray_rgbd"
    assert a == b


@pytest.mark.parametrize("robot", ["r6", "gantry"])
@pytest.mark.parametrize("task", ["sort", "stack"])
def test_three_object_tasks(robot, task):
    from robostudio.studio_runtime import run_experiment
    from robostudio.studio_scene import default_scene

    cfg = default_scene(robot)
    cfg["task"] = task
    result = run_experiment(cfg)
    assert result["status"] == "SUCCEEDED"
    assert result["metrics"]["placed"] == 3
    assert result["metrics"]["collision_steps"] == 0
    if task == "stack":
        heights = [obj["position"][2] for obj in result["frames"][-1]["objects"]]
        assert heights == pytest.approx([0.02, 0.06, 0.10], abs=0.008)


def test_same_obstacle_scene_baseline_blocks_and_rrt_succeeds():
    from robostudio.studio_runtime import run_experiment
    from robostudio.studio_scene import default_scene

    cfg = default_scene()
    cfg["obstacles"] = [{"xy": [0.43, 0.015], "size": [0.035, 0.045, 0.115], "yaw": 0}]
    planned = run_experiment(cfg)
    cfg["planner"] = "direct"
    direct = run_experiment(cfg)
    assert planned["status"] == "SUCCEEDED"
    assert direct["status"] == "FAILED"
    assert direct["error"] == "PATH_OR_IK_BLOCKED"
    assert any(p["iterations"] > 0 for p in planned["plans"])
    assert planned["metrics"]["collision_steps"] == 0


def test_grasp_failure_retries_with_actual_contact_and_slip():
    from robostudio.studio_runtime import run_experiment
    from robostudio.studio_scene import default_scene

    cfg = default_scene()
    cfg["objects"][0].update(mass=0.28, friction=0.05)
    result = run_experiment(cfg)
    assert result["status"] == "FAILED"
    assert result["error"] == "GRASP_LOST"
    assert result["metrics"]["retries"] == 1
    assert result["metrics"]["bilateral_steps"] > 0
    assert result["metrics"]["lift_m"] < 0.02
    assert len(result["grasps"]) == 2


def test_camera_noise_latency_and_full_occlusion():
    from robostudio.studio_runtime import Session
    from robostudio.studio_scene import StudioCell, default_scene
    from robostudio.studio_sensor import capture, detect

    cfg = default_scene()
    cfg.update(noise_mm=4, dropout=0.2, latency_ms=300)
    c = StudioCell(cfg)
    a, b = capture(c, 10), capture(c, 10)
    assert a == b
    assert a != capture(c, 11)
    assert a["depth_m"].count(0) > 100
    sim = Session(cfg)
    sim.advance(250)
    assert sim.cell.data.time - sim.image["time_s"] >= 0.299
    # A physical occluder between the camera and orange object blocks its color pixels.
    plain = default_scene()
    plain["objects"] = plain["objects"][:1]
    plain["obstacles"] = [{"xy": [0.42, -0.16], "size": [0.06, 0.06, 0.1], "yaw": 0}]
    hidden = capture(StudioCell(plain), 0)
    assert not any(item["color"] == "orange" for item in detect(hidden))


def test_collision_probe_and_oriented_ik_leave_physics_unchanged():
    import numpy as np

    from robostudio.cell_ik import solve
    from robostudio.studio_planning import CollisionProbe
    from robostudio.studio_runtime import down_quaternion
    from robostudio.studio_scene import StudioCell, default_scene

    cell = StudioCell(default_scene())
    before = cell.data.qpos.copy()
    solved = solve(cell, [0.42, -0.1, 0.2], quaternion_wxyz=down_quaternion(0.7))
    assert solved["status"] == "CONVERGED"
    assert solved["orientation_error_rad"] < 0.002
    CollisionProbe(cell)(solved["q"])
    assert np.array_equal(cell.data.qpos, before)
    assert cell.self_pairs


def test_manual_motion_is_dynamics_and_teaching_preserves_gripper():
    import numpy as np

    from robostudio.studio_runtime import Session, down_quaternion
    from robostudio.studio_scene import default_scene

    sim = Session(default_scene())
    before = sim.cell.data.qpos.copy()
    sim.command(
        {
            "action": "target",
            "position_m": [0.42, -0.1, 0.23],
            "quaternion_wxyz": down_quaternion(0),
        }
    )
    assert np.array_equal(sim.cell.data.qpos, before)
    while sim.segment is not None:
        sim.advance(100)
    assert np.linalg.norm(sim.cell.data.site_xpos[sim.cell.tcp] - [0.42, -0.1, 0.23]) < 0.003
    sim.command({"action": "gripper", "close": True})
    sim.command({"action": "teach_add"})
    sim.command({"action": "gripper", "close": False})
    sim.command({"action": "teach_play"})
    while sim.segment is not None:
        sim.advance(100)
    assert sim.close is True


def test_record_integrity_and_manual_action_replay(tmp_path):
    from robostudio.studio_api import StudioAPI, load_report
    from robostudio.studio_scene import default_scene

    api = StudioAPI(tmp_path)
    created = api.post("create", {"config": default_scene()})
    identifier = created["session"]
    api.post("step", {"session": identifier, "steps": 50})
    api.post(
        "command",
        {"session": identifier, "command": {"action": "push", "object": 0, "force_n": [0.3, 0, 0]}},
    )
    api.post("step", {"session": identifier, "steps": 100})
    report = api.post("save", {"session": identifier})
    assert api.post("replay", {"run_id": report["run_id"]})["status"] == "IDENTICAL"
    path = tmp_path / report["run_id"] / "run.json"
    path.write_text(path.read_text("utf-8").replace('"gravity": 9.81', '"gravity": 9.82'), "utf-8")
    with pytest.raises(RoboStudioError, match="hash mismatch"):
        load_report(tmp_path, report["run_id"])
