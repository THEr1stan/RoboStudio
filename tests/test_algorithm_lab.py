"""Observable contracts, added before the algorithm-library implementation."""

import json
from pathlib import Path

import numpy as np
import pytest

from robostudio.cell_ik import solve
from robostudio.core import RoboStudioError
from robostudio.studio_planning import plan_path
from robostudio.studio_scene import StudioCell, default_scene, validate_scene


def clear(q):
    return not (0.35 < q[0] < 0.65 and abs(q[1]) < 0.25)


ARGS = ([0.0, 0.0], [1.0, 0.0], [(-0.1, 1.1), (-1.0, 1.0)], clear)


def test_frozen_v050_algorithm_outputs():
    frozen = json.loads(Path("tests/fixtures/studio_algorithms_v050.json").read_text())
    result = plan_path(*ARGS)
    for key, expected in frozen["planner"].items():
        assert result[key] == expected
    cell = StudioCell(default_scene())
    for method, expected in frozen["ik"].items():
        actual = solve(cell, [0.42, -0.14, 0.23], method)
        for key, value in expected.items():
            assert actual[key] == pytest.approx(value, abs=1e-12)


@pytest.mark.parametrize("method", ["rrt", "rrt_connect", "prm"])
def test_search_records_are_real_seeded_and_collision_free(method):
    a = plan_path(*ARGS, method=method)
    assert a == plan_path(*ARGS, method=method)
    assert a["status"] == "FOUND"
    assert a["search_trace"] and a["raw_path"]
    assert a["collision_checks"] > len(a["search_trace"])
    for x, y in zip(a["path"], a["path"][1:], strict=False):
        for t in np.linspace(0, 1, 200):
            assert clear(np.array(x) + (np.array(y) - x) * t)
    assert any(not row["accepted"] for row in a["search_trace"])


def test_adaptive_dls_records_actual_damping_and_preserves_physics():
    cell = StudioCell(default_scene())
    before = cell.frame("idle")
    result = solve(cell, [0.42, -0.14, 0.23], "adaptive_dls", initial=[0.0] * 6)
    assert result["status"] == "CONVERGED"
    assert len({row["damping"] for row in result["trace"]}) > 1
    assert result["position_error_m"] < 0.0005
    assert cell.frame("idle") == before


@pytest.mark.parametrize("method", ["quintic", "cubic", "trapezoidal"])
def test_time_profile_derivatives_and_endpoints(method):
    from robostudio.studio_trajectory import time_profile

    assert time_profile(method, 0)[:2] == (0, 0)
    assert time_profile(method, 1)[:2] == (1, 0)
    for u in (0.12, 0.37, 0.62, 0.89):
        h = 1e-5
        s, v, a = time_profile(method, u)
        left, right = time_profile(method, u - h), time_profile(method, u + h)
        assert 0 <= s <= 1
        assert (right[0] - left[0]) / (2 * h) == pytest.approx(v, abs=1e-7)
        assert (right[1] - left[1]) / (2 * h) == pytest.approx(a, abs=1e-7)


def test_config_accepts_old_scene_and_rejects_unknown_profile():
    cfg = default_scene()
    cfg.pop("trajectory", None)
    assert validate_scene(cfg)["trajectory"] == "quintic"
    with pytest.raises(RoboStudioError):
        validate_scene({**cfg, "trajectory": "execute_code"})


def test_lab_provenance_frames_and_current_session_isolation(tmp_path):
    from robostudio.studio_api import StudioAPI

    api = StudioAPI(tmp_path)
    cfg = default_scene()
    created = api.post("create", {"config": cfg})
    sim = api.sessions[created["session"]]
    before = sim.snapshot()
    report = api.post(
        "lab", {"config": cfg, "family": "ik", "method": "dls", "target_m": [0.42, -0.14, 0.23]}
    )
    assert sim.snapshot() == before
    assert report["source_hash"] and report["model_hash"]
    assert report["frame_id"] == "world"
    assert len(report["frames"]) == len(report["result"]["trace"])
    for frame, row in zip(report["frames"], report["result"]["trace"], strict=True):
        assert frame["q"] == row["q"]
        assert frame["tcp"] == pytest.approx(row["tcp"], abs=1e-12)
    with pytest.raises(RoboStudioError):
        api.post("lab", {"config": cfg, "family": "ik", "method": "dls", "target_m": None})


@pytest.mark.parametrize("robot", ["r6", "gantry"])
@pytest.mark.parametrize(
    "family,method",
    [
        ("ik", "adaptive_dls"),
        ("planner", "rrt"),
        ("planner", "prm"),
        ("trajectory", "cubic"),
        ("trajectory", "trapezoidal"),
        ("controller", "computed"),
        ("controller", "pd_bias"),
    ],
)
def test_new_algorithms_have_reproducible_engine_data(robot, family, method):
    from robostudio.studio_lab import run_lab

    cfg = default_scene(robot)
    target = [0.42, -0.14, 0.23]
    if family == "planner":
        cfg["obstacles"] = [{"xy": [0.43, 0.015], "size": [0.035, 0.045, 0.115], "yaw": 0}]
        target[1] = 0.20
    a = run_lab(cfg, family, method, target)
    assert a == run_lab(cfg, family, method, target)
    assert a["result"]["status"] in ("CONVERGED", "FOUND", "COMPLETED")
    if family == "controller":
        assert a["mode"] == "physics"
        assert a["result"]["physics_steps"] > 1000
        assert 0 < a["result"]["tracking_rms_si"] < 0.1
        assert a["frames"][-1]["time_s"] > 2.5
        assert any(abs(x) > 1e-3 for row in a["result"]["trace"] for x in row["effort"])
    if family == "trajectory":
        assert "tracking_rms_si" not in a["result"]
        assert a["result"]["physics_steps"] == 0


def test_task_inspection_uses_recorded_scene_without_live_writes(tmp_path):
    from robostudio.studio_api import StudioAPI

    api = StudioAPI(tmp_path)
    sim_id = api.post("create", {"config": default_scene()})["session"]
    api.post(
        "command",
        {
            "session": sim_id,
            "command": {
                "action": "target",
                "position_m": [0.42, -0.14, 0.23],
                "quaternion_wxyz": [2**-0.5, 0, 2**-0.5, 0],
            },
        },
    )
    before = api.sessions[sim_id].snapshot()
    records = api.post("algorithm_records", {"session": sim_id})
    assert records["planner"] and records["ik"]
    for family in ("planner", "ik"):
        viewed = api.post("inspect", {"session": sim_id, "family": family, "index": 0})
        assert viewed["origin"] == "actual_task_record"
        assert viewed["base_frame"]["tcp"] == pytest.approx(before["frame"]["tcp"])
    assert before == api.sessions[sim_id].snapshot()


def test_collision_budget_and_invalid_algorithms():
    result = plan_path(*ARGS, method="rrt", collision_budget=3)
    assert result["collision_checks"] == 3
    assert result["status"] == "BLOCKED"
    assert result["reason"] == "COLLISION_BUDGET"
    for kwargs in ({"budget": True}, {"collision_budget": 0}, {"method": "RRT*"}):
        with pytest.raises(RoboStudioError):
            plan_path(*ARGS, **kwargs)


@pytest.mark.parametrize("profile", ["cubic", "trapezoidal"])
def test_selected_time_profile_is_used_in_live_physics(profile):
    from robostudio.studio_runtime import Session
    from robostudio.studio_trajectory import time_profile

    sim = Session({**default_scene(), "trajectory": profile})
    start = sim.reference.copy()
    goal = start + np.array([0.05, 0, 0, 0, 0, 0])
    sim._move([start.tolist(), goal.tolist()], "teleop", minimum=1.0)
    sim.advance(100)
    expected = start + (goal - start) * time_profile(profile, 0.2)[0]
    assert sim.reference == pytest.approx(expected)
    assert sim.cell.data.time == pytest.approx(0.2)
    assert not np.array_equal(sim.cell.data.qpos[sim.cell.q_indices], start)
