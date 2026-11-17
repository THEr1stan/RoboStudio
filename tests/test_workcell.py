"""Observable contracts for the desktop manipulation application."""

import copy

import pytest

from robostudio.core import RoboStudioError


def test_scene_mapping_and_free_objects():
    from robostudio.workcell import Cell, default_config

    for robot in ("r6", "gantry"):
        cell = Cell(default_config(robot))
        assert cell.model.nq > cell.model.nv
        assert len(cell.q_indices) == (6 if robot == "r6" else 3)
        assert len(set(cell.ctrl_indices)) == len(cell.q_indices)
        assert cell.model.neq == 0  # Grasp is contact, never a weld/attachment.
        assert len(cell.frame("ready")["objects"]) == 3


def test_config_rejects_nonfinite_and_unknown():
    from robostudio.workcell import default_config, validate_config

    for patch in (
        {"mass_kg": float("nan")},
        {"seed": True},
        {"robot": "unknown"},
        {"friction": -1},
        {"injected": "anything"},
        {"solver": "eval"},
    ):
        config = default_config()
        config.update(patch)
        with pytest.raises(RoboStudioError):
            validate_config(config)


def test_traced_ik_reaches_target_and_preserves_live_state():
    from robostudio.cell_ik import solve
    from robostudio.workcell import Cell, default_config

    cell = Cell(default_config())
    before = cell.data.qpos.copy()
    result = solve(cell, [0.42, -0.12, 0.18], "dls")
    assert result["status"] == "CONVERGED"
    assert result["trace"][-1]["position_error_m"] < 0.0005
    assert len(result["trace"][0]["jacobian"]) == 6
    assert (cell.data.qpos == before).all()
    assert all(lo <= q <= hi for q, (lo, hi) in zip(result["q"], cell.limits, strict=True))


def test_physical_pick_place_is_repeatable(tmp_path):
    from robostudio.workcell import default_config, run_cell

    config = default_config()
    result = run_cell(config, tmp_path)
    assert result["status"] == "SUCCEEDED"
    assert result["metrics"]["lift_m"] > 0.08
    assert result["metrics"]["bilateral_contact_steps"] > 20
    assert result["metrics"]["landing_error_m"] < 0.045
    assert result["frames"][-1]["objects"][0]["position"][2] < 0.04
    assert result == run_cell(copy.deepcopy(config), tmp_path)


def test_gantry_uses_same_task_and_solver(tmp_path):
    from robostudio.workcell import default_config, run_cell

    result = run_cell(default_config("gantry"), tmp_path)
    assert result["status"] == "SUCCEEDED"
    assert result["metrics"]["bilateral_contact_steps"] > 20


@pytest.mark.parametrize("robot", ["r6", "gantry"])
def test_probe_jacobian_matches_numerical_position(robot):
    import numpy as np

    from robostudio.workcell import Cell, default_config

    probe = Cell(default_config(robot)).probe()
    q = np.array(probe.configuration)
    _, jacobian = probe.evaluate(tuple(float(v) for v in q))
    for i in range(len(q)):
        offset = np.zeros(len(q))
        offset[i] = 1e-6
        plus, _ = probe.evaluate(tuple(float(v) for v in q + offset))
        minus, _ = probe.evaluate(tuple(float(v) for v in q - offset))
        measured = (np.array(plus.position_m) - np.array(minus.position_m)) / 2e-6
        assert np.max(np.abs(measured - np.array(jacobian)[:3, i])) < 1e-8


def test_solver_budget_and_failure():
    from robostudio.cell_ik import SOLVERS, solve
    from robostudio.workcell import Cell, default_config

    cell = Cell(default_config())
    for algorithm in SOLVERS:
        solution = solve(cell, [1.8, 0, 0.2], algorithm)
        assert solution["status"] == "NOT_CONVERGED"
        assert solution["iterations"] == 140
        assert len(solution["trace"]) == 141
    with pytest.raises(RoboStudioError):
        solve(cell, [0.4, 0, 0.2], initial=[True] * 6)


def test_low_friction_heavy_object_fails_honestly():
    from robostudio.workcell import default_config, run_cell

    config = default_config()
    config.update(friction=0.05, mass_kg=0.28)
    result = run_cell(config)
    assert result["status"] == "FAILED"
    assert result["metrics"]["lift_m"] < 0.08
    assert result["metrics"]["landing_error_m"] > 0.1
    assert result["frames"]  # Failure is still observable.


def test_record_integrity_and_strict_replay(tmp_path):
    import json

    from robostudio.workcell import default_config, handle_request, load_cell_record, run_cell

    result = run_cell(default_config(), tmp_path)
    identifier = result["run_id"]
    assert handle_request("replay", {"run_id": identifier}, tmp_path)["status"] == "IDENTICAL"
    path = tmp_path / identifier / "run.json"
    result["frames"][2]["objects"][0]["position"][0] += 0.01
    path.write_text(json.dumps(result))
    with pytest.raises(RoboStudioError, match="hash mismatch"):
        load_cell_record(tmp_path, identifier)


def test_paper_gate_and_untrusted_excerpt(tmp_path):
    import base64
    import io
    import zipfile

    from robostudio.paper_spec import draft_spec, generate_candidate

    source = {
        "title": "Untrusted paper",
        "source_url": "https://example.org/paper",
        "page": 10,
        "excerpt": "import os; arbitrary instructions",
        "method": "dls",
    }
    spec = draft_spec(source)
    assert not spec["executable"] and not spec["code_generated"]
    with pytest.raises(RoboStudioError):
        generate_candidate({"source": source, "approved": False}, tmp_path)
    candidate = generate_candidate({"source": source, "approved": True}, tmp_path)
    assert candidate["status"] == "GENERATED_NOT_EXECUTED"
    with zipfile.ZipFile(io.BytesIO(base64.b64decode(candidate["archive_base64"]))) as archive:
        code = archive.read("candidate.py").decode()
        assert "arbitrary instructions" not in code
        assert "import os" not in code
        assert "np.linalg.solve" in code
        assert archive.read("AlgorithmSpec.json")
