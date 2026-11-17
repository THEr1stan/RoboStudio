import json

import pytest

from robostudio.core import RoboStudioError
from robostudio.demo import replay
from robostudio.tracking import preset_config, run_tracking


def test_real_tracking_repeatability_replay_and_tamper(tmp_path):
    pytest.importorskip("mujoco")
    config = preset_config("reach")
    run = run_tracking(config, tmp_path)
    assert run["result"]["status"] == "SUCCEEDED"
    assert run["result"]["metrics"]["final_position_error_m"] < 0.002
    assert run["result"]["metrics"]["final_orientation_error_rad"] < 0.01
    assert run["result"]["metrics"]["joint_limit_violations"] == 0
    assert len(run["frames"]) > 100
    assert run["frames"][0]["qpos_si"] != run["frames"][-1]["qpos_si"]
    assert run_tracking(config, tmp_path) == run
    path = tmp_path / run["result"]["run_id"] / "run.json"
    assert replay(path, tmp_path).as_dict() == run["result"]
    observations = path.parent / "observations.json"
    data = json.loads(observations.read_text())
    data[-1]["tcp"][0] += 0.01
    observations.write_text(json.dumps(data))
    with pytest.raises(RoboStudioError, match="integrity"):
        replay(path, tmp_path)


def test_failed_ik_does_not_simulate(tmp_path):
    pytest.importorskip("mujoco")
    config = preset_config("reach")
    config["target"]["position_m"] = [3, 0, 0]
    result = run_tracking(config, tmp_path)
    assert result["result"]["status"] == "FAILED"
    assert result["frames"] == []
    assert result["error"]["code"] == "CONVERGENCE_FAILED"


@pytest.mark.parametrize("bad", [True, -1, float("nan"), 10000])
def test_bounded_tracking_configuration(tmp_path, bad):
    config = preset_config("reach")
    config["duration_s"] = bad
    with pytest.raises(RoboStudioError):
        run_tracking(config, tmp_path)


@pytest.mark.parametrize("preset", ["sweep", "lift"])
def test_other_presets_reach_target_with_actual_dynamics(tmp_path, preset):
    pytest.importorskip("mujoco")
    result = run_tracking(preset_config(preset), tmp_path)
    assert result["result"]["status"] == "SUCCEEDED"
    assert result["result"]["metrics"]["max_playback_fk_error_m"] < 1e-10


def test_matched_pd_baseline_has_measured_residual(tmp_path):
    pytest.importorskip("mujoco")
    config = preset_config("reach")
    compensated = run_tracking(config, tmp_path)
    config["controller"] = "pd"
    baseline = run_tracking(config, tmp_path)
    a, b = compensated["result"]["metrics"], baseline["result"]["metrics"]
    assert a["steps"] == b["steps"]
    assert compensated["solution"] == baseline["solution"]
    assert a["tcp_tracking_rms_m"] < b["tcp_tracking_rms_m"]
    assert baseline["result"]["status"] == "FAILED"
    assert baseline["error"]["code"] == "TRACKING_FAILED"
    assert baseline["frames"]
