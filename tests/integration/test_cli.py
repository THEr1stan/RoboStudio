import json
import subprocess
import sys
from pathlib import Path

from robostudio.demo import replay, run_demo


def cli(*args, cwd=None):
    return subprocess.run(
        [sys.executable, "-m", "robostudio", *args],
        cwd=cwd,
        capture_output=True,
        text=True,
        check=False,
    )


def test_lists_work_outside_repository(tmp_path):
    result = cli("list", "robots", cwd=tmp_path)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)[0]["robot_id"] == "fake_arm2"
    result = cli("list", "plugins", cwd=tmp_path)
    assert result.returncode == 0, result.stderr
    assert [p["plugin_id"] for p in json.loads(result.stdout)] == [
        "dls_pose_v1",
        "joint_pd_v1",
        "quintic_v1",
    ]


def test_invalid_fixture_has_structured_error():
    result = cli("validate", "robot", "examples/manifests/invalid_robot.json")
    assert result.returncode == 2
    assert json.loads(result.stderr)["error"]["code"] == "INVALID_MANIFEST"
    assert "Traceback" not in result.stderr


def test_demo_and_replay_are_equal(tmp_path):
    first = run_demo(seed=42, output_dir=tmp_path / "first")
    second = run_demo(seed=42, output_dir=tmp_path / "second")
    assert first == second
    a = tmp_path / "first" / first.run_id / "run.json"
    b = tmp_path / "second" / second.run_id / "run.json"
    assert a.read_bytes() == b.read_bytes()
    assert replay(a, output_dir=tmp_path / "replay") == first
    document = json.loads(a.read_text("utf-8"))
    assert document["result"]["status"] == "SUCCEEDED"
    assert document["result"]["metrics"]["steps"] == 20
    assert document["result"]["metrics"]["sim_time_s"] > 0
    assert "wall_time_s" not in document["result"]["metrics"]
    assert document["result"]["source_hash"]
    assert document["replay_command"]


def test_cli_demo_and_invalid_config(tmp_path):
    result = cli("demo", "fake", "--seed", "42", "--output", str(tmp_path))
    assert result.returncode == 0, result.stderr
    run_id = json.loads(result.stdout)["run_id"]
    assert (tmp_path / run_id / "observations.json").exists()
    result = cli("demo", "fake", "--steps", "0", "--output", str(tmp_path))
    assert result.returncode == 2
    assert json.loads(result.stderr)["error"]["code"] == "INVALID_VALUE"


def test_replay_refuses_tampered_inputs(tmp_path):
    import pytest

    from robostudio.core import RoboStudioError

    result = run_demo(seed=42, output_dir=tmp_path)
    path = Path(tmp_path, result.run_id, "run.json")
    document = json.loads(path.read_text("utf-8"))
    document["config"]["seed"] = 99
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(RoboStudioError, match="integrity"):
        replay(path, output_dir=tmp_path / "replay")


def test_jacobian_cli_values_and_invalid_rate():
    import pytest

    pytest.importorskip("mujoco")
    result = cli("jacobian")
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["twist_si"] == pytest.approx([0, 0.84, 0, 0, 0, 1])
    result = cli("jacobian", "--qvel", "5", "0", "0", "0", "0", "0")
    assert result.returncode == 2
    assert json.loads(result.stderr)["error"]["code"] == "INVALID_VALUE"
