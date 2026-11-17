import json

import pytest

pytest.importorskip("mujoco", reason="optional physics extra is not installed")

from robostudio.arm_experiment import run_arm
from robostudio.core import RoboStudioError
from robostudio.demo import replay


def test_physics_experiment_records_engine_frames_and_replays(tmp_path):
    first = run_arm(seed=42, output_dir=tmp_path)
    second = run_arm(seed=42, output_dir=tmp_path)
    assert first == second
    assert first["result"]["status"] == "SUCCEEDED"
    assert first["result"]["metrics"]["steps"] == 1000
    assert len(first["frames"]) == 101
    assert first["frames"][0]["qpos_si"] != first["frames"][-1]["qpos_si"]
    assert first["frames"][-1]["time_s"] == pytest.approx(2.0)
    artifact = tmp_path / first["result"]["run_id"] / "run.json"
    assert replay(artifact, tmp_path / "replayed").as_dict() == first["result"]
    document = json.loads(artifact.read_text("utf-8"))
    assert document["environment"]["mujoco"]
    assert document["verification"]["max_position_error_m"] < 1e-12
    assert document["verification"]["jacobian"]["configurations"] == 105
    assert document["verification"]["jacobian"]["max_fd_error"] < 1e-8
    observations = artifact.with_name("observations.json")
    data = json.loads(observations.read_text("utf-8"))
    data[1]["tcp"][0] += 0.01
    observations.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(RoboStudioError, match="integrity"):
        replay(artifact, tmp_path / "tampered")


@pytest.mark.parametrize("seed", [True, -1, 2**32, 4.5])
def test_invalid_seed(seed, tmp_path):
    with pytest.raises(RoboStudioError):
        run_arm(seed=seed, output_dir=tmp_path)
    assert not list(tmp_path.iterdir())


def test_failure_preserves_completed_steps(tmp_path, monkeypatch):
    from robostudio.backends.mujoco_backend import MuJoCoBackend
    from robostudio.core import ErrorCode

    original = MuJoCoBackend.step

    def fail_after_three(self, dt):
        if self.observe().time_s >= 0.006:
            raise RoboStudioError(ErrorCode.INVALID_VALUE, "injected failure")
        original(self, dt)

    monkeypatch.setattr(MuJoCoBackend, "step", fail_after_three)
    result = run_arm(output_dir=tmp_path)
    assert result["result"]["status"] == "FAILED"
    assert result["result"]["metrics"]["steps"] == 3
    assert result["error"]["message"] == "injected failure"
    samples = json.loads((tmp_path / result["result"]["run_id"] / "observations.json").read_text())
    assert len(samples) == 4
    assert samples[-1]["time_s"] == pytest.approx(0.006)
