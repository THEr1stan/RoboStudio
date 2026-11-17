from dataclasses import replace
from pathlib import Path

import pytest

from robostudio.backends.fake import FakeBackend
from robostudio.core import ErrorCode, RoboStudioError
from robostudio.demo import DemoConfig, run_demo
from robostudio.registry import load_robot, parse_plugin, parse_robot
from robostudio.serialization import parse_json, read_json


@pytest.mark.parametrize("value", ['{"a":1,"a":2}', '{"n":NaN}', '{"n":Infinity}', "[]x"])
def test_nonstandard_and_ambiguous_json_is_rejected(value):
    with pytest.raises(RoboStudioError) as error:
        parse_json(value)
    assert error.value.code == ErrorCode.INVALID_MANIFEST


def test_external_model_hash_is_checked(tmp_path):
    fixtures = Path("examples/manifests")
    for name in ("fake_arm2.json", "fake_model.json"):
        (tmp_path / name).write_bytes((fixtures / name).read_bytes())
    assert load_robot(tmp_path / "fake_arm2.json").robot_id == "fake_arm2"
    (tmp_path / "fake_model.json").write_text("changed", encoding="utf-8")
    with pytest.raises(RoboStudioError) as error:
        load_robot(tmp_path / "fake_arm2.json")
    assert error.value.code == ErrorCode.MODEL_MISMATCH


@pytest.mark.parametrize("seed", [True, -1, 2**32, 0.1, "42"])
def test_seed_is_strict(robot, seed):
    backend = FakeBackend()
    backend.load(robot)
    with pytest.raises(RoboStudioError) as error:
        backend.reset(seed)
    assert error.value.code == ErrorCode.INVALID_VALUE


@pytest.mark.parametrize("dt", [0, -0.1, float("inf"), float("nan"), True, 0.2])
def test_invalid_step_preserves_state(robot, dt):
    backend = FakeBackend()
    backend.load(robot)
    backend.reset(0)
    before = backend.observe()
    with pytest.raises(RoboStudioError):
        backend.step(dt)
    assert backend.observe() == before


def test_fake_rejects_floating_base_before_running(manifest):
    manifest["joints"].append(
        {
            "name": "base",
            "kind": "free",
            "qpos_indices": list(range(2, 9)),
            "qvel_indices": list(range(2, 8)),
        }
    )
    manifest["qpos_size"], manifest["qvel_size"] = 9, 8
    backend = FakeBackend()
    with pytest.raises(RoboStudioError) as error:
        backend.load(parse_robot(manifest))
    assert error.value.code == ErrorCode.UNSUPPORTED_CAPABILITY


def test_fake_does_not_select_by_robot_name(robot):
    backend = FakeBackend()
    backend.load(replace(robot, robot_id="another_name", family="another_family"))
    backend.reset(42)
    assert backend.observe().robot_id == "another_name"


def test_failed_runs_keep_actual_partial_observations(tmp_path, monkeypatch):
    def failing_step(self, dt_s):
        raise RoboStudioError(ErrorCode.INVALID_VALUE, "injected numerical failure")

    monkeypatch.setattr(FakeBackend, "step", failing_step)
    result = run_demo(output_dir=tmp_path)
    assert result.status == "FAILED"
    assert result.error_code == ErrorCode.INVALID_VALUE
    assert result.metrics["steps"] == 0
    assert result.metrics["sim_time_s"] == 0
    document = read_json(tmp_path / result.run_id / "run.json")
    assert document["error"]["message"] == "injected numerical failure"
    assert len(read_json(tmp_path / result.run_id / "observations.json")) == 1


def test_artifacts_are_idempotent_and_never_silently_overwritten(tmp_path):
    first = run_demo(output_dir=tmp_path)
    assert run_demo(output_dir=tmp_path) == first
    assert run_demo(seed=43, output_dir=tmp_path).run_id != first.run_id
    path = tmp_path / first.run_id / "observations.json"
    path.write_text("[]", encoding="utf-8")
    with pytest.raises(RoboStudioError) as error:
        run_demo(output_dir=tmp_path)
    assert error.value.code == ErrorCode.INTEGRITY_ERROR
    assert path.read_text("utf-8") == "[]"


def test_plugin_manifest_cannot_smuggle_code_loading_fields():
    with pytest.raises(RoboStudioError):
        parse_plugin(
            {
                "api_version": "v1alpha1",
                "plugin_id": "unknown",
                "requires_capabilities": [],
                "provides_capability": "cartesian_ik",
                "parameters": {},
                "entry_point": "untrusted:execute",
            }
        )


def test_extreme_dt_has_a_structured_failure():
    with pytest.raises(RoboStudioError) as error:
        DemoConfig(physics_dt_s=5e-324)
    assert error.value.code == ErrorCode.INVALID_VALUE


@pytest.mark.parametrize("field", ["qpos_size", "qvel_size", "ctrl_size"])
def test_unbounded_dimensions_fail_without_allocating_arrays(manifest, field):
    manifest[field] = 10**100
    with pytest.raises(RoboStudioError) as error:
        parse_robot(manifest)
    assert error.value.code == ErrorCode.INVALID_MAPPING
