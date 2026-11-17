"""Portable scene files are validated data, independent of a running session."""

import copy

import pytest

from robostudio.core import RoboStudioError
from robostudio.serialization import canonical
from robostudio.studio_api import StudioAPI
from robostudio.studio_scene import default_scene


def document(robot="r6"):
    return {
        "schema": "robostudio-scene-v1",
        "name": "我的桌面",
        "units": {"length": "m", "angle": "rad", "mass": "kg"},
        "config": default_scene(robot),
    }


@pytest.mark.parametrize("robot", ["r6", "gantry"])
def test_scene_files_validate_and_keep_legacy_config(tmp_path, robot):
    api = StudioAPI(tmp_path)
    result = api.post("scene_validate", {"document": document(robot)})
    assert result == document(robot)
    legacy = api.post("scene_validate", {"document": default_scene(robot)})
    assert legacy["schema"] == "robostudio-scene-v1"
    assert legacy["config"] == result["config"]
    first = api.post("create_world", {"config": result["config"]})
    reopened = api.post("reset", {"session": first["session"], "config": legacy["config"]})
    assert reopened["frame"] == first["frame"]


@pytest.mark.parametrize(
    "bad",
    [
        {"schema": "studio-world-v1"},
        {**document(), "units": {"length": "mm", "angle": "rad", "mass": "kg"}},
        {**document(), "name": " "},
        {**document(), "name": "x" * 81},
        {**document(), "script": "run.py"},
        {**document(), "config": {**default_scene(), "gravity": -1}},
    ],
)
def test_invalid_import_never_mutates_live_scene(tmp_path, bad):
    api = StudioAPI(tmp_path)
    sid = api.post("create_world", {"config": default_scene()})["session"]
    before = copy.deepcopy(api.sessions[sid].snapshot())
    with pytest.raises(RoboStudioError):
        api.post("scene_validate", {"document": bad})
    assert canonical(api.sessions[sid].snapshot()) == canonical(before)
