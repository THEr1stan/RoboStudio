"""Live transport may omit cached panels; physics, full snapshots and reports stay exact."""

import copy

import pytest

from robostudio.core import RoboStudioError
from robostudio.serialization import canonical
from robostudio.studio_api import StudioAPI
from robostudio.studio_scene import default_scene


@pytest.mark.parametrize("robot", ["r6", "gantry"])
def test_live_view_roundtrip_and_physics_match_full_api(tmp_path, robot):
    api = StudioAPI(tmp_path)
    full = api.post("create_world", {"config": default_scene(robot)})["session"]
    live = api.post("create_world", {"config": default_scene(robot)})["session"]
    view, merged = {}, {}
    actions = [("step", {"steps": 60}), ("command", {"command": {"action": "start"}})]
    actions += [("step", {"steps": 40})] * 8
    for action, body in actions:
        expected = api.post(action, {"session": full, **body})
        received = api.post(action, {"session": live, **body, "view": view})
        view = received.pop("view")
        merged.update(received)
        for key in expected:
            assert canonical(merged[key]) == canonical(expected[key]), key
    assert canonical(api.sessions[full].report()) == canonical(api.sessions[live].report())
    cached = api.post("snapshot", {"session": live, "view": view})
    assert not set(cached) & {"sensor", "solution", "plan", "grasp", "trace", "events"}
    assert cached["frame"] == merged["frame"]
    complete = api.post("snapshot", {"session": live})
    assert "sensor" in complete and "solution" in complete and "view" not in complete
    reset = api.post("reset", {"session": live, "config": default_scene(robot)})
    refreshed = api.post("snapshot", {"session": live, "view": view})
    assert refreshed["sensor"] == reset["sensor"]
    assert refreshed["solution"] is None


@pytest.mark.parametrize("view", [None, [], {"x": 1}, {"plan": True}, {"plan": -1}])
def test_invalid_live_cursor_is_rejected_before_advancing(tmp_path, view):
    api = StudioAPI(tmp_path)
    sid = api.post("create_world", {"config": default_scene()})["session"]
    before = canonical(api.sessions[sid].snapshot())
    with pytest.raises(RoboStudioError):
        api.post("step", {"session": sid, "steps": 40, "view": view})
    assert canonical(api.sessions[sid].snapshot()) == before


def test_live_cache_recovers_when_client_loses_response(tmp_path):
    api = StudioAPI(tmp_path)
    sid = api.post("create_world", {"config": default_scene()})["session"]
    first = api.post("snapshot", {"session": sid, "view": {}})
    old = copy.deepcopy(first["view"])
    api.post("step", {"session": sid, "steps": 100, "view": old})
    retry = api.post("snapshot", {"session": sid, "view": old})
    assert "sensor" in retry and "trace" in retry
    assert retry["frame"]["time_s"] == pytest.approx(0.2)
