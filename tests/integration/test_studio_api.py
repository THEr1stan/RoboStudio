import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from robostudio.studio_scene import default_scene
from robostudio.workbench import make_server


@pytest.fixture
def studio_server(tmp_path):
    server = make_server(0, tmp_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{server.server_port}"
    server.shutdown()
    server.server_close()
    thread.join(2)


def post(base, action, value, origin=None):
    request = Request(
        base + "/api/studio/" + action,
        data=json.dumps(value).encode(),
        headers={"Content-Type": "application/json", "Origin": origin or base},
    )
    with urlopen(request) as response:
        return json.loads(response.read())


def test_studio_http_session_and_isolation(studio_server):
    a = post(studio_server, "create", {"config": default_scene()})
    b = post(studio_server, "create", {"config": default_scene("gantry")})
    advanced = post(studio_server, "step", {"session": a["session"], "steps": 40})
    assert advanced["frame"]["time_s"] == pytest.approx(0.08)
    assert post(studio_server, "snapshot", {"session": b["session"]})["frame"]["time_s"] == 0
    assert advanced["sensor"]["source"] == "mujoco_ray_rgbd"
    saved = post(studio_server, "save", {"session": a["session"]})
    assert post(studio_server, "replay", {"run_id": saved["run_id"]})["status"] == "IDENTICAL"
    post(studio_server, "close", {"session": a["session"]})
    with pytest.raises(HTTPError):
        post(studio_server, "snapshot", {"session": a["session"]})


@pytest.mark.parametrize(
    "patch",
    [
        {"destination": None},
        {"objects": None},
        {"obstacles": [None]},
        {"seed": True},
        {"noise_mm": "1"},
        {"latency_ms": 0.1},
    ],
)
def test_invalid_scene_does_not_kill_service(studio_server, patch):
    with pytest.raises(HTTPError) as error:
        post(studio_server, "create", {"config": {**default_scene(), **patch}})
    assert error.value.code == 400
    assert post(studio_server, "create", {"config": default_scene()})["status"] == "READY"


def test_action_bounds_and_same_origin(studio_server):
    with pytest.raises(HTTPError) as error:
        post(studio_server, "create", {"config": default_scene()}, "https://example.org")
    assert error.value.code == 403
    sim = post(studio_server, "create", {"config": default_scene()})
    for command in (
        {"action": "target", "position_m": None, "quaternion_wxyz": None},
        {"action": "push", "object": True, "force_n": [1, 0, 0]},
        {"action": "joints", "q": None},
        {"action": "start", "injected": "eval"},
    ):
        with pytest.raises(HTTPError) as error:
            post(studio_server, "command", {"session": sim["session"], "command": command})
        assert error.value.code == 400
    with pytest.raises(HTTPError):
        post(studio_server, "step", {"session": sim["session"], "steps": 100000000})
    assert post(studio_server, "snapshot", {"session": sim["session"]})["frame"]["time_s"] == 0


def test_algorithm_lab_routes_validation_and_session_inspection(studio_server):
    args = {
        "config": default_scene(),
        "family": "ik",
        "method": "dls",
        "target_m": [0.42, -0.14, 0.23],
    }
    result = post(studio_server, "lab", args)
    assert result["result"]["status"] == "CONVERGED"
    for patch in ({"method": "exec"}, {"family": []}, {"target_m": [1, 2]}):
        with pytest.raises(HTTPError) as error:
            post(studio_server, "lab", {**args, **patch})
        assert error.value.code == 400
    sim = post(studio_server, "create", {"config": default_scene()})
    assert post(studio_server, "algorithm_records", {"session": sim["session"]}) == {
        "ik": [],
        "planner": [],
    }
    with pytest.raises(HTTPError) as error:
        post(studio_server, "inspect", {"session": sim["session"], "family": "ik", "index": 0})
    assert error.value.code == 400
    with urlopen(studio_server + "/studio-lab.js") as response:
        assert response.status == 200


def test_scene_import_and_live_cursor_http_contract(studio_server):
    with urlopen(studio_server + "/studio-scenes.js") as response:
        assert response.status == 200
    doc = post(studio_server, "scene_validate", {"document": default_scene()})
    assert doc["schema"] == "robostudio-scene-v1"
    with pytest.raises(HTTPError) as error:
        post(studio_server, "scene_validate", {"document": {"schema": "studio-world-v1"}})
    assert error.value.code == 400
    sim = post(studio_server, "create_world", {"config": doc["config"]})
    initial = post(studio_server, "snapshot", {"session": sim["session"], "view": {}})
    cached = post(studio_server, "snapshot", {"session": sim["session"], "view": initial["view"]})
    assert "sensor" not in cached
    assert cached["frame"] == initial["frame"]
    assert len(json.dumps(cached)) < len(json.dumps(initial)) / 4
