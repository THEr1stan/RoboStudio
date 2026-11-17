import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from robostudio.workbench import make_server


@pytest.fixture
def server(tmp_path):
    instance = make_server(port=0, output_dir=tmp_path)
    thread = threading.Thread(target=instance.serve_forever, daemon=True)
    thread.start()
    yield instance
    instance.shutdown()
    instance.server_close()
    thread.join(timeout=2)


def test_workbench_serves_only_packaged_assets(server):
    base = f"http://127.0.0.1:{server.server_port}"
    with urlopen(base + "/") as response:
        assert response.status == 200
        assert "RoboStudio" in response.read().decode()
        assert response.headers["Content-Security-Policy"]
    for path in ("/kinematics.js", "/app.js", "/styles.css", "/renderer.js", "/api/status"):
        with urlopen(base + path) as response:
            assert response.status == 200
    with pytest.raises(HTTPError) as error:
        urlopen(base + "/../pyproject.toml")
    assert error.value.code == 404


def test_workbench_refuses_duplicate_listener_and_reuses_existing_app(server, tmp_path, capsys):
    from robostudio.workbench import serve

    with pytest.raises(OSError):
        duplicate = make_server(port=server.server_port, output_dir=tmp_path)
        duplicate.server_close()
    serve(server.server_port, tmp_path, False)
    assert "already running" in capsys.readouterr().out


def test_workbench_runs_real_fake_backend_and_rejects_cross_origin(server):
    base = f"http://127.0.0.1:{server.server_port}"
    request = Request(
        base + "/api/demo",
        data=b'{"seed":42}',
        headers={"Content-Type": "application/json", "Origin": base},
    )
    with urlopen(request) as response:
        result = json.loads(response.read())
        assert result["status"] == "SUCCEEDED"
        assert result["metrics"]["steps"] == 20
    request.add_header("Origin", "https://unrelated.example")
    with pytest.raises(HTTPError) as error:
        urlopen(request)
    assert error.value.code == 403


def test_workbench_rejects_invalid_config(server):
    base = f"http://127.0.0.1:{server.server_port}"
    request = Request(
        base + "/api/demo",
        data=b'{"seed":true}',
        headers={"Content-Type": "application/json", "Origin": base},
    )
    with pytest.raises(HTTPError) as error:
        urlopen(request)
    assert error.value.code == 400
    assert json.loads(error.value.read())["error"]["code"] == "INVALID_VALUE"


def test_physics_api_is_bounded_and_same_origin(server):
    pytest.importorskip("mujoco")
    base = f"http://127.0.0.1:{server.server_port}"
    with urlopen(base + "/api/status") as response:
        assert json.loads(response.read())["physics_available"] is True
    request = Request(
        base + "/api/arm/run",
        data=b'{"seed":42}',
        headers={"Content-Type": "application/json", "Origin": base},
    )
    with urlopen(request) as response:
        data = json.loads(response.read())
        assert data["result"]["status"] == "SUCCEEDED"
        assert len(data["frames"]) == 101
        assert data["frames"][0]["frame_id"] == "world"
    request.data = b'{"seed":42,"steps":1000000000}'
    with pytest.raises(HTTPError) as error:
        urlopen(request)
    assert error.value.code == 400
    request.data = b'{"seed":42}'
    request.add_header("Origin", "https://unrelated.example")
    with pytest.raises(HTTPError) as error:
        urlopen(request)
    assert error.value.code == 403


def test_missing_engine_keeps_geometry_available(server, monkeypatch):
    from robostudio.core import ErrorCode, RoboStudioError

    def unavailable():
        raise RoboStudioError(ErrorCode.UNSUPPORTED_CAPABILITY, "engine unavailable")

    monkeypatch.setattr("robostudio.workbench.engine", unavailable)
    base = f"http://127.0.0.1:{server.server_port}"
    with urlopen(base + "/api/status") as response:
        data = json.loads(response.read())
        assert data["physics_available"] is False
        assert data["physics_error"] == "engine unavailable"
    with urlopen(base + "/app.js") as response:
        assert response.status == 200


def test_jacobian_inspection_values_and_contract(server):
    pytest.importorskip("mujoco")
    base = f"http://127.0.0.1:{server.server_port}"
    payload = {"q_rad": [0] * 6, "qvel_rad_s": [1, 0, 0, 0, 0, 0]}
    request = Request(
        base + "/api/arm/analyze",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "Origin": base},
    )
    with urlopen(request) as response:
        data = json.loads(response.read())
        assert data["twist_si"] == pytest.approx([0, 0.84, 0, 0, 0, 1])
        assert data["row_order"] == ["vx", "vy", "vz", "wx", "wy", "wz"]
    for invalid in (
        {**payload, "frame_id": "tool"},
        {**payload, "q_rad": {}},
        {**payload, "q_rad": [False] * 6},
        {**payload, "qvel_rad_s": [100] * 6},
    ):
        request.data = json.dumps(invalid).encode()
        with pytest.raises(HTTPError) as error:
            urlopen(request)
        assert error.value.code == 400
    request.add_header("Origin", "https://unrelated.example")
    with pytest.raises(HTTPError) as error:
        urlopen(request)
    assert error.value.code == 403
