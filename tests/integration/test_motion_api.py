import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from robostudio.tracking import preset_config
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


def post(server, path, data, origin=None):
    base = f"http://127.0.0.1:{server.server_port}"
    request = Request(
        base + path,
        data=json.dumps(data).encode(),
        headers={"Content-Type": "application/json", "Origin": origin or base},
    )
    with urlopen(request) as response:
        return json.loads(response.read())


def test_target_solve_run_list_and_saved_playback(server):
    pytest.importorskip("mujoco")
    base = f"http://127.0.0.1:{server.server_port}"
    with urlopen(base + "/api/catalog") as response:
        catalog = json.loads(response.read())
    assert len(catalog["presets"]) == 3
    assert len(catalog["algorithms"]) == 3
    config = catalog["presets"][0]["config"]
    solution = post(server, "/api/arm/ik", config)
    assert solution["solution"]["status"] == "CONVERGED"
    run = post(server, "/api/arm/track", config)
    assert run["result"]["status"] == "SUCCEEDED"
    with urlopen(base + "/api/runs") as response:
        records = json.loads(response.read())
    assert records[0]["result"] == run["result"]
    with urlopen(base + "/api/runs/" + run["result"]["run_id"]) as response:
        assert json.loads(response.read()) == run
    with pytest.raises(HTTPError) as error:
        post(server, "/api/arm/track", config, "http://unrelated.example")
    assert error.value.code == 403


@pytest.mark.parametrize(
    "mutation",
    [
        {"seed": True},
        {"seed_q_rad": [99] * 6},
        {"duration_s": 0},
        {"controller": "execute_python"},
        {"seed_q_rad": {}},
        {"extra": "unexpected"},
        {"target": {"position_m": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0], "frame_id": "tool"}},
        {
            "target": {
                "position_m": [True, 0, 0],
                "quaternion_wxyz": [1, 0, 0, 0],
                "frame_id": "world",
            }
        },
        {"target": {"position_m": [0, 0, 0], "quaternion_wxyz": [2, 0, 0, 0], "frame_id": "world"}},
    ],
)
def test_target_api_rejects_invalid_request_before_running(server, mutation):
    config = preset_config("reach")
    config.update(mutation)
    with pytest.raises(HTTPError) as error:
        post(server, "/api/arm/track", config)
    assert error.value.code == 400
    assert json.loads(error.value.read())["error"]["code"]


def test_records_do_not_allow_paths(server):
    base = f"http://127.0.0.1:{server.server_port}"
    for name in ("..%2fpyproject.toml", "not-a-record", "track_" + "0" * 24):
        with pytest.raises(HTTPError):
            urlopen(base + "/api/runs/" + name)
