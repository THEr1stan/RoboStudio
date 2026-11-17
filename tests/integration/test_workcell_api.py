import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from robostudio.workbench import make_server
from robostudio.workcell import default_config


@pytest.fixture
def cell_server(tmp_path):
    server = make_server(0, tmp_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{server.server_port}"
    server.shutdown()
    server.server_close()
    thread.join(2)


def post(base, action, value, origin=None):
    request = Request(
        base + "/api/cell/" + action,
        data=json.dumps(value).encode(),
        headers={"Content-Type": "application/json", "Origin": origin or base},
    )
    with urlopen(request) as response:
        return json.loads(response.read())


def test_cell_flow_and_equal_budget_comparison(cell_server):
    cfg = default_config()
    data = post(cell_server, "preview", cfg)
    assert data["nq"] != data["nv"]
    compared = post(
        cell_server, "compare", {"config": cfg, "target_m": [0.42, -0.14, 0.073], "initial_q": None}
    )
    assert {r["algorithm"] for r in compared["results"]} == {"dls", "pinv", "transpose"}
    assert {r["budget"] for r in compared["results"]} == {140}
    assert all(r["status"] == "CONVERGED" for r in compared["results"])
    run = post(cell_server, "run", cfg)
    assert run["status"] == "SUCCEEDED"
    assert post(cell_server, "replay", {"run_id": run["run_id"]})["status"] == "IDENTICAL"
    with urlopen(cell_server + "/api/cell/records/" + run["run_id"]) as response:
        assert json.loads(response.read()) == run
    sample = run["frames"][5]
    assert sample["state"]["time_s"] > sample["command"]["time_s"]
    assert sample["command"]["time_s"] == sample["reference"]["time_s"]
    assert sample["state"]["model_hash"] == run["model_hash"]


@pytest.mark.parametrize(
    "bad",
    [{"robot": []}, {"seed": True}, {"friction": "1"}, {"object_xy": [0, 0]}, {"damping": None}],
)
def test_cell_bad_inputs_fail_without_crashing_server(cell_server, bad):
    cfg = default_config()
    cfg.update(bad)
    with pytest.raises(HTTPError) as error:
        post(cell_server, "preview", cfg)
    assert error.value.code == 400
    assert post(cell_server, "preview", default_config())["frame"]["robot"] == "r6"


def test_cell_origin_bounds_and_generation_gate(cell_server):
    with pytest.raises(HTTPError) as error:
        post(cell_server, "run", default_config(), origin="https://example.org")
    assert error.value.code == 403
    with pytest.raises(HTTPError) as error:
        post(
            cell_server,
            "ik",
            {"config": default_config(), "target_m": [True, 0, 0], "initial_q": None},
        )
    assert error.value.code == 400
    with pytest.raises(HTTPError) as error:
        post(cell_server, "candidate", {"source": {}, "approved": False})
    assert error.value.code == 400
