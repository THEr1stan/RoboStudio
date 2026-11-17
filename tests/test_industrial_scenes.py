"""Industrial presets are editable data with observable physical task outcomes."""

import json

import numpy as np
import pytest

from robostudio.serialization import canonical
from robostudio.studio_api import StudioAPI
from robostudio.studio_documents import scene_document
from robostudio.studio_world import WorldSession, replay_world

IDS = ("part_loading", "incoming_sorting", "case_stacking", "partition_transfer")


def test_industrial_catalog_is_portable_and_independent(tmp_path):
    api = StudioAPI(tmp_path)
    rows = api.get("catalog")["industrial_scenes"]
    assert tuple(row["id"] for row in rows) == IDS
    for row in rows:
        assert row["description"] and len(row["process"]) >= 3 and row["scope"]
        doc = json.loads(json.dumps(row["document"]))
        assert scene_document(doc) == doc
        assert doc["config"]["obstacles"], "fixtures must be collision geometry"
        assert len(json.dumps(doc).encode()) < 14000
        first = WorldSession(doc["config"])
        second = WorldSession(doc["config"])
        assert canonical(first.snapshot()) == canonical(second.snapshot())
        assert len(first.cell.obstacle_geoms) == len(doc["config"]["obstacles"])
    rows[0]["document"]["config"]["objects"][0]["mass"] = 999
    assert (
        api.get("catalog")["industrial_scenes"][0]["document"]["config"]["objects"][0]["mass"] < 1
    )


@pytest.mark.parametrize("identifier", IDS)
def test_industrial_task_contact_completion_and_replay(tmp_path, identifier):
    row = next(
        r for r in StudioAPI(tmp_path).get("catalog")["industrial_scenes"] if r["id"] == identifier
    )
    cfg = row["document"]["config"]
    sim = WorldSession(cfg)
    sim.command({"action": "start"})
    for _ in range(100):
        sim.advance(500)
        if sim.goal_state == "satisfied":
            break
        assert not sim.failures, sim.failures
    assert sim.goal_state == "satisfied"
    report = sim.report()
    count = 1 if cfg["task"] == "single" else len(cfg["objects"])
    assert report["metrics"]["placed"] == count
    assert report["metrics"]["bilateral_steps"] > 100
    assert report["metrics"]["collision_steps"] == 0
    assert all(p["success"] and p["camera_verified"] for p in report["placements"])
    if cfg["task"] == "stack":
        expected = np.cumsum([2 * o["size"][2] for o in cfg["objects"]])
        centers = expected - np.array([o["size"][2] for o in cfg["objects"]])
        assert [o["position"][2] for o in report["frames"][-1]["objects"]] == pytest.approx(
            centers, abs=0.008
        )
    if identifier == "partition_transfer":
        assert any(p["iterations"] > 0 for p in report["plans"])
    assert canonical(report) == canonical(replay_world(report))
