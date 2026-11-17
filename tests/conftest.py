import json
from importlib.resources import files

import pytest

from robostudio.core import RobotSpec
from robostudio.registry import default_registry


@pytest.fixture
def robot() -> RobotSpec:
    return default_registry().robot("fake_arm2")


@pytest.fixture
def manifest() -> dict:
    return json.loads(files("robostudio").joinpath("data/fake_arm2.json").read_text("utf-8"))
