import pytest

from robostudio.backends.mujoco_backend import engine
from robostudio.core import RoboStudioError


def test_missing_optional_engine_has_structured_error(monkeypatch):
    import importlib

    original = importlib.import_module

    def unavailable(name, *args, **kwargs):
        if name == "mujoco":
            raise ImportError("not installed")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(importlib, "import_module", unavailable)
    with pytest.raises(RoboStudioError, match="UNSUPPORTED_CAPABILITY.*physics"):
        engine()
