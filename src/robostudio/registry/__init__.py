import hashlib
from importlib.resources import files
from pathlib import Path

from robostudio.core import Capability, ErrorCode, RoboStudioError, RobotSpec
from robostudio.core.errors import require
from robostudio.serialization import parse_json, read_json

from .manifests import PluginManifest, parse_plugin, parse_robot

__all__ = [
    "PluginManifest",
    "Registry",
    "default_registry",
    "load_robot",
    "parse_plugin",
    "parse_robot",
]


class Registry:
    def __init__(self) -> None:
        self._robots: dict[str, RobotSpec] = {}
        self._plugins: dict[str, PluginManifest] = {}

    def add_robot(self, robot: RobotSpec) -> None:
        require(
            robot.robot_id not in self._robots,
            ErrorCode.DUPLICATE_ID,
            "robot already registered",
            robot.robot_id,
        )
        self._robots[robot.robot_id] = robot

    def add_plugin(self, plugin: PluginManifest) -> None:
        require(
            plugin.plugin_id not in self._plugins,
            ErrorCode.DUPLICATE_ID,
            "plugin already registered",
            plugin.plugin_id,
        )
        self._plugins[plugin.plugin_id] = plugin

    def robot(self, robot_id: str) -> RobotSpec:
        require(robot_id in self._robots, ErrorCode.NOT_FOUND, "unknown robot", robot_id)
        return self._robots[robot_id]

    @property
    def robots(self) -> tuple[RobotSpec, ...]:
        return tuple(self._robots[key] for key in sorted(self._robots))

    @property
    def plugins(self) -> tuple[PluginManifest, ...]:
        return tuple(self._plugins[key] for key in sorted(self._plugins))

    def compatible_plugins(
        self, robot_id: str, capability: Capability
    ) -> tuple[PluginManifest, ...]:
        robot = self.robot(robot_id)
        return tuple(
            p
            for p in self.plugins
            if p.provides_capability == capability
            and robot.capabilities.supports(p.requires_capabilities)
        )

    def require_plugin(self, plugin_id: str, robot_id: str) -> PluginManifest:
        require(plugin_id in self._plugins, ErrorCode.NOT_FOUND, "unknown plugin", plugin_id)
        plugin = self._plugins[plugin_id]
        require(
            self.robot(robot_id).capabilities.supports(plugin.requires_capabilities),
            ErrorCode.UNSUPPORTED_CAPABILITY,
            "robot lacks required capabilities",
            plugin_id,
        )
        return plugin


def load_robot(path: Path) -> RobotSpec:
    robot = parse_robot(read_json(path))
    model_path = (path.parent / robot.model.path).resolve()
    require(
        model_path.is_relative_to(path.parent.resolve()),
        ErrorCode.INVALID_MANIFEST,
        "model path must remain inside the manifest directory",
        "model.path",
    )
    try:
        actual = hashlib.sha256(model_path.read_bytes()).hexdigest()
    except OSError as error:
        raise RoboStudioError(ErrorCode.IO_ERROR, str(error), str(model_path)) from error
    require(
        actual == robot.model.sha256,
        ErrorCode.MODEL_MISMATCH,
        "model file SHA256 mismatch",
        "model.sha256",
    )
    return robot


def default_registry() -> Registry:
    data = files("robostudio").joinpath("data")
    registry = Registry()
    for name in ("fake_arm2", "r6_compact"):
        robot = parse_robot(parse_json(data.joinpath(name + ".json").read_text("utf-8")))
        require(
            hashlib.sha256(data.joinpath(robot.model.path).read_bytes()).hexdigest()
            == robot.model.sha256,
            ErrorCode.MODEL_MISMATCH,
            "bundled model hash mismatch",
        )
        registry.add_robot(robot)
    for name in ("dls_pose", "joint_pd", "quintic"):
        registry.add_plugin(
            parse_plugin(parse_json(data.joinpath(name + ".json").read_text("utf-8")))
        )
    return registry
