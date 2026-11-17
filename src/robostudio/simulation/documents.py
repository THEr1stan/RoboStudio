"""Portable scene configurations; data only, no model or code loading."""

from typing import Any

from ..core import ErrorCode
from ..core.errors import require
from .scene import validate_scene

SCHEMA = "robostudio-scene-v1"
UNITS = {"length": "m", "angle": "rad", "mass": "kg"}


def scene_document(value: Any) -> dict[str, Any]:
    require(isinstance(value, dict), ErrorCode.INVALID_VALUE, "场景文件需要是 JSON 对象。")
    if "schema" not in value:
        value = {"schema": SCHEMA, "name": "导入的场景", "units": UNITS, "config": value}
    require(
        value.get("schema") == SCHEMA,
        ErrorCode.INVALID_VALUE,
        "这不是 RoboStudio 场景文件；运行记录请在“运行记录”中查看。",
    )
    require(
        set(value) == {"schema", "name", "units", "config"},
        ErrorCode.INVALID_VALUE,
        "场景需包含 schema、name、units、config 四项；请使用下载的示例。",
    )
    name = value["name"]
    require(
        isinstance(name, str) and 1 <= len(name.strip()) <= 80,
        ErrorCode.INVALID_VALUE,
        "场景名称需要 1–80 个字符。",
    )
    require(
        value["units"] == UNITS,
        ErrorCode.INVALID_VALUE,
        "场景文件的单位必须为 m、rad、kg；界面会转换为 mm、°、g。",
    )
    return {
        "schema": SCHEMA,
        "name": name.strip(),
        "units": dict(UNITS),
        "config": validate_scene(value["config"]),
    }
