"""Project-authored industrial workcells using the existing bounded scene contract."""

from typing import Any

from .documents import SCHEMA, UNITS, scene_document
from .scene import default_scene


def industrial_scenes() -> list[dict[str, Any]]:
    def rail(x: float, y: float, sx: float, sy: float, sz: float) -> dict[str, Any]:
        return {"xy": [x, y], "size": [sx, sy, sz], "yaw": 0.0}

    loading = default_scene()
    loading["objects"] = [loading["objects"][0]]
    loading["objects"][0].update(shape="cylinder", mass=0.06)
    loading["obstacles"] = [
        rail(0.32, 0.21, 0.015, 0.075, 0.025),
        rail(0.52, 0.21, 0.015, 0.075, 0.025),
        rail(0.42, 0.30, 0.115, 0.01, 0.025),
    ]
    sorting = default_scene("gantry")
    sorting["task"] = "sort"
    for i, obj in enumerate(sorting["objects"]):
        obj["xy"] = [0.32 + i * 0.10, -0.20]
    sorting["obstacles"] = [
        rail(0.42, -0.30, 0.16, 0.01, 0.025),
        rail(0.42, -0.10, 0.16, 0.01, 0.025),
    ]
    stacking = default_scene("gantry")
    stacking["task"] = "stack"
    stacking["destination"] = [0.44, 0.20]
    stacking["obstacles"] = [
        rail(0.28, 0.22, 0.01, 0.075, 0.03),
        rail(0.60, 0.22, 0.01, 0.075, 0.03),
        rail(0.44, 0.30, 0.16, 0.01, 0.03),
    ]
    transfer = default_scene()
    transfer["objects"] = [transfer["objects"][0]]
    transfer["obstacles"] = [rail(0.43, 0.015, 0.035, 0.045, 0.115)]
    specs = [
        (
            "part_loading",
            "零件上下料",
            "R6 · 圆柱坯料",
            loading,
            "将圆柱零件从待料区搬入 U 形定位挡块之间，完成一次接驳上料。",
            ["相机定位", "圆柱夹取", "定位区放置"],
            "模拟上料转运与接触放置；不包含机床切削、插装或门控联动。",
            "待料区",
            "定位区",
        ),
        (
            "incoming_sorting",
            "来料分拣工位",
            "XYZ · 三色分流",
            sorting,
            "从双导轨静态料道逐件取料，按颜色送入三个独立接收位置。",
            ["颜色识别", "逐件抓取", "分流摆放"],
            "料道与挡块参与真实碰撞；来料静置，可手动拖动物料补入，无运动传送带。",
            "静态料道",
            "分拣区",
        ),
        (
            "case_stacking",
            "成品码垛",
            "XYZ · 三层堆叠",
            stacking,
            "把三个箱件依次叠放到有围挡的桌面垛位，检查每层落点和高度。",
            ["识别箱件", "计算层高", "逐层码垛"],
            "三件小尺寸箱体的竖直堆叠，靠重力和接触保持；不含缠绕、打包或整托盘搬运。",
            "成品区",
            "码垛区",
        ),
        (
            "partition_transfer",
            "隔板避障转运",
            "R6 · 路径规划",
            transfer,
            "在来料区与接收区之间加入 230 mm 高隔板，用 RRT-Connect 搜索可行路径。",
            ["抓取来料", "碰撞路径搜索", "跨区放置"],
            "隔板是可拖动的实体障碍；移动后可能使任务受阻，可在运算面板检查实际规划。",
            "来料区",
            "接收区",
        ),
    ]
    return [
        {
            "id": ident,
            "subtitle": subtitle,
            "description": description,
            "process": process,
            "scope": scope,
            "zones": {"input": incoming, "output": outgoing},
            "document": scene_document(
                {"schema": SCHEMA, "name": name, "units": dict(UNITS), "config": config}
            ),
        }
        for ident, name, subtitle, config, description, process, scope, incoming, outgoing in specs
    ]
