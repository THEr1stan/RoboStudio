"""Explicit manual position constraint; autonomous grasping never uses this adapter."""

import copy
import math
from typing import Any


class ObjectHandle:
    """Translate one free body, preserving orientation and the clicked world offset.

    This is a scene-edit interaction, not a force actuator. Translation is swept in
    <=2 mm increments on private data; the last clear pose is held until release.
    """

    def __init__(self, cell: Any, index: int, point: list[float]):
        self.cell = cell
        self.body = cell.object_bodies[index]
        self.geom = cell.object_geoms[index]
        joint = cell.model.joint(f"free{index}").id
        self.q = int(cell.model.jnt_qposadr[joint])
        self.v = int(cell.model.jnt_dofadr[joint])
        self.pose = cell.data.qpos[self.q : self.q + 7].copy()
        self.offset = cell.np.array(point) - self.pose[:3]
        self.probe = cell.mj.MjData(cell.model)
        self.blocking: dict[str, Any] | None = None
        self.others = [
            g
            for g in range(cell.model.ngeom)
            if g != self.geom and (cell.model.geom_contype[g] or cell.model.geom_conaffinity[g])
        ]

    def move(self, point: list[float]) -> bool:
        c, d = self.cell, self.probe
        self.blocking = None
        d.qpos[:] = c.data.qpos
        d.mocap_pos[:] = c.data.mocap_pos
        d.mocap_quat[:] = c.data.mocap_quat
        d.qpos[self.q : self.q + 7] = self.pose
        c.mj.mj_forward(c.model, d)
        # Existing resting contacts may separate, but cannot be driven deeper.
        limits = [
            min(0.001, c.mj.mj_geomDistance(c.model, d, self.geom, g, 0.01, None) - 0.00005)
            for g in self.others
        ]
        start = self.pose[:3].copy()
        target = c.np.array(point) - self.offset
        delta = target - start
        samples = max(1, math.ceil(float(c.np.linalg.norm(delta)) / 0.002))
        reached = True
        for i in range(1, samples + 1):
            candidate = start + delta * (i / samples)
            d.qpos[self.q : self.q + 3] = candidate
            c.mj.mj_forward(c.model, d)
            for g, limit in zip(self.others, limits, strict=True):
                distance = c.mj.mj_geomDistance(c.model, d, self.geom, g, 0.01, None)
                if distance < limit:
                    self.blocking = {
                        "source": "manual_translation_guard",
                        "frame": "world",
                        "geom_ids": [self.geom, g],
                        "geom_names": [c.model.geom(self.geom).name, c.model.geom(g).name],
                        **self._describe(g),
                        "requested_point_m": point.copy(),
                        "rejected_position_m": candidate.tolist(),
                        "rejected_clearance_m": float(distance),
                        "limit_m": float(limit),
                    }
                    break
            if self.blocking is not None:
                reached = False
                break
            self.pose[:3] = candidate
        self.enforce()
        return reached

    def _describe(self, geom: int) -> dict[str, Any]:
        c = self.cell
        if geom in c.object_geoms:
            index = c.object_geoms.index(geom)
            obj = c.scene["objects"][index]
            color = {"orange": "橙色", "blue": "蓝色", "mint": "绿色"}[obj["color"]]
            return {
                "kind": "object",
                "index": index,
                "label": color + ("积木" if obj["shape"] == "box" else "圆柱"),
            }
        if geom in c.obstacle_geoms:
            index = c.obstacle_geoms.index(geom)
            return {"kind": "obstacle", "index": index, "label": f"障碍 {index + 1}"}
        name = c.model.geom(geom).name
        if name in ("table", "floor"):
            return {
                "kind": "surface",
                "index": None,
                "label": "桌面" if name == "table" else "地面",
            }
        if geom in c.robot_geoms:
            label = "机械臂夹指" if geom in c.pad_ids else "机械臂部件"
            return {"kind": "body", "index": int(c.model.geom_bodyid[geom]), "label": label}
        return {"kind": "fixed", "index": None, "label": "固定部件"}

    def feedback(self) -> dict[str, Any] | None:
        """Read current accepted geometry, never display rejected probe coordinates as real."""
        if self.blocking is None:
            return None
        c = self.cell
        points = c.np.zeros(6)
        distance = c.mj.mj_geomDistance(
            c.model, c.data, self.geom, self.blocking["geom_ids"][1], 10.0, points
        )
        return {
            **copy.deepcopy(self.blocking),
            "clearance_m": float(distance),
            "points_m": points.reshape(2, 3).tolist(),
            "accepted_position_m": c.data.xpos[self.body].tolist(),
        }

    def enforce(self) -> None:
        c = self.cell
        c.data.qpos[self.q : self.q + 7] = self.pose
        c.data.qvel[self.v : self.v + 6] = 0
        c.data.qacc_warmstart[self.v : self.v + 6] = 0
        c.mj.mj_forward(c.model, c.data)
