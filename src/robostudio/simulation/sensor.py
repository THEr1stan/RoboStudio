"""Calibrated ray-cast color/depth sensor and image-only connected-component perception.

This is a synthetic flat-color camera, not photorealistic RGB or a learned detector.
Geometry identities are used only to form radiance; they are never given to detection.
"""

import importlib
import math
from typing import Any

from ..cell_plugins import require_algorithm
from .scene import COLORS, StudioCell


def capture(cell: StudioCell, sequence: int) -> dict[str, Any]:
    np, m, d = cell.np, cell.model, cell.data
    width, height, focal = 128, 112, 160.0
    origin = np.array([0.40, 0.0, 1.0])
    u, v = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    rays = np.stack(((u - width / 2) / focal, -(v - height / 2) / focal, -np.ones_like(u)), axis=-1)
    rays = rays.reshape(-1, 3)
    rays /= np.linalg.norm(rays, axis=1)[:, None]
    ids, distances = np.full(len(rays), -1, dtype=np.int32), np.zeros(len(rays))
    cell.mj.mj_multiRay(
        m, d, origin, rays.flatten(), None, True, -1, ids, distances, None, len(rays), 2.0
    )
    valid = (ids >= 0) & (distances > 0)
    rgb = np.full((len(rays), 3), 245, dtype=np.uint8)
    rgb[valid] = (m.geom_rgba[ids[valid], :3] * 255).astype(np.uint8)
    depth = np.where(valid, distances * -rays[:, 2], 0.0)
    rng = np.random.default_rng(cell.scene["seed"] + sequence * 7919)
    depth[valid] += rng.normal(0, cell.scene["noise_mm"] / 1000, int(valid.sum()))
    missing = rng.random(len(depth)) < cell.scene["dropout"]
    depth[missing] = 0
    return {
        "source": "mujoco_ray_rgbd",
        "width": width,
        "height": height,
        "rgb": rgb.flatten().tolist(),
        "depth_m": np.round(np.maximum(0, depth), 6).tolist(),
        "intrinsics": {"fx": focal, "fy": focal, "cx": width / 2, "cy": height / 2},
        "camera_position_m": origin.tolist(),
        "camera_axes": [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
        "time_s": float(d.time),
        "sequence": sequence,
        "frame_id": "world",
        "noise_mm": cell.scene["noise_mm"],
        "dropout": cell.scene["dropout"],
        "latency_ms": cell.scene["latency_ms"],
    }


def detect(image: dict[str, Any]) -> list[dict[str, Any]]:
    """Color threshold → 4-connected components → calibrated backprojection → XY PCA."""
    require_algorithm("rgbd_components", frozenset({"rgbd_observation"}))
    np = importlib.import_module("numpy")
    w, h = image["width"], image["height"]
    rgb = np.array(image["rgb"], dtype=float).reshape(h, w, 3)
    depth = np.array(image["depth_m"]).reshape(h, w)
    intr, origin = image["intrinsics"], np.array(image["camera_position_m"])
    results = []
    for name, color in COLORS.items():
        mask = (np.linalg.norm(rgb - np.array(color) * 255, axis=2) < 35) & (depth > 0)
        seen = np.zeros((h, w), dtype=bool)
        components: list[list[tuple[int, int]]] = []
        for yy, xx in zip(*np.where(mask), strict=True):
            if seen[yy, xx]:
                continue
            component, queue = [], [(int(yy), int(xx))]
            seen[yy, xx] = True
            while queue:
                y, x = queue.pop()
                component.append((y, x))
                for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                    if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        queue.append((ny, nx))
            components.append(component)
        if not components:
            continue
        component = max(components, key=len)
        if len(component) < 5:
            continue
        yy, xx = np.array(component).T
        z = depth[yy, xx]
        points = np.column_stack(
            (
                origin[0] + (xx + 0.5 - intr["cx"]) * z / intr["fx"],
                origin[1] - (yy + 0.5 - intr["cy"]) * z / intr["fy"],
                origin[2] - z,
            )
        )
        # Table is z=0. Graspable color components must have meaningful height.
        if float(np.median(points[:, 2])) < 0.008:
            continue
        center = np.mean(points, axis=0)
        eigenvalues, axes = np.linalg.eigh(np.cov(points[:, :2].T))
        direction = axes[:, -1]
        yaw = math.atan2(float(direction[1]), float(direction[0]))
        yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
        ratio = float(eigenvalues[-1] / max(1e-10, eigenvalues[0]))
        results.append(
            {
                "color": name,
                "position_m": center.tolist(),
                "yaw_rad": yaw if ratio > 1.3 else 0.0,
                "anisotropy": ratio,
                "pixels": len(component),
                "confidence": min(1, len(component) / 36),
                "bbox_px": [int(xx.min()), int(yy.min()), int(xx.max() + 1), int(yy.max() + 1)],
                "time_s": image["time_s"],
                "source": "rgb_threshold_components_depth_pca",
                "points_m": points[:: max(1, len(points) // 70)].tolist(),
            }
        )
    return results
