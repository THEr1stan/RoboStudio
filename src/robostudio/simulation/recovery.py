"""Bounded upward contact retreat; private geometry checks, never live pose writes."""

from typing import Any

from ..cell_ik import solve
from .planning import CollisionProbe
from .scene import StudioCell


def contact_retreat(cell: StudioCell, solver: str, damping: float) -> dict[str, Any]:
    """Allow existing shallow contact to separate, reject new or deeper contact.

    This is a local recovery guard, not a general collision-free path planner.
    Every sampled configuration must improve initial contact clearance; all other
    pairs retain the normal planner margin. Final configuration must be clear.
    """
    c, np = cell, cell.np
    probe = CollisionProbe(c)
    q = c.data.qpos[c.q_indices].tolist()
    result: dict[str, Any] = {
        "status": "CLEAR",
        "reason": "START_CLEAR",
        "path": [],
        "collision_checks": 0,
        "sample_count": 80,
        "max_penetration_m": 0.004,
        "rise_m": 0.08,
    }
    if probe(q):
        return result
    result.update(status="BLOCKED", reason="CONTACT_RETREAT_BLOCKED")
    if any(not low <= value <= high for value, (low, high) in zip(q, c.limits, strict=True)):
        result["reason"] = "CONTACT_RETREAT_JOINT_LIMIT"
        return result
    world = [c.model.geom("table").id, c.model.geom("floor").id, *c.obstacle_geoms, *c.object_geoms]
    pairs = [(a, b, 0.001) for a, b in c.self_pairs]
    pairs += [
        (a, b, 0.0015)
        for a in c.robot_geoms
        for b in world
        if c.model.geom_bodyid[a] != c.model.jnt_bodyid[c.joint_ids[0]]
    ]

    def distances(values: list[float]) -> Any:
        probe.data.qpos[c.q_indices] = values
        c.mj.mj_forward(c.model, probe.data)
        result["collision_checks"] += len(pairs)
        return np.array(
            [c.mj.mj_geomDistance(c.model, probe.data, a, b, 0.02, None) for a, b, _ in pairs]
        )

    initial = distances(q)
    margins = np.array([margin for _, _, margin in pairs])
    touching = initial < margins
    result["initial_pairs"] = [
        {"geoms": [a, b], "clearance_m": float(distance)}
        for (a, b, margin), distance in zip(pairs, initial, strict=True)
        if distance < margin
    ]
    # Self contact and obstacle contact require scene adjustment, not automatic escape.
    allowed = {c.model.geom("table").id, c.model.geom("floor").id, *c.object_geoms}
    if any(
        distance < -0.004 or b not in allowed
        for (a, b, margin), distance in zip(pairs, initial, strict=True)
        if distance < margin
    ):
        return result
    start_tcp = probe.data.site_xpos[c.tcp].copy()
    quat = np.zeros(4)
    c.mj.mju_mat2Quat(quat, c.data.site_xmat[c.tcp])
    target = c.data.site_xpos[c.tcp].copy()
    target[2] += result["rise_m"]
    solved = solve(c, target.tolist(), solver, damping=damping, quaternion_wxyz=quat.tolist())
    result["solution"] = solved
    if solved["status"] != "CONVERGED":
        result["reason"] = "CONTACT_RETREAT_IK_FAILED"
        return result
    previous = initial.copy()
    minimum = float(initial.min())
    for i in range(1, result["sample_count"] + 1):
        values = (np.array(q) + (np.array(solved["q"]) - q) * i / result["sample_count"]).tolist()
        current = distances(values)
        tcp = probe.data.site_xpos[c.tcp]
        if (
            np.any(current[~touching] < margins[~touching])
            or np.any(current[touching] < previous[touching] - 0.00005)
            or np.any(current[touching] < initial[touching] - 0.00005)
            or tcp[2] < start_tcp[2] - 0.00005
            or float(np.linalg.norm(tcp[:2] - start_tcp[:2])) > 0.01
        ):
            return result
        minimum = min(minimum, float(current.min()))
        previous = np.minimum(current, margins)
    if not probe(solved["q"]):
        return result
    result.update(
        status="FOUND",
        reason="SEPARATING_CONTACT",
        path=[q, solved["q"]],
        min_clearance_m=minimum,
        final_clearance_m=float(current.min()),
    )
    return result
