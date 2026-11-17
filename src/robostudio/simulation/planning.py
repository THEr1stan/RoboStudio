"""Seeded bidirectional RRT-Connect, collision checks and bounded shortcut smoothing.

Path interpolation is explicitly sampled, not a continuous-collision guarantee.
The baseline uses the very same edge checker and rejects blocked straight paths.
"""

import heapq
import importlib
import math
import random
from typing import Any

from ..cell_plugins import require_algorithm
from ..core import ErrorCode
from ..core.contracts import vector
from ..core.errors import require
from ..core.manipulation import CollisionQuery
from .scene import StudioCell


def plan_path(
    start: list[float],
    goal: list[float],
    limits: list[tuple[float, ...]],
    clear: CollisionQuery,
    *,
    method: str = "rrt_connect",
    seed: int = 42,
    budget: int = 450,
    collision_budget: int = 12000,
) -> dict[str, Any]:
    require(
        method in ("direct", "rrt", "rrt_connect", "prm"),
        ErrorCode.INVALID_VALUE,
        "unknown planner",
    )
    require(
        type(budget) is int and 1 <= budget <= 1000,
        ErrorCode.INVALID_VALUE,
        "sampling budget must be 1..1000",
    )
    require(
        type(collision_budget) is int and 1 <= collision_budget <= 30000,
        ErrorCode.INVALID_VALUE,
        "collision budget must be 1..30000",
    )
    vector(tuple(start), "start", len(limits))
    vector(tuple(goal), "goal", len(limits))
    require(1 <= len(limits) <= 12, ErrorCode.INVALID_VALUE, "DOF must be 1..12")
    for i, bounds in enumerate(limits):
        vector(tuple(bounds), "limits", 2)
        require(
            bounds[0] < bounds[1]
            and bounds[0] <= start[i] <= bounds[1]
            and bounds[0] <= goal[i] <= bounds[1],
            ErrorCode.INVALID_VALUE,
            "joint limits",
        )
    require_algorithm(method, frozenset({"collision_check"}))
    np = importlib.import_module("numpy")
    checks = 0
    edges: list[list[list[float]]] = []
    search_trace: list[dict[str, Any]] = []
    last_collision: list[float] | None = None
    trace_count = 0
    step, resolution = 0.22, 0.028

    def valid(q: Any) -> bool:
        nonlocal checks, last_collision
        if checks >= collision_budget:
            return False
        checks += 1
        values = [float(x) for x in q]
        ok = clear(values)
        if not ok:
            last_collision = values
        return ok

    def edge(a: Any, b: Any) -> bool:
        nonlocal last_collision
        last_collision = None
        count = max(1, math.ceil(float(np.max(np.abs(b - a))) / resolution))
        return all(valid(a + (b - a) * i / count) for i in range(1, count + 1))

    def record(old: Any, new: Any, sample: Any, accepted: bool, tree: int) -> None:
        nonlocal trace_count
        trace_count += 1
        if len(search_trace) < 2500:
            search_trace.append(
                {
                    "iteration": iterations,
                    "tree": tree,
                    "from_q": old.tolist(),
                    "q": new.tolist(),
                    "sample_q": sample.tolist(),
                    "accepted": accepted,
                    "collision_q": last_collision,
                    "collision_checks": checks,
                }
            )

    a, b = np.array(start), np.array(goal)
    path: list[Any] = []
    raw_path: list[list[float]] = []
    iterations = 0
    if valid(a) and valid(b):
        direct_ok = edge(a, b)
        record(a, b, b, direct_ok, -1)
        if direct_ok:
            path = [a, b]
        elif method == "rrt_connect":
            rng = random.Random(seed)
            trees: list[list[tuple[Any, int]]] = [[(a, -1)], [(b, -1)]]

            def extend(tree: list[tuple[Any, int]], target: Any) -> tuple[int, bool]:
                nearest = min(
                    range(len(tree)), key=lambda i: float(np.linalg.norm(tree[i][0] - target))
                )
                old = tree[nearest][0]
                dist = float(np.linalg.norm(target - old))
                new = old + (target - old) * min(1.0, step / max(1e-12, dist))
                ok = edge(old, new)
                record(old, new, target, ok, 0 if tree is trees[0] else 1)
                if not ok:
                    return -1, False
                tree.append((new, nearest))
                if len(edges) < 350:
                    edges.append([old.tolist(), new.tolist()])
                return len(tree) - 1, dist <= step

            def branch(tree: list[tuple[Any, int]], index: int) -> list[Any]:
                result = []
                while index >= 0:
                    value, index = tree[index]
                    result.append(value)
                return result[::-1]

            for iterations in range(1, budget + 1):  # noqa: B007 - consumed by trace closure
                if checks >= collision_budget:
                    break
                active = iterations % 2
                first, second = trees[active], trees[1 - active]
                target = (
                    second[0][0]
                    if rng.random() < 0.2
                    else np.array([rng.uniform(lo, hi) for lo, hi in limits])
                )
                ia, _ = extend(first, target)
                if ia < 0:
                    continue
                for _ in range(100):
                    ib, reached = extend(second, first[ia][0])
                    if ib < 0:
                        break
                    if reached:
                        one, two = branch(first, ia), branch(second, ib)
                        path = one + two[-2::-1]
                        if active == 1:
                            path.reverse()
                        break
                if path:
                    raw_path = [q.tolist() for q in path]
                    # Deterministic shortcut pass using the same sampled collision oracle.
                    for _ in range(50):
                        if len(path) <= 2:
                            break
                        i, j = sorted(rng.sample(range(len(path)), 2))
                        if j > i + 1 and edge(path[i], path[j]):
                            path[i + 1 : j] = []
                    break
        elif method in ("rrt", "prm"):
            rng = random.Random(seed)
            nodes = [a] if method == "rrt" else [a, b]
            parents = [-1]
            graph: list[list[tuple[int, float]]] = [[], []]

            def shortest() -> list[Any]:
                queue = [(0.0, 0)]
                distances = {0: 0.0}
                previous: dict[int, int] = {}
                while queue:
                    cost, node = heapq.heappop(queue)
                    if cost > distances[node]:
                        continue
                    if node == 1:
                        indices = [1]
                        while indices[-1] != 0:
                            indices.append(previous[indices[-1]])
                        return [nodes[i] for i in reversed(indices)]
                    for neighbor, length in graph[node]:
                        candidate = cost + length
                        if candidate < distances.get(neighbor, math.inf):
                            distances[neighbor], previous[neighbor] = candidate, node
                            heapq.heappush(queue, (candidate, neighbor))
                return []

            for iterations in range(1, budget + 1):  # noqa: B007 - consumed by trace closure
                if checks >= collision_budget:
                    break
                target = (
                    b
                    if method == "rrt" and rng.random() < 0.2
                    else np.array([rng.uniform(lo, hi) for lo, hi in limits])
                )
                near = min(
                    range(len(nodes)), key=lambda i: float(np.linalg.norm(nodes[i] - target))
                )
                old = nodes[near]
                if method == "rrt":
                    dist = float(np.linalg.norm(target - old))
                    new = old + (target - old) * min(1.0, step / max(1e-12, dist))
                    ok = edge(old, new)
                    record(old, new, target, ok, 0)
                    if not ok:
                        continue
                    nodes.append(new)
                    parents.append(near)
                    if len(edges) < 350:
                        edges.append([old.tolist(), new.tolist()])
                    if float(np.linalg.norm(new - b)) <= step:
                        ok = edge(new, b)
                        record(new, b, b, ok, 0)
                        if ok:
                            index = len(nodes) - 1
                            path = [b]
                            while index >= 0:
                                path.append(nodes[index])
                                index = parents[index]
                            path.reverse()
                else:
                    last_collision = None
                    if not valid(target):
                        record(old, target, target, False, 2)
                        continue
                    neighbors = sorted(
                        range(len(nodes)), key=lambda i: float(np.linalg.norm(nodes[i] - target))
                    )[:10]
                    new_id = len(nodes)
                    nodes.append(target)
                    graph.append([])
                    for neighbor in neighbors:
                        if checks >= collision_budget:
                            break
                        old = nodes[neighbor]
                        ok = edge(old, target)
                        record(old, target, target, ok, 2)
                        if ok:
                            length = float(np.linalg.norm(target - old))
                            graph[new_id].append((neighbor, length))
                            graph[neighbor].append((new_id, length))
                            if len(edges) < 350:
                                edges.append([old.tolist(), target.tolist()])
                    path = shortest()
                if path:
                    raw_path = [q.tolist() for q in path]
                    for _ in range(50):
                        if len(path) <= 2 or checks >= collision_budget:
                            break
                        i, j = sorted(rng.sample(range(len(path)), 2))
                        if j > i + 1 and edge(path[i], path[j]):
                            path[i + 1 : j] = []
                    break
    return {
        "status": "FOUND" if path else "BLOCKED",
        "algorithm": method,
        "path": [q.tolist() for q in path],
        "iterations": iterations,
        "budget": budget,
        "collision_checks": checks,
        "sample_resolution_si": resolution,
        "seed": seed,
        "length_joint_space": sum(
            float(np.linalg.norm(y - x)) for x, y in zip(path, path[1:], strict=False)
        ),
        "tree_edges": edges,
        "search_trace": search_trace,
        "trace_truncated": trace_count > len(search_trace),
        "search_events": trace_count,
        "raw_path": raw_path or [q.tolist() for q in path],
        "collision_budget": collision_budget,
        "reason": (
            "FOUND"
            if path
            else "COLLISION_BUDGET"
            if checks >= collision_budget
            else "NO_PATH_WITHIN_BUDGET"
        ),
        "scene_source": "known_simulator_collision_geometry",
    }


class CollisionProbe:
    """Private data; obstacle/robot/link checks including an estimated carried object."""

    def __init__(
        self,
        cell: StudioCell,
        target: int | None = None,
        held: bool = False,
        allow_touch: bool = False,
        close: bool = False,
    ):
        self.cell, self.target, self.held, self.allow_touch = cell, target, held, allow_touch
        self.data = cell.mj.MjData(cell.model)
        self.data.qpos[:] = cell.data.qpos
        self.data.mocap_pos[:] = cell.data.mocap_pos
        self.data.mocap_quat[:] = cell.data.mocap_quat
        self.checks = 0
        for name in ("left", "right"):
            jid = cell.model.joint(f"finger_{name}").id
            address = cell.model.jnt_qposadr[jid]
            if close:
                # Actual current width when holding, nominal width for candidate grasp.
                width = cell.scene["objects"][target]["size"][1] if target is not None else 0.02
                self.data.qpos[address] = min(0.043, 0.042 - width)
            else:
                self.data.qpos[address] = 0.0
        self.last_pair: list[int] | None = None

    def __call__(self, q: list[float]) -> bool:
        c, d, m = self.cell, self.data, self.cell.model
        d.qpos[c.q_indices] = q
        c.mj.mj_forward(m, d)
        if self.held and self.target is not None:
            jid = m.joint(f"free{self.target}").id
            adr = m.jnt_qposadr[jid]
            # Conservative carried-volume check, never a weld or write to the live object.
            d.qpos[adr : adr + 3] = d.site_xpos[c.tcp] + d.site_xmat[c.tcp].reshape(
                3, 3
            ) @ c.np.array([0.05, 0, 0])
            c.mj.mj_forward(m, d)
        for a, b in c.self_pairs:
            if c.mj.mj_geomDistance(m, d, a, b, 0.01, None) < 0.001:
                self.last_pair = [a, b]
                return False
        table = m.geom("table").id
        obstacles = [table, m.geom("floor").id, *c.obstacle_geoms, *c.object_geoms]
        for robot_geom in c.robot_geoms:
            if int(m.geom_bodyid[robot_geom]) == int(m.jnt_bodyid[c.joint_ids[0]]):
                continue  # anchored base and its floor intersection
            for other in obstacles:
                if (
                    self.target is not None
                    and other == c.object_geoms[self.target]
                    and (self.held or (self.allow_touch and robot_geom in c.pad_ids))
                ):
                    continue
                self.checks += 1
                distance = c.mj.mj_geomDistance(m, d, robot_geom, other, 0.01, None)
                if distance < 0.0015:
                    self.last_pair = [robot_geom, other]
                    return False
        if self.held and self.target is not None:
            carried = c.object_geoms[self.target]
            for other in [*c.obstacle_geoms, *(x for x in c.object_geoms if x != carried)]:
                if c.mj.mj_geomDistance(m, d, carried, other, 0.01, None) < 0.002:
                    self.last_pair = [carried, other]
                    return False
        return True
