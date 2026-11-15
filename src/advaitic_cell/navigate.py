"""Known-map planner and differential-drive tracker. No base teleportation."""

from __future__ import annotations

import heapq
import math

import numpy as np

from advaitic_cell.frames import wrap
from advaitic_cell.spec import dock_pose

RES = 0.05
RADIUS = 0.32
CONVEYORS = ((3.0, 1.2, 1.0, 0.25), (3.0, 4.8, 1.0, 0.25))


def obstacle_clear(x: float, y: float, radius: float = RADIUS) -> bool:
    if x < radius or y < radius or x > 6.0 - radius or y > 6.0 - radius:
        return False
    for cx, cy, hx, hy in CONVEYORS:
        dx = max(abs(x - cx) - hx, 0.0)
        dy = max(abs(y - cy) - hy, 0.0)
        if dx * dx + dy * dy < radius * radius:
            return False
    return True


def astar(start: tuple[float, float], goal: tuple[float, float], radius: float = RADIUS) -> list[np.ndarray]:
    def cell(point: tuple[float, float]) -> tuple[int, int]:
        return int(round(point[0] / RES)), int(round(point[1] / RES))

    def world(ix: int, iy: int) -> np.ndarray:
        return np.array([ix * RES, iy * RES], dtype=float)

    start_c = cell(start)
    goal_c = cell(goal)
    if not obstacle_clear(*world(*goal_c), radius):
        raise RuntimeError(f"dock approach {goal} is inside an obstacle")
    neighbors = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                 (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
    open_heap = [(0.0, start_c)]
    came: dict[tuple[int, int], tuple[int, int] | None] = {start_c: None}
    cost = {start_c: 0.0}
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal_c:
            break
        cx, cy = current
        for dx, dy, step in neighbors:
            nxt = (cx + dx, cy + dy)
            wx, wy = world(*nxt)
            if not (0.0 <= wx <= 6.0 and 0.0 <= wy <= 6.0):
                continue
            if not obstacle_clear(wx, wy, radius):
                continue
            # Don't cut a corner through an obstacle.
            if dx != 0 and dy != 0 and not (
                obstacle_clear(*world(cx + dx, cy), radius) and obstacle_clear(*world(cx, cy + dy), radius)
            ):
                continue
            new_cost = cost[current] + step
            if new_cost < cost.get(nxt, 1e9):
                cost[nxt] = new_cost
                came[nxt] = current
                heuristic = math.hypot(nxt[0] - goal_c[0], nxt[1] - goal_c[1])
                heapq.heappush(open_heap, (new_cost + heuristic, nxt))
    if goal_c not in came:
        raise RuntimeError(f"no path from {start} to {goal}")
    chain = []
    cursor: tuple[int, int] | None = goal_c
    while cursor is not None:
        chain.append(world(*cursor))
        cursor = came[cursor]
    chain.reverse()
    return _simplify(chain)


def _simplify(points: list[np.ndarray]) -> list[np.ndarray]:
    if len(points) < 3:
        return points
    kept = [points[0]]
    anchor = 0
    for index in range(2, len(points)):
        if not _segment_clear(points[anchor], points[index]):
            kept.append(points[index - 1])
            anchor = index - 1
    kept.append(points[-1])
    return kept


def _segment_clear(a: np.ndarray, b: np.ndarray) -> bool:
    length = float(np.linalg.norm(b - a))
    steps = max(2, int(length / RES))
    for i in range(steps + 1):
        point = a + (b - a) * (i / steps)
        if not obstacle_clear(float(point[0]), float(point[1])):
            return False
    return True


def approach_point(station: int, standoff: float = 0.60) -> np.ndarray:
    x, y, yaw = dock_pose(station)
    return np.array([x - standoff * math.cos(yaw), y - standoff * math.sin(yaw)])


class DiffDriveTracker:
    """Follow a path, align with the dock heading, then creep in along that heading."""

    def __init__(self, path: list[np.ndarray], dock_xy: tuple[float, float], dock_yaw: float):
        self.path = path
        self.dock = np.array(dock_xy, dtype=float)
        self.dock_yaw = dock_yaw
        self.index = 0
        self.mode = "follow"

    def command(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        pos = np.array([x, y], dtype=float)
        if self.mode == "follow":
            while self.index < len(self.path) - 1 and np.linalg.norm(pos - self.path[self.index]) < 0.14:
                self.index += 1
            if np.linalg.norm(pos - self.path[-1]) < 0.04:
                self.mode = "align"
            else:
                target = self.path[min(self.index, len(self.path) - 1)]
                heading = math.atan2(target[1] - y, target[0] - x)
                err = wrap(heading - yaw)
                dist = float(np.linalg.norm(self.path[-1] - pos))
                speed = 0.28 if abs(err) < 0.6 else 0.06
                speed = min(speed, 0.35 * dist + 0.04)
                return speed, float(np.clip(1.8 * err, -0.50, 0.50))
        if self.mode == "align":
            err = wrap(self.dock_yaw - yaw)
            if abs(err) < 0.04:
                self.mode = "creep"
            return 0.0, float(np.clip(2.0 * err, -0.50, 0.50))
        if self.mode == "creep":
            delta = self.dock - pos
            dist = float(np.linalg.norm(delta))
            along = float(delta[0] * math.cos(yaw) + delta[1] * math.sin(yaw))
            lateral = float(-delta[0] * math.sin(yaw) + delta[1] * math.cos(yaw))
            yaw_err = wrap(self.dock_yaw - yaw)
            if dist < 0.01 and abs(yaw_err) < 0.015:
                self.mode = "done"
                return 0.0, 0.0
            if abs(lateral) > 0.008:
                face = math.atan2(float(delta[1]), float(delta[0]))
                face_err = wrap(face - yaw)
                if abs(face_err) > 0.12:
                    return 0.0, float(np.clip(2.0 * face_err, -0.45, 0.45))
                return (
                    float(np.clip(0.55 * dist, 0.015, 0.06)),
                    float(np.clip(1.4 * face_err, -0.30, 0.30)),
                )
            return (
                float(np.clip(0.7 * along, -0.04, 0.07)),
                float(np.clip(2.0 * yaw_err, -0.40, 0.40)),
            )
        return 0.0, 0.0

    @property
    def done(self) -> bool:
        return self.mode == "done"


def plan_to_dock(start_xy: tuple[float, float], station: int) -> tuple[list[np.ndarray], tuple[float, float], float]:
    dock = dock_pose(station)
    approach = approach_point(station)
    path = astar(start_xy, (float(approach[0]), float(approach[1])))
    if np.linalg.norm(path[-1] - approach) > RES:
        path.append(approach)
    dock_y = dock[1] + 0.008 if station == 2 else dock[1]
    return path, (dock[0], dock_y), dock[2]


def integrate_unicycle(x: float, y: float, yaw: float, speed: float, yaw_rate: float, dt: float):
    if abs(yaw_rate) < 1e-6:
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
    else:
        x += speed / yaw_rate * (math.sin(yaw + yaw_rate * dt) - math.sin(yaw))
        y += speed / yaw_rate * (-math.cos(yaw + yaw_rate * dt) + math.cos(yaw))
    return x, y, wrap(yaw + yaw_rate * dt)
