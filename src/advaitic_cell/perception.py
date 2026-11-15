"""Simulated sensors. Navigation consumes the estimate; manipulation consumes the encoder and wrist depth."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from advaitic_cell.frames import ring_world, wrap
from advaitic_cell.scene import ModelIndex, base_pose
from advaitic_cell.spec import ENTRY_X, RING_CENTER_Z

# Documented interaction zone: the aisle between the dock and the near belt edge.
ZONES = {
    1: (2.2, 3.8, 1.45, 2.75),
    2: (2.2, 3.8, 3.25, 4.55),
}


@dataclass
class RayHit:
    origin: np.ndarray
    direction: np.ndarray
    distance: float
    geom: int
    kind: str

    @property
    def end(self) -> np.ndarray:
        span = self.distance if self.distance >= 0.0 else 5.0
        return self.origin + span * self.direction


@dataclass
class RayLog:
    """Last controller rays, keyed by the code path that cast them."""

    by_kind: dict[str, list[RayHit]] = field(default_factory=dict)

    def replace(self, kind: str, hits: list[RayHit]) -> None:
        self.by_kind[kind] = hits

    def all(self) -> list[RayHit]:
        hits: list[RayHit] = []
        for group in self.by_kind.values():
            hits.extend(group)
        return hits


def _ray(model, data, origin, direction, groups: tuple[int, ...]) -> tuple[float, int]:
    mask = np.zeros(6, dtype=np.uint8)
    for group in groups:
        mask[group] = 1
    geomid = np.zeros(1, dtype=np.int32)
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    dist = mujoco.mj_ray(
        model, data, np.asarray(origin, dtype=float), direction, mask, 1, -1, geomid,
    )
    return float(dist), int(geomid[0])


def _record_ray(origin, direction, dist, gid, kind: str) -> RayHit:
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    return RayHit(
        origin=np.asarray(origin, dtype=float).copy(),
        direction=direction.copy(),
        distance=float(dist),
        geom=int(gid),
        kind=kind,
    )


class Estimator:
    """Unicycle odometry with a constant scale bias, corrected by a lidar scan match."""

    def __init__(self, x: float, y: float, yaw: float, rng: np.random.Generator, raylog: RayLog | None = None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.scale = 1.0 + float(rng.uniform(-0.01, 0.01))
        self.rng = rng
        self.raylog = raylog

    def predict(self, forward: float, yaw_rate: float, dt: float) -> None:
        noisy_v = forward * self.scale + float(self.rng.normal(0.0, 0.0015))
        noisy_w = yaw_rate + float(self.rng.normal(0.0, 0.004))
        self.x += noisy_v * math.cos(self.yaw) * dt
        self.y += noisy_v * math.sin(self.yaw) * dt
        self.yaw = wrap(self.yaw + noisy_w * dt)

    def correct(self, model, data, index: ModelIndex) -> None:
        angles = np.linspace(-math.pi, math.pi, 8, endpoint=False)
        measured = _scan(model, data, index, *base_pose(data, index), angles, self.raylog, "nav")
        best = (self.x, self.y, self.yaw)
        best_cost = _cost(model, data, index, best, angles, measured)
        for dx in (-0.03, -0.015, 0.0, 0.015, 0.03):
            for dy in (-0.03, -0.015, 0.0, 0.015, 0.03):
                for dyaw in (-0.04, 0.0, 0.04):
                    guess = (self.x + dx, self.y + dy, wrap(self.yaw + dyaw))
                    cost = _cost(model, data, index, guess, angles, measured)
                    if cost < best_cost:
                        best, best_cost = guess, cost
        if best_cost < 1.5:
            self.x, self.y, self.yaw = best

    def walls(self, model, data, index: ModelIndex) -> None:
        """Range to the known walls. Used only while creeping into a dock.

        A belt in front of the dock blocks one axis; that ray is ignored because
        it disagrees with the estimate by much more than the search window.
        """
        true = base_pose(data, index)
        origin = (true[0], true[1], 0.30)
        hits = []
        for kind_dir in ((-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 1.0, 0.0)):
            dist, gid = _ray(model, data, origin, kind_dir, (0,))
            hits.append(_record_ray(origin, kind_dir, dist, gid, "dock"))
        if self.raylog is not None:
            self.raylog.replace("dock", hits)
        west, east, south, north = (hit.distance for hit in hits)
        if west > 0 and abs(west - self.x) < 0.12:
            self.x = west
        elif east > 0 and abs((6.0 - east) - self.x) < 0.12:
            self.x = 6.0 - east
        if south > 0 and abs(south - self.y) < 0.12:
            self.y = south
        elif north > 0 and abs((6.0 - north) - self.y) < 0.12:
            self.y = 6.0 - north


def _scan(model, data, index, x, y, yaw, angles, raylog: RayLog | None = None, kind: str = "nav") -> np.ndarray:
    origin = (x, y, 0.30)
    ranges = []
    hits = []
    for angle in angles:
        direction = (math.cos(yaw + angle), math.sin(yaw + angle), 0.0)
        dist, gid = _ray(model, data, origin, direction, (0,))
        ranges.append(dist if dist >= 0 else 8.0)
        hits.append(_record_ray(origin, direction, dist, gid, kind))
    if raylog is not None:
        raylog.replace(kind, hits)
    del index
    return np.array(ranges)


def _cost(model, data, index, pose, angles, measured) -> float:
    predicted = _scan(model, data, index, pose[0], pose[1], pose[2], angles)
    return float(np.mean(np.abs(predicted - measured)))


class LinePerception:
    """Conveyor encoder plus a wrist depth camera. Oracle mode is explicit and off by default."""

    def __init__(
        self, model, data, index: ModelIndex, speed: float, rng: np.random.Generator, oracle: bool,
        raylog: RayLog | None = None,
    ):
        self.model = model
        self.data = data
        self.index = index
        self.speed = speed
        self.rng = rng
        self.oracle = oracle
        self.scale = 1.0 + float(rng.uniform(-0.002, 0.002))
        self.raylog = raylog

    def encoder(self, t: float) -> float:
        # A 0.2% scale bias stands in for filtered encoder noise. Drawing fresh
        # noise every control tick would make the arm chase its own sensor.
        return self.speed * t * self.scale

    def ring_point(self, station: int, encoder_seen: float, t: float, slot: int | None) -> np.ndarray:
        if self.oracle and slot is not None:
            body = self.index.slots[slot].ring_body
            return self.data.xpos[body].copy()
        local_x = ENTRY_X + (self.encoder(t) - encoder_seen)
        predicted = np.array(ring_world(station, local_x))
        refined = self._depth(predicted)
        return refined if refined is not None else predicted

    def _depth(self, predicted: np.ndarray) -> np.ndarray | None:
        hits = []
        rays = []
        for dx in (-0.03, -0.015, 0.0, 0.015, 0.03):
            for dy in (-0.03, -0.015, 0.0, 0.015, 0.03):
                origin = predicted + np.array([dx, dy, 0.12])
                dist, gid = _ray(self.model, self.data, origin, (0.0, 0.0, -1.0), (2,))
                rays.append(_record_ray(origin, (0.0, 0.0, -1.0), dist, gid, "depth"))
                if dist < 0 or gid < 0:
                    continue
                point = origin + dist * np.array([0.0, 0.0, -1.0])
                if 0.780 <= point[2] <= 0.792 and np.linalg.norm(point[:2] - predicted[:2]) < 0.045:
                    hits.append(point)
        if self.raylog is not None:
            self.raylog.replace("depth", rays)
        if len(hits) < 2:
            return None
        centroid = np.mean(hits, axis=0)
        centroid[2] = RING_CENTER_Z
        return centroid


def person_visible(
    model, data, index: ModelIndex, station: int | None, raylog: RayLog | None = None,
) -> bool:
    x, y, yaw = base_pose(data, index)
    zone = ZONES.get(station) if station else None
    origin = (x, y, 0.6)
    rays = []
    seen = False
    for angle in np.linspace(-math.pi, math.pi, 64, endpoint=False):
        direction = (math.cos(yaw + angle), math.sin(yaw + angle), 0.0)
        dist, gid = _ray(model, data, origin, direction, (0, 1))
        rays.append(_record_ray(origin, direction, dist, gid, "human"))
        if dist < 0 or gid < 0:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if not name.startswith("human"):
            continue
        if dist < 1.0:
            seen = True
            continue
        if zone is not None:
            hit = np.array([x, y]) + dist * np.array(direction[:2])
            if zone[0] <= hit[0] <= zone[1] and zone[2] <= hit[1] <= zone[3]:
                seen = True
    if raylog is not None:
        raylog.replace("human", rays)
    return seen
