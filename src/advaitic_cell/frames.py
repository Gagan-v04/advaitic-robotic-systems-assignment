"""Station frames shared by the plant and the robot's line-signal model."""

from __future__ import annotations

import math

from advaitic_cell.spec import (
    BOTTLE_OFFSET,
    ENTRY_X,
    INSPECT_X,
    KIT_TOP_Z,
    RING_AHEAD_OF_BOTTLE,
    RING_CENTER_Z,
    SEAT_Z,
)

CONVEYOR_CENTER = {1: (3.0, 1.2), 2: (3.0, 4.8)}
STATION_YAW = {1: 0.0, 2: math.pi}


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def local_to_world(station: int, local_x: float, local_y: float, z: float = 0.0) -> tuple[float, float, float]:
    """Station local +x follows belt travel. Station B is yaw pi, so +x is −world x."""
    cx, cy = CONVEYOR_CENTER[station]
    if station == 1:
        return cx + local_x, cy + local_y, z
    if station == 2:
        return cx - local_x, cy - local_y, z
    raise ValueError(station)


def world_to_local(station: int, x: float, y: float) -> tuple[float, float]:
    cx, cy = CONVEYOR_CENTER[station]
    if station == 1:
        return x - cx, y - cy
    if station == 2:
        return cx - x, cy - y
    raise ValueError(station)


def belt_velocity(station: int, speed: float) -> tuple[float, float, float]:
    if station == 1:
        return speed, 0.0, 0.0
    if station == 2:
        return -speed, 0.0, 0.0
    raise ValueError(station)


def bottle_local_x(arrival_t: float, t: float, speed: float) -> float:
    return ENTRY_X + speed * (t - arrival_t)


def kit_center_local(bottle_local_x_m: float) -> tuple[float, float]:
    # Kit frame is aligned with the station frame, so bottle local = kit local + offset.
    return bottle_local_x_m - BOTTLE_OFFSET[0], 0.0


def bottle_world(station: int, bottle_local_x_m: float) -> tuple[float, float, float]:
    return local_to_world(station, bottle_local_x_m, BOTTLE_OFFSET[1], KIT_TOP_Z)


def ring_world(station: int, bottle_local_x_m: float, z: float = RING_CENTER_Z) -> tuple[float, float, float]:
    return local_to_world(station, bottle_local_x_m + RING_AHEAD_OF_BOTTLE, BOTTLE_OFFSET[1], z)


def seat_world(station: int, bottle_local_x_m: float) -> tuple[float, float, float]:
    return local_to_world(station, bottle_local_x_m, BOTTLE_OFFSET[1], KIT_TOP_Z + SEAT_Z)


def kit_world(station: int, bottle_local_x_m: float) -> tuple[float, float, float, float]:
    local_x, local_y = kit_center_local(bottle_local_x_m)
    x, y, z = local_to_world(station, local_x, local_y, KIT_TOP_Z)
    return x, y, z, STATION_YAW[station]


def inspection_delay(speed: float) -> float:
    return (INSPECT_X - ENTRY_X) / speed


def yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)
