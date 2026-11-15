"""Cartesian trajectories sampled every physics step.

IK in `arm.py` is a servo around these setpoints. It is not allowed to jump the
hand to a new target in one retarget. Minimum-jerk is used on committed vertical
segments (pick, lift, place, retreat). A trapezoid with acceleration limits
tracks moving belt targets in the other axes.
"""

from __future__ import annotations

import math

import numpy as np


def minjerk(tau: float) -> float:
    """Quintic 0→1 interpolant with zero velocity and acceleration at both ends."""
    tau = float(np.clip(tau, 0.0, 1.0))
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


def minjerk_duration(distance: float, vmax: float, amax: float) -> float:
    """Time for a min-jerk segment whose peak speed stays near `vmax`."""
    distance = abs(float(distance))
    if distance < 1e-6:
        return 0.05
    # Peak speed of min-jerk is 1.875 * d / T. Match vmax, then floor by accel.
    duration = max(0.18, 1.875 * distance / max(vmax, 1e-6))
    duration = max(duration, math.sqrt(distance / max(amax, 1e-6)))
    return duration


def _trap_axis(pos: float, vel: float, goal: float, dt: float, vmax: float, amax: float) -> tuple[float, float]:
    remaining = goal - pos
    if abs(remaining) < 1e-5 and abs(vel) < 1e-4:
        return goal, 0.0
    v_stop = math.copysign(math.sqrt(2.0 * amax * abs(remaining)), remaining) if remaining else 0.0
    v_des = float(np.clip(v_stop, -vmax, vmax))
    dv = float(np.clip(v_des - vel, -amax * dt, amax * dt))
    vel = vel + dv
    pos = pos + vel * dt
    if remaining * (goal - pos) <= 0.0 and abs(vel) < 0.08:
        return goal, 0.0
    return pos, vel


class CartesianServo:
    """World-frame hand setpoint. `step` must run at the physics rate."""

    def __init__(self, vmax_xy: float = 0.28, vmax_z: float = 0.26, amax: float = 1.8):
        self.vmax_xy = vmax_xy
        self.vmax_z = vmax_z
        self.amax = amax
        self.pos: np.ndarray | None = None
        self.vel = np.zeros(3)
        self.vertical = False
        self.z0 = 0.0
        self.z1 = 0.0
        self.seg_t = 0.0
        self.seg_T = 0.2

    def reset(self, point: np.ndarray) -> None:
        self.pos = np.asarray(point, dtype=float).copy()
        self.vel[:] = 0.0
        self.stop_vertical()

    def stop_vertical(self) -> None:
        self.vertical = False
        self.seg_t = 0.0

    def start_vertical(self, z_goal: float) -> None:
        if self.pos is None:
            return
        self.vertical = True
        self.z0 = float(self.pos[2])
        self.z1 = float(z_goal)
        self.seg_t = 0.0
        self.seg_T = minjerk_duration(self.z1 - self.z0, self.vmax_z, self.amax)
        self.vel[2] = 0.0

    def step(
        self, target: np.ndarray, dt: float, *, vertical: bool = False, z_floor: float | None = None,
    ) -> np.ndarray:
        target = np.asarray(target, dtype=float)
        if self.pos is None:
            self.reset(target)
            if vertical:
                self.start_vertical(float(target[2]))
            return self.pos.copy()
        if vertical and not self.vertical:
            self.start_vertical(float(target[2]))
        if not vertical and self.vertical:
            self.stop_vertical()
        self.pos[0], self.vel[0] = _trap_axis(
            float(self.pos[0]), float(self.vel[0]), float(target[0]), dt, self.vmax_xy, self.amax,
        )
        self.pos[1], self.vel[1] = _trap_axis(
            float(self.pos[1]), float(self.vel[1]), float(target[1]), dt, self.vmax_xy, self.amax,
        )
        z_goal = float(target[2])
        if not vertical and z_floor is not None:
            xy_err = math.hypot(float(target[0] - self.pos[0]), float(target[1] - self.pos[1]))
            if xy_err > 0.10:
                # Descend only as far as the keep-out. Holding the current Z
                # forever parked the unstow at stow height; dropping with no
                # floor sent the setpoint through the kit while the arm lagged.
                z_goal = max(z_goal, float(z_floor))
        if self.vertical:
            self.seg_t += dt
            tau = self.seg_t / max(self.seg_T, 1e-6)
            self.pos[2] = self.z0 + (self.z1 - self.z0) * minjerk(tau)
            self.vel[2] = 0.0
            if tau >= 1.0:
                self.pos[2] = self.z1
        else:
            self.pos[2], self.vel[2] = _trap_axis(
                float(self.pos[2]), float(self.vel[2]), z_goal, dt, self.vmax_z, self.amax,
            )
        return self.pos.copy()

    def error(self, target: np.ndarray) -> float:
        if self.pos is None:
            return 1.0
        return float(np.linalg.norm(self.pos - np.asarray(target, dtype=float)))

    def vertical_done(self, z_tol: float = 0.006) -> bool:
        if self.pos is None:
            return False
        if not self.vertical:
            return abs(float(self.pos[2]) - self.z1) < z_tol
        return self.seg_t >= self.seg_T - 1e-9 and abs(float(self.pos[2]) - self.z1) < z_tol
