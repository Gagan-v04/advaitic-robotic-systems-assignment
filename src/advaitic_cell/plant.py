"""Kinematic belt carriers, feeder, and the customer measurement bridge.

The robot never writes accepted counts. Seating is computed from body poses,
contacts and finite-difference velocity, then handed to the line controller.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from advaitic_cell.feeder import Feeder
from advaitic_cell.frames import (
    belt_velocity,
    bottle_local_x,
    bottle_world,
    inspection_delay,
    kit_world,
    ring_world,
    yaw_quat,
)
from advaitic_cell.scene import ModelIndex, SlotIndex
from advaitic_cell.spec import (
    DT,
    ENTRY_X,
    EXIT_X,
    HOLD_S,
    INSPECT_X,
    KIT_TOP_Z,
    REL_ANG_MAX,
    REL_LIN_MAX,
    RING_CENTER_Z,
)
from advaitic_systems.contract import inspection_passes
from advaitic_systems.modbus import PLCServer


@dataclass
class Carrier:
    slot: int
    station: int
    kit_id: int
    arrival_t: float
    speed: float
    arrived: bool = False
    inspected: bool = False
    seat_logged: bool = False
    hold: float = 0.0
    history: list = field(default_factory=list)
    samples: list[dict] = field(default_factory=list)


class Plant:
    def __init__(self, model, data, index: ModelIndex, server: PLCServer, speed: float, phase: float, log):
        self.model = model
        self.data = data
        self.index = index
        self.server = server
        self.speed = speed
        self.log = log
        self.feeders = {
            1: Feeder(1, speed, phase, 101),
            2: Feeder(2, speed, phase, 201),
        }
        self.carriers: dict[int, Carrier] = {}
        self.by_kit: dict[int, Carrier] = {}
        self.hold_log: dict[int, list[dict]] = {}
        self.free = {1: list(range(6)), 2: list(range(6, 12))}
        for slot in index.slots:
            self._park(slot)

    def _park(self, slot: SlotIndex) -> None:
        self.data.mocap_pos[slot.mocap] = (-4.0 - 0.4 * slot.slot, -3.0, -2.0)
        self.data.mocap_quat[slot.mocap] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[slot.ring_q:slot.ring_q + 3] = self.data.mocap_pos[slot.mocap]
        self.data.qpos[slot.ring_q + 3:slot.ring_q + 7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qvel[slot.ring_v:slot.ring_v + 6] = 0.0
        self.data.eq_active[slot.equality] = 0
        if 0 <= slot.equality < self.model.neq:
            self.model.eq_data[slot.equality, 10] = 1.0
        for geom in slot.geoms:
            self.model.geom_contype[geom] = 0
            self.model.geom_conaffinity[geom] = 0

    def _enable(self, slot: SlotIndex) -> None:
        for geom, contype, affinity in zip(slot.geoms, slot.contype, slot.conaffinity, strict=True):
            self.model.geom_contype[geom] = contype
            self.model.geom_conaffinity[geom] = affinity

    def accepted(self, station: int) -> int:
        with self.server.lock:
            return int(self.server.model.accepted[station])

    def ready(self, station: int) -> bool:
        with self.server.lock:
            flag = bool(self.server.model.ready[station])
            if station == 2 and self.server.model.accepted[1] < 5:
                return False
            return flag

    def slot_for(self, kit_id: int) -> int | None:
        carrier = self.by_kit.get(kit_id)
        return None if carrier is None else carrier.slot

    def drive(self, t: float) -> None:
        for station, feeder in self.feeders.items():
            if not self.free[station]:
                continue
            released = feeder.try_release(t, self.ready(station), self.accepted(station))
            if released is not None:
                self._spawn(t, station, *released)
        active = {carrier.slot for carrier in self.carriers.values()}
        for slot in self.index.slots:
            if slot.slot in active:
                continue
            park = self.data.mocap_pos[slot.mocap]
            self.data.qpos[slot.ring_q:slot.ring_q + 3] = park
            self.data.qvel[slot.ring_v:slot.ring_v + 6] = 0.0
        for carrier in self.carriers.values():
            if carrier.inspected and bottle_local_x(carrier.arrival_t, t, carrier.speed) > 1.05:
                continue
            self._place_kit(carrier, t)

    def measure(self, t: float) -> None:
        done = []
        for kit_id, carrier in self.carriers.items():
            local_x = bottle_local_x(carrier.arrival_t, t, carrier.speed)
            if not carrier.arrived and local_x >= ENTRY_X - 1e-9:
                carrier.arrived = True
                with self.server.lock:
                    self.server.model.arrive(carrier.station, carrier.kit_id)
                self.log.add({
                    "type": "kit_arrived", "t": t, "station": carrier.station,
                    "kit_id": carrier.kit_id, "belt_speed_mps": carrier.speed,
                })
            if carrier.arrived and not carrier.inspected:
                self._update_hold(carrier, t)
            if carrier.arrived and not carrier.inspected and local_x >= INSPECT_X:
                self._inspect(carrier, t)
            if carrier.inspected and local_x > EXIT_X + 0.8:
                done.append(kit_id)
        for kit_id in done:
            carrier = self.carriers.pop(kit_id)
            self.by_kit.pop(kit_id, None)
            self._park(self.index.slots[carrier.slot])
            self.hold_log[kit_id] = carrier.samples
            self.free[carrier.station].append(carrier.slot)

    def flush_holds(self) -> None:
        for kit_id, carrier in self.carriers.items():
            self.hold_log[kit_id] = carrier.samples

    def grasped(self, slot: int) -> bool:
        return bool(self.data.eq_active[self.index.slots[slot].equality])

    def closest_ring(self) -> tuple[int, float] | None:
        """Fingertip-to-ring distance for rings still on the line. None if the cell is empty."""
        best: tuple[int, float] | None = None
        for carrier in self.carriers.values():
            if carrier.inspected:
                continue
            distance = self.proximity(carrier.slot)
            if best is None or distance < best[1]:
                best = (carrier.slot, distance)
        return best

    def proximity(self, slot: int) -> float:
        """Minimum finger-to-ring distance. Negative means the shapes overlap."""
        slot_index = self.index.slots[slot]
        ring_geoms = [g for g in slot_index.geoms if (mujoco.mj_id2name(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("ring_")]
        finger_bodies = {
            self.index.tip_right, self.index.tip_left,
        }
        best = 1.0
        fromto = np.zeros(6)
        for geom in range(self.model.ngeom):
            if int(self.model.geom_bodyid[geom]) not in finger_bodies:
                continue
            for ring in ring_geoms:
                dist = mujoco.mj_geomDistance(self.model, self.data, geom, ring, 0.2, fromto)
                best = min(best, float(dist))
        return best

    def _spawn(self, t: float, station: int, kit_id: int, arrival_t: float) -> None:
        if not self.free[station]:
            return
        slot_id = self.free[station].pop(0)
        slot = self.index.slots[slot_id]
        self._enable(slot)
        carrier = Carrier(slot_id, station, kit_id, arrival_t, self.speed)
        self.carriers[kit_id] = carrier
        self.by_kit[kit_id] = carrier
        self._place_kit(carrier, t)
        point = ring_world(station, bottle_local_x(arrival_t, t, self.speed), RING_CENTER_Z + 0.002)
        self.data.qpos[slot.ring_q:slot.ring_q + 3] = point
        self.data.qpos[slot.ring_q + 3:slot.ring_q + 7] = (1.0, 0.0, 0.0, 0.0)
        velocity = belt_velocity(station, self.speed)
        self.data.qvel[slot.ring_v:slot.ring_v + 3] = velocity
        self.data.qvel[slot.ring_v + 3:slot.ring_v + 6] = 0.0
        carrier.history.clear()
        carrier.history.append((t, np.array(point, dtype=float), np.eye(3)))
        mujoco.mj_forward(self.model, self.data)

    def _place_kit(self, carrier: Carrier, t: float) -> None:
        slot = self.index.slots[carrier.slot]
        local_x = bottle_local_x(carrier.arrival_t, t, carrier.speed)
        x, y, z, yaw = kit_world(carrier.station, local_x)
        self.data.mocap_pos[slot.mocap] = (x, y, z)
        self.data.mocap_quat[slot.mocap] = yaw_quat(yaw)

    def _inspect(self, carrier: Carrier, t: float) -> None:
        measured = self._measurement(carrier, t, for_check=True)
        with self.server.lock:
            self.server.model.inspect(carrier.station, carrier.kit_id, measured)
        self.feeders[carrier.station].mark_inspected(carrier.kit_id)
        carrier.inspected = True
        self.log.add({
            "type": "inspection", "t": t, "station": carrier.station, "kit_id": carrier.kit_id,
            "belt_speed_mps": carrier.speed, **event_fields(measured),
        })
        print(
            f"inspect kit {carrier.kit_id} t={t:.2f} dx={measured['dx_m']:+.4f} "
            f"dy={measured['dy_m']:+.4f} z={measured['z_m']:.4f} hold={carrier.hold:.2f} "
            f"bottle={measured['bottle_contact']} kit={measured['kit_contact']} "
            f"released={measured['released']} tilt={measured['tilt_rad']:.3f}",
            flush=True,
        )
        # The crossing should sit on the customer's 0.85 m / speed delay.
        expected = carrier.arrival_t + inspection_delay(carrier.speed)
        carrier.samples.append({"t": t, "expected_inspection_t": expected})

    def _update_hold(self, carrier: Carrier, t: float) -> None:
        measured = self._measurement(carrier, t, for_check=False)
        settled = measured["rel_lin"] <= REL_LIN_MAX and measured["rel_ang"] <= REL_ANG_MAX
        geometry_ok = inspection_passes({**measured, "hold_s": HOLD_S})
        if geometry_ok and settled and not self.grasped(carrier.slot):
            carrier.hold += DT
        else:
            carrier.hold = 0.0
        carrier.samples.append({
            "t": round(t, 4),
            "dt": DT,
            "geom_ok": bool(geometry_ok and settled and not self.grasped(carrier.slot)),
            "rel_lin": round(measured["rel_lin"], 6),
            "rel_ang": round(measured["rel_ang"], 6),
            "hold": round(carrier.hold, 4),
            "dx_m": round(measured["dx_m"], 6),
            "dy_m": round(measured["dy_m"], 6),
            "z_m": round(measured["z_m"], 6),
        })
        if carrier.hold >= HOLD_S and not carrier.seat_logged:
            carrier.seat_logged = True
            seated = event_fields(self._measurement(carrier, t, for_check=True))
            seated["hold_s"] = float(carrier.hold)
            self.log.add({
                "type": "seat_verified", "t": t, "station": carrier.station,
                "kit_id": carrier.kit_id, **seated,
            })
            print(
                f"seat kit {carrier.kit_id} t={t:.2f} dx={seated['dx_m']:+.4f} "
                f"dy={seated['dy_m']:+.4f} z={seated['z_m']:.4f}",
                flush=True,
            )

    def _measurement(self, carrier: Carrier, t: float, for_check: bool) -> dict:
        slot = self.index.slots[carrier.slot]
        ring = self.data.xpos[slot.ring_body].copy()
        mat = self.data.xmat[slot.ring_body].reshape(3, 3).copy()
        local_x = bottle_local_x(carrier.arrival_t, t, carrier.speed)
        base = np.array(bottle_world(carrier.station, local_x))
        # Bottle frame yaw matches the station frame.
        yaw = 0.0 if carrier.station == 1 else np.pi
        cosine, sine = np.cos(yaw), np.sin(yaw)
        delta = ring - base
        dx = cosine * delta[0] + sine * delta[1]
        dy = -sine * delta[0] + cosine * delta[1]
        # 50 ms window. A single 4 ms contact spike is not the ring leaving the bottle;
        # the position over this window is what "settled" means in the acceptance text.
        carrier.history.append((float(t), ring.copy(), mat.copy()))
        cutoff = float(t) - 0.05
        while len(carrier.history) > 2 and carrier.history[0][0] < cutoff:
            carrier.history.pop(0)
        t0, origin, previous = carrier.history[0]
        span = max(float(t) - t0, DT)
        velocity = (ring - origin) / span
        rel_lin = float(np.linalg.norm(velocity - np.array(belt_velocity(carrier.station, carrier.speed))))
        turned = previous.T @ mat
        rel_ang = math_acos(0.5 * (float(np.trace(turned)) - 1.0)) / span
        bottle_hit, kit_hit = self._contacts(slot)
        released = not self.grasped(carrier.slot)
        return {
            "dx_m": float(dx), "dy_m": float(dy), "z_m": float(ring[2] - KIT_TOP_Z),
            "tilt_rad": float(np.arccos(np.clip(mat[2, 2], -1.0, 1.0))),
            "hold_s": float(carrier.hold if not for_check else max(carrier.hold, 0.0)),
            "released": bool(released),
            "bottle_contact": bool(bottle_hit),
            "kit_contact": bool(kit_hit),
            "rel_lin": float(rel_lin),
            "rel_ang": float(rel_ang),
        }

    def _contacts(self, slot: SlotIndex) -> tuple[bool, bool]:
        bottle = kit = False
        ring_body = slot.ring_body
        for contact in range(self.data.ncon):
            geom1 = int(self.data.contact[contact].geom1)
            geom2 = int(self.data.contact[contact].geom2)
            bodies = {int(self.model.geom_bodyid[geom1]), int(self.model.geom_bodyid[geom2])}
            if ring_body not in bodies:
                continue
            other = (bodies - {ring_body}).pop() if len(bodies) == 2 else ring_body
            if other == slot.bottle_body:
                bottle = True
            elif other == slot.kit_body:
                kit = True
        return bottle, kit


def math_acos(value: float) -> float:
    return float(np.arccos(np.clip(value, -1.0, 1.0)))


_EVENT_FIELDS = (
    "dx_m", "dy_m", "z_m", "tilt_rad", "hold_s", "released", "bottle_contact", "kit_contact",
)


def event_fields(measured: dict) -> dict:
    return {key: measured[key] for key in _EVENT_FIELDS}
