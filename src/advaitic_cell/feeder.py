"""Kit release schedule. Belts never pause; this only decides when a kit is placed."""

from __future__ import annotations

from advaitic_cell.spec import ENTRY_X, PITCH_M


class Feeder:
    """Release kits after the station is ready, keeping accepted + in-flight <= 5."""

    def __init__(self, station: int, speed: float, phase_s: float, first_id: int):
        if station not in (1, 2):
            raise ValueError(station)
        if not 0.025 <= speed <= 0.035:
            raise ValueError(speed)
        self.station = station
        self.speed = speed
        self.phase_s = phase_s
        self.next_id = first_id
        self.ready_t: float | None = None
        self.next_arrival: float | None = None
        self.last_arrival: float | None = None
        self.in_flight: set[int] = set()

    def set_ready(self, t: float, ready: bool) -> None:
        if ready and self.ready_t is None:
            self.ready_t = t
            self.next_arrival = t + self.phase_s
        if not ready:
            # Leaving the station freezes the schedule; coming back starts a new batch
            # only if the caller constructs a fresh feeder. NOT_READY just stops releases.
            self.ready_t = None if self.next_arrival is None else self.ready_t

    def mark_inspected(self, kit_id: int) -> None:
        self.in_flight.discard(kit_id)

    def try_release(self, t: float, ready: bool, accepted: int) -> tuple[int, float] | None:
        if ready and self.ready_t is None:
            self.ready_t = t
            self.next_arrival = t + self.phase_s
        if not ready or self.next_arrival is None or self.ready_t is None:
            return None
        if accepted + len(self.in_flight) >= 5:
            return None
        interval = PITCH_M / self.speed
        earliest = self.ready_t if self.last_arrival is None else self.last_arrival + interval
        arrival = max(self.next_arrival, earliest, t)
        spawn_at = arrival - min(0.20, interval / 2)
        if t + 1e-9 < max(self.ready_t, spawn_at):
            return None
        # If we are already late, put the bottle on the entry line now.
        if t > arrival:
            arrival = max(t, earliest)
        kit_id = self.next_id
        self.next_id += 1
        self.in_flight.add(kit_id)
        self.last_arrival = arrival
        self.next_arrival = arrival + interval
        return kit_id, arrival

    @staticmethod
    def bottle_local_at(arrival_t: float, t: float, speed: float) -> float:
        return ENTRY_X + speed * (t - arrival_t)
