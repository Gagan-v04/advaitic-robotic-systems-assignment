"""Customer-spec accessors and reproducible scenario parameters."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC_PATH = ROOT / "spec" / "customer_cell.json"
ROBOT_DIR = ROOT / "third_party" / "mujoco_menagerie" / "google_robot"
ROBOT_XML = ROBOT_DIR / "robot.xml"

SPEC = json.loads(SPEC_PATH.read_text(encoding="utf-8"))

DT = 0.004
SAMPLE_DT = 0.1
PITCH_M = float(SPEC["conveyor"]["kit_pitch_m"])
ENTRY_X = float(SPEC["conveyor"]["work_entry_local_x_m"])
EXIT_X = float(SPEC["conveyor"]["work_exit_local_x_m"])
INSPECT_X = float(SPEC["conveyor"]["inspection_local_x_m"])
BELT_TOP_Z = 0.75
KIT_TOP_Z = 0.775
RING_CENTER_Z = KIT_TOP_Z + 0.004
BOTTLE_OFFSET = tuple(SPEC["kit"]["bottle_xy_in_kit_m"])
RING_OFFSET = tuple(SPEC["kit"]["ring_xy_in_kit_m"])
# Station-local ring x minus bottle x when the kit frame matches the station frame.
RING_AHEAD_OF_BOTTLE = float(RING_OFFSET[0] - BOTTLE_OFFSET[0])
SEAT_Z = 0.133
HOLD_S = float(SPEC["acceptance"]["settled_hold_s"])
RADIAL_MAX = float(SPEC["acceptance"]["radial_error_max_m"])
TILT_MAX = float(SPEC["acceptance"]["tilt_max_rad"])
REL_LIN_MAX = 0.01
REL_ANG_MAX = 0.1
DOCK_POS_TOL = float(SPEC["robot"]["dock_position_tolerance_m"])
DOCK_YAW_TOL = float(SPEC["robot"]["dock_yaw_tolerance_rad"])
MAX_TRANSIT = float(SPEC["robot"]["max_transit_speed_mps"])
MAX_YAW_RATE = float(SPEC["robot"]["max_yaw_rate_radps"])
CYCLE_LIMIT = float(SPEC["timing"]["assembly_target_s"])
MISSION_LIMIT = float(SPEC["timing"]["nominal_mission_target_s"])
FAULT_LIMIT = 360.0


def station_key(station: int) -> str:
    if station == 1:
        return "A"
    if station == 2:
        return "B"
    raise ValueError(f"unknown station {station}")


def dock_pose(station: int) -> tuple[float, float, float]:
    raw = SPEC["stations"][station_key(station)]["dock_world"]
    return float(raw[0]), float(raw[1]), float(raw[2])


def start_pose() -> tuple[float, float, float]:
    raw = SPEC["start_pose_world"]
    return float(raw[0]), float(raw[1]), float(raw[2])


def _unit(seed: int, stream: int) -> float:
    """Stable unit interval. Independent of NumPy's RNG stream."""
    value = (seed + 1) * 1_103_515_245 + (stream + 1) * 12_345
    value = (value * 1_664_525 + 1_013_904_223) & 0xFFFFFFFF
    return value / 4_294_967_296


@dataclass(frozen=True)
class RunParams:
    scenario: str
    seed: int
    belt_speed_mps: float
    release_phase_s: float
    human_duration_s: float
    oracle_perception: bool = False

    @property
    def mission_limit_s(self) -> float:
        return MISSION_LIMIT if self.scenario == "nominal" else FAULT_LIMIT


def scenario_params(
    scenario: str,
    seed: int,
    human_duration: float = 3.0,
    oracle_perception: bool = False,
) -> RunParams:
    if scenario not in {"nominal", "human_stop", "plc_retry"}:
        raise ValueError(f"unknown scenario {scenario}")
    if scenario != "nominal" and seed != 0:
        # Fault scenarios are specified at seed 0. Other seeds still run,
        # but the published commands use seed 0.
        pass
    if not 2.0 <= human_duration <= 4.0:
        raise ValueError("human duration must be inside 2–4 s")
    speed = round(0.025 + 0.010 * _unit(seed, 1), 6)
    phase = round(_unit(seed, 2), 4)
    return RunParams(scenario, seed, speed, phase, human_duration, oracle_perception)
