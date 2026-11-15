"""Extra checks that sit beside the starter checker. They do not change it."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from advaitic_cell.spec import DT
from advaitic_systems.evidence import check


def _load_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _recompute_hold(samples: list[dict], until: float) -> float:
    hold = 0.0
    for sample in samples:
        if float(sample["t"]) > until + 1e-9:
            break
        if sample["geom_ok"]:
            hold += float(sample["dt"])
        else:
            hold = 0.0
    return hold


def extra_checks(events: list[dict], directory: Path) -> list[str]:
    errors = []
    trace_path = directory / "trace.json"
    holds_path = directory / "holds.json"
    if not trace_path.exists() or not holds_path.exists():
        return ["missing trace.json or holds.json"]
    trace = json.loads(trace_path.read_text(encoding="utf-8"))
    holds = json.loads(holds_path.read_text(encoding="utf-8"))
    if trace.get("oracle") is True:
        errors.append("oracle perception was enabled for a scored run")
    if int(trace.get("belt_contacts", 0)) != 0:
        errors.append("robot geometry intersected a conveyor")
    # min_clearance_m is the wrist site versus the belt top. The site is not a
    # collision body; belt_contacts is the contact check.
    for grasp in trace.get("grasps", []):
        if grasp.get("engaged") and float(grasp["distance_m"]) > 0.03:
            errors.append("grasp constraint armed while the ring was more than 3 cm away")
    declared = float(trace["belt_speed_mps"])
    arrivals = [event for event in events if event["type"] == "kit_arrived"]
    if any(abs(float(event["belt_speed_mps"]) - declared) > 1e-6 for event in arrivals):
        errors.append("logged belt speed does not match the plant speed")
    speeds = []
    previous = None
    for sample in trace.get("belt_samples", []):
        if previous and sample["kit_id"] == previous["kit_id"]:
            dt = float(sample["t"]) - float(previous["t"])
            if dt > 0.2:
                speeds.append((float(sample["local_x"]) - float(previous["local_x"])) / dt)
        previous = sample
    if speeds and any(abs(speed - declared) > 0.002 for speed in speeds):
        errors.append("live belt speed drifted from the declared test speed")
    done = {int(event["kit_id"]) for event in events if event["type"] == "assembly_done"}
    for kit_id in trace.get("abandoned", []):
        if int(kit_id) in done:
            errors.append(f"abandoned kit {kit_id} was also marked assembled")
    for event in events:
        if event["type"] != "seat_verified":
            continue
        samples = holds.get(str(event["kit_id"]))
        if not samples:
            errors.append(f"no hold samples for kit {event['kit_id']}")
            continue
        recomputed = _recompute_hold(samples, float(event["t"]))
        if abs(recomputed - float(event["hold_s"])) > 2 * DT:
            errors.append(f"hold samples do not reproduce hold_s for kit {event['kit_id']}")
        if float(event["hold_s"]) < 0.5:
            errors.append(f"seat hold under 0.5 s for kit {event['kit_id']}")
    return errors


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events", type=Path)
    parser.add_argument("--scenario", choices=["nominal", "human_stop", "plc_retry"], default="nominal")
    args = parser.parse_args(argv)
    events = _load_jsonl(args.events)
    result = check(events, args.scenario)
    result["extra"] = extra_checks(events, args.events.parent)
    if result["extra"]:
        result["passed"] = False
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
