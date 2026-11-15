"""Headless cell runner. Writes evidence, then returns the starter checker's status."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from advaitic_cell.mission import Mission
from advaitic_cell.spec import ROOT, scenario_params
from advaitic_systems.evidence import check


def default_output(scenario: str, seed: int) -> Path:
    if scenario == "nominal":
        return ROOT / "results" / f"nominal-seed-{seed}"
    if scenario == "human_stop":
        return ROOT / "results" / "human-stop"
    return ROOT / "results" / "plc-retry"


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=["nominal", "human_stop", "plc_retry"], default="nominal")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--headless", action="store_true", help="No viewer. This is the only mode.")
    parser.add_argument("--duration", type=float, default=3.0, help="Human presence, 2–4 s.")
    parser.add_argument("--interrupt-at", default="transfer", help="Human stop is armed at ring transfer.")
    parser.add_argument("--oracle-perception", action="store_true",
                        help="Debug only. Scored logs must leave this off.")
    parser.add_argument("--record", action="store_true", help="Write demo.gif if OSMesa is available.")
    parser.add_argument(
        "--video", type=Path, default=None,
        help="Write a mosaic mp4 plus per-camera files under <dir>/views/.",
    )
    parser.add_argument("--fps", type=int, default=30, help="Video recording frame rate in fps (default: 30).")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--no-record", action="store_true", help="Skip demo.gif even on human_stop.")
    args = parser.parse_args(argv)
    if args.interrupt_at != "transfer":
        parser.error("only --interrupt-at transfer is implemented")
    params = scenario_params(args.scenario, args.seed, args.duration, args.oracle_perception)
    out = args.out or default_output(args.scenario, args.seed)
    mission = Mission(
        params, out,
        record=(args.record or args.scenario == "human_stop") and args.video is None and not args.no_record,
        video_path=args.video,
        video_fps=args.fps,
    )
    mission.trace["interrupt_at"] = args.interrupt_at
    summary = {}
    try:
        mission.run()
    finally:
        try:
            summary = mission.save()
        finally:
            mission.close()
    result = check(mission.log.events, params.scenario)
    print(json.dumps({"summary": summary, "check": result}, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
