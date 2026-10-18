#!/usr/bin/env python3
"""
Post-flight analysis for the CDE1302 mini-project report.

Combines the PX4 .ulg flight log with the tag-mission run folder and produces:

    flight_overview.png   top view: full path, start, landing, tag IDs and positions,
                          plus altitude and speed against time
    flight_3d.png         3-D path (with --3d)
    flight_summary.md     numbers and tables, with warnings
    flight_summary.json   the same, machine-readable

Examples (from the repo root):

    python laptop/analyze_flight.py --ulg log_12_2026-11-12.ulg \
        --run laptop/mission_logs/20261112_101500_drone22

    # with measured tag positions to get the localisation error
    python laptop/analyze_flight.py --ulg flight.ulg --run <run> \
        --truth 3:1.20,-0.40 --truth 5:2.60,0.85

    # before the .ulg is downloaded: telemetry only
    python laptop/analyze_flight.py --run <run>

Getting the .ulg: copy it from the flight controller's SD card (log/ folder),
or in QGroundControl use Analyze Tools > Log Download.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

from flight_data import load_run, load_ulg, sync_times
from flight_plots import analyze, plot_3d, plot_overview, write_report
from mission_config import parse_config

HERE = Path(__file__).resolve().parent


def parse_truth(items: list[str]) -> dict[int, tuple[float, float]]:
    out = {}
    for item in items:
        try:
            ident, pos = item.split(":")
            x, y = pos.split(",")
            out[int(ident)] = (float(x), float(y))
        except ValueError:
            raise SystemExit(f"--truth must look like ID:x,y (for example 3:1.2,-0.4), got {item!r}")
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Analyse a PX4 .ulg log and a tag-mission run.")
    p.add_argument("--ulg", help="PX4 flight log (.ulg).")
    p.add_argument("--run", help="Mission run folder from run_tag_mission.py.")
    p.add_argument("--config", default=str(HERE / "tag_mission.yaml"),
                   help="Mission config (camera geometry and search area for the plot).")
    p.add_argument("--truth", action="append", default=[],
                   help="Known tag position ID:x,y in the local frame (repeatable).")
    p.add_argument("--out", help="Output folder (default: <run>/analysis or ./flight_analysis).")
    p.add_argument("--sync", choices=["auto", "arm"], default="auto")
    p.add_argument("--arm-delay", type=float, default=2.0,
                   help="Assumed seconds from the takeoff command to arming (default 2).")
    p.add_argument("--offset-s", type=float,
                   help="Manual offset: t_ulog = t_mission + offset (overrides --sync).")
    p.add_argument("--3d", dest="plot3d", action="store_true", help="Also write flight_3d.png.")
    args = p.parse_args(argv)

    if not args.ulg and not args.run:
        p.error("give --ulg, --run, or both")

    flog = load_ulg(args.ulg) if args.ulg else None
    run = load_run(args.run) if args.run else None

    cam = area = None
    cfg_path = Path(args.config)
    if cfg_path.exists():
        cfg = parse_config(yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {})
        cam = cfg.camera
        area = (cfg.area.min_x, cfg.area.max_x, cfg.area.min_y, cfg.area.max_y)

    sync = None
    if flog is not None and run is not None:
        sync = sync_times(flog, run, mode=args.sync, arm_delay_s=args.arm_delay,
                          manual_offset_s=args.offset_s)
        print(f"Time alignment: {sync.method} ({sync.quality}) - {sync.detail}")

    out = Path(args.out) if args.out else (Path(args.run) / "analysis" if args.run
                                            else Path("flight_analysis"))
    out.mkdir(parents=True, exist_ok=True)

    result = analyze(flog, run, sync, camera=cam, truth=parse_truth(args.truth))
    plot_overview(result, out / "flight_overview.png", area=area)
    if args.plot3d:
        plot_3d(result, out / "flight_3d.png")
    write_report(result, out / "flight_summary.md", out / "flight_summary.json",
                 args.ulg, args.run)
    print(f"Wrote {out / 'flight_overview.png'}")
    print(f"Wrote {out / 'flight_summary.md'}")
    for n in result.notes:
        print(f"WARNING: {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
