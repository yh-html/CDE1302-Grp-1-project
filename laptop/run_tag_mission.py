#!/usr/bin/env python3
"""
CDE1302 mini-project: autonomous AprilTag search mission (single drone).

    takeoff command -> sweep the search area -> log tags (ID, position, pose,
    timestamp) -> return to the start -> landing command

All tunable parameters are in laptop/tag_mission.yaml.  Coordinates are the
drone's local frame: origin at the takeoff point, x = North, y = East.

Examples (from the repo root):

    # Check the plan without flying or connecting
    python laptop/run_tag_mission.py --drone-id 22 --plan-only

    # Full dry run with the built-in simulator (no drone needed)
    python laptop/run_tag_mission.py --drone-id 22 --simulate

    # Real drone: asks for TAKEOFF-22 before sending the takeoff command
    python laptop/run_tag_mission.py --drone-id 22

Outputs go to <output_dir>/<time>_drone<ID>/ (events.csv, observations.csv,
tags.csv, trajectory.csv, summary.json).

Exit code: 0 completed (or cancelled), 1 aborted (stale telemetry, geofence,
error), 130 Ctrl+C.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from datetime import datetime
from pathlib import Path

from drone_link import RealClock, SimClock, SimLink, UdpLink
from mission_config import load_config
from mission_log import MissionLog
from protocol import AT_DEBUG_PORT
from sweep_planner import (path_length, plan_sweep, suggest_lane_spacing_m,
                           swath_halfwidth_m)
from tag_mission import TagMission

HERE = Path(__file__).resolve().parent


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Autonomous AprilTag search mission for one drone.")
    p.add_argument("--drone-id", type=int, required=True,
                   help="Flashed ESP32 CONFIG_DRONE_ID.")
    p.add_argument("--config", default=str(HERE / "tag_mission.yaml"),
                   help="Mission config (default: laptop/tag_mission.yaml).")
    p.add_argument("--simulate", action="store_true",
                   help="Run against the built-in simulator instead of a drone.")
    p.add_argument("--plan-only", action="store_true",
                   help="Print the sweep plan and exit; no connection, no flight.")
    p.add_argument("--yes", action="store_true",
                   help="Skip the TAKEOFF-<id> confirmation prompt.")
    p.add_argument("--out-dir", help="Override output_dir from the config.")
    p.add_argument("--telem-port", type=int, default=5005)
    p.add_argument("--cmd-port", type=int, default=5006)
    p.add_argument("--at-port", type=int, default=AT_DEBUG_PORT)
    return p.parse_args(argv)


def make_confirm(drone_id: int):
    word = f"TAKEOFF-{drone_id}"

    def confirm() -> bool:
        typed = input(f"Type {word} to send the autonomous takeoff command: ").strip()
        return typed == word
    return confirm


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = load_config(args.config)

    if args.plan_only:
        plan = plan_sweep(cfg.area, cfg.lane_spacing_m, cfg.edge_margin_m,
                          (0.0, 0.0), cfg.lane_axis)
        print(f"Search area: {cfg.area}")
        for i, (x, y) in enumerate(plan, 1):
            print(f"  {i:2d}: x={x:6.2f}  y={y:6.2f}")
        print("  then return to (0.00, 0.00) and land")
        flown = path_length(plan, (0.0, 0.0), return_to_start=True)
        half = swath_halfwidth_m(cfg.camera, cfg.altitude_m, cfg.max_range_m,
                                 cfg.min_view_elevation_deg)
        print(f"\nPlan: {len(plan)} waypoints, {flown:.1f} m flown including the return.")
        print("Estimated mission time (takeoff wait + flight + landing wait) by cruise speed:")
        for v in (0.1, 0.2, 0.3, 0.5):
            mark = "  <- configured" if abs(v - cfg.cruise_speed_mps) < 1e-9 else ""
            print(f"  {v:.1f} m/s: {cfg.takeoff_wait_s + flown / v + cfg.land_wait_s:5.0f} s{mark}")
        print(f"\nCamera sees floor tags up to {half:.2f} m either side of the flight line "
              f"(pitch {cfg.camera.pitch_deg:g} deg, FOV {cfg.camera.hfov_deg:g}x{cfg.camera.vfov_deg:g} deg, "
              f"altitude {cfg.altitude_m:g} m, range limit {cfg.max_range_m:g} m, "
              f"min viewing angle {cfg.min_view_elevation_deg:g} deg above the floor).")
        print(f"Largest gap-free lane spacing: {2 * half:.2f} m; suggested (80%): "
              f"{suggest_lane_spacing_m(half):.2f} m; configured: {cfg.lane_spacing_m:g} m.")
        print("ASSUMES tags lie flat on the floor. Range and viewing angle stand in for 'tag decodable':")
        print("verify max_range_m and min_view_elevation_deg with a real tag.")
        return 0

    if args.simulate:
        clock = SimClock()
        link = SimLink(args.drone_id, clock, cfg.camera, cfg.sim.tags,
                       speed_mps=cfg.cruise_speed_mps, takeoff_s=cfg.sim.takeoff_s,
                       altitude_m=cfg.altitude_m, hfov_deg=cfg.camera.hfov_deg,
                       vfov_deg=cfg.camera.vfov_deg, max_range_m=cfg.sim.max_range_m,
                       min_elevation_deg=cfg.min_view_elevation_deg,
                       noise_m=cfg.sim.noise_m)
    else:
        clock = RealClock()
        link = UdpLink(args.drone_id, args.telem_port, args.cmd_port, args.at_port)

    out_root = Path(args.out_dir or cfg.output_dir)
    if not out_root.is_absolute():
        out_root = HERE / out_root
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = out_root / f"{stamp}_drone{args.drone_id}{'_sim' if args.simulate else ''}"

    mlog = MissionLog(out_dir, wall=clock.wall, now=clock.now)
    confirm = None if (args.yes or args.simulate) else make_confirm(args.drone_id)
    mission = TagMission(cfg, link, clock, mlog, drone_id=args.drone_id, confirm=confirm)

    def on_sigint(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, on_sigint)

    print(f"Output folder: {out_dir}", flush=True)
    return mission.run()


if __name__ == "__main__":
    sys.exit(main())
