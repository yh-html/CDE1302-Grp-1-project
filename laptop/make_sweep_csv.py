#!/usr/bin/env python3
"""Write the smooth sweep path of a mission config as a t,x,y,z CSV (same format as trajectory/*.csv).

    python make_sweep_csv.py --config tag_mission_demo.yaml --speed 0.12 --out ../trajectory/tag_sweep_official.csv

Fly it standalone with laptop/send_trajectory.py, or let run_tag_mission.py do it (config `trajectory:` block),
which also logs tags and returns early.  dt 0.1 keeps the official sweep under the firmware's 2400-point limit.
"""
import argparse
from pathlib import Path

from mission_config import load_config
from sweep_planner import plan_sweep
from traj_gen import smooth_path, stats, write_csv

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=str(Path(__file__).with_name("tag_mission_demo.yaml")))
ap.add_argument("--speed", type=float, default=0.12)
ap.add_argument("--corner", type=float, default=0.35)
ap.add_argument("--accel", type=float, default=0.08)
ap.add_argument("--dt", type=float, default=0.1)
ap.add_argument("--out", required=True)
a = ap.parse_args()
cfg = load_config(a.config)
plan = plan_sweep(cfg.area, cfg.lane_spacing_m, cfg.edge_margin_m, (0.0, 0.0), cfg.lane_axis)
tr = smooth_path(plan, (0.0, 0.0), a.speed, a.corner, a.accel, dt=a.dt)
write_csv(a.out, tr)
s = stats(tr)
print(f"{a.out}: {s['n_points']} points, {s['duration_s']:.0f} s, {s['length_m']:.1f} m, "
      f"max speed {s['max_speed']:.2f} m/s, max accel {s['max_accel']:.2f} m/s^2")
