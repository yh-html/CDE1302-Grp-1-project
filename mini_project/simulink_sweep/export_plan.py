#!/usr/bin/env python3
"""
Export the real mission plan to plan.json so the MATLAB/Simulink sweep simulation
flies exactly the waypoints the mission would send.

Reads laptop/tag_mission.yaml (search area, lane spacing, camera, detection and
mission settings) and calls the mission's own plan_sweep().

    python mini_project/simulink_sweep/export_plan.py
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
LAPTOP = HERE.parents[1] / "laptop"
sys.path.insert(0, str(LAPTOP))

from mission_config import load_config          # noqa: E402
from sweep_planner import path_length, plan_sweep  # noqa: E402

# The six possible tag spots in "CDE1302-Mini Project-Flying Area.pdf": a 3 x 2 grid
# 0.45 m inside the search area edges. Derived from the diagram, not measured.
TAG_SPOTS_X = [1.35, 2.70, 4.05]
TAG_SPOTS_Y = [-1.35, 0.90]


def main() -> None:
    cfg = load_config(LAPTOP / "tag_mission.yaml")
    start = (0.0, 0.0)
    plan = plan_sweep(cfg.area, cfg.lane_spacing_m, cfg.edge_margin_m, start, cfg.lane_axis)
    out = {
        "source": "laptop/tag_mission.yaml via sweep_planner.plan_sweep",
        "start": list(start),
        "waypoints": [list(p) for p in plan],
        "path_length_m": path_length(plan, start, return_to_start=True),
        "search_area": {"min_x": cfg.area.min_x, "max_x": cfg.area.max_x,
                        "min_y": cfg.area.min_y, "max_y": cfg.area.max_y},
        "altitude_m": cfg.altitude_m,
        "cruise_speed_mps": cfg.cruise_speed_mps,
        "arrival_radius_m": cfg.arrival_radius_m,
        "takeoff_wait_s": cfg.takeoff_wait_s,
        "land_wait_s": cfg.land_wait_s,
        "camera": {"pitch_deg": cfg.camera.pitch_deg,
                   "fwd_offset_m": cfg.camera.fwd_offset_m,
                   "right_offset_m": cfg.camera.right_offset_m,
                   "down_offset_m": cfg.camera.down_offset_m,
                   "hfov_deg": cfg.camera.hfov_deg, "vfov_deg": cfg.camera.vfov_deg},
        "detection": {"max_range_m": cfg.max_range_m,
                      "min_view_elevation_deg": cfg.min_view_elevation_deg},
        "tag_spots": [[x, y] for y in TAG_SPOTS_Y for x in TAG_SPOTS_X],
        # 0.9 x 0.9 m launch square centred on the launch point (Flying Area PDF).
        # The room outline is not drawn: the PDF diagram is not to scale.
        "launch_square": {"min_x": -0.45, "max_x": 0.45, "min_y": -0.45, "max_y": 0.45},
    }
    path = HERE / "plan.json"
    path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(f"Wrote {path}: {len(plan)} waypoints, {out['path_length_m']:.1f} m incl. return")


if __name__ == "__main__":
    main()
