"""
Loads tag_mission.yaml into typed, validated settings.

Every tunable parameter of the tag-search mission lives in that file (the brief
asks for parameters in configuration files); nothing here is hard-coded except
defaults used when a key is omitted.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from sweep_planner import Area
from tag_localizer import CameraGeometry


@dataclass(frozen=True)
class SimSettings:
    """Used only with --simulate."""
    tags: dict = field(default_factory=dict)   # id -> (x, y), local frame
    noise_m: float = 0.0
    takeoff_s: float = 8.0
    max_range_m: float = 4.0


@dataclass(frozen=True)
class MissionConfig:
    area: Area
    lane_spacing_m: float
    edge_margin_m: float
    lane_axis: str
    camera: CameraGeometry
    max_pose_err: float
    max_range_m: float
    min_view_elevation_deg: float
    detection_latency_s: float
    update_threshold_m: float
    expected_tags: int | None
    stop_when_all_found: bool
    cruise_speed_mps: float
    altitude_m: float
    takeoff_wait_s: float
    arrival_radius_m: float
    min_leg_timeout_s: float
    max_mission_s: float
    geofence_margin_m: float
    stale_timeout_s: float
    connect_timeout_s: float
    land_wait_s: float
    max_consecutive_misses: int
    output_dir: str
    sim: SimSettings


def _num(section: dict, key: str, default: float, positive: bool = False,
         minimum: float | None = None) -> float:
    v = section.get(key, default)
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"{key} must be a number, got {v!r}") from None
    if positive and v <= 0:
        raise ValueError(f"{key} must be positive, got {v}")
    if minimum is not None and v < minimum:
        raise ValueError(f"{key} must be >= {minimum}, got {v}")
    return v


def parse_config(cfg: dict) -> MissionConfig:
    a = cfg.get("search_area")
    if not isinstance(a, dict):
        raise ValueError("search_area (min_x, max_x, min_y, max_y) is required")
    try:
        area = Area(float(a["min_x"]), float(a["max_x"]),
                    float(a["min_y"]), float(a["max_y"]))
    except KeyError as exc:
        raise ValueError(f"search_area is missing {exc}") from None

    sweep = cfg.get("sweep", {}) or {}
    cam = cfg.get("camera", {}) or {}
    det = cfg.get("detection", {}) or {}
    mis = cfg.get("mission", {}) or {}
    sim = cfg.get("simulation", {}) or {}

    lane_axis = str(sweep.get("lane_axis", "x"))
    if lane_axis not in ("x", "y"):
        raise ValueError("sweep.lane_axis must be 'x' or 'y'")

    expected = det.get("expected_tags")
    if expected is not None:
        expected = int(expected)
        if expected < 1:
            raise ValueError("detection.expected_tags must be >= 1 or omitted")

    tags = {int(k): (float(v[0]), float(v[1]))
            for k, v in (sim.get("tags", {}) or {}).items()}

    return MissionConfig(
        area=area,
        lane_spacing_m=_num(sweep, "lane_spacing_m", 1.0, positive=True),
        edge_margin_m=_num(sweep, "edge_margin_m", 0.3, minimum=0.0),
        lane_axis=lane_axis,
        camera=CameraGeometry(
            pitch_deg=_num(cam, "pitch_deg", 45.0),
            fwd_offset_m=_num(cam, "fwd_offset_m", 0.0),
            right_offset_m=_num(cam, "right_offset_m", 0.0),
            down_offset_m=_num(cam, "down_offset_m", 0.0),
            hfov_deg=_num(cam, "hfov_deg", 91.0, positive=True),
            vfov_deg=_num(cam, "vfov_deg", 82.0, positive=True),
        ),
        max_pose_err=_num(det, "max_pose_err", 0.5, positive=True),
        max_range_m=_num(det, "max_range_m", 3.0, positive=True),
        min_view_elevation_deg=_num(det, "min_view_elevation_deg", 30.0, minimum=0.0),
        detection_latency_s=_num(det, "latency_s", 0.0, minimum=0.0),
        update_threshold_m=_num(det, "update_threshold_m", 0.2, positive=True),
        expected_tags=expected,
        stop_when_all_found=bool(det.get("stop_when_all_found", False)),
        cruise_speed_mps=_num(mis, "cruise_speed_mps", 0.20, positive=True),
        altitude_m=_num(mis, "altitude_m", 0.5, positive=True),
        takeoff_wait_s=_num(mis, "takeoff_wait_s", 12.0, minimum=0.0),
        arrival_radius_m=_num(mis, "arrival_radius_m", 0.25, positive=True),
        min_leg_timeout_s=_num(mis, "min_leg_timeout_s", 15.0, positive=True),
        max_mission_s=_num(mis, "max_mission_s", 600.0, positive=True),
        geofence_margin_m=_num(mis, "geofence_margin_m", 0.5, minimum=0.0),
        stale_timeout_s=_num(mis, "stale_timeout_s", 3.0, positive=True),
        connect_timeout_s=_num(mis, "connect_timeout_s", 30.0, positive=True),
        land_wait_s=_num(mis, "land_wait_s", 8.0, minimum=0.0),
        max_consecutive_misses=int(_num(mis, "max_consecutive_misses", 3, minimum=1)),
        output_dir=str(mis.get("output_dir", "mission_logs")),
        sim=SimSettings(
            tags=tags,
            noise_m=_num(sim, "noise_m", 0.0, minimum=0.0),
            takeoff_s=_num(sim, "takeoff_s", 8.0, minimum=0.0),
            max_range_m=_num(sim, "max_range_m", 4.0, positive=True),
        ),
    )


def load_config(path: str | Path) -> MissionConfig:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"mission config not found: {p}")
    data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    return parse_config(data)
