"""
End-to-end tests of the tag-search mission against the simulator.

These prove the mission logic (sequencing, logging, safety, fusion).  They do
NOT prove real-world localisation accuracy: the simulator uses the same camera
geometry as the localiser, so a wrong measured mount offset would not show here.
"""

import csv
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_link import SimClock, SimLink
from mission_config import parse_config
from mission_log import MissionLog
from tag_mission import TagMission

TAGS = {1: (1.5, -1.0), 2: (2.5, 1.2), 3: (3.5, -0.4)}


def raw_cfg(**mission):
    m = {"cruise_speed_mps": 0.5, "takeoff_wait_s": 10.0, "land_wait_s": 2.0,
         "max_mission_s": 600.0}
    m.update(mission)
    return {
        "search_area": {"min_x": 0, "max_x": 4, "min_y": -2, "max_y": 2},
        "sweep": {"lane_spacing_m": 1.0, "edge_margin_m": 0.3},
        "camera": {"pitch_deg": 45.0, "fwd_offset_m": 0.054, "down_offset_m": 0.034},
        "detection": {"expected_tags": 3},
        "mission": m,
        "simulation": {"noise_m": 0.02},
    }


def run_mission(tmp_path, raw=None, link_cls=SimLink, link_kw=None, confirm=None,
                cfg_extra=None):
    raw = raw or raw_cfg()
    if cfg_extra:
        for section, vals in cfg_extra.items():
            raw.setdefault(section, {}).update(vals)
    cfg = parse_config(raw)
    clock = SimClock()
    kw = dict(speed_mps=0.5, takeoff_s=8.0, noise_m=0.02)
    kw.update(link_kw or {})
    link = link_cls(22, clock, cfg.camera, TAGS, **kw)
    out = tmp_path / "run"
    mlog = MissionLog(out, wall=clock.wall, now=clock.now)
    mission = TagMission(cfg, link, clock, mlog, drone_id=22, confirm=confirm)
    code = mission.run()
    return code, mission, link, mlog, out


def names(mlog):
    return [e["event"] for e in mlog.events()]


def test_full_mission_finds_all_tags_returns_and_lands(tmp_path):
    code, mission, link, mlog, _ = run_mission(tmp_path)
    assert code == 0
    assert link.landed
    assert math.hypot(link.x, link.y) <= mission.cfg.arrival_radius_m + 0.05
    assert mission.tracker.ids() == [1, 2, 3]
    for e in mission.tracker.estimates():
        assert math.hypot(e.x - TAGS[e.tag_id][0], e.y - TAGS[e.tag_id][1]) < 0.1


def test_required_events_are_logged_in_order(tmp_path):
    _, _, _, mlog, _ = run_mission(tmp_path)
    n = names(mlog)
    assert n.index("TAKEOFF_CMD_SENT") < n.index("TAG_DETECTED") < n.index("LANDING_CMD_SENT")
    assert n.count("TAG_DETECTED") == 3                     # once per unique tag
    first_land = n.index("LANDING_CMD_SENT")
    assert "RETURNING_TO_START" in n[:first_land]
    # Mission time starts at the takeoff command.
    takeoff = next(e for e in mlog.events() if e["event"] == "TAKEOFF_CMD_SENT")
    assert takeoff["t_mission_s"] == pytest.approx(0.0)
    land = next(e for e in mlog.events() if e["event"] == "LANDING_CMD_SENT")
    assert land["t_mission_s"] > 30


def test_detection_events_carry_id_position_pose_and_timestamp(tmp_path):
    _, _, _, mlog, _ = run_mission(tmp_path)
    ev = next(e for e in mlog.events() if e["event"] == "TAG_DETECTED")
    d = ev["details"]
    assert {"id", "x", "y", "range_m", "cam_xyz_m", "frame_ms"} <= set(d)
    assert ev["time"] and ev["t_mission_s"] is not None


def test_output_files_are_written(tmp_path):
    _, _, _, _, out = run_mission(tmp_path)
    for f in ("events.csv", "observations.csv", "tags.csv", "trajectory.csv",
              "summary.json"):
        assert (out / f).stat().st_size > 0, f
    tags = list(csv.DictReader(open(out / "tags.csv")))
    assert sorted(int(t["tag_id"]) for t in tags) == [1, 2, 3]
    summary = json.loads((out / "summary.json").read_text())
    assert summary["exit_code"] == 0
    assert summary["takeoff_cmd_time"] and summary["landing_cmd_time"]
    assert summary["waypoints_reached"] == summary["waypoints_planned"]
    obs = list(csv.DictReader(open(out / "observations.csv")))
    assert len(obs) > 10 and {"local_x", "local_y", "tx", "tz", "accepted"} <= set(obs[0])


def test_stop_when_all_found_ends_search_early_but_still_lands(tmp_path):
    code, mission, link, mlog, _ = run_mission(
        tmp_path, cfg_extra={"detection": {"stop_when_all_found": True}})
    assert code == 0 and link.landed
    stop = next(e for e in mlog.events() if e["event"] == "SEARCH_STOPPED")
    assert "expected tags found" in stop["details"]["reason"]
    assert mission._reached < len(mission.plan)


def test_time_limit_stops_search_then_returns_and_lands(tmp_path):
    code, mission, link, mlog, _ = run_mission(
        tmp_path, cfg_extra={"mission": {"max_mission_s": 35.0}})
    assert code == 0 and link.landed
    stop = next(e for e in mlog.events() if e["event"] == "SEARCH_STOPPED")
    assert "time limit" in stop["details"]["reason"]
    assert math.hypot(link.x, link.y) <= mission.cfg.arrival_radius_m + 0.05


def test_cancelled_before_takeoff_sends_nothing(tmp_path):
    code, _, link, mlog, _ = run_mission(tmp_path, confirm=lambda: False)
    assert code == 0
    assert link.commands == []
    assert "TAKEOFF_CMD_SENT" not in names(mlog)


class StaleLink(SimLink):
    """Telemetry stops after 25 s of simulated time."""
    def _emit_telemetry(self, s):
        if self._clock.now() < 25.0:
            super()._emit_telemetry(s)


class EscapingLink(SimLink):
    """Drone is pushed far outside the area after 25 s."""
    def _step(self, dt):
        super()._step(dt)
        if self._clock.now() > 25.0:
            self.x = 20.0


def test_stale_telemetry_aborts_and_lands(tmp_path):
    code, _, link, mlog, _ = run_mission(tmp_path, link_cls=StaleLink)
    assert code == 1 and link.landed
    abort = next(e for e in mlog.events() if e["event"] == "ABORT")
    assert "telemetry stale" in abort["details"]["reason"]
    assert any(e["event"] == "LANDING_CMD_SENT" and e["details"].get("emergency")
               for e in mlog.events())


def test_geofence_breach_aborts_and_lands(tmp_path):
    code, _, link, mlog, _ = run_mission(tmp_path, link_cls=EscapingLink)
    assert code == 1 and link.landed
    abort = next(e for e in mlog.events() if e["event"] == "ABORT")
    assert "geofence" in abort["details"]["reason"]


def test_repeated_waypoint_timeouts_stop_search_and_land(tmp_path):
    # speed 0: telemetry keeps flowing but the drone never reaches a waypoint.
    code, mission, link, mlog, _ = run_mission(
        tmp_path, link_kw={"speed_mps": 0.0},
        cfg_extra={"mission": {"max_consecutive_misses": 2}})
    assert code == 0 and link.landed
    n = names(mlog)
    assert n.count("WAYPOINT_TIMEOUT") == 2
    stop = next(e for e in mlog.events() if e["event"] == "SEARCH_STOPPED")
    assert "consecutive waypoint timeouts" in stop["details"]["reason"]
    assert "HOLD" in [c for _, c in link.commands]


def test_no_flight_commands_before_start(tmp_path):
    _, _, link, _, _ = run_mission(tmp_path)
    cmds = [c for _, c in link.commands]
    assert cmds[0] == "START"
    assert cmds[-1] == "LAND"


def test_config_defaults_and_camera_fov():
    raw = raw_cfg()
    del raw["mission"]["cruise_speed_mps"]
    cfg = parse_config(raw)
    assert cfg.cruise_speed_mps == pytest.approx(0.20)       # PX4 MPC_XY_VEL_MAX, not the firmware constant
    assert cfg.altitude_m == pytest.approx(0.5)
    assert cfg.min_view_elevation_deg == pytest.approx(30.0)
    assert (cfg.camera.hfov_deg, cfg.camera.vfov_deg) == (91.0, 82.0)
    bad = raw_cfg()
    bad["camera"]["hfov_deg"] = 0
    with pytest.raises(ValueError):
        parse_config(bad)


def test_config_validation():
    bad = raw_cfg()
    del bad["search_area"]
    with pytest.raises(ValueError):
        parse_config(bad)
    bad = raw_cfg()
    bad["sweep"]["lane_axis"] = "z"
    with pytest.raises(ValueError):
        parse_config(bad)
    bad = raw_cfg()
    bad["mission"]["arrival_radius_m"] = 0
    with pytest.raises(ValueError):
        parse_config(bad)
    bad = raw_cfg()
    bad["search_area"] = {"min_x": 4, "max_x": 0, "min_y": -2, "max_y": 2}
    with pytest.raises(ValueError):
        parse_config(bad)
