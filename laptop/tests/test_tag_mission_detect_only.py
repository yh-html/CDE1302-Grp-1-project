"""
Detect-only mode (walk-through test with the drone carried by hand).

The drone is a SimLink that moves itself along the sweep plan, like a person
carrying it.  The mission must localise every tag, report the error against the
reference positions, and never send a takeoff, goto, hold or land command.
"""

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import run_tag_mission
from drone_link import SimClock, SimLink
from mission_config import parse_config
from mission_log import MissionLog, format_truth_table, truth_errors
from sweep_planner import plan_sweep
from tag_mission import TagMission

TAGS = {1: (1.35, -1.35), 2: (4.0, -1.35), 3: (2.7, 0.9)}


def raw_cfg():
    return {
        "search_area": {"min_x": 0.9, "max_x": 4.5, "min_y": -1.8, "max_y": 1.35},
        "sweep": {"lane_spacing_m": 1.0, "edge_margin_m": 0.3},
        "camera": {"pitch_deg": 45.0, "fwd_offset_m": 0.07, "down_offset_m": 0.008},
        "detection": {"expected_tags": 3},
        "mission": {"cruise_speed_mps": 0.5, "stale_timeout_s": 3.0},
        "simulation": {},
    }


class CarriedLink(SimLink):
    """A SimLink that walks its waypoint list by itself, as if carried by hand."""

    def __init__(self, *args, waypoints, **kw):
        super().__init__(*args, **kw)
        self.mode = "hover"
        self._queue = list(waypoints)

    def _step(self, dt):
        if self.mode == "hover" and self._queue:
            self._goal = self._queue.pop(0)
            self.mode = "fly"
        super()._step(dt)


def run_detect_only(tmp_path, duration_s=90.0, truth=TAGS):
    cfg = parse_config(raw_cfg())
    clock = SimClock()
    plan = plan_sweep(cfg.area, cfg.lane_spacing_m, cfg.edge_margin_m, (0.0, 0.0), cfg.lane_axis)
    link = CarriedLink(22, clock, cfg.camera, TAGS, waypoints=plan, speed_mps=0.5,
                       noise_m=0.01, altitude_m=0.5)
    mlog = MissionLog(tmp_path / "run", wall=clock.wall, now=clock.now)
    mission = TagMission(cfg, link, clock, mlog, drone_id=22, detect_only=True,
                         detect_duration_s=duration_s, truth=truth)
    code = mission.run()
    return code, mission, link, mlog, tmp_path / "run"


def test_walk_through_finds_all_tags_and_sends_no_commands(tmp_path):
    code, mission, link, mlog, _ = run_detect_only(tmp_path)
    assert code == 0
    assert link.commands == []                       # nothing reached the drone
    assert mission.tracker.ids() == [1, 2, 3]
    for e in mission.tracker.estimates():
        assert math.hypot(e.x - TAGS[e.tag_id][0], e.y - TAGS[e.tag_id][1]) < 0.1


def test_events_show_detect_only_and_no_takeoff_or_landing(tmp_path):
    _, _, _, mlog, _ = run_detect_only(tmp_path)
    names = [e["event"] for e in mlog.events()]
    assert "DETECT_ONLY_START" in names and "DETECT_ONLY_END" in names
    assert names.count("TAG_DETECTED") == 3
    for forbidden in ("TAKEOFF_CMD_SENT", "LANDING_CMD_SENT", "WAYPOINT_SENT", "ABORT"):
        assert forbidden not in names


def test_summary_records_mode_and_truth_errors(tmp_path):
    _, _, _, _, out = run_detect_only(tmp_path)
    summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert summary["mode"] == "detect-only"
    assert summary["exit_code"] == 0
    errors = {r["tag_id"]: r["error_m"] for r in summary["truth_errors"]}
    assert set(errors) == {1, 2, 3} and all(e is not None and e < 0.1 for e in errors.values())


def test_wrapped_link_refuses_every_command(tmp_path):
    cfg = parse_config(raw_cfg())
    clock = SimClock()
    link = SimLink(22, clock, cfg.camera, TAGS)
    mission = TagMission(cfg, link, clock, MissionLog(None, clock.wall, clock.now),
                         drone_id=22, detect_only=True)
    for call in (lambda: mission.link.send_start(), lambda: mission.link.send_goto(1.0, 1.0),
                 lambda: mission.link.send_hold(), lambda: mission.link.send_land()):
        with pytest.raises(AssertionError):
            call()
    assert link.commands == []


def test_truth_errors_report_missing_and_unlisted_tags():
    est = [SimpleNamespace(tag_id=1, x=1.40, y=-1.30), SimpleNamespace(tag_id=9, x=0.0, y=0.0)]
    rows = {r["tag_id"]: r for r in truth_errors(est, {1: (1.35, -1.35), 2: (4.0, -1.35)})}
    assert rows[1]["error_m"] == pytest.approx(math.hypot(0.05, 0.05), abs=1e-3)
    assert rows[2]["error_m"] is None and rows[2]["est_x"] is None     # never detected
    assert rows[9]["ref_x"] is None and rows[9]["error_m"] is None     # not in the reference list
    table = format_truth_table(list(rows.values()))
    assert "not detected" in table and "not in the list" in table and "mean error" in table


def test_parse_truth():
    assert run_tag_mission.parse_truth("1:1.35,-1.35 2:4,-1.35;3:2.7,0.9") == {
        1: (1.35, -1.35), 2: (4.0, -1.35), 3: (2.7, 0.9)}
    assert run_tag_mission.parse_truth(None) == {}
    with pytest.raises(SystemExit):
        run_tag_mission.parse_truth("1:1.35")


def test_command_line_rejects_bad_combinations():
    with pytest.raises(SystemExit):
        run_tag_mission.main(["--drone-id", "22", "--detect-only", "--simulate"])
    with pytest.raises(SystemExit):
        run_tag_mission.main(["--drone-id", "22", "--duration", "10"])
