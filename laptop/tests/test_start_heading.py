"""Takeoff is refused unless PX4's heading is near 0 at the start (no compass)."""

import math
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from drone_link import SimClock, SimLink  # noqa: E402
from mission_config import parse_config  # noqa: E402
from mission_log import MissionLog  # noqa: E402
from tag_mission import TagMission  # noqa: E402


def _run(tmp_path, heading_deg, limit=None):
    raw = {
        "search_area": {"min_x": 0, "max_x": 2, "min_y": -1, "max_y": 1},
        "sweep": {"lane_spacing_m": 1.0, "edge_margin_m": 0.3},
        "detection": {"expected_tags": 1},
        "mission": {"cruise_speed_mps": 0.5, "takeoff_wait_s": 2.0, "land_wait_s": 1.0},
    }
    if limit is not None:
        raw["mission"]["max_start_heading_deg"] = limit
    cfg = parse_config(raw)
    clock = SimClock()
    link = SimLink(1, clock, cfg.camera, {}, speed_mps=0.5, takeoff_s=2.0,
                   heading_rad=math.radians(heading_deg))
    mlog = MissionLog(tmp_path / "run", wall=clock.wall, now=clock.now)
    mission = TagMission(cfg, link, clock, mlog, drone_id=1)
    code = mission.run()
    return code, mission, [c[1] for c in link.commands]


@pytest.mark.parametrize("deg", [6.0, -6.0, 354.0])
def test_rotated_start_refuses_takeoff(tmp_path, deg):
    code, mission, cmds = _run(tmp_path, deg)
    assert code == 1
    assert "START" not in cmds
    assert "heading" in mission.exit_reason


@pytest.mark.parametrize("deg", [0.0, 2.0, 359.0])
def test_zeroed_start_flies(tmp_path, deg):
    code, _, cmds = _run(tmp_path, deg)
    assert code == 0
    assert "START" in cmds


def test_check_can_be_disabled(tmp_path):
    code, _, cmds = _run(tmp_path, 20.0, limit=0)
    assert code == 0
    assert "START" in cmds
