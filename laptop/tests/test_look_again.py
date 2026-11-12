"""
look_again: after a partial sighting the rest of the path will not see again, the
mission pauses, hovers just short of it (camera faces +x) and then resumes the sweep.

A tag boxed in on three sides is only visible from close behind it. GlimpseSim models
that crudely without touching drone_link.py: a "hidden" tag is reported only from a
narrow window short of it, plus a short band farther out to the side (the glimpse).
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_link import SimClock, SimLink
from mission_config import parse_config
from mission_log import MissionLog
from tag_mission import TagMission


class GlimpseSim(SimLink):
    def __init__(self, *a, hidden=(), **kw):
        super().__init__(*a, **kw)
        self.hidden = set(hidden)

    def _make_packet(self):
        pkt = super()._make_packet()
        keep = []
        for d in pkt.detections:
            if d.id in self.hidden:
                tx, ty = self._tags[d.id]
                ahead, side = tx - self.x, abs(self.y - ty)
                full = 0.03 <= ahead <= 0.35 and side <= 0.12       # over the wall, close behind
                glimpse = 0.25 <= ahead <= 0.40 and side <= 0.45    # brief view from the lane
                if not (full or glimpse):
                    continue
            keep.append(d)
        pkt.detections = keep
        pkt.n = len(keep) if hasattr(pkt, "n") else None
        return pkt


def raw_cfg(look=None, stop_when_all_found=True, expected=3):
    cfg = {
        "search_area": {"min_x": 0.9, "max_x": 4.5, "min_y": -1.8, "max_y": 1.35},
        "sweep": {"lane_spacing_m": 2.25, "edge_margin_m": 0.45, "lane_axis": "x",
                  "entry": "left", "return_leg": "lateral"},
        "camera": {"pitch_deg": 45.0, "fwd_offset_m": 0.07, "down_offset_m": 0.008},
        "detection": {"expected_tags": expected, "min_frames": 3, "merge_radius_m": 0.5,
                      "stop_when_all_found": stop_when_all_found, "settle_s": 1.0,
                      "max_range_m": 3.0},
        "mission": {"cruise_speed_mps": 0.2, "takeoff_wait_s": 10.0, "land_wait_s": 2.0,
                    "max_mission_s": 900.0, "geofence_margin_m": 0.45,
                    "soft_fence_margin_m": 0.3},
        "trajectory": {"enabled": True, "speed_mps": 0.144, "corner_radius_m": 0.35,
                       "accel_mps2": 0.08, "dt_s": 0.1},
    }
    if look is not None:
        cfg["look_again"] = look
    return cfg


TAGS = {2: (1.35, -1.35), 4: (2.7, -1.35), 6: (3.5, 0.55)}     # 6: off the lane, boxed


def run(tmp_path, raw, tags=TAGS, hidden=(6,)):
    cfg = parse_config(raw)
    clock = SimClock()
    link = GlimpseSim(1, clock, cfg.camera, tags, speed_mps=0.3, takeoff_s=8.0,
                      max_range_m=3.0, min_elevation_deg=30.0, noise_m=0.01,
                      packet_hz=2.0, hidden=hidden)
    mlog = MissionLog(tmp_path / "run", wall=clock.wall, now=clock.now)
    mission = TagMission(cfg, link, clock, mlog, drone_id=1)
    code = mission.run()
    return code, mission, link, mlog


def events(mlog, name=None):
    ev = mlog.events()
    return [e for e in ev if name is None or e["event"] == name]


LOOK = {"enabled": True}


def test_glimpse_only_tag_gets_one_look_and_is_confirmed(tmp_path):
    code, mission, link, mlog = run(tmp_path, raw_cfg(LOOK))
    assert code == 0 and link.landed
    starts = events(mlog, "LOOK_AGAIN_START")
    assert len(starts) == 1
    d = starts[0]["details"]
    assert abs(d["x"] - 3.5) < 0.2 and abs(d["y"] - 0.55) < 0.2
    assert d["look_x"] < d["x"]                         # short of it: camera faces +x
    res = events(mlog, "LOOK_AGAIN_RESULT")[0]["details"]
    assert res["confirmed"] is True and res["id"] == 6
    assert {e.tag_id for e in mission.tracker.confirmed()} == {2, 4, 6}
    assert math.hypot(link.x, link.y) < 0.3            # back home


def test_without_look_again_the_boxed_tag_is_missed(tmp_path):
    code, mission, link, mlog = run(tmp_path, raw_cfg(None))
    assert code == 0 and link.landed
    assert 6 not in {e.tag_id for e in mission.tracker.confirmed()}
    assert not events(mlog, "LOOK_AGAIN_START")


def test_no_glimpse_means_no_look_and_identical_run(tmp_path):
    plain = {2: (1.35, -1.35), 4: (2.7, -1.35), 6: (2.7, 0.9)}
    _, _, link_a, mlog_a = run(tmp_path / "a", raw_cfg(None), tags=plain, hidden=())
    _, _, link_b, mlog_b = run(tmp_path / "b", raw_cfg(LOOK), tags=plain, hidden=())
    assert not any(e["event"].startswith("LOOK_AGAIN") for e in mlog_b.events())
    assert [e["event"] for e in mlog_a.events()] == [e["event"] for e in mlog_b.events()]
    assert [c for _, c in link_a.commands] == [c for _, c in link_b.commands]


def test_max_looks_is_respected(tmp_path):
    tags = {2: (1.35, -1.35), 4: (3.5, 0.55), 6: (2.0, 0.55)}
    raw = raw_cfg({"enabled": True, "max_looks": 1}, stop_when_all_found=False)
    code, mission, link, mlog = run(tmp_path, raw, tags=tags, hidden=(4, 6))
    assert code == 0 and link.landed
    assert len(events(mlog, "LOOK_AGAIN_START")) == 1


def test_resumed_sweep_runs_to_the_end_then_returns_and_lands(tmp_path):
    raw = raw_cfg(LOOK, stop_when_all_found=False)
    code, mission, link, mlog = run(tmp_path, raw)
    assert code == 0 and link.landed
    names = [e["event"] for e in mlog.events()]
    i = names.index("LOOK_AGAIN_RESULT")
    assert "TRAJECTORY_RESUME" in names[i:]
    assert "TRAJECTORY_END" in names[i:]
    assert names.index("TRAJECTORY_END") > names.index("TRAJECTORY_RESUME")
    assert "LANDING_CMD_SENT" in names and "ABORT" not in names
    assert math.hypot(link.x, link.y) < 0.3
    assert {e.tag_id for e in mission.tracker.confirmed()} == {2, 4, 6}
