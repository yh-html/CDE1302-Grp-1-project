"""
Trajectory pre-upload: the sweep's chunks go up during the takeoff wait (two passes,
same id), so at GO only CMD_TRAJ_START is needed. If that start does not take, the
mission falls back to the normal full-upload retry loop.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_link import SimClock, SimLink
from mission_config import parse_config
from mission_log import MissionLog
from tag_mission import TagMission
from test_look_again import raw_cfg

TAGS = {2: (1.35, -1.35), 4: (2.7, -1.35), 6: (2.7, 0.9)}


class LossyUploadSim(SimLink):
    """Drops every pre-upload (as if the drone never got the chunks)."""

    def upload_trajectory(self, pts, traj_id):
        self._log_cmd("TRAJ_UPLOAD")


class LateFlyingSim(SimLink):
    """Like the firmware when the climb is slow: START is ignored until 14 s after takeoff
    (past the mission's takeoff wait), and is not consumed while ignored."""

    def send_start(self):
        self._t_takeoff = self._clock.now()
        super().send_start()

    def start_trajectory(self, traj_id, dt_ms):
        if self._clock.now() - self._t_takeoff < 14.0:
            self._log_cmd("TRAJ_START")
            return
        super().start_trajectory(traj_id, dt_ms)


def run(tmp_path, link_cls=SimLink, go_after_s=None):
    cfg = parse_config(raw_cfg(None, stop_when_all_found=False))
    clock = SimClock()
    link = link_cls(1, clock, cfg.camera, TAGS, speed_mps=0.3, takeoff_s=8.0,
                    max_range_m=3.0, min_elevation_deg=30.0, packet_hz=2.0)
    mlog = MissionLog(tmp_path / "run", wall=clock.wall, now=clock.now)
    gate = None
    if go_after_s is not None:
        t_go = [None]

        def gate():
            if t_go[0] is None:
                t_go[0] = clock.now()
            return "go" if clock.now() - t_go[0] >= go_after_s else None
    mission = TagMission(cfg, link, clock, mlog, drone_id=1, go_gate=gate)
    code = mission.run()
    return code, link, mlog


def names(mlog):
    return [e["event"] for e in mlog.events()]


def test_preupload_happens_during_takeoff_wait_before_go(tmp_path):
    code, link, mlog = run(tmp_path, go_after_s=3.0)
    assert code == 0 and link.landed
    n = names(mlog)
    assert n.index("TAKEOFF_CMD_SENT") < n.index("TRAJECTORY_PREUPLOADED") < n.index("TAKEOFF_WAIT_DONE")
    assert n.index("TRAJECTORY_PREUPLOADED") < n.index("GO_RECEIVED")
    pre = next(e for e in mlog.events() if e["event"] == "TRAJECTORY_PREUPLOADED")["details"]
    assert pre["passes"] == 2 and pre["chunks"] == -(-pre["points"] // 80)
    cmds = [c for _, c in link.commands]
    i_start = cmds.index("START")
    assert cmds[i_start + 1:i_start + 3] == ["TRAJ_UPLOAD", "TRAJ_UPLOAD"]


def test_start_at_go_needs_no_retry(tmp_path):
    code, link, mlog = run(tmp_path, go_after_s=3.0)
    assert code == 0
    start = next(e for e in mlog.events() if e["event"] == "TRAJECTORY_START")["details"]
    assert start["preuploaded"] is True
    assert "TRAJECTORY_RETRY" not in names(mlog)
    assert "TRAJECTORY_PREUPLOAD_NOT_STARTED" not in names(mlog)
    cmds = [c for _, c in link.commands]
    # one bare start, then the stored trajectory plays (no full re-upload)
    assert cmds.count("TRAJ_START") == 1 and cmds.count("TRAJ") == 1
    assert "TRAJECTORY_END" in names(mlog)


def test_no_wait_for_go_starts_right_after_takeoff(tmp_path):
    code, link, mlog = run(tmp_path)
    assert code == 0 and link.landed
    n = names(mlog)
    assert "TRAJECTORY_RETRY" not in n
    assert n.index("TAKEOFF_WAIT_DONE") < n.index("TRAJECTORY_START")


def test_fallback_to_full_upload_when_preupload_was_lost(tmp_path):
    code, link, mlog = run(tmp_path, link_cls=LossyUploadSim, go_after_s=3.0)
    assert code == 0 and link.landed
    n = names(mlog)
    assert "TRAJECTORY_PREUPLOAD_NOT_STARTED" in n
    assert "TRAJECTORY_END" in n and "ABORT" not in n
    cmds = [c for _, c in link.commands]
    # bare STARTs re-sent every 0.5 s for the whole start window (plus one same-id
    # re-upload halfway), then the full-upload fallback
    assert cmds.count("TRAJ_START") >= 5 and cmds.count("TRAJ") >= 1
    assert cmds.index("TRAJ") > max(i for i, c in enumerate(cmds) if c == "TRAJ_START")


def test_start_retries_until_the_drone_is_ready(tmp_path):
    """The firmware ignores START until it is in its flying phase; keep sending it."""
    code, link, mlog = run(tmp_path, link_cls=LateFlyingSim)
    assert code == 0 and link.landed
    n = names(mlog)
    assert "TRAJECTORY_PREUPLOAD_STARTED" in n and "TRAJECTORY_PREUPLOAD_NOT_STARTED" not in n
    cmds = [c for _, c in link.commands]
    assert cmds.count("TRAJ_START") >= 2 and cmds.count("TRAJ") == 1   # no full re-upload
