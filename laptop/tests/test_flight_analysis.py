"""
Post-flight analysis: ULog loading, time alignment, tag re-localisation, outputs.

The ULog files here are written in the real binary format (ulog_builder.py), and
the mission run folder comes from a real run of the mission in the simulator,
so loader, alignment and plotting are exercised on realistic data.  No real
PX4 log was available when these were written.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

pytest.importorskip("pyulog")

import analyze_flight                                              # noqa: E402
from drone_link import SimClock, SimLink                           # noqa: E402
from flight_data import ARMED, load_run, load_ulg, sync_times      # noqa: E402
from flight_plots import analyze                                   # noqa: E402
from mission_config import parse_config                            # noqa: E402
from mission_log import MissionLog                                 # noqa: E402
from tag_localizer import (CameraGeometry, body_to_local,          # noqa: E402
                           camera_to_local_full, quat_to_matrix)
from tag_mission import TagMission                                 # noqa: E402
from ulog_builder import ULogWriter, yaw_pitch_roll_quat           # noqa: E402

TAGS = {1: (1.5, -1.0), 2: (2.5, 1.2), 3: (3.5, -0.4)}
BOOT_OFFSET_S = 5000.0          # PX4 clock reads 5000 s when the sim clock reads 0
ARM_AFTER_CMD_S = 3.0           # PX4 arms 3 s after the takeoff command (default guess is 2)


def raw_cfg():
    return {
        "search_area": {"min_x": 0, "max_x": 4, "min_y": -2, "max_y": 2},
        "sweep": {"lane_spacing_m": 1.0, "edge_margin_m": 0.3},
        "camera": {"pitch_deg": 45.0, "fwd_offset_m": 0.054, "down_offset_m": 0.034},
        "detection": {"expected_tags": 3},
        "mission": {"cruise_speed_mps": 0.5, "takeoff_wait_s": 10.0, "land_wait_s": 2.0},
        "simulation": {"noise_m": 0.0},
    }


@pytest.fixture(scope="module")
def flight(tmp_path_factory):
    """Run a real simulated mission; build a matching .ulg; return paths and truth."""
    base = tmp_path_factory.mktemp("flight")
    cfg = parse_config(raw_cfg())
    clock = SimClock()
    link = SimLink(22, clock, cfg.camera, TAGS, speed_mps=0.5, takeoff_s=8.0, noise_m=0.0)
    samples = []
    link.on_telemetry(lambda s: samples.append((s.t, s.x, s.y, s.heading_rad)))
    run_dir = base / "run"
    mlog = MissionLog(run_dir, wall=clock.wall, now=clock.now)
    mission = TagMission(cfg, link, clock, mlog, drone_id=22)
    assert mission.run() == 0

    t_cmd = next(c[0] for c in link.commands if c[1] == "START")
    t_land = next(c[0] for c in link.commands if c[1] == "LAND")

    w = ULogWriter()
    arm_t, disarm_t = t_cmd + ARM_AFTER_CMD_S, t_land + 1.0
    for st, state in ((0.0, 1), (arm_t, ARMED), (disarm_t, 1)):
        w.add("vehicle_status", int((st + BOOT_OFFSET_S) * 1e6), state, 14)
    for t, x, y, h in samples:
        us = int((t + BOOT_OFFSET_S) * 1e6)
        alt = 0.5 if (arm_t + 8 <= t <= t_land) else 0.0
        w.add("vehicle_local_position", us, x, y, -alt, 0.0, 0.0, 0.0, h)
        w.add("vehicle_attitude", us, yaw_pitch_roll_quat(h))
        w.add("vehicle_land_detected", us, t >= t_land + 0.5)
    ulg = base / "flight.ulg"
    w.write(ulg)
    return {"ulg": ulg, "run": run_dir, "cfg": cfg, "t_cmd": t_cmd, "arm_t": arm_t,
            "disarm_t": disarm_t, "samples": samples}


def test_ulog_loader_reads_real_binary_format(flight):
    f = load_ulg(flight["ulg"])
    assert len(f.t) == len(flight["samples"])
    assert f.t[0] == pytest.approx(BOOT_OFFSET_S + flight["samples"][0][0])
    assert f.arm_intervals == [pytest.approx((BOOT_OFFSET_S + flight["arm_t"],
                                              BOOT_OFFSET_S + flight["disarm_t"]))]
    assert f.att_q is not None and f.att_q.shape[1] == 4
    # the builder sets landed 0.5 s after the landing command, 1 s before disarm
    assert f.touchdown_time() == pytest.approx(BOOT_OFFSET_S + flight["disarm_t"] - 0.5, abs=0.2)


def test_missing_local_position_gives_a_clear_error(tmp_path):
    w = ULogWriter(topics=["vehicle_status"])
    w.add("vehicle_status", 1_000_000, ARMED, 14)
    p = tmp_path / "bad.ulg"
    w.write(p)
    with pytest.raises(ValueError, match="vehicle_local_position"):
        load_ulg(p)


def test_sync_recovers_the_clock_offset_despite_wrong_arm_delay(flight):
    f, run = load_ulg(flight["ulg"]), load_run(flight["run"])
    true_offset = BOOT_OFFSET_S + flight["t_cmd"]
    s = sync_times(f, run, mode="auto", arm_delay_s=2.0)        # real delay is 3 s
    assert s.method == "xcorr" and s.quality == "good"
    assert s.offset_s == pytest.approx(true_offset, abs=0.15)
    assert s.rms_m < 0.05
    # arm-only is off by the 1 s error in the assumed delay
    s_arm = sync_times(f, run, mode="arm", arm_delay_s=2.0)
    assert abs(s_arm.offset_s - true_offset) == pytest.approx(1.0, abs=0.05)


def test_manual_offset_is_respected(flight):
    f, run = load_ulg(flight["ulg"]), load_run(flight["run"])
    s = sync_times(f, run, manual_offset_s=123.0)
    assert s.offset_s == 123.0 and s.quality == "manual"


def test_analysis_metrics_and_tags(flight):
    f, run = load_ulg(flight["ulg"]), load_run(flight["run"])
    sync = sync_times(f, run)
    a = analyze(f, run, sync, camera=flight["cfg"].camera, truth=TAGS)
    m = a.metrics
    assert m["return_error_m"] < 0.3                       # mission returns to start
    assert m["max_altitude_m"] == pytest.approx(0.5, abs=0.01)
    assert m["path_length_m"] > 20
    # the synthetic log has zero velocity fields, so speed is derived from position
    assert 0.3 < m["max_horizontal_speed_mps"] < 0.8      # drone flies at 0.5 m/s
    assert [t.tag_id for t in a.tags] == [1, 2, 3]
    for t in a.tags:
        assert t.err() < 0.05                              # mission estimate vs truth
        assert t.err_att() < 0.05                          # zero tilt: corrected agrees
        assert t.shift_att() < 0.05
    assert a.max_tilt_deg == pytest.approx(0.0, abs=0.5)


def test_command_line_writes_all_outputs(flight, tmp_path, capsys):
    out = tmp_path / "out"
    code = analyze_flight.main([
        "--ulg", str(flight["ulg"]), "--run", str(flight["run"]), "--out", str(out),
        "--3d", "--truth", "1:1.5,-1.0", "--truth", "2:2.5,1.2"])
    assert code == 0
    for name in ("flight_overview.png", "flight_3d.png", "flight_summary.md",
                 "flight_summary.json"):
        assert (out / name).stat().st_size > 1000, name
    text = (out / "flight_summary.md").read_text()
    assert "| 1 |" in text and "Horizontal error against truth" in text
    assert "Time alignment" in capsys.readouterr().out


def test_telemetry_only_mode_without_ulg(flight, tmp_path):
    out = tmp_path / "t_only"
    assert analyze_flight.main(["--run", str(flight["run"]), "--out", str(out)]) == 0
    assert (out / "flight_overview.png").stat().st_size > 1000


def test_ulg_only_mode_without_run(flight, tmp_path):
    out = tmp_path / "u_only"
    assert analyze_flight.main(["--ulg", str(flight["ulg"]), "--out", str(out)]) == 0
    assert "Landing" in (out / "flight_summary.md").read_text()


def test_requires_an_input():
    with pytest.raises(SystemExit):
        analyze_flight.main([])


def test_bad_truth_argument_is_rejected(flight):
    with pytest.raises(SystemExit):
        analyze_flight.main(["--run", str(flight["run"]), "--truth", "oops"])


# -- attitude-corrected localisation ----------------------------------------

def test_quaternion_matrix_matches_heading_only_rotation():
    h = math.radians(37)
    r = quat_to_matrix(yaw_pitch_roll_quat(h))
    assert r[0][0] == pytest.approx(math.cos(h)) and r[1][0] == pytest.approx(math.sin(h))
    assert r[2][2] == pytest.approx(1.0)


def test_full_attitude_equals_heading_only_when_level():
    g = CameraGeometry(45.0, 0.054, 0.0, 0.034)
    h = math.radians(70)
    fwd, right, _ = g.camera_to_body(0.2, -0.1, 1.4)
    want = body_to_local(fwd, right, 1.0, 2.0, h)
    got = camera_to_local_full(g, 0.2, -0.1, 1.4, 1.0, 2.0, yaw_pitch_roll_quat(h))
    assert got == pytest.approx(want)


def test_ignoring_pitch_costs_accuracy_and_the_correction_removes_it():
    """Drone pitched up 8 deg: heading-only geometry misplaces the tag, full attitude does not."""
    g = CameraGeometry(45.0, 0.054, 0.0, 0.034)
    tag, drone, yaw = np.array([2.0, 0.5, 0.5]), np.array([0.0, 0.0, 0.0]), math.radians(15)
    q = yaw_pitch_roll_quat(yaw, pitch=math.radians(-8))       # nose up (negative pitch)
    r = np.array(quat_to_matrix(q))
    v_body = r.T @ (tag - drone)                              # tag in the tilted body frame
    cam = g.body_to_camera(*v_body)
    heading_only = body_to_local(*g.camera_to_body(*cam)[:2], 0.0, 0.0, yaw)
    full = camera_to_local_full(g, *cam, 0.0, 0.0, q)
    assert full == pytest.approx(tag[:2], abs=1e-9)
    # about 5 cm for 8 degrees at this range and camera pitch
    assert math.hypot(heading_only[0] - tag[0], heading_only[1] - tag[1]) > 0.03
