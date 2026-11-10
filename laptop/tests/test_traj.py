import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import TRAJ_CHUNK_PTS, build_traj_packets, build_traj_start
from send_trajectory import load_trajectory


def test_traj_packets_match_firmware_layout() -> None:
    # Offsets pinned to handle_traj_data()/handle_traj_start() in main/wifi_task.c.
    pts = [(i * 0.01, -i * 0.02, 0.0) for i in range(170)]
    packets = build_traj_packets(7, pts)
    assert [p[7] for p in packets] == [80, 80, 10]
    assert max(len(p) for p in packets) <= 1024          # WIFI_CMD_BUF_SIZE

    got = []
    for p in packets:
        pkt, cmd, tid, total, offset, n = struct.unpack_from("<BBBHHB", p)
        assert (pkt, cmd, tid, total) == (0x02, 0x08, 7, 170)
        assert offset == len(got) and len(p) == 8 + n * 12
        got += [struct.unpack_from("<fff", p, 8 + 12 * i) for i in range(n)]
    assert got == [struct.unpack("<fff", struct.pack("<fff", *q)) for q in pts]
    assert TRAJ_CHUNK_PTS == 80

    assert struct.unpack("<BBBH", build_traj_start(7, 50)) == (0x02, 0x09, 7, 50)


def test_load_trajectory_resamples_relative_to_first_point(tmp_path) -> None:
    csv = tmp_path / "traj.csv"
    csv.write_text("t,x,y,z\n0,1,2,0\n0.1,1.1,2,0\n0.2,1.2,2,0\n")
    pts = load_trajectory(str(csv), 0.05)
    assert pts.shape == (5, 3)
    assert abs(pts[1][0] - 0.05) < 1e-9 and abs(pts[-1][0] - 0.2) < 1e-9
    assert not pts[0].any()


def test_slow_zones_cap_speed_inside_and_keep_accel_limit() -> None:
    import numpy as np
    from traj_gen import smooth_path
    pts = [(4.0, 0.0)]
    zone = (1.5, 2.0, -0.3, 0.3)
    tr = smooth_path(pts, (0.0, 0.0), speed=0.15, accel=0.08, dt=0.05,
                     end=(4.0, 0.0), slow_zones=[zone], slow_speed=0.05)
    t, x, y = tr.T
    v = np.hypot(np.diff(x), np.diff(y)) / np.diff(t)
    xm = (x[1:] + x[:-1]) / 2
    inside = (xm > zone[0] + 0.02) & (xm < zone[1] - 0.02)
    assert v[inside].max() <= 0.05 + 1e-3
    assert v[xm > 3.0].max() > 0.12                       # speeds up again after the zone
    # braking stays near the accel limit (speed averaged over 1 s, so sampling noise cancels)
    w = 20
    v1 = np.hypot(x[w:] - x[:-w], y[w:] - y[:-w]) / (t[w:] - t[:-w])
    a = np.diff(v1[::w]) / 1.0
    assert np.abs(a).max() <= 0.08 * 1.3


def test_no_slow_zones_gives_the_same_path_as_before() -> None:
    import numpy as np
    from traj_gen import smooth_path
    a = smooth_path([(2.0, 1.0)], (0.0, 0.0), speed=0.12)
    b = smooth_path([(2.0, 1.0)], (0.0, 0.0), speed=0.12, slow_zones=[], slow_speed=0.05)
    assert np.allclose(a, b)
