"""
SimLink realism: a tag boxed in by a U-shaped foam block, and trajectory drift.

The 10 Nov arena had one tag inside a 0.15 m-high foam U, open towards +x. The
drone keeps its takeoff heading (+x) in trajectory mode, so the camera has to look
over the closed -x wall: only possible from close behind the tag and nearly in line.
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_link import SimClock, SimLink
from tag_localizer import CameraGeometry

CAM = CameraGeometry(pitch_deg=45.0, fwd_offset_m=0.07, right_offset_m=0.0, down_offset_m=0.008)
TAG = (4.2, 0.62)
BOX = {6: dict(wall_h=0.15, inner_half=0.12, wall_t=0.05, open_side="+x")}


def seen(dx, dy, heading=0.0, boxed=True):
    """Is tag 6 reported with the drone at TAG + (dx, dy), hovering at `heading`?"""
    link = SimLink(1, SimClock(), CAM, {6: TAG}, max_range_m=3.0, min_elevation_deg=30.0,
                   boxed=BOX if boxed else None)
    link.x, link.y, link.heading = TAG[0] + dx, TAG[1] + dy, heading
    link.mode = "hover"
    return bool(link._make_packet().detections)


@pytest.mark.parametrize("short", [0.2, 0.3])
def test_boxed_tag_visible_from_close_behind_in_line(short):
    assert seen(-short, 0.0)


def test_boxed_tag_hidden_from_the_side():
    # 0.5 m short and 0.6 m to the side: an open tag is in view, the boxed one is not
    assert seen(-0.5, 0.6, boxed=False)
    assert not seen(-0.5, 0.6)


def test_boxed_tag_hidden_behind_the_closed_wall_further_back():
    # 0.6 m short, in line: the sight line is too flat to clear the 0.15 m back wall
    assert seen(-0.6, 0.0, boxed=False)
    assert not seen(-0.6, 0.0)


def test_boxed_tag_visible_through_the_open_side():
    # beyond the tag on the open (+x) side, facing back towards it
    for d in (0.5, 0.8):
        assert seen(d, 0.0, heading=math.pi)


def test_bad_open_side_is_rejected():
    with pytest.raises(ValueError, match="open_side"):
        SimLink(1, SimClock(), CAM, {6: TAG}, boxed={6: dict(open_side="up")})


def test_drift_is_reproducible_bounded_and_seeded():
    a = SimLink(1, SimClock(), CAM, {}, drift_m=0.25, drift_seed=3)
    b = SimLink(1, SimClock(), CAM, {}, drift_m=0.25, drift_seed=3)
    c = SimLink(1, SimClock(), CAM, {}, drift_m=0.25, drift_seed=4)
    ts = [i * 0.5 for i in range(300)]
    da = [a.drift_at(t) for t in ts]
    assert da == [b.drift_at(t) for t in ts]
    assert da != [c.drift_at(t) for t in ts]
    ys = [abs(y) for _, y in da]
    assert max(ys) <= 0.25 + 1e-9 and max(ys) > 0.1        # sideways, up to the amplitude
    assert max(abs(x) for x, _ in da) <= 0.4 * 0.25 + 1e-9  # forward error is smaller


def test_drift_moves_the_drone_during_a_trajectory_and_is_reported():
    clock = SimClock()
    link = SimLink(1, clock, CAM, {}, takeoff_s=0.5, drift_m=0.25, drift_seed=1)
    seen_y, gaps = [], []

    def on_telem(s):
        seen_y.append(s.y)
        gaps.append(abs(s.y - link.y) + abs(s.x - link.x))   # telemetry vs true position
    link.on_telemetry(on_telem)
    link.send_start()
    clock.sleep(1.0)
    pts = [(0.1 * i, 0.0, 0.0) for i in range(300)]          # 30 m straight along x, 30 s
    link.send_trajectory(pts, 100)
    clock.sleep(20.0)
    assert max(abs(y) for y in seen_y) > 0.05                 # left the commanded line
    assert max(gaps) < 1e-9                                   # telemetry = true position


def test_defaults_leave_the_sim_unchanged():
    clock = SimClock()
    link = SimLink(1, clock, CAM, {6: TAG}, takeoff_s=0.5)
    assert link.drift_at(12.3) == (0.0, 0.0)
    link.send_start()
    clock.sleep(1.0)
    pts = [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0)]
    link.send_trajectory(pts, 100)
    clock.sleep(2.0)
    assert (link.x, link.y) == pytest.approx((1.0, 0.5))      # exactly on the path
    # an unboxed tag 0.6 m short and in line is visible, as before
    assert seen(-0.6, 0.0, boxed=False)


def _seen_half(dx, dy, tag_half):
    box = {6: dict(BOX[6], tag_half=tag_half)}
    link = SimLink(1, SimClock(), CAM, {6: TAG}, max_range_m=3.0, min_elevation_deg=30.0, boxed=box)
    link.x, link.y, link.heading = TAG[0] + dx, TAG[1] + dy, 0.0
    link.mode = "hover"
    return bool(link._make_packet().detections)


def test_whole_tag_must_clear_the_wall_when_tag_half_is_set():
    # centre clears the back wall from 0.4 m short, the near corners do not
    assert _seen_half(-0.4, 0.0, 0.0)
    assert not _seen_half(-0.4, 0.0, 0.05)
    assert _seen_half(-0.2, 0.0, 0.05)


def test_detect_prob_drops_some_frames_reproducibly():
    def count(seed, p):
        link = SimLink(1, SimClock(), CAM, {6: TAG}, max_range_m=3.0, seed=seed, detect_prob=p)
        link.x, link.y, link.heading, link.mode = TAG[0] - 0.4, TAG[1], 0.0, "hover"
        return sum(bool(link._make_packet().detections) for _ in range(200))
    assert count(1, 1.0) == 200
    n = count(1, 0.6)
    assert 90 < n < 150 and n == count(1, 0.6)
    with pytest.raises(ValueError, match="detect_prob"):
        SimLink(1, SimClock(), CAM, {}, detect_prob=0.0)
