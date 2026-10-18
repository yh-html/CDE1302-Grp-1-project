"""Camera -> body -> local localisation maths and tag fusion."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tag_localizer import (CameraGeometry, DetectionGate, Observation,
                           PoseHistory, TagTracker, body_to_local)

GEOM = CameraGeometry(pitch_deg=45.0, fwd_offset_m=0.054,
                      right_offset_m=0.0, down_offset_m=0.034)


def test_camera_to_body_agrees_with_tag_debug():
    """The mission and the live debug viewer must use the same geometry."""
    tag_debug = pytest.importorskip("tag_debug")
    mounts = getattr(tag_debug, "CAMERA_MOUNTS", None)
    if not mounts or "astra_under_nose" not in mounts:
        pytest.skip("tag_debug.CAMERA_MOUNTS['astra_under_nose'] not available")
    m = mounts["astra_under_nose"]
    g = CameraGeometry(45.0, m.fwd_offset_m, m.right_offset_m, m.down_offset_m)
    for tx, ty, tz in [(0.1, -0.2, 1.3), (-0.4, 0.3, 0.8), (0.0, 0.0, 2.0)]:
        want = tag_debug.cam_to_body(tx, ty, tz, pitch_deg=45.0,
                                     mount_name="astra_under_nose")[:3]
        assert g.camera_to_body(tx, ty, tz) == pytest.approx(want)


@pytest.mark.parametrize("pitch", [30.0, 45.0, 60.0, 75.0, 90.0])
def test_body_camera_round_trip(pitch):
    g = CameraGeometry(pitch, 0.054, 0.01, 0.034)
    body = (1.2, -0.3, 0.5)
    assert g.camera_to_body(*g.body_to_camera(*body)) == pytest.approx(body)


def test_optical_axis_points_nose_down():
    """A tag straight along the optical axis is forward AND below the camera."""
    g = CameraGeometry(45.0, 0.0, 0.0, 0.0)
    fwd, right, down = g.camera_to_body(0.0, 0.0, 1.0)
    assert fwd == pytest.approx(math.cos(math.radians(45)))
    assert down == pytest.approx(math.sin(math.radians(45)))
    assert right == pytest.approx(0.0)


def test_pitch_range_validated():
    with pytest.raises(ValueError):
        CameraGeometry(-5.0, 0, 0, 0)
    with pytest.raises(ValueError):
        CameraGeometry(95.0, 0, 0, 0)


@pytest.mark.parametrize("heading_deg,fwd,right,expect", [
    (0, 1, 0, (1, 0)),        # facing North: forward is +x
    (90, 1, 0, (0, 1)),       # facing East: forward is +y
    (180, 1, 0, (-1, 0)),
    (0, 0, 1, (0, 1)),        # right of a North-facing drone is East
    (90, 0, 1, (-1, 0)),      # right of an East-facing drone is South
])
def test_body_to_local_headings(heading_deg, fwd, right, expect):
    x, y = body_to_local(fwd, right, 0.0, 0.0, math.radians(heading_deg))
    assert (x, y) == pytest.approx(expect, abs=1e-9)


def test_body_to_local_adds_drone_position():
    assert body_to_local(1, 0, 2.0, 3.0, 0.0) == pytest.approx((3.0, 3.0))


@pytest.mark.parametrize("heading_deg", [0, 37, 90, 135, 180, -90, -170])
def test_tag_position_recovered_at_any_heading(heading_deg):
    """Place a floor tag, project it into the camera, localise it back."""
    tag = (2.3, -0.7)
    drone = (1.0, 0.2)
    h = math.radians(heading_deg)
    dx, dy = tag[0] - drone[0], tag[1] - drone[1]
    fwd = dx * math.cos(h) + dy * math.sin(h)
    right = -dx * math.sin(h) + dy * math.cos(h)
    cam = GEOM.body_to_camera(fwd, right, 0.5)        # tag 0.5 m below the drone
    f2, r2, _ = GEOM.camera_to_body(*cam)
    assert body_to_local(f2, r2, drone[0], drone[1], h) == pytest.approx(tag, abs=1e-9)


def test_pose_history_interpolates_and_clamps():
    h = PoseHistory()
    assert h.at(1.0) is None
    h.add(0.0, 0.0, 0.0, 0.0)
    h.add(2.0, 2.0, 4.0, 0.0)
    assert h.at(1.0) == pytest.approx((1.0, 2.0, 0.0))
    assert h.at(-5.0) == pytest.approx((0.0, 0.0, 0.0))
    assert h.at(9.0) == pytest.approx((2.0, 4.0, 0.0))


def test_pose_history_heading_wraps_the_short_way():
    h = PoseHistory()
    h.add(0.0, 0, 0, math.radians(170))
    h.add(2.0, 0, 0, math.radians(-170))
    mid = h.at(1.0)[2]
    assert abs(abs(mid) - math.pi) < 1e-9       # 180 degrees, not 0


def test_pose_history_ignores_out_of_order_and_caps_size():
    h = PoseHistory(max_samples=3)
    h.add(1.0, 1, 0, 0)
    h.add(0.5, 9, 9, 0)                 # older than the last: ignored
    assert len(h) == 1
    for t in (2.0, 3.0, 4.0):
        h.add(t, t, 0, 0)
    assert len(h) == 3
    assert h.at(0.0)[0] == pytest.approx(2.0)   # oldest kept sample


def _obs(tag_id, x, y, rng, accepted=True, t=1.0):
    return Observation("t", t, 0, tag_id, 0, 0, 0, 0, 0, 0, 0, 0, 0, x, y,
                       rng, 100.0, 0.1, accepted)


def test_tracker_weights_near_sightings_more():
    tr = TagTracker()
    tr.add(_obs(5, 1.00, 0.0, rng=0.5))
    est = tr.add(_obs(5, 2.00, 0.0, rng=2.0))
    assert 1.0 < est.x < 1.1            # dominated by the close sighting
    assert est.n_obs == 2 and est.best_range_m == 0.5


def test_tracker_ignores_rejected_observations():
    tr = TagTracker()
    assert tr.add(_obs(7, 9, 9, 1.0, accepted=False)) is None
    assert tr.estimate(7) is None and tr.ids() == []


def test_tracker_tracks_first_and_last_seen():
    tr = TagTracker()
    tr.add(_obs(1, 0, 0, 1.0, t=3.0))
    tr.add(_obs(1, 0, 0, 1.0, t=9.0))
    e = tr.estimate(1)
    assert (e.first_seen_s, e.last_seen_s) == (3.0, 9.0)


def test_reject_reasons():
    tr = TagTracker(DetectionGate(max_pose_err=0.5, max_range_m=3.0))
    assert tr.reject_reason(True, 0.1, 1.0) == ""
    assert tr.reject_reason(False, -1.0, 1.0) == "no_pose"
    assert tr.reject_reason(True, 0.9, 1.0) == "pose_err"
    assert tr.reject_reason(True, 0.1, 3.5) == "range"
