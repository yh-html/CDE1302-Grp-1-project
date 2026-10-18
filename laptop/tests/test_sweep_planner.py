"""Coverage sweep planner: geometry only."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sweep_planner import Area, path_length, plan_sweep


def test_waypoints_stay_inside_shrunken_area():
    area = Area(0, 4, -2, 2)
    for x, y in plan_sweep(area, 1.0, 0.3):
        assert 0.3 - 1e-9 <= x <= 3.7 + 1e-9
        assert -1.7 - 1e-9 <= y <= 1.7 + 1e-9


def test_lane_spacing_never_exceeds_request_and_covers_width():
    pts = plan_sweep(Area(0, 4, -2, 2), 1.0, 0.3)
    lane_y = sorted({round(p[1], 6) for p in pts})
    assert lane_y[0] == pytest.approx(-1.7) and lane_y[-1] == pytest.approx(1.7)
    gaps = [b - a for a, b in zip(lane_y, lane_y[1:])]
    assert max(gaps) <= 1.0 + 1e-9
    assert len(lane_y) == math.ceil(3.4 / 1.0) + 1


def test_lawn_mower_alternates_direction():
    pts = plan_sweep(Area(0, 4, -2, 2), 1.0, 0.3)
    # Each lane contributes two points with the same y; x directions alternate.
    lanes = [(pts[i], pts[i + 1]) for i in range(0, len(pts), 2)]
    dirs = [math.copysign(1, b[0] - a[0]) for a, b in lanes]
    assert all(d1 != d2 for d1, d2 in zip(dirs, dirs[1:]))
    assert all(a[1] == pytest.approx(b[1]) for a, b in lanes)


def test_chooses_cheapest_orientation_from_start():
    area = Area(0, 4, -2, 2)
    plan = plan_sweep(area, 1.0, 0.3, start=(0.0, 0.0))
    best = path_length(plan, (0, 0), return_to_start=True)
    # A naive plan that begins at the far corner is never better.
    naive = [(3.7, 1.7), (0.3, 1.7), (0.3, 0.85), (3.7, 0.85), (3.7, 0.0),
             (0.3, 0.0), (0.3, -0.85), (3.7, -0.85), (3.7, -1.7), (0.3, -1.7)]
    assert best <= path_length(naive, (0, 0), return_to_start=True) + 1e-9


def test_narrow_area_collapses_to_one_lane():
    pts = plan_sweep(Area(0, 4, -0.2, 0.2), 1.0, 0.3)
    assert len({round(p[1], 6) for p in pts}) == 1


def test_lane_axis_y_swaps_axes():
    pts = plan_sweep(Area(0, 2, -4, 4), 1.0, 0.3, lane_axis="y")
    assert pts and len({round(p[0], 6) for p in pts}) >= 2     # lanes step across x
    # Within a lane the y coordinate spans the long side.
    ys = [p[1] for p in pts]
    assert min(ys) == pytest.approx(-3.7) and max(ys) == pytest.approx(3.7)


def test_start_outside_the_area_is_fine():
    pts = plan_sweep(Area(2, 6, -1, 1), 1.0, 0.2, start=(0.0, 0.0))
    assert pts[0][0] < 3.0          # begins at the near (x) end


@pytest.mark.parametrize("kwargs", [
    dict(lane_spacing_m=0, edge_margin_m=0.3),
    dict(lane_spacing_m=1, edge_margin_m=-0.1),
    dict(lane_spacing_m=1, edge_margin_m=0.3, lane_axis="z"),
])
def test_invalid_arguments_raise(kwargs):
    with pytest.raises(ValueError):
        plan_sweep(Area(0, 4, -2, 2), **kwargs)


def test_area_validation_and_helpers():
    with pytest.raises(ValueError):
        Area(1, 1, 0, 1)
    a = Area(0, 4, -2, 2)
    assert a.contains(0, 0) and not a.contains(5, 0)
    assert a.contains(4.4, 0, margin=0.5)
    assert a.including(-1, 0).min_x == -1
    assert a.expanded(0.5).max_y == 2.5


# -- camera swath ------------------------------------------------------------

from tag_localizer import CameraGeometry                       # noqa: E402
from sweep_planner import suggest_lane_spacing_m, swath_halfwidth_m  # noqa: E402

CAM = CameraGeometry(45.0, 0.054, 0.0, 0.034)      # FOV defaults 91 x 82 deg


def test_swath_is_limited_by_the_minimum_viewing_angle():
    free = swath_halfwidth_m(CAM, 0.5, 3.0, min_elevation_deg=0.0)
    limited = swath_halfwidth_m(CAM, 0.5, 3.0, min_elevation_deg=30.0)
    assert limited < free
    # at 0.5 m altitude a 30 degree viewing angle means within 0.5 / tan(30) of the drone
    assert limited <= 0.5 / math.tan(math.radians(30)) + 0.03


def test_swath_grows_with_altitude_and_range_and_shrinks_with_fov():
    base = swath_halfwidth_m(CAM, 0.5, 3.0, 30.0)
    assert swath_halfwidth_m(CAM, 1.0, 3.0, 30.0) > base
    assert swath_halfwidth_m(CAM, 0.5, 1.0, 0.0) < swath_halfwidth_m(CAM, 0.5, 3.0, 0.0)
    narrow = CameraGeometry(45.0, 0.054, 0.0, 0.034, hfov_deg=40.0, vfov_deg=30.0)
    assert swath_halfwidth_m(narrow, 0.5, 3.0, 0.0) < swath_halfwidth_m(CAM, 0.5, 3.0, 0.0)


def test_suggested_spacing_keeps_a_margin():
    assert suggest_lane_spacing_m(0.66) == pytest.approx(2 * 0.66 * 0.8)
    assert suggest_lane_spacing_m(0.66, safety=1.0) == pytest.approx(1.32)


def _tag_seen_flying_a_lane(lateral_m: float) -> bool:
    """Fly a straight lane in the simulator; was a floor tag `lateral_m` to the side seen?"""
    from drone_link import SimClock, SimLink
    clock = SimClock()
    link = SimLink(1, clock, CAM, {9: (2.0, lateral_m)}, speed_mps=0.5, takeoff_s=0.5,
                   altitude_m=0.5, max_range_m=3.0, min_elevation_deg=30.0)
    seen = []
    link.on_detection(lambda t, p: seen.extend(d.id for d in p.detections))
    link.send_start()
    clock.sleep(1.0)
    link.send_goto(4.0, 0.0)                 # lane along x through y = 0
    clock.sleep(12.0)
    return 9 in seen


def test_swath_matches_what_the_simulated_camera_sees():
    half = swath_halfwidth_m(CAM, 0.5, 3.0, 30.0)
    assert _tag_seen_flying_a_lane(half - 0.06)         # just inside: seen
    assert not _tag_seen_flying_a_lane(half + 0.10)     # just outside: never seen
