"""
Boustrophedon (lawn-mower) sweep path for the tag-search mission.

Pure geometry, no I/O.  All coordinates are in the drone's local frame:
origin at the takeoff point, x = North/forward, y = East/right (NED, as in
the rest of laptop/).

    area = Area(min_x=0, max_x=4, min_y=-2, max_y=2)
    waypoints = plan_sweep(area, lane_spacing_m=1.0, edge_margin_m=0.3)

The returned waypoints do NOT include the return-to-start leg; the mission
appends that itself.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Point = tuple[float, float]


@dataclass(frozen=True)
class Area:
    """Axis-aligned rectangle in the local frame (metres)."""
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def __post_init__(self) -> None:
        if not (self.max_x > self.min_x and self.max_y > self.min_y):
            raise ValueError(
                f"search area must have max > min on both axes, got {self}")

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.min_x - margin <= x <= self.max_x + margin
                and self.min_y - margin <= y <= self.max_y + margin)

    def including(self, x: float, y: float) -> "Area":
        """Smallest area containing this one and the point (x, y)."""
        return Area(min(self.min_x, x), max(self.max_x, x),
                    min(self.min_y, y), max(self.max_y, y))

    def expanded(self, margin: float) -> "Area":
        return Area(self.min_x - margin, self.max_x + margin,
                    self.min_y - margin, self.max_y + margin)


def path_length(points: list[Point], start: Point = (0.0, 0.0),
                return_to_start: bool = False) -> float:
    """Length of start -> points[0] -> ... (-> start)."""
    total, prev = 0.0, start
    for p in points:
        total += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    if return_to_start and points:
        total += math.hypot(start[0] - prev[0], start[1] - prev[1])
    return total


def _lane_positions(lo: float, hi: float, spacing: float) -> list[float]:
    """Evenly spaced lane centres from lo to hi, no further apart than spacing."""
    span = hi - lo
    if span <= 1e-9:
        return [lo]
    n = max(2, math.ceil(span / spacing) + 1)
    return [lo + span * i / (n - 1) for i in range(n)]


def plan_sweep(area: Area, lane_spacing_m: float, edge_margin_m: float,
               start: Point = (0.0, 0.0), lane_axis: str = "x") -> list[Point]:
    """
    Lawn-mower waypoints covering `area`.

    lane_axis "x": lanes run along x, stepping across in y (and vice versa).
    The area is shrunk by edge_margin_m so the drone never aims at the
    boundary.  Lane order and direction are chosen to minimise the flown
    distance from `start` and back to it.
    """
    if lane_spacing_m <= 0:
        raise ValueError("lane_spacing_m must be positive")
    if edge_margin_m < 0:
        raise ValueError("edge_margin_m must be non-negative")
    if lane_axis not in ("x", "y"):
        raise ValueError("lane_axis must be 'x' or 'y'")

    # Work in (u, v): u runs along a lane, v steps between lanes.
    if lane_axis == "x":
        u_lo, u_hi, v_lo, v_hi = area.min_x, area.max_x, area.min_y, area.max_y
        to_xy = lambda u, v: (u, v)          # noqa: E731
    else:
        u_lo, u_hi, v_lo, v_hi = area.min_y, area.max_y, area.min_x, area.max_x
        to_xy = lambda u, v: (v, u)          # noqa: E731

    def shrink(lo: float, hi: float) -> tuple[float, float]:
        if hi - lo <= 2 * edge_margin_m:     # too narrow: use the centre line
            mid = (lo + hi) / 2
            return mid, mid
        return lo + edge_margin_m, hi - edge_margin_m

    u_lo, u_hi = shrink(u_lo, u_hi)
    v_lo, v_hi = shrink(v_lo, v_hi)
    lanes = _lane_positions(v_lo, v_hi, lane_spacing_m)

    best: list[Point] | None = None
    best_cost = math.inf
    for ordered in (lanes, lanes[::-1]):
        for first_forward in (True, False):
            pts: list[Point] = []
            forward = first_forward
            for v in ordered:
                ends = (u_lo, u_hi) if forward else (u_hi, u_lo)
                for u in ends:
                    p = to_xy(u, v)
                    if not pts or math.hypot(p[0] - pts[-1][0],
                                             p[1] - pts[-1][1]) > 1e-9:
                        pts.append(p)
                forward = not forward
            cost = path_length(pts, start, return_to_start=True)
            if cost < best_cost - 1e-9:
                best, best_cost = pts, cost
    assert best is not None
    return best


def swath_halfwidth_m(camera, altitude_m: float, max_range_m: float,
                      min_elevation_deg: float = 0.0,
                      step_m: float = 0.02) -> float:
    """
    Widest sideways distance (m) from the flight line at which a tag lying on
    the floor can come into view, for a camera at the given altitude.

    A floor point (a ahead, d to the right) is visible if it is in front of the
    camera, within max_range_m, inside the horizontal and vertical field of
    view, and seen at least min_elevation_deg above the floor plane (a tag seen
    at a grazing angle is a thin sliver that neither decodes nor localises well).
    As the drone flies along a lane, a tag at sideways offset d is seen
    if some forward distance a makes (a, d) visible, so this is the largest such
    d.  Lane spacing up to 2 x this value leaves no gap; use less for margin.

    `camera` is a tag_localizer.CameraGeometry.  Range and elevation are stand-ins
    for "the tag is big enough and face-on enough to decode", which depends on the
    tag size and mounting and is not modelled: measure both limits with a real tag.
    Assumes tags lie flat on the floor.
    """
    hf = math.radians(camera.hfov_deg) / 2
    vf = math.radians(camera.vfov_deg) / 2
    min_el = math.radians(min_elevation_deg)
    best = 0.0
    n = int(max_range_m / step_m) + 1
    for i in range(n + 1):
        a = i * step_m
        for j in range(n + 1):
            d = j * step_m
            if d <= best:
                continue
            if math.atan2(altitude_m, math.hypot(a, d)) < min_el:
                continue
            tx, ty, tz = camera.body_to_camera(a, d, altitude_m)
            if tz < 0.1:
                continue
            if (math.sqrt(tx * tx + ty * ty + tz * tz) <= max_range_m
                    and abs(math.atan2(tx, tz)) <= hf
                    and abs(math.atan2(ty, tz)) <= vf):
                best = d
    return best


def suggest_lane_spacing_m(halfwidth_m: float, safety: float = 0.8) -> float:
    """Lane spacing that keeps `safety` x the gap-free maximum (2 x halfwidth)."""
    return 2.0 * halfwidth_m * safety

