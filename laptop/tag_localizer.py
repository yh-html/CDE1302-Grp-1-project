"""
Tag localisation for the exploration mission: camera frame -> drone body ->
local (takeoff-origin NED) frame, plus aggregation of repeated sightings.

Frames
------
camera : X right, Y image-down, Z forward (AprilTag / OpenCV convention).
body   : NED-aligned, +X forward, +Y right, +Z down, origin at the drone's
         position estimate.  The camera is mounted `pitch_deg` nose-down.
local  : NED, origin at the takeoff point.  x = North, y = East, heading is
         clockwise-positive from North (same as telemetry).

Only horizontal (x, y) is used for the result: the brief scores horizontal
error.  Drone roll/pitch are not in the telemetry packet and are ignored
(small at hover / cruise).  The AprilTag debug packet carries the tag's
translation only, so "relative pose" is the camera-frame translation.

Camera geometry values come from the camera.* section of tag_mission.yaml.
They must be measured on the assembled drone: they set the localisation error.
"""

from __future__ import annotations

import math
from bisect import bisect_left
from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class CameraGeometry:
    """Where the camera sits on the body and how it is tilted."""
    pitch_deg: float        # nose-down tilt of the optical axis
    fwd_offset_m: float     # lens forward of the body origin
    right_offset_m: float   # lens right of the body origin
    down_offset_m: float    # lens below the body origin
    hfov_deg: float = 91.0  # horizontal field of view (firmware calibration, main/at_detect.c)
    vfov_deg: float = 82.0  # vertical field of view

    def __post_init__(self) -> None:
        if not 0.0 <= self.pitch_deg <= 90.0:
            raise ValueError(f"pitch_deg must be in [0, 90], got {self.pitch_deg}")
        if self.hfov_deg <= 0 or self.vfov_deg <= 0:
            raise ValueError("hfov_deg and vfov_deg must be positive")

    def camera_to_body(self, tx: float, ty: float, tz: float
                       ) -> tuple[float, float, float]:
        """Tag translation in the camera frame -> (forward, right, down) in body."""
        p = math.radians(self.pitch_deg)
        fwd = math.cos(p) * tz - math.sin(p) * ty + self.fwd_offset_m
        right = tx + self.right_offset_m
        down = math.sin(p) * tz + math.cos(p) * ty + self.down_offset_m
        return fwd, right, down

    def body_to_camera(self, fwd: float, right: float, down: float
                       ) -> tuple[float, float, float]:
        """Inverse of camera_to_body (used by the simulator)."""
        p = math.radians(self.pitch_deg)
        a = fwd - self.fwd_offset_m
        b = down - self.down_offset_m
        tx = right - self.right_offset_m
        tz = math.cos(p) * a + math.sin(p) * b
        ty = -math.sin(p) * a + math.cos(p) * b
        return tx, ty, tz


def body_to_local(fwd: float, right: float, drone_x: float, drone_y: float,
                  heading_rad: float) -> tuple[float, float]:
    """Body-frame horizontal offset -> local-frame position, using heading."""
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    return (drone_x + fwd * c - right * s,
            drone_y + fwd * s + right * c)


def quat_to_matrix(q) -> list[list[float]]:
    """Rotation matrix body (FRD) -> NED from a unit quaternion (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def camera_to_local_full(geom: CameraGeometry, tx: float, ty: float, tz: float,
                         drone_x: float, drone_y: float, q) -> tuple[float, float]:
    """
    Like camera_to_body + body_to_local, but rotates by the drone's FULL attitude
    (roll, pitch and yaw from the quaternion) instead of heading only.  The
    mission uses the heading-only version because telemetry has no roll/pitch;
    the post-flight analysis uses this to measure what that cost.
    """
    fwd, right, down = geom.camera_to_body(tx, ty, tz)
    r = quat_to_matrix(q)
    return (drone_x + r[0][0] * fwd + r[0][1] * right + r[0][2] * down,
            drone_y + r[1][0] * fwd + r[1][1] * right + r[1][2] * down)


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class PoseHistory:
    """Time-ordered drone poses so a detection can use the pose at its frame time."""

    def __init__(self, max_samples: int = 600) -> None:
        self._t: list[float] = []
        self._p: list[tuple[float, float, float]] = []
        self._max = max_samples

    def add(self, t: float, x: float, y: float, heading: float) -> None:
        if self._t and t < self._t[-1]:
            return                           # ignore out-of-order samples
        self._t.append(t)
        self._p.append((x, y, heading))
        if len(self._t) > self._max:
            del self._t[0], self._p[0]

    def __len__(self) -> int:
        return len(self._t)

    def at(self, t: float) -> Optional[tuple[float, float, float]]:
        """Pose at time t, linearly interpolated; clamped to the ends."""
        if not self._t:
            return None
        if t <= self._t[0]:
            return self._p[0]
        if t >= self._t[-1]:
            return self._p[-1]
        i = bisect_left(self._t, t)
        t0, t1 = self._t[i - 1], self._t[i]
        (x0, y0, h0), (x1, y1, h1) = self._p[i - 1], self._p[i]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return (x0 + f * (x1 - x0), y0 + f * (y1 - y0),
                _wrap_pi(h0 + f * _wrap_pi(h1 - h0)))


@dataclass(frozen=True)
class DetectionGate:
    """Which detections are trusted for localisation."""
    max_pose_err: float = 0.5     # firmware pose reprojection error limit
    max_range_m: float = 3.0      # farther tags have poor depth accuracy


@dataclass
class Observation:
    """One tag sighting with everything needed to audit the estimate."""
    t_wall: str                   # ISO-8601 wall-clock time of receipt
    t_mission_s: Optional[float]  # seconds since the takeoff command
    frame_ms: int                 # drone esp_timer time of the frame
    tag_id: int
    tx: float                     # camera-frame translation (m)
    ty: float
    tz: float
    fwd: float                    # body-frame offset (m)
    right: float
    down: float
    drone_x: float                # drone pose used (local frame)
    drone_y: float
    heading_rad: float
    local_x: float                # estimated tag position, local frame (m)
    local_y: float
    range_m: float
    margin: float
    pose_err: float
    accepted: bool
    reason: str = ""              # why it was rejected, if it was


@dataclass
class TagEstimate:
    tag_id: int
    x: float
    y: float
    n_obs: int
    spread_m: float               # weighted std-dev of the sightings
    first_seen_s: Optional[float]
    last_seen_s: Optional[float]
    best_range_m: float


@dataclass
class _Acc:
    obs: list = field(default_factory=list)
    first_s: Optional[float] = None
    last_s: Optional[float] = None


class TagTracker:
    """Collects observations per tag ID and fuses them into one position."""

    def __init__(self, gate: DetectionGate | None = None) -> None:
        self.gate = gate or DetectionGate()
        self._acc: dict[int, _Acc] = {}

    def reject_reason(self, has_pose: bool, pose_err: float,
                      range_m: float) -> str:
        if not has_pose:
            return "no_pose"
        if pose_err > self.gate.max_pose_err:
            return "pose_err"
        if range_m > self.gate.max_range_m:
            return "range"
        return ""

    def add(self, o: Observation) -> Optional[TagEstimate]:
        """Record an observation; return the updated estimate if it was accepted."""
        if not o.accepted:
            return None
        a = self._acc.setdefault(o.tag_id, _Acc())
        a.obs.append(o)
        if a.first_s is None:
            a.first_s = o.t_mission_s
        a.last_s = o.t_mission_s
        return self.estimate(o.tag_id)

    def estimate(self, tag_id: int) -> Optional[TagEstimate]:
        a = self._acc.get(tag_id)
        if not a or not a.obs:
            return None
        # Nearer sightings are more accurate: weight by 1 / range^2.
        w = [1.0 / max(o.range_m, 0.2) ** 2 for o in a.obs]
        sw = sum(w)
        x = sum(wi * o.local_x for wi, o in zip(w, a.obs)) / sw
        y = sum(wi * o.local_y for wi, o in zip(w, a.obs)) / sw
        var = sum(wi * ((o.local_x - x) ** 2 + (o.local_y - y) ** 2)
                  for wi, o in zip(w, a.obs)) / sw
        return TagEstimate(tag_id, x, y, len(a.obs), math.sqrt(var),
                           a.first_s, a.last_s,
                           min(o.range_m for o in a.obs))

    def ids(self) -> list[int]:
        return sorted(self._acc)

    def estimates(self) -> list[TagEstimate]:
        return [e for e in (self.estimate(i) for i in self.ids()) if e]
