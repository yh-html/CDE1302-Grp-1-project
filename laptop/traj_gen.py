"""
Smooth, slow trajectory through a list of waypoints (pure geometry, no I/O).

    pts, dt = smooth_path(plan, start=(0, 0), speed=0.12)

Corners are rounded with a quadratic Bezier fillet, then the path is timed with
a speed profile that:
  * never exceeds `speed`,
  * starts and ends at rest, ramping at `accel` (m/s^2),
  * slows in curves so the sideways acceleration stays below `lat_accel`.
The drone therefore never stops at a waypoint, which keeps the camera moving
smoothly and avoids the pitch/roll jerk of stop-and-go goto legs.

Output: array of (t, x, y) at a fixed period `dt`, local frame, starting at
`start`.  Offsets from the first point are what the drone plays back
(laptop/send_trajectory.py convention, z = 0 holds altitude).
"""

from __future__ import annotations

import math

import numpy as np

Point = tuple[float, float]


def _fillet(corner: np.ndarray, a: np.ndarray, c: np.ndarray, d: float, n: int = 24) -> np.ndarray:
    """Quadratic Bezier from d before `corner` (towards a) to d after it (towards c)."""
    p0 = corner + (a - corner) / np.linalg.norm(a - corner) * d
    p2 = corner + (c - corner) / np.linalg.norm(c - corner) * d
    s = np.linspace(0.0, 1.0, n)[:, None]
    return (1 - s) ** 2 * p0 + 2 * (1 - s) * s * corner + s ** 2 * p2


def rounded_polyline(points: list[Point], corner_radius_m: float) -> np.ndarray:
    """Dense polyline through `points` with every interior corner rounded."""
    P = [np.array(p, dtype=float) for p in points]
    # drop repeated points
    Q = [P[0]]
    for p in P[1:]:
        if np.linalg.norm(p - Q[-1]) > 1e-6:
            Q.append(p)
    out = [Q[0][None, :]]
    for i in range(1, len(Q) - 1):
        a, b, c = Q[i - 1], Q[i], Q[i + 1]
        u, v = (a - b), (c - b)
        cosang = float(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v)))
        if cosang < -0.9999:                       # straight on: no corner to round
            out.append(b[None, :])
            continue
        d = min(corner_radius_m, 0.5 * np.linalg.norm(u), 0.5 * np.linalg.norm(v))
        out.append(_fillet(b, a, c, d))
    out.append(Q[-1][None, :])
    path = np.vstack(out)
    # remove duplicate consecutive samples
    keep = np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-9]
    return path[keep]


def _resample_by_arclength(path: np.ndarray, ds: float) -> tuple[np.ndarray, np.ndarray]:
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    sq = np.arange(0.0, s[-1], ds)
    sq = np.r_[sq, s[-1]]
    return sq, np.column_stack([np.interp(sq, s, path[:, 0]), np.interp(sq, s, path[:, 1])])


def _curvature(xy: np.ndarray, ds: float) -> np.ndarray:
    d1 = np.gradient(xy, ds, axis=0)
    d2 = np.gradient(d1, ds, axis=0)
    num = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    den = np.maximum((d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5, 1e-9)
    return num / den


def smooth_path(points: list[Point], start: Point = (0.0, 0.0), speed: float = 0.12,
                corner_radius_m: float = 0.35, accel: float = 0.08,
                lat_accel: float = 0.08, dt: float = 0.05,
                end: Point | None = None) -> np.ndarray:
    """
    Timed smooth path start -> points... -> end (default: back to start).
    Returns an (N, 3) array of t, x, y sampled every `dt` seconds.
    """
    if speed <= 0 or accel <= 0 or lat_accel <= 0 or dt <= 0:
        raise ValueError("speed, accel, lat_accel and dt must be positive")
    end = start if end is None else end
    poly = rounded_polyline([start, *points, end], corner_radius_m)
    ds = 0.01
    s, xy = _resample_by_arclength(poly, ds)
    total = s[-1]
    if total < 1e-6:
        raise ValueError("path has zero length")

    kappa = _curvature(xy, ds)
    v_curve = np.sqrt(lat_accel / np.maximum(kappa, 1e-6))
    v_start = np.sqrt(2.0 * accel * s)
    v_end = np.sqrt(2.0 * accel * (total - s))
    v = np.minimum.reduce([np.full_like(s, speed), v_curve, v_start, v_end])
    v = np.maximum(v, 0.02)                        # creep, so time stays finite
    # light smoothing of the profile (about 0.3 m window) to soften speed changes
    k = max(3, int(0.3 / ds)) | 1
    kern = np.ones(k) / k
    vp = np.pad(v, (k // 2, k // 2), mode="edge")
    v = np.maximum(np.convolve(vp, kern, mode="valid"), 0.02)

    # time at each arclength sample: dt_i = ds / v_mid
    vmid = 0.5 * (v[1:] + v[:-1])
    t = np.r_[0.0, np.cumsum(np.diff(s) / vmid)]
    tq = np.arange(0.0, t[-1], dt)
    tq = np.r_[tq, t[-1]]
    x = np.interp(tq, t, xy[:, 0])
    y = np.interp(tq, t, xy[:, 1])
    return np.column_stack([tq, x, y])


def stats(traj: np.ndarray) -> dict:
    t, x, y = traj[:, 0], traj[:, 1], traj[:, 2]
    dt = np.diff(t)
    v = np.hypot(np.diff(x), np.diff(y)) / dt
    a = np.diff(v) / dt[1:]
    return {"duration_s": float(t[-1]), "length_m": float(np.hypot(np.diff(x), np.diff(y)).sum()),
            "max_speed": float(v.max()), "mean_speed": float(v.mean()),
            "max_accel": float(np.abs(a).max()) if len(a) else 0.0,
            "n_points": int(len(t))}


def write_csv(path, traj: np.ndarray, z: float = 0.0) -> None:
    with open(path, "w", newline="") as f:
        f.write("t,x,y,z\n")
        for t, x, y in traj:
            f.write(f"{t:.3f},{x:.4f},{y:.4f},{z:.4f}\n")
