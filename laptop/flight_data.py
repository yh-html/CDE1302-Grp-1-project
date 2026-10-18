"""
Loading and time-alignment for the post-flight analysis.

Two data sources describe the same flight:

  * the PX4 .ulg log (vehicle_local_position, vehicle_status, vehicle_attitude,
    vehicle_land_detected), timestamped in seconds since the flight controller
    booted, and
  * the mission run folder written by run_tag_mission.py (events, tag
    estimates, every observation, the drone telemetry), timestamped in wall
    clock time.

sync_times() finds the offset between them.  Everything is in the local NED
frame: x = North, y = East, z = Down (negative z is altitude above takeoff).
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

ARMED = 2          # vehicle_status.arming_state value for "armed"
_TOPICS = ["vehicle_local_position", "vehicle_status", "vehicle_attitude",
           "vehicle_land_detected"]


# ---------------------------------------------------------------------------
# PX4 log
# ---------------------------------------------------------------------------

@dataclass
class FlightLog:
    path: str
    t: np.ndarray                       # s since boot
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    vz: np.ndarray
    arm_intervals: list[tuple[float, float]] = field(default_factory=list)
    att_t: Optional[np.ndarray] = None
    att_q: Optional[np.ndarray] = None  # (N, 4) w, x, y, z  (body FRD -> NED)
    land_t: Optional[np.ndarray] = None
    landed: Optional[np.ndarray] = None
    warnings: list[str] = field(default_factory=list)

    def pos_at(self, t: float) -> tuple[float, float, float]:
        return (float(np.interp(t, self.t, self.x)),
                float(np.interp(t, self.t, self.y)),
                float(np.interp(t, self.t, self.z)))

    def window(self) -> tuple[float, float]:
        """First arm to last disarm; the whole log if arming was not recorded."""
        if self.arm_intervals:
            return self.arm_intervals[0][0], self.arm_intervals[-1][1]
        return float(self.t[0]), float(self.t[-1])

    def speed(self) -> np.ndarray:
        """Horizontal speed (m/s) per sample.  Uses the logged velocity unless it is
        absent or all zero, in which case it is derived from position."""
        v = np.hypot(self.vx, self.vy)
        if np.isfinite(v).all() and v.max() > 1e-6:
            return v
        if len(self.t) < 3:
            return np.zeros_like(self.t)
        return np.hypot(np.gradient(self.x, self.t), np.gradient(self.y, self.t))

    def quat_at(self, t: float) -> Optional[np.ndarray]:
        if self.att_t is None or len(self.att_t) == 0:
            return None
        i = int(np.clip(np.searchsorted(self.att_t, t), 0, len(self.att_t) - 1))
        if i > 0 and abs(self.att_t[i - 1] - t) < abs(self.att_t[i] - t):
            i -= 1
        return self.att_q[i]

    def touchdown_time(self) -> Optional[float]:
        """Last False -> True transition of vehicle_land_detected.landed."""
        if self.land_t is None or len(self.land_t) < 2:
            return None
        flips = np.where((~self.landed[:-1]) & self.landed[1:])[0]
        return float(self.land_t[flips[-1] + 1]) if len(flips) else None


def _dataset(ulog, name: str):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == 0 and len(d.data["timestamp"]):
            return d.data
    return None


def _arm_intervals(t: np.ndarray, state: np.ndarray) -> list[tuple[float, float]]:
    armed = state == ARMED
    out, start = [], None
    for i, a in enumerate(armed):
        if a and start is None:
            start = float(t[i])
        elif not a and start is not None:
            out.append((start, float(t[i])))
            start = None
    if start is not None:
        out.append((start, float(t[-1])))
    return out


def load_ulg(path: str | Path) -> FlightLog:
    """Read the topics the analysis needs from a PX4 .ulg file."""
    from pyulog import ULog                      # imported late: optional dependency

    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"ULog not found: {path}")
    ulog = ULog(str(path), message_name_filter_list=_TOPICS)

    lp = _dataset(ulog, "vehicle_local_position")
    if lp is None:
        everything = ULog(str(path))
        names = sorted({d.name for d in everything.data_list})
        raise ValueError(f"{path.name} has no vehicle_local_position topic. "
                         f"Topics present: {', '.join(names) or 'none'}")

    t = np.asarray(lp["timestamp"], dtype=float) * 1e-6
    flog = FlightLog(
        path=str(path), t=t,
        x=np.asarray(lp["x"], float), y=np.asarray(lp["y"], float),
        z=np.asarray(lp["z"], float),
        vx=np.asarray(lp["vx"], float), vy=np.asarray(lp["vy"], float),
        vz=np.asarray(lp["vz"], float))

    vs = _dataset(ulog, "vehicle_status")
    if vs is not None:
        flog.arm_intervals = _arm_intervals(
            np.asarray(vs["timestamp"], float) * 1e-6, np.asarray(vs["arming_state"]))
    if not flog.arm_intervals:
        flog.warnings.append("no armed interval in vehicle_status: using the whole log "
                             "as the flight, so start/landing positions are the first/last samples")

    att = _dataset(ulog, "vehicle_attitude")
    if att is not None and all(f"q[{i}]" in att for i in range(4)):
        flog.att_t = np.asarray(att["timestamp"], float) * 1e-6
        flog.att_q = np.column_stack([np.asarray(att[f"q[{i}]"], float) for i in range(4)])
    else:
        flog.warnings.append("no vehicle_attitude topic: attitude-corrected tag positions unavailable")

    ld = _dataset(ulog, "vehicle_land_detected")
    if ld is not None:
        flog.land_t = np.asarray(ld["timestamp"], float) * 1e-6
        flog.landed = np.asarray(ld["landed"]).astype(bool)
    return flog


# ---------------------------------------------------------------------------
# Mission run folder
# ---------------------------------------------------------------------------

@dataclass
class MissionRun:
    folder: Path
    events: list[dict]                      # time(datetime), t_mission_s(float|None), event, details
    t_cmd_wall: Optional[datetime]          # wall time of TAKEOFF_CMD_SENT
    traj_t: np.ndarray                      # mission seconds
    traj_x: np.ndarray
    traj_y: np.ndarray
    tags: list[dict]
    observations: list[dict]
    summary: dict

    def event_times(self, name: str) -> list[float]:
        return [e["t_mission_s"] for e in self.events
                if e["event"] == name and e["t_mission_s"] is not None]


def _rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def _f(v: str, default: float = float("nan")) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def load_run(folder: str | Path) -> MissionRun:
    folder = Path(folder)
    if not folder.is_dir():
        raise FileNotFoundError(f"mission run folder not found: {folder}")

    raw_events = _rows(folder / "events.csv")
    t_cmd = None
    for e in raw_events:
        if e["event"] == "TAKEOFF_CMD_SENT":
            t_cmd = datetime.fromisoformat(e["time"])
            break

    def rel(stamp: str) -> float:
        if t_cmd is None:
            return float("nan")
        return (datetime.fromisoformat(stamp) - t_cmd).total_seconds()

    events = []
    for e in raw_events:
        try:
            details = json.loads(e["details"]) if e["details"] else {}
        except json.JSONDecodeError:
            details = {}
        events.append({"time": datetime.fromisoformat(e["time"]),
                       "t_mission_s": rel(e["time"]), "event": e["event"],
                       "details": details})

    traj = _rows(folder / "trajectory.csv")
    tt = np.array([rel(r["time"]) for r in traj], float)
    tx = np.array([_f(r["x"]) for r in traj], float)
    ty = np.array([_f(r["y"]) for r in traj], float)

    tags = [{"tag_id": int(r["tag_id"]), "x": _f(r["x_m"]), "y": _f(r["y_m"]),
             "n_obs": int(r["n_obs"]), "spread_m": _f(r["spread_m"]),
             "first_seen_s": _f(r["first_seen_s"]), "best_range_m": _f(r["best_range_m"])}
            for r in _rows(folder / "tags.csv")]

    obs = []
    for r in _rows(folder / "observations.csv"):
        obs.append({
            "t_mission_s": rel(r["t_wall"]), "tag_id": int(r["tag_id"]),
            "tx": _f(r["tx"]), "ty": _f(r["ty"]), "tz": _f(r["tz"]),
            "drone_x": _f(r["drone_x"]), "drone_y": _f(r["drone_y"]),
            "local_x": _f(r["local_x"]), "local_y": _f(r["local_y"]),
            "range_m": _f(r["range_m"]), "accepted": r["accepted"] == "True"})

    summary = {}
    if (folder / "summary.json").exists():
        summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    return MissionRun(folder, events, t_cmd, tt, tx, ty, tags, obs, summary)


# ---------------------------------------------------------------------------
# Time alignment
# ---------------------------------------------------------------------------

@dataclass
class SyncResult:
    offset_s: float                 # t_ulog = t_mission + offset_s
    method: str
    rms_m: Optional[float]          # residual after alignment (translation removed)
    frame_offset_m: Optional[tuple[float, float]]   # mean (telemetry - ulog) position
    quality: str                    # good | ambiguous | arm-only | manual
    detail: str


def _residual(run: MissionRun, flog: FlightLog, offset: float, sel: np.ndarray):
    tm = run.traj_t[sel]
    ux = np.interp(tm + offset, flog.t, flog.x)
    uy = np.interp(tm + offset, flog.t, flog.y)
    return run.traj_x[sel] - ux, run.traj_y[sel] - uy


def sync_times(flog: FlightLog, run: MissionRun, mode: str = "auto",
               arm_delay_s: float = 2.0, manual_offset_s: Optional[float] = None,
               window_s: float = 20.0, step_s: float = 0.05) -> SyncResult:
    """
    Find offset_s so that t_ulog = t_mission + offset_s.

    arm  : the ULog arming time minus arm_delay_s (CMD_START -> armed latency).
    auto : start from the arm estimate, then search +-window_s for the offset at
           which the drone telemetry best matches the ULog position (a constant
           translation between the two frames is ignored).
    """
    if manual_offset_s is not None:
        return SyncResult(manual_offset_s, "manual", None, None, "manual",
                          f"offset given on the command line: {manual_offset_s:.2f} s")

    if not flog.arm_intervals:
        raise ValueError("cannot sync: the log has no armed interval. "
                         "Pass --offset-s with the offset in seconds (t_ulog = t_mission + offset).")
    coarse = flog.arm_intervals[0][0] - arm_delay_s
    if mode == "arm":
        return SyncResult(coarse, "arm", None, None, "arm-only",
                          f"first arm at {flog.arm_intervals[0][0]:.2f} s minus assumed delay {arm_delay_s:.1f} s")

    sel = np.isfinite(run.traj_t) & (run.traj_t >= 0) & np.isfinite(run.traj_x)
    if sel.sum() < 20 or (np.ptp(run.traj_x[sel]) + np.ptp(run.traj_y[sel])) < 0.5:
        return SyncResult(coarse, "arm", None, None, "arm-only",
                          "telemetry too short or too still to refine; used the arm time "
                          f"minus {arm_delay_s:.1f} s")

    offsets = coarse + np.arange(-window_s, window_s + step_s, step_s)
    rms = np.empty(len(offsets))
    for i, off in enumerate(offsets):
        rx, ry = _residual(run, flog, off, sel)
        rms[i] = math.sqrt(np.mean((rx - rx.mean()) ** 2 + (ry - ry.mean()) ** 2))
    k = int(np.argmin(rms))
    best, med = float(rms[k]), float(np.median(rms))
    rx, ry = _residual(run, flog, float(offsets[k]), sel)
    quality = "good" if (best <= 0.25 and best <= 0.5 * med) else "ambiguous"
    detail = (f"best offset {offsets[k]:.2f} s (arm-based guess {coarse:.2f} s), "
              f"RMS {best:.3f} m vs median {med:.3f} m over +-{window_s:.0f} s")
    if quality == "ambiguous":
        detail += " - alignment not distinct; check --arm-delay or pass --offset-s"
    return SyncResult(float(offsets[k]), "xcorr", best, (float(rx.mean()), float(ry.mean())),
                      quality, detail)
