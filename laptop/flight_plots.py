"""
Post-flight analysis: numbers, figures and a written summary from a PX4 .ulg
log and/or a mission run folder (see flight_data.py).

All positions are in the local NED frame, origin at the takeoff point.  Maps
are drawn with East to the right and North up.  Tag heights are not measured:
in the 3-D view tags are drawn on the floor (z = 0), which is an assumption.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
import numpy as np                                  # noqa: E402

from flight_data import FlightLog, MissionRun, SyncResult     # noqa: E402
from tag_localizer import CameraGeometry, camera_to_local_full  # noqa: E402


@dataclass
class TagResult:
    tag_id: int
    x: float                         # mission estimate
    y: float
    n_obs: int
    spread_m: float
    first_seen_s: float
    x_att: Optional[float] = None    # attitude-corrected estimate
    y_att: Optional[float] = None
    truth: Optional[tuple[float, float]] = None

    def err(self) -> Optional[float]:
        return None if self.truth is None else math.hypot(self.x - self.truth[0], self.y - self.truth[1])

    def err_att(self) -> Optional[float]:
        if self.truth is None or self.x_att is None:
            return None
        return math.hypot(self.x_att - self.truth[0], self.y_att - self.truth[1])

    def shift_att(self) -> Optional[float]:
        return None if self.x_att is None else math.hypot(self.x_att - self.x, self.y_att - self.y)


@dataclass
class Analysis:
    flog: Optional[FlightLog]
    run: Optional[MissionRun]
    sync: Optional[SyncResult]
    tags: list[TagResult] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    max_tilt_deg: Optional[float] = None


def _tilt_deg(q) -> float:
    w, x, y, z = q
    # angle between body z (down) and world z (down)
    cz = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cz))))


def analyze(flog: Optional[FlightLog], run: Optional[MissionRun],
            sync: Optional[SyncResult], camera: Optional[CameraGeometry] = None,
            truth: Optional[dict[int, tuple[float, float]]] = None) -> Analysis:
    a = Analysis(flog, run, sync)
    truth = truth or {}
    if flog is not None:
        a.notes.extend(flog.warnings)
    if sync is not None and sync.quality in ("ambiguous", "arm-only"):
        a.notes.append(f"time alignment is {sync.quality}: {sync.detail}")

    m = a.metrics
    if flog is not None:
        w0, w1 = flog.window()
        sel = (flog.t >= w0) & (flog.t <= w1)
        x, y, z = flog.x[sel], flog.y[sel], flog.z[sel]
        m["flight_window_s"] = [round(w0, 2), round(w1, 2)]
        m["armed_duration_s"] = round(w1 - w0, 2)
        m["start_position_m"] = [round(float(x[0]), 3), round(float(y[0]), 3)]
        m["landing_position_m"] = [round(float(x[-1]), 3), round(float(y[-1]), 3)]
        m["return_error_m"] = round(math.hypot(x[-1] - x[0], y[-1] - y[0]), 3)
        m["path_length_m"] = round(float(np.sum(np.hypot(np.diff(x), np.diff(y)))), 2)
        m["max_altitude_m"] = round(float(-(z.min() - z[0])), 3)
        spd = flog.speed()[sel]
        m["max_horizontal_speed_mps"] = round(float(spd.max()), 3)
        td = flog.touchdown_time()
        if td is not None:
            m["touchdown_time_s"] = round(td, 2)
        if flog.att_q is not None:
            att_sel = (flog.att_t >= w0) & (flog.att_t <= w1)
            if att_sel.any():
                a.max_tilt_deg = round(max(_tilt_deg(q) for q in flog.att_q[att_sel][::5]), 1)
                m["max_tilt_deg"] = a.max_tilt_deg

    if run is not None:
        for key in ("TAKEOFF_CMD_SENT", "LANDING_CMD_SENT"):
            ts = run.event_times(key)
            if ts:
                m[key.lower() + "_s"] = round(ts[0], 2)
        if run.event_times("LANDING_CMD_SENT"):
            m["mission_time_to_landing_cmd_s"] = round(run.event_times("LANDING_CMD_SENT")[0], 2)
        for t in run.tags:
            r = TagResult(t["tag_id"], t["x"], t["y"], t["n_obs"], t["spread_m"],
                          t["first_seen_s"], truth=truth.get(t["tag_id"]))
            if flog is not None and sync is not None and camera is not None \
                    and flog.att_q is not None:
                r.x_att, r.y_att = _relocalize(run, flog, sync, camera, t["tag_id"])
            a.tags.append(r)
        unknown = sorted(set(truth) - {t.tag_id for t in a.tags})
        if unknown:
            a.notes.append(f"truth given for tags that were never detected: {unknown}")
        if camera is None and flog is not None and flog.att_q is not None:
            a.notes.append("no camera geometry (--config): attitude-corrected tag positions skipped")
    return a


def _relocalize(run: MissionRun, flog: FlightLog, sync: SyncResult,
                cam: CameraGeometry, tag_id: int) -> tuple[Optional[float], Optional[float]]:
    """Re-localise one tag from its accepted sightings using ULog position + full attitude."""
    sw = sx = sy = 0.0
    for o in run.observations:
        if o["tag_id"] != tag_id or not o["accepted"] or not math.isfinite(o["t_mission_s"]):
            continue
        t = o["t_mission_s"] + sync.offset_s
        if not (flog.t[0] <= t <= flog.t[-1]):
            continue
        q = flog.quat_at(t)
        if q is None:
            return None, None
        px, py, _ = flog.pos_at(t)
        lx, ly = camera_to_local_full(cam, o["tx"], o["ty"], o["tz"], px, py, q)
        w = 1.0 / max(o["range_m"], 0.2) ** 2
        sw += w
        sx += w * lx
        sy += w * ly
    return (sx / sw, sy / sw) if sw > 0 else (None, None)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def plot_overview(a: Analysis, path: Path, area: Optional[tuple] = None) -> None:
    """2-D top view plus altitude and speed against time."""
    fig = plt.figure(figsize=(13, 7.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.6, 1], hspace=0.35, wspace=0.22)
    ax = fig.add_subplot(gs[:, 0])
    ax_z = fig.add_subplot(gs[0, 1])
    ax_v = fig.add_subplot(gs[1, 1])
    f, run, sync = a.flog, a.run, a.sync

    if area is not None:
        x0, x1, y0, y1 = area
        ax.add_patch(plt.Rectangle((y0, x0), y1 - y0, x1 - x0, fill=False, ls="--",
                                   ec="0.5", lw=1.2, label="search area"))

    if f is not None:
        w0, w1 = f.window()
        sel = (f.t >= w0) & (f.t <= w1)
        sc = ax.scatter(f.y[sel], f.x[sel], c=f.t[sel] - w0, s=6, cmap="viridis", zorder=3)
        ax.plot(f.y[sel], f.x[sel], color="0.6", lw=0.8, zorder=2, label="PX4 flight path")
        cb = fig.colorbar(sc, ax=ax, fraction=0.035, pad=0.02)
        cb.set_label("time since arming (s)")
        ax.plot(f.y[sel][0], f.x[sel][0], "^", ms=13, mfc="limegreen", mec="k", zorder=6,
                label="start (arm)")
        ax.plot(f.y[sel][-1], f.x[sel][-1], "s", ms=12, mfc="tomato", mec="k", zorder=6,
                label="landing (disarm)")
    if run is not None and len(run.traj_t):
        ok = np.isfinite(run.traj_x)
        lab = "drone telemetry" if f is not None else "drone telemetry (no .ulg)"
        ax.plot(run.traj_y[ok], run.traj_x[ok], ls=":", color="tab:blue", lw=1.4, zorder=4, label=lab)

    for t in a.tags:
        ax.plot(t.y, t.x, "*", ms=17, mfc="gold", mec="k", zorder=7)
        ax.annotate(f"ID {t.tag_id}", (t.y, t.x), xytext=(7, 7), textcoords="offset points",
                    fontsize=10, weight="bold", zorder=8)
        if t.x_att is not None:
            ax.plot(t.y_att, t.x_att, "x", ms=9, mec="purple", mew=2, zorder=7)
        if t.truth is not None:
            ax.plot(t.truth[1], t.truth[0], "o", ms=9, mfc="none", mec="red", mew=2, zorder=7)
            ax.plot([t.y, t.truth[1]], [t.x, t.truth[0]], "r-", lw=1, zorder=6)
    if a.tags:
        ax.plot([], [], "*", ms=14, mfc="gold", mec="k", ls="", label="tag estimate (mission)")
        if any(t.x_att is not None for t in a.tags):
            ax.plot([], [], "x", mec="purple", mew=2, ls="", label="tag, attitude-corrected")
        if any(t.truth is not None for t in a.tags):
            ax.plot([], [], "o", mfc="none", mec="red", mew=2, ls="", label="tag truth")

    ax.set_xlabel("East, y (m)")
    ax.set_ylabel("North, x (m)")
    ax.set_title("Flight path and AprilTag positions (local frame, origin = takeoff)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    if f is not None:
        w0, w1 = f.window()
        sel = (f.t >= w0 - 2) & (f.t <= w1 + 2)
        tt = f.t[sel] - w0
        ax_z.plot(tt, -(f.z[sel] - f.z[sel][0]), color="tab:green")
        ax_v.plot(tt, f.speed()[sel], color="tab:orange")
        if run is not None and sync is not None:
            for name, color in (("TAKEOFF_CMD_SENT", "tab:blue"), ("LANDING_CMD_SENT", "tab:red")):
                for ts in run.event_times(name):
                    for axx in (ax_z, ax_v):
                        axx.axvline(ts + sync.offset_s - w0, color=color, ls="--", lw=1)
            for t in a.tags:
                if math.isfinite(t.first_seen_s):
                    for axx in (ax_z, ax_v):
                        axx.axvline(t.first_seen_s + sync.offset_s - w0, color="gold", lw=1)
        ax_z.set_xlabel("time since arming (s)")
        ax_v.set_xlabel("time since arming (s)")
    else:
        for axx in (ax_z, ax_v):
            axx.text(0.5, 0.5, "no .ulg given", ha="center", va="center", transform=axx.transAxes)
    ax_z.set_ylabel("altitude (m)")
    ax_v.set_ylabel("horizontal speed (m/s)")
    ax_z.set_title("Altitude  (blue dashed: takeoff cmd, red dashed: landing cmd, gold: first sightings)",
                   fontsize=8)
    ax_v.set_title("Speed", fontsize=9)
    for axx in (ax_z, ax_v):
        axx.grid(alpha=0.3)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_3d(a: Analysis, path: Path) -> None:
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    f = a.flog
    if f is not None:
        w0, w1 = f.window()
        s = (f.t >= w0) & (f.t <= w1)
        ax.plot(f.y[s], f.x[s], -(f.z[s] - f.z[s][0]), color="tab:blue", lw=1.5, label="PX4 flight path")
        ax.plot([f.y[s][0]], [f.x[s][0]], [0], "^", ms=11, mfc="limegreen", mec="k", label="start")
        ax.plot([f.y[s][-1]], [f.x[s][-1]], [0], "s", ms=10, mfc="tomato", mec="k", label="landing")
    for t in a.tags:
        ax.plot([t.y], [t.x], [0], "*", ms=14, mfc="gold", mec="k")
        ax.text(t.y, t.x, 0.03, f"ID {t.tag_id}")
    ax.set_xlabel("East y (m)")
    ax.set_ylabel("North x (m)")
    ax.set_zlabel("altitude (m)")
    ax.set_title("3-D flight path (tags drawn on the floor: height not measured)")
    ax.legend()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Written summary
# ---------------------------------------------------------------------------

def _fmt(v, nd=2):
    return "-" if v is None else f"{v:.{nd}f}"


def write_report(a: Analysis, md_path: Path, json_path: Path, ulg: Optional[str],
                 run_dir: Optional[str]) -> None:
    L = ["# Flight analysis", "",
         f"- PX4 log: `{ulg or 'not given'}`", f"- Mission run: `{run_dir or 'not given'}`"]
    if a.sync:
        L.append(f"- Time alignment ({a.sync.method}, {a.sync.quality}): {a.sync.detail}")
        if a.sync.frame_offset_m is not None:
            L.append(f"- Mean position offset telemetry minus ULog: "
                     f"({a.sync.frame_offset_m[0]:+.3f}, {a.sync.frame_offset_m[1]:+.3f}) m")
    L += ["", "## Flight summary", "", "| Quantity | Value |", "|---|---|"]
    labels = [("armed_duration_s", "Armed duration (s)"), ("start_position_m", "Start (x N, y E) (m)"),
              ("landing_position_m", "Landing (x N, y E) (m)"), ("return_error_m", "Landing vs start, horizontal (m)"),
              ("path_length_m", "Path length (m)"), ("max_altitude_m", "Max altitude (m)"),
              ("max_horizontal_speed_mps", "Max horizontal speed (m/s)"), ("max_tilt_deg", "Max tilt (deg)"),
              ("touchdown_time_s", "Touchdown (s, log time)"),
              ("mission_time_to_landing_cmd_s", "Mission time to landing command (s)")]
    for k, lab in labels:
        if k in a.metrics:
            L.append(f"| {lab} | {a.metrics[k]} |")

    if a.tags:
        L += ["", "## AprilTags", "",
              "| ID | x N (m) | y E (m) | sightings | spread (m) | first seen (s) | attitude-corrected x, y (m) | shift (m) | truth | error (m) | error corrected (m) |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        for t in a.tags:
            att = "-" if t.x_att is None else f"{t.x_att:.2f}, {t.y_att:.2f}"
            tr = "-" if t.truth is None else f"{t.truth[0]:.2f}, {t.truth[1]:.2f}"
            L.append(f"| {t.tag_id} | {t.x:.2f} | {t.y:.2f} | {t.n_obs} | {t.spread_m:.2f} | "
                     f"{_fmt(t.first_seen_s, 1)} | {att} | {_fmt(t.shift_att(), 3)} | {tr} | "
                     f"{_fmt(t.err(), 3)} | {_fmt(t.err_att(), 3)} |")
        errs = [t.err() for t in a.tags if t.err() is not None]
        if errs:
            L += ["", f"Horizontal error against truth: mean {np.mean(errs):.3f} m, max {max(errs):.3f} m "
                      f"over {len(errs)} tags."]
    if a.notes:
        L += ["", "## Warnings", ""] + [f"- {n}" for n in a.notes]
    L += ["", "## How to check these results (do this before using them in the report)", "",
          "- Compare the figure with what you watched: start, path shape and landing spot.",
          "- Confirm the tag IDs and positions against the printed tags and a tape measure.",
          "- If the alignment is not `good`, do not trust the event markers; pass `--offset-s`.",
          "- The `attitude-corrected` column re-localises each sighting with the logged roll and pitch; "
          "a large shift means the mission's heading-only geometry cost accuracy.",
          "- AI tools help with the code and plots; the numbers still have to be verified against the log "
          "(for example plot the raw `vehicle_local_position` in PlotJuggler or Flight Review)."]
    md_path.write_text("\n".join(L) + "\n", encoding="utf-8")

    payload = {"metrics": a.metrics, "notes": a.notes,
               "sync": None if a.sync is None else {
                   "offset_s": a.sync.offset_s, "method": a.sync.method,
                   "quality": a.sync.quality, "rms_m": a.sync.rms_m},
               "tags": [{"id": t.tag_id, "x": t.x, "y": t.y, "x_att": t.x_att, "y_att": t.y_att,
                         "truth": t.truth, "error_m": t.err(), "error_corrected_m": t.err_att()}
                        for t in a.tags]}
    json_path.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
