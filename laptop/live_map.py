#!/usr/bin/env python3
"""
Live 2D map of the tag-search mission: run it in a second terminal while
run_tag_mission.py flies the sweep.

It only READS the CSV files the mission writes to its run folder
(trajectory.csv, events.csv, observations.csv), so it cannot interfere with
the drone link: no UDP sockets, no commands. Start it before or after the
mission; with no --run it follows the newest run folder and switches to a
new one as soon as the mission creates it.

What is drawn (top-down, forward/North up, East right; origin = takeoff):
    search area, soft fence and hard geofence (from the same config the mission uses)
    planned sweep (smooth trajectory or waypoints), flown trail, drone + heading
    approximate camera floor footprint, accepted sightings (small dots),
    tag positions as the mission reported them (TAG_DETECTED / TAG_UPDATED, stars),
    reference positions from --truth (or the simulator's tags for a _sim run)

Examples (from laptop/):
    python live_map.py                                         # follow the newest run
    python live_map.py --config tag_mission_small_traj.yaml    # same YAML as the mission!
    python live_map.py --truth "1:1.35,-1.35 2:4.0,-1.35"
    python live_map.py --run mission_logs/20261109_190020_drone1_sim   # replay a finished run
    python live_map.py --config tag_mission_small_traj.yaml --drone-id 1   # + live camera panel

With --drone-id (or --esp-ip) a second panel shows the drone's camera (same
stream as camera_stream.py; close that viewer first, both need UDP 5009). A red
"TAG n" badge flashes on the video when the mission accepts a sighting.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
import select
import socket
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Rectangle

from mission_config import fences, load_config, mission_plan, mission_trajectory
from sweep_planner import plan_sweep
from traj_gen import smooth_path

HERE = Path(__file__).resolve().parent
TAG_COLORS = ["#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf",
              "#e377c2", "#8c564b", "#bcbd22", "#1f77b4", "#7f7f7f"]


def tag_color(tag_id: int) -> str:
    return TAG_COLORS[tag_id % len(TAG_COLORS)]


class CsvTail:
    """Returns rows appended to a CSV file since the last call (complete lines only)."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._pos = 0
        self._header: list[str] | None = None
        self._buf = ""

    def read_new(self) -> list[dict]:
        try:
            with open(self.path, "r", encoding="utf-8", newline="") as fh:
                fh.seek(self._pos)
                chunk = fh.read()
                self._pos = fh.tell()
        except FileNotFoundError:
            return []
        self._buf += chunk
        if "\n" not in self._buf:
            return []
        complete, self._buf = self._buf.rsplit("\n", 1)
        rows = list(csv.reader(io.StringIO(complete)))
        if self._header is None and rows:
            self._header, rows = rows[0], rows[1:]
        return [dict(zip(self._header, r)) for r in rows if r]


def _f(v, default=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def newest_run(root: Path) -> Path | None:
    runs = [p for p in root.iterdir() if p.is_dir() and (p / "trajectory.csv").exists()] \
        if root.is_dir() else []
    return max(runs, key=lambda p: p.name) if runs else None


def parse_truth(text: str | None) -> dict[int, tuple[float, float]]:
    truth = {}
    for token in (text or "").replace(";", " ").split():
        tag_id, xy = token.split(":")
        x, y = xy.split(",")
        truth[int(tag_id)] = (float(x), float(y))
    return truth


def camera_footprint(cfg, x: float, y: float, heading: float) -> np.ndarray | None:
    """Floor polygon (local x, y) seen by the camera, clipped to the detection range."""
    cam = cfg.camera
    h = cfg.altitude_m + cam.down_offset_m           # lens height above the floor (approx.)
    th, tv = math.tan(math.radians(cam.hfov_deg / 2)), math.tan(math.radians(cam.vfov_deg / 2))
    pts = []
    # walk the image border so a clipped top edge still gives a sensible shape
    border = [(u, -1.0) for u in np.linspace(-1, 1, 9)] + [(1.0, v) for v in np.linspace(-1, 1, 9)] \
        + [(u, 1.0) for u in np.linspace(1, -1, 9)] + [(-1.0, v) for v in np.linspace(1, -1, 9)]
    o = cam.camera_to_body(0.0, 0.0, 0.0)
    for u, v in border:
        d = cam.camera_to_body(u * th, v * tv, 1.0)
        fwd, right, down = d[0] - o[0], d[1] - o[1], d[2] - o[2]
        horiz = math.hypot(fwd, right)
        if down > 1e-3:
            s = h / down
            dist = horiz * s
        else:
            dist = float("inf")
        dist = min(dist, cfg.max_range_m)
        if horiz < 1e-6:
            continue
        bf, br = o[0] + fwd / horiz * dist, o[1] + right / horiz * dist
        c, s_ = math.cos(heading), math.sin(heading)
        pts.append((x + bf * c - br * s_, y + bf * s_ + br * c))
    return np.array(pts) if len(pts) >= 3 else None


def planned_path(cfg) -> tuple[np.ndarray, np.ndarray]:
    """(path xy, waypoint xy) in the local frame."""
    plan = mission_plan(cfg)
    wps = np.array(plan) if plan else np.zeros((0, 2))
    if cfg.traj_csv:
        p = Path(cfg.traj_csv)
        if not p.is_absolute() and not p.exists():
            p = HERE.parent / cfg.traj_csv
        data = np.atleast_2d(np.genfromtxt(p, delimiter=",", comments="#"))
        data = data[~np.isnan(data).any(axis=1)]
        xy = data[:, 1:3] - data[0, 1:3]
        return xy, np.zeros((0, 2))
    if cfg.traj_speed_mps:
        tr = mission_trajectory(cfg, plan)
        return tr[:, 1:3], wps
    return np.vstack([[0.0, 0.0], wps, [0.0, 0.0]]), wps


class CameraFeed:
    """
    Receives the drone's JPEG preview (UDP 5009) on a background thread and keeps
    the newest decoded frame. Same wire format and keepalive as camera_stream.py.
    The mission never listens on 5009, so this does not touch the mission's link;
    the firmware detects tags on a copy of the frame while the preview is on.

    Self-healing: the receive loop never dies on a bad packet or decode error
    (it counts them), keepalives go out every KEEPALIVE_S whatever happens, the
    loop is restarted if it ever exits, and after a gap the half-built frames
    are discarded so the next complete frame shows at once. The firmware stops
    streaming 2.5 s after the last keepalive (main/wifi_task.c,
    CAMERA_STREAM_TIMEOUT_MS), so sending every 0.5 s rides out four lost
    keepalives in a row.
    """

    KEEPALIVE_S = 0.5           # firmware keeps streaming for 2.5 s after the last one
    GAP_S = 1.5                 # no frame for this long = shown as a gap on the panel

    def __init__(self, esp_ip: str, fps: int, quality: int, *,
                 stream_port: int | None = None, command_port: int | None = None) -> None:
        import cv2                                            # only needed with a camera
        from camera_stream import COMMAND_PORT, FrameAssembler, drain_socket
        from protocol import CAMERA_STREAM_PORT, build_camera_stream_command
        self._cv2, self._drain = cv2, drain_socket
        self.esp_ip = socket.gethostbyname(esp_ip)
        self._target = (self.esp_ip, command_port or COMMAND_PORT)
        self._on = build_camera_stream_command(True, fps, quality)
        self._off = build_camera_stream_command(False)
        self._asm = FrameAssembler()
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        port = stream_port or CAMERA_STREAM_PORT
        try:
            self._rx.bind(("0.0.0.0", port))
        except OSError as exc:
            raise SystemExit(f"Cannot listen on UDP {port} (is camera_stream.py or "
                             f"tag_stream.py already open?): {exc}")
        self._rx.setblocking(False)
        # Own unbound socket: an ephemeral source port, never 5005/5006/5008 on this laptop.
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()
        self._img = None
        self._frame_id = 0
        self._age_ms = 0
        self._last_rx = None
        self._shown = 0
        self._fps = 0.0
        self.frames = 0                   # decoded frames since start
        self.keepalives = 0               # keepalives sent
        self.errors = 0                   # packets / frames that could not be used
        self.gaps = 0                     # times the video stopped for > GAP_S
        self.restarts = 0                 # receive loop restarted after an unexpected exit
        self.last_error = ""
        self._in_gap = False
        self._running = True
        self._thread = None
        self._start_thread()

    def _start_thread(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="camera-feed", daemon=True)
        self._thread.start()

    def _note_error(self, exc: BaseException) -> None:
        with self._lock:
            self.errors += 1
            self.last_error = f"{type(exc).__name__}: {exc}"[:80]

    def _send_keepalive(self) -> None:
        try:
            self._tx.sendto(self._on, self._target)
            self.keepalives += 1
        except OSError as exc:                                # drone off / WiFi down: retry
            self._note_error(exc)

    def _decode(self, jpeg: bytes):
        cv2 = self._cv2
        try:
            return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
        except cv2.error as exc:                              # corrupt JPEG: skip this frame
            self._note_error(exc)
            return None

    def _step(self, next_keep: float) -> float:
        now = time.monotonic()
        if now >= next_keep:
            next_keep = now + self.KEEPALIVE_S
            self._send_keepalive()
        try:
            select.select([self._rx], [], [], 0.05)
            frame, _, _ = self._drain(self._rx, self._asm, self.esp_ip)
        except OSError as exc:                                # e.g. WSAECONNRESET on Windows
            self._note_error(exc)
            frame = None
        if frame is not None:
            gray = self._decode(frame.jpeg)
            with self._lock:
                if gray is None:
                    self.errors += 1
                else:
                    self._img = gray
                    self._frame_id, self._age_ms = frame.frame_id, frame.age_ms
                    self._last_rx = time.monotonic()
                    self._shown += 1
                    self.frames += 1
                    self._in_gap = False
        now = time.monotonic()
        self._asm.expire(now)
        with self._lock:
            if (self._last_rx is not None and now - self._last_rx > self.GAP_S
                    and not self._in_gap):
                self._in_gap = True
                self.gaps += 1
                self._asm.pending.clear()                     # stale half-frames: start fresh
        return next_keep

    def _loop(self) -> None:
        next_keep = stats_t = time.monotonic()
        while self._running:
            try:
                next_keep = self._step(next_keep)
                now = time.monotonic()
                if now - stats_t >= 1.0:
                    with self._lock:
                        self._fps, self._shown = self._shown / (now - stats_t), 0
                    stats_t = now
            except Exception as exc:                          # noqa: BLE001 - keep the feed alive
                self._note_error(exc)
                time.sleep(0.1)

    def latest(self) -> tuple[np.ndarray | None, str]:
        if self._running and (self._thread is None or not self._thread.is_alive()):
            self.restarts += 1                                # should not happen; heal anyway
            self._start_thread()
        with self._lock:
            img = self._img
            err = f"  err {self.errors}" if self.errors else ""
            if self._last_rx is None:
                return None, (f"waiting for video from {self.esp_ip} "
                              f"(keepalives sent: {self.keepalives}){err}")
            stale = time.monotonic() - self._last_rx
            fps = self._fps if stale <= self.GAP_S else 0.0
            info = (f"#{self._frame_id}  {fps:4.1f} fps  esp {self._age_ms} ms  "
                    f"drops {self._asm.dropped}  gaps {self.gaps}{err}")
            if stale > self.GAP_S:
                info += (f"\n!! no video for {stale:.0f} s - still asking "
                         f"(drone off / out of WiFi?)")
            return img, info

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            self._tx.sendto(self._off, self._target)
        except OSError:
            pass
        self._rx.close()
        self._tx.close()


class LiveMap:
    def __init__(self, cfg, run_dir: Path | None, follow_root: Path | None,
                 truth: dict[int, tuple[float, float]], show_rejected: bool,
                 camera: "CameraFeed | None" = None) -> None:
        self.cfg = cfg
        self.follow_root = follow_root
        self.truth_arg = truth
        self.show_rejected = show_rejected
        self.run_dir: Path | None = None
        self.camera = camera
        if camera is None:
            self.fig, self.ax = plt.subplots(figsize=(8, 8))
            self.cam_ax = None
        else:
            self.fig, (self.ax, self.cam_ax) = plt.subplots(
                1, 2, figsize=(15, 8), gridspec_kw={"width_ratios": [1, 1.25]})
            self._setup_camera_axes()
        self.fig.canvas.manager.set_window_title("Tag mission live map")
        self._draw_static()
        self._open_run(run_dir)

    def _setup_camera_axes(self) -> None:
        ax = self.cam_ax
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(f"Drone camera  ({self.camera.esp_ip})", fontsize=11)
        self.cam_img = ax.imshow(np.zeros((240, 320)), cmap="gray", vmin=0, vmax=255,
                                 interpolation="nearest")
        self.cam_text = ax.text(0.01, 0.99, "waiting for video...", transform=ax.transAxes,
                                va="top", ha="left", family="monospace", fontsize=9,
                                color="#00ff00", bbox=dict(fc="black", alpha=0.5))
        self.cam_tag_text = ax.text(0.5, 0.02, "", transform=ax.transAxes, va="bottom",
                                    ha="center", fontsize=13, weight="bold", color="white",
                                    bbox=dict(fc="#d62728", alpha=0.8, boxstyle="round"),
                                    visible=False)
        self._recent_tags: dict[int, float] = {}       # id -> monotonic time last sighted

    def _update_camera(self) -> None:
        cam = self.camera
        img, info = cam.latest()
        if img is not None:
            if img.shape != self.cam_img.get_array().shape:
                self.cam_img.set_extent((-0.5, img.shape[1] - 0.5, img.shape[0] - 0.5, -0.5))
            self.cam_img.set_data(img)
        self.cam_text.set_text(info)
        # flash which tags the mission accepted in the last second
        now = time.monotonic()
        live = sorted(i for i, t in self._recent_tags.items() if now - t < 1.0)
        self.cam_tag_text.set_text("TAG " + ", ".join(map(str, live)) if live else "")
        self.cam_tag_text.set_visible(bool(live))

    # -- setup ----------------------------------------------------------------

    @staticmethod
    def _xy(x, y):
        """Local (x North, y East) -> plot (horizontal = East, vertical = North)."""
        return y, x

    def _rect(self, area, margin, **kw):
        a = area.including(0.0, 0.0).expanded(margin) if margin is not None else area
        return Rectangle((a.min_y, a.min_x), a.max_y - a.min_y, a.max_x - a.min_x, **kw)

    def _draw_static(self) -> None:
        cfg, ax = self.cfg, self.ax
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("y  East / right (m)")
        ax.set_ylabel("x  North / forward (m)")
        hard, soft = fences(cfg)
        if cfg.walls is not None:
            ax.add_patch(self._rect(cfg.walls, None, fill=False, ec="k", lw=2.5,
                                    label="arena walls"))
        ax.add_patch(self._rect(hard, None, fill=False, ec="#d62728",
                                ls="--", lw=1.2, label="hard geofence (lands)"))
        ax.add_patch(self._rect(soft, None, fill=False, ec="#ff7f0e", ls=":", lw=1.2,
                                label="soft fence (returns)"))
        ax.add_patch(self._rect(cfg.area, None, fc="#1f77b4", alpha=0.07, ec="#1f77b4", lw=1.5,
                                label="search area"))
        try:
            path, wps = planned_path(cfg)
            ax.plot(path[:, 1], path[:, 0], color="0.55", lw=1, ls="--", label="planned path")
            if len(wps):
                ax.plot(wps[:, 1], wps[:, 0], "o", ms=4, mfc="none", mec="0.45")
        except Exception as exc:                       # plan is a nice-to-have
            print(f"(planned path not drawn: {exc})")
        ax.plot(0, 0, marker="s", ms=10, mfc="none", mec="k", mew=1.5, label="start / takeoff")

        fence = (cfg.walls or hard).expanded(0.4)
        ax.set_xlim(fence.min_y, fence.max_y)
        ax.set_ylim(fence.min_x, fence.max_x)

        self.trail, = ax.plot([], [], color="#1f77b4", lw=2, label="flown")
        self.fov = Polygon(np.zeros((3, 2)), closed=True, fc="#2ca02c", alpha=0.12, ec="#2ca02c",
                           lw=0.8, label="camera view (approx.)")
        ax.add_patch(self.fov)
        self.drone, = ax.plot([], [], marker="o", ms=11, color="#1f77b4", mec="k", ls="none")
        self.arrow = None
        self.obs_ok = ax.scatter([], [], s=10, alpha=0.35)
        self.obs_bad = ax.scatter([], [], s=12, marker="x", c="0.5", alpha=0.5,
                                  label="rejected sighting" if self.show_rejected else None)
        self.status = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", ha="left",
                              family="monospace", fontsize=9,
                              bbox=dict(boxstyle="round", fc="white", alpha=0.85))
        self.tag_artists: list = []
        self.truth_artists: list = []
        self.fig.legend(*ax.get_legend_handles_labels(), loc="lower center", ncol=4, fontsize=8,
                        frameon=False)
        self.fig.subplots_adjust(bottom=0.14)

    def _open_run(self, run_dir: Path | None) -> None:
        self.run_dir = run_dir
        self.traj = CsvTail(run_dir / "trajectory.csv") if run_dir else None
        self.events = CsvTail(run_dir / "events.csv") if run_dir else None
        self.obs = CsvTail(run_dir / "observations.csv") if run_dir else None
        self.path_x: list[float] = []
        self.path_y: list[float] = []
        self.pose = None                  # (x, y, heading, nav_state, t_mission)
        self.last_event = "waiting for the mission to start"
        self.tags: dict[int, tuple[float, float, str]] = {}   # id -> (x, y, time)
        self.ok_pts: list[tuple[float, float, int]] = []
        self.bad_pts: list[tuple[float, float]] = []
        self.last_row_wall = None
        self.finished = False
        truth = dict(self.truth_arg)
        if not truth and run_dir and run_dir.name.endswith("_sim"):
            truth = dict(self.cfg.sim.tags)
        self._draw_truth(truth)
        name = run_dir.name if run_dir else "(no run yet)"
        self.ax.set_title(f"Tag mission live map  |  {name}", fontsize=11)

    def _draw_truth(self, truth) -> None:
        for a in self.truth_artists:
            a.remove()
        self.truth_artists = []
        for tag_id, (x, y) in truth.items():
            self.truth_artists.append(self.ax.plot(y, x, marker="+", ms=16, mew=2,
                                                   color=tag_color(tag_id), alpha=0.6)[0])
            self.truth_artists.append(self.ax.annotate(f"ref {tag_id}", (y, x), xytext=(6, -12),
                                                       textcoords="offset points", fontsize=8,
                                                       color=tag_color(tag_id), alpha=0.8))
        self.truth = truth

    # -- update -----------------------------------------------------------------

    def _ingest(self) -> None:
        if self.traj is None:
            return
        for r in self.traj.read_new():
            x, y, h = _f(r.get("x")), _f(r.get("y")), _f(r.get("heading_rad"), 0.0)
            if x is None or y is None:
                continue
            self.path_x.append(x)
            self.path_y.append(y)
            self.pose = (x, y, h, r.get("nav_state", ""), _f(r.get("t_mission_s")))
            self.last_row_wall = time.monotonic()
        for r in self.events.read_new():
            name = r.get("event", "")
            try:
                d = json.loads(r.get("details") or "{}")
            except json.JSONDecodeError:
                d = {}
            if name in ("TAG_DETECTED", "TAG_UPDATED") and "id" in d:
                tm = r.get("t_mission_s") or ""
                self.tags[int(d["id"])] = (float(d["x"]), float(d["y"]),
                                           f"{float(tm):.1f}s" if tm else "")
            if name not in ("POSITION", "WAYPOINT_SENT", "WAYPOINT_REACHED", "PLAN"):
                self.last_event = name
            if name in ("MISSION_END", "DETECT_ONLY_END", "CANCELLED"):
                self.finished = True
        for r in self.obs.read_new():
            x, y = _f(r.get("local_x")), _f(r.get("local_y"))
            if x is None or y is None:
                continue
            if r.get("accepted") == "True":
                self.ok_pts.append((x, y, int(r.get("tag_id", 0))))
                if self.camera is not None:
                    self._recent_tags[int(r.get("tag_id", 0))] = time.monotonic()
            else:
                self.bad_pts.append((x, y))
        # tags.csv is written at the end: it holds the final fused positions
        final = self.run_dir / "tags.csv"
        if self.finished and final.exists():
            with open(final, encoding="utf-8") as fh:
                for r in csv.DictReader(fh):
                    self.tags[int(r["tag_id"])] = (float(r["x_m"]), float(r["y_m"]),
                                                   self.tags.get(int(r["tag_id"]), (0, 0, ""))[2])

    def _maybe_switch_run(self) -> None:
        if self.follow_root is None:
            return
        newest = newest_run(self.follow_root)
        if newest is not None and newest != self.run_dir:
            print(f"Following run: {newest}")
            for a in self.tag_artists:
                a.remove()
            self.tag_artists = []
            self._open_run(newest)

    def update(self, _frame) -> list:
        self._maybe_switch_run()
        self._ingest()
        ax = self.ax
        self.trail.set_data(self.path_y, self.path_x)

        if self.pose:
            x, y, h, state, tm = self.pose
            self.drone.set_data([y], [x])
            if self.arrow is not None:
                self.arrow.remove()
            L = 0.35
            self.arrow = ax.annotate("", xy=(y + L * math.sin(h), x + L * math.cos(h)), xytext=(y, x),
                                     arrowprops=dict(arrowstyle="-|>", color="k", lw=1.5))
            poly = camera_footprint(self.cfg, x, y, h)
            airborne = state not in ("IDLE", "LANDED", "")
            if poly is not None and airborne:
                self.fov.set_xy(poly[:, ::-1])
                self.fov.set_visible(True)
            else:
                self.fov.set_visible(False)

        if self.ok_pts:
            arr = np.array(self.ok_pts)
            self.obs_ok.set_offsets(arr[:, 1::-1])
            self.obs_ok.set_color([tag_color(int(i)) for i in arr[:, 2]])
        if self.show_rejected and self.bad_pts:
            self.obs_bad.set_offsets(np.array(self.bad_pts)[:, ::-1])

        for a in self.tag_artists:
            a.remove()
        self.tag_artists = []
        for tag_id, (tx, ty, seen) in sorted(self.tags.items()):
            c = tag_color(tag_id)
            self.tag_artists.append(ax.plot(ty, tx, marker="*", ms=18, color=c, mec="k")[0])
            label = f"ID {tag_id}\n({tx:.2f}, {ty:.2f})"
            if tag_id in self.truth:
                rx, ry = self.truth[tag_id]
                label += f"\nerr {math.hypot(tx - rx, ty - ry):.2f} m"
            self.tag_artists.append(ax.annotate(label, (ty, tx), xytext=(8, 6),
                                                textcoords="offset points", fontsize=8,
                                                color=c, weight="bold"))

        lines = []
        if self.pose:
            x, y, h, state, tm = self.pose
            lines.append(f"t      {tm:7.1f} s" if tm is not None else "t      (before takeoff)")
            lines.append(f"pos    x={x:5.2f}  y={y:5.2f} m")
            lines.append(f"hdg    {math.degrees(h):6.1f} deg   {state}")
        lines.append(f"event  {self.last_event}")
        exp = self.cfg.expected_tags
        found = ", ".join(str(i) for i in sorted(self.tags)) or "-"
        lines.append(f"tags   {len(self.tags)}{'/' + str(exp) if exp else ''}  [{found}]")
        if self.last_row_wall is not None and not self.finished:
            age = time.monotonic() - self.last_row_wall
            if age > 2.0:
                lines.append(f"!! no new trajectory data for {age:.0f} s")
        if self.finished:
            lines.append("run finished")
        self.status.set_text("\n".join(lines))
        if self.camera is not None:
            self._update_camera()
        return []


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Live 2D map of a tag-search mission run.")
    p.add_argument("--config", default=str(HERE / "tag_mission.yaml"),
                   help="The SAME mission YAML run_tag_mission.py uses (area, fences, plan).")
    p.add_argument("--run", help="Run folder to show (default: follow the newest one).")
    p.add_argument("--out-dir", help="Folder holding the run folders (default: config output_dir).")
    p.add_argument("--truth", metavar="'ID:X,Y ...'",
                   help="Measured tag positions, e.g. '1:1.35,-1.35 2:4.0,-1.35'.")
    p.add_argument("--rejected", action="store_true", help="Also plot rejected sightings.")
    p.add_argument("--fps", type=float, default=5.0, help="Map refresh rate (default 5).")
    cam = p.add_argument_group("camera panel (live drone video beside the map)")
    cam.add_argument("--drone-id", type=int,
                     help="Show the camera of this drone (IP 192.168.1.(200 + id)).")
    cam.add_argument("--esp-ip", help="Drone IP for the camera, instead of --drone-id.")
    cam.add_argument("--cam-fps", type=int, default=6,
                     help="Video frame rate asked of the drone, 1-15 (default 6, light on WiFi).")
    cam.add_argument("--cam-quality", type=int, default=45, help="JPEG quality 10-90 (default 45).")
    args = p.parse_args(argv)

    cfg = load_config(args.config)
    root = Path(args.out_dir or cfg.output_dir)
    if not root.is_absolute():
        root = HERE / root
    if args.run:
        run_dir, follow = Path(args.run), None
        if not (run_dir / "trajectory.csv").exists():
            raise SystemExit(f"{run_dir} has no trajectory.csv")
    else:
        run_dir, follow = newest_run(root), root
        print(f"Following the newest run in {root}" +
              (f" (now {run_dir.name})" if run_dir else " (none yet; waiting)"))

    camera = None
    if args.esp_ip or args.drone_id is not None:
        camera = CameraFeed(args.esp_ip or f"192.168.1.{200 + args.drone_id}",
                            args.cam_fps, args.cam_quality)
    m = LiveMap(cfg, run_dir, follow, parse_truth(args.truth), args.rejected, camera)
    _anim = FuncAnimation(m.fig, m.update, interval=1000 / args.fps, cache_frame_data=False)
    try:
        plt.show()
    finally:
        if camera is not None:
            camera.close()                 # tell the drone to stop streaming
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
