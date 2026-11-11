"""
Autonomous AprilTag search mission (CDE1302 mini-project).

    takeoff command -> lawn-mower sweep of the search area -> log every tag
    (ID, local-frame position, relative pose, timestamp) -> return to the start
    -> landing command.

No manual input after the takeoff command.  The mission is written against
DroneLink / Clock (drone_link.py) so it runs unchanged on the real drone and
in the simulator.

Safety behaviour (all lands the drone and says why):
  * telemetry older than stale_timeout_s,
  * position outside the search area plus geofence_margin_m (the start point is
    always inside the fence),
  * Ctrl+C or any unexpected error after takeoff.
Running out of time, finding all expected tags, or repeated waypoint timeouts
end the search early but still return to the start and land.

Limits: there is no touchdown signal in telemetry, so "landing" is the landing
command being sent plus land_wait_s; and no obstacle avoidance (the firmware has
none, and the brief only requires staying inside the boundaries).

Detect-only mode (detect_only=True): a walk-through test with the drone carried
by hand, props off.  It connects, prints every tag with the same localisation as
a real run, and never sends a takeoff, goto, hold or land command (the link is
wrapped so any such call raises).  Ctrl+C or detect_duration_s ends it normally.
Pass truth={tag_id: (x, y)} to get the horizontal error of each tag at the end.
"""

from __future__ import annotations

import dataclasses
import math
import threading
from typing import Callable, Optional

from drone_link import DroneLink
from mission_config import MissionConfig, fences, known_stops, mission_plan, mission_trajectory
from mission_log import MissionLog, format_tag_table, format_truth_table, truth_errors
from protocol import (TRAJ_CHUNK_PTS, TRAJ_MAX_PTS, NAV_FLYING, NAV_ROTATING, NAV_STUCK, NAV_TRAJ,
                      AtDebugPacket)
from sweep_planner import path_length, plan_sweep, square_corner
from traj_gen import smooth_path, stats as traj_stats
from tag_localizer import (DetectionGate, Observation, PoseHistory, TagEstimate,
                           TagTracker, body_to_local)

# Camera intrinsics from main/at_detect.c (pixels)
_F_X, _F_Y, _C_X, _C_Y = 163.5047216, 153.22210511, 154.00573087, 107.10222796
GOTO_RESEND_S = 2.0
DETECT_STATUS_S = 5.0          # detect-only: how often the position is printed
_MOVING = (NAV_ROTATING, NAV_FLYING, NAV_STUCK, NAV_TRAJ)


class MissionAbort(Exception):
    """Unsafe condition: stop and land."""


class _StopExploring(Exception):
    """End the search early; still return to the start and land."""


class _AllTagsFound(_StopExploring):
    """Search done early because every expected tag was found (a normal finish)."""


class _LookAgain(Exception):
    """Pause the sweep to take a closer look at a partial sighting (see _look_candidate)."""

    def __init__(self, cluster: dict, resume_from: tuple[float, float]) -> None:
        super().__init__(f"look again at ({cluster['x']:.2f}, {cluster['y']:.2f})")
        self.cluster = cluster
        self.resume_from = resume_from


class _ReadOnlyLink:
    """Wraps a DroneLink for detect-only mode: reads pass through, every send_* raises."""

    def __init__(self, link) -> None:
        self._link = link

    def __getattr__(self, name):
        if name.startswith("send_"):
            raise AssertionError(f"detect-only mode must never call {name}")
        return getattr(self._link, name)


class TagMission:
    def __init__(self, cfg: MissionConfig, link: DroneLink, clock,
                 mlog: MissionLog, *, drone_id: int,
                 confirm: Optional[Callable[[], bool]] = None,
                 detect_only: bool = False,
                 detect_duration_s: Optional[float] = None,
                 truth: Optional[dict[int, tuple[float, float]]] = None,
                 go_gate: Optional[Callable[[], Optional[str]]] = None,
                 hold_timeout_s: float = 120.0) -> None:
        self.cfg = cfg
        # go_gate: polled while hovering at the start after takeoff; returns "go",
        # "land" or None (no answer yet). None = no hover, sweep straight after takeoff.
        self._go_gate = go_gate
        self.hold_timeout_s = hold_timeout_s
        self._hold_s = 0.0                 # time spent hovering for GO (not counted as search time)
        self.detect_only = detect_only
        self.detect_duration_s = detect_duration_s
        self.truth = dict(truth) if truth else None
        self.link = _ReadOnlyLink(link) if detect_only else link
        self.clock = clock
        self.mlog = mlog
        self.drone_id = drone_id
        self._confirm_fn = confirm

        self.tracker = TagTracker(DetectionGate(cfg.max_pose_err, cfg.max_range_m),
                                  min_frames=cfg.min_frames, merge_radius_m=cfg.merge_radius_m)
        self.history = PoseHistory()
        self._lock = threading.Lock()
        self._reported: dict[int, tuple[float, float]] = {}

        self.start_point = (0.0, 0.0)
        self.plan = mission_plan(cfg, self.start_point)
        self._known_hover = {(round(x, 6), round(y, 6)): h
                             for x, y, h, _ in known_stops(cfg, self.start_point)}
        # Hard fence: land where it is. Soft fence (just inside it): end the search, hold,
        # fly back to the start and land. Both stay wall_clearance_m inside arena_walls.
        self.fence, self.soft_fence = fences(cfg, self.start_point)
        outside = [p for p in self.plan if not self.soft_fence.contains(*p)]
        if outside:
            raise ValueError(f"planned waypoints {outside} are outside the soft fence "
                             f"{self.soft_fence}; they would end the search")

        self._started = False
        self._land_sent = False
        self._reached = 0
        self._missed = 0
        self._stop_reason = ""
        self._all_found_t = None
        self.exit_reason = "completed"

        # look-again state: partial sightings, where we already looked, the current
        # trajectory (absolute xy samples + start time) and the route it follows
        self._glimpses: list[tuple] = []     # (t, x, y, tag_id, frame_ms, drone_x, drone_y)
        self._looked: list[tuple[float, float]] = []
        self._looks = 0
        self._route: Optional[list[tuple[float, float]]] = None
        self._traj_xy = None
        self._traj_t0: Optional[float] = None
        self._traj_dt_s = 0.1
        self._pre: Optional[tuple] = None    # (pts, dt_ms, traj_id) pre-uploaded during takeoff

        link.on_telemetry(self._on_telemetry)
        link.on_detection(self._on_detection)

    # -- background data ---------------------------------------------------

    def _on_telemetry(self, s) -> None:
        self.history.add(s.t, s.x, s.y, s.heading_rad)
        self.mlog.trajectory(s.x, s.y, s.heading_rad, s.nav_state_name)

    def _on_detection(self, recv_t: float, pkt: AtDebugPacket) -> None:
        pose = self.history.at(recv_t - self.cfg.detection_latency_s)
        if pose is None:
            return
        px, py, ph = pose
        with self._lock:
            for det in pkt.detections:
                if det.id < 0 or (self.cfg.id_max is not None and det.id > self.cfg.id_max):
                    continue                          # not one of the course tags
                tx, ty, tz, pose_err, has_pose, rescued = (
                    det.tx, det.ty, det.tz, det.pose_err, det.has_pose, False)
                if (not has_pose and self.cfg.rescue_margin is not None
                        and det.margin >= self.cfg.rescue_margin
                        and det.hamming <= self.cfg.rescue_max_hamming):
                    r = self._rescue_translation(det)
                    if r is not None:
                        tx, ty, tz = r
                        pose_err, has_pose, rescued = 0.0, True, True
                fwd, right, down = self.cfg.camera.camera_to_body(tx, ty, tz)
                lx, ly = body_to_local(fwd, right, px, py, ph)
                rng = math.sqrt(tx * tx + ty * ty + tz * tz)
                reason = self.tracker.reject_reason(has_pose, pose_err, rng)
                obs = Observation(
                    t_wall=self.mlog.stamp(), t_mission_s=self.mlog.mission_time(),
                    frame_ms=pkt.frame_ms, tag_id=det.id,
                    tx=tx, ty=ty, tz=tz,
                    fwd=fwd, right=right, down=down,
                    drone_x=px, drone_y=py, heading_rad=ph,
                    local_x=lx, local_y=ly, range_m=rng,
                    margin=det.margin, pose_err=pose_err,
                    accepted=not reason, reason=reason,
                    hamming=det.hamming, rescued=rescued)
                self.mlog.observation(obs)
                if self.cfg.look_again and (obs.accepted or reason == "pose_err"):
                    self._glimpses.append((self.clock.now(), lx, ly, det.id, pkt.frame_ms, px, py))
                est = self.tracker.add(obs)
                if est is not None:
                    # report only tags that count: enough frames, not a misread of another
                    confirmed = {e.tag_id: e for e in self.tracker.confirmed()}
                    if est.tag_id in confirmed:
                        self._report(confirmed[est.tag_id], obs)

    def _rescue_translation(self, det):
        """Camera-frame position of a confident tag the firmware gave no pose for.

        Uses the tag's pixel centre and the known camera height above the floor
        (ray-floor intersection), assuming the tag lies flat on the floor.  Less
        precise than the firmware pose (it trusts the configured altitude), so
        it is meant to catch tags the firmware gate (hamming <= 1) dropped.
        """
        ray = ((det.cx - _C_X) / _F_X, (det.cy - _C_Y) / _F_Y, 1.0)
        cam = self.cfg.camera
        o = cam.camera_to_body(0.0, 0.0, 0.0)
        b = cam.camera_to_body(*ray)
        down_dir = b[2] - o[2]
        if down_dir < 0.1:                       # ray does not hit the floor
            return None
        s = (self.cfg.altitude_m - cam.down_offset_m) / down_dir
        return ray[0] * s, ray[1] * s, s

    def _report(self, est: TagEstimate, obs: Observation) -> None:
        last = self._reported.get(est.tag_id)
        rel = f"({obs.tx:.2f},{obs.ty:.2f},{obs.tz:.2f})"
        if last is None:
            self.mlog.event("TAG_DETECTED", id=est.tag_id,
                            x=round(est.x, 2), y=round(est.y, 2),
                            first_seen_s=None if est.first_seen_s is None else round(est.first_seen_s, 2),
                            frames=est.n_frames,
                            range_m=round(obs.range_m, 2), cam_xyz_m=rel,
                            frame_ms=obs.frame_ms)
        elif math.hypot(est.x - last[0], est.y - last[1]) > self.cfg.update_threshold_m:
            self.mlog.event("TAG_UPDATED", id=est.tag_id,
                            x=round(est.x, 2), y=round(est.y, 2), sightings=est.n_obs)
        else:
            return
        self._reported[est.tag_id] = (est.x, est.y)

    def _found(self) -> int:
        """Number of confirmed tags (thread-safe)."""
        with self._lock:
            return len(self.tracker.confirmed())

    # -- safety ------------------------------------------------------------

    def _check_safety(self, allow_stop: bool) -> None:
        s = self.link.latest()
        now = self.clock.now()
        if s is None or now - s.t > self.cfg.stale_timeout_s:
            age = "never" if s is None else f"{now - s.t:.1f}s"
            raise MissionAbort(f"telemetry stale (last packet {age} ago)")
        if not self.fence.contains(s.x, s.y):
            raise MissionAbort(f"geofence breach at ({s.x:.2f}, {s.y:.2f})")
        if allow_stop:
            if not self.soft_fence.contains(s.x, s.y):
                raise _StopExploring(f"soft fence: ({s.x:.2f}, {s.y:.2f}) near the boundary, returning")
            tm = self.mlog.mission_time()
            if tm is not None and tm - self._hold_s > self.cfg.max_mission_s:
                raise _StopExploring(f"mission time limit {self.cfg.max_mission_s:.0f}s")
            exp = self.cfg.expected_tags
            if (self.cfg.stop_when_all_found and exp is not None
                    and self._found() >= exp):
                if self.cfg.settle_s <= 0:
                    raise _AllTagsFound(f"all {exp} expected tags found")
                if self._all_found_t is None:        # hover and keep averaging sightings
                    self._all_found_t = now
                    self.link.send_hold()
                    self.mlog.event("ALL_TAGS_FOUND_HOVERING", settle_s=self.cfg.settle_s)
                elif now - self._all_found_t >= self.cfg.settle_s:
                    raise _AllTagsFound(f"all {exp} expected tags found, hovered {self.cfg.settle_s:g}s")

    def _leg_timeout(self, goal: tuple[float, float]) -> float:
        s = self.link.latest()
        dist = math.hypot(goal[0] - s.x, goal[1] - s.y) if s else 5.0
        return self.cfg.min_leg_timeout_s + 2.0 * dist / self.cfg.cruise_speed_mps

    def _wait_arrival(self, goal: tuple[float, float], timeout_s: float,
                      allow_stop: bool) -> bool:
        start = last_sent = self.clock.now()
        saw_moving = False
        while self.clock.now() - start < timeout_s:
            self._check_safety(allow_stop)
            s = self.link.latest()
            if s.nav_state in _MOVING:
                saw_moving = True
            if math.hypot(s.x - goal[0], s.y - goal[1]) <= self.cfg.arrival_radius_m:
                return True
            now = self.clock.now()
            if not saw_moving and now - last_sent >= GOTO_RESEND_S:
                self.link.send_goto(*goal)
                last_sent = now
            self.clock.sleep(0.1)
        return False

    # -- phases ------------------------------------------------------------

    def _connect(self) -> None:
        self.mlog.event("WAITING_FOR_TELEMETRY", drone=self.drone_id)
        self.link.wait_for_link(self.cfg.connect_timeout_s)
        s = self.link.latest()
        self.mlog.event("LINK_ESTABLISHED", drone=self.drone_id,
                        x=round(s.x, 2), y=round(s.y, 2),
                        heading_rad=round(s.heading_rad, 2))

    def describe_plan(self) -> str:
        length = path_length(self.plan, self.start_point, return_to_start=True)
        eta = length / self.cfg.cruise_speed_mps
        a = self.cfg.area
        mode = (f"known tags: hover over {len(self.cfg.known_tags)} listed tag(s), "
                if self.cfg.known_tags else "")
        return (f"{mode}search area x[{a.min_x:g},{a.max_x:g}] y[{a.min_y:g},{a.max_y:g}] m, "
                f"{len(self.plan)} waypoints, {length:.1f} m flown incl. return, "
                f"about {eta:.0f}s at {self.cfg.cruise_speed_mps:g} m/s")

    def _takeoff(self) -> None:
        self.mlog.start_clock()                       # mission time zero
        self.link.send_start()
        self._started = True
        self.mlog.event("TAKEOFF_CMD_SENT", drone=self.drone_id)
        end = self.clock.now() + self.cfg.takeoff_wait_s
        second_pass = self._preupload_first_pass()
        while self.clock.now() < end:
            self._check_safety(allow_stop=False)
            if second_pass is not None and self.clock.now() >= second_pass:
                self._preupload_second_pass()
                second_pass = None
            self.clock.sleep(0.1)
        if second_pass is not None:              # takeoff wait shorter than the gap
            self._preupload_second_pass()
        self.mlog.event("TAKEOFF_WAIT_DONE")

    def _preupload_first_pass(self) -> Optional[float]:
        """Trajectory mode: send the sweep's chunks while the drone climbs, so GO only
        needs CMD_TRAJ_START. The firmware stores chunks in any phase (only START is
        gated on flying in OFFBOARD), and the first burst on a fresh link often loses a
        chunk, so a second pass with the SAME id fills the gaps (same id + total is not a
        new upload; the chunks just overwrite). Offsets are relative to wherever the drone
        is when START arrives (the firmware stores s_traj_org at START), so drift during
        the hover does not matter. Returns when to send the second pass."""
        if not (self.cfg.traj_speed_mps or self.cfg.traj_csv):
            return None
        import random
        pts, dt_ms = self._trajectory_points()
        traj_id = random.randint(1, 255)
        try:
            self.link.upload_trajectory(pts, traj_id)
        except NotImplementedError:
            return None                          # link cannot split upload / start
        self._pre = (pts, dt_ms, traj_id)
        return self.clock.now() + 2.5

    def _preupload_second_pass(self) -> None:
        pts, dt_ms, traj_id = self._pre
        self.link.upload_trajectory(pts, traj_id)
        n_chunks = -(-len(pts) // TRAJ_CHUNK_PTS)
        self.mlog.event("TRAJECTORY_PREUPLOADED", points=len(pts), chunks=n_chunks, passes=2)

    def _hold_for_go(self) -> None:
        """Hover over the launch box until the operator types GO (or LAND / timeout)."""
        self.mlog.event("HOVERING_AT_START", note="type GO + Enter to start the sweep, "
                                                  "LAND + Enter to land",
                        timeout_s=self.hold_timeout_s)
        start = self.clock.now()
        while True:
            self._check_safety(allow_stop=False)
            answer = self._go_gate()
            waited = self.clock.now() - start
            if answer == "go":
                self._hold_s = waited
                self.mlog.event("GO_RECEIVED", hovered_s=round(waited, 1))
                return
            if answer == "land":
                self._hold_s = waited
                raise _StopExploring("operator chose LAND while hovering at the start")
            if waited > self.hold_timeout_s:
                self._hold_s = waited
                raise _StopExploring(f"no GO within {self.hold_timeout_s:.0f}s of hovering, landing")
            self.clock.sleep(0.1)

    def _make_traj(self, points, start, end):
        """(pts offsets, dt_ms, absolute xy) for start -> points -> end, as the firmware plays it."""
        import numpy as np
        dt_ms = round(self.cfg.traj_dt_s * 1000)
        # dt_s rounded to whole ms, exactly as the firmware will play it
        tr = mission_trajectory(dataclasses.replace(self.cfg, traj_dt_s=dt_ms / 1000),
                                list(points), start, end)
        xy = tr[:, 1:3]
        pts = np.column_stack([xy - xy[0], np.zeros(len(xy))])
        if len(pts) > TRAJ_MAX_PTS:
            raise MissionAbort(f"trajectory has {len(pts)} points, firmware limit {TRAJ_MAX_PTS}; "
                               "raise trajectory.dt_s or speed")
        return pts, dt_ms, xy

    def _trajectory_points(self):
        """(N,3) array of (x, y, z) offsets and dt_ms for smooth-trajectory mode."""
        import numpy as np
        dt_ms = round(self.cfg.traj_dt_s * 1000)
        if self.cfg.traj_csv:
            data = np.atleast_2d(np.genfromtxt(self.cfg.traj_csv, delimiter=",", comments="#"))
            data = data[~np.isnan(data).any(axis=1)]
            t = data[:, 0]
            tq = np.arange(t[0], t[-1] + 1e-9, dt_ms / 1000)
            xy = np.column_stack([np.interp(tq, t, data[:, c]) for c in (1, 2)])
            self._route = None                  # unknown route: look-again cannot resume
        else:
            _, dt_ms, xy = self._make_traj(self.plan, self.start_point, self.start_point)
            self._route = [self.start_point, *self.plan, self.start_point]
        self._traj_xy = xy
        pts = np.column_stack([xy - xy[0], np.zeros(len(xy))])
        if len(pts) > TRAJ_MAX_PTS:
            raise MissionAbort(f"trajectory has {len(pts)} points, firmware limit {TRAJ_MAX_PTS}; "
                               "raise trajectory.dt_s or speed")
        return pts, dt_ms

    def _explore_trajectory(self) -> None:
        if self._pre is not None:
            pts, dt_ms, pre_id = self._pre       # already on the drone (_preupload_*)
            self._pre = None
        else:
            pts, dt_ms = self._trajectory_points()
            pre_id = None
        duration = (len(pts) - 1) * dt_ms / 1000
        self.mlog.event("TRAJECTORY_START", points=len(pts), duration_s=round(duration, 1),
                        source=self.cfg.traj_csv or ("known tags" if self.cfg.known_tags else "generated sweep"),
                        preuploaded=pre_id is not None)
        watch = self.cfg.look_again and self._route is not None
        try:
            while True:
                try:
                    self._play_trajectory(pts, dt_ms, duration, xy=self._traj_xy, watch=watch,
                                          pre_id=pre_id)
                    return
                except _LookAgain as la:
                    self._look(la.cluster)
                    pts, dt_ms = self._resume_trajectory(la.resume_from)
                    duration = (len(pts) - 1) * dt_ms / 1000
        except _StopExploring:
            self.link.send_hold()          # nav_cancel: drone hovers where it is
            raise

    def _play_trajectory(self, pts, dt_ms: int, duration: float, *, xy=None,
                         watch: bool = False, must_start: bool = True,
                         log_end: bool = True, pre_id: Optional[int] = None,
                         returning: bool = False) -> bool:
        """Upload and play a trajectory until it ends. With watch=True, raise _LookAgain
        when a partial sighting needs a closer look. Returns False only if it never
        started and must_start is False. pre_id: the upload is already on the drone
        under this id, so try a bare START first; if that does not start within 1.5 s,
        fall back to full uploads (new ids). returning=True: the flight home, so no
        search stops (soft fence, all-tags-found) and it ends when the trajectory does."""
        started = False
        allow_stop = not returning
        deadline = self.clock.now() + duration + 15.0
        if pre_id is not None:
            self.link.start_trajectory(pre_id, dt_ms)
            t_end = self.clock.now() + 1.5
            while self.clock.now() < t_end:
                self._check_safety(allow_stop)
                if self.link.latest().nav_state == NAV_TRAJ:
                    started = True
                    break
                self.clock.sleep(0.1)
            if not started:
                self.mlog.event("TRAJECTORY_PREUPLOAD_NOT_STARTED",
                                note="falling back to full upload")
        for attempt in range(1, 6):
            if started:
                break
            self.link.send_trajectory(pts, dt_ms)
            t_end = self.clock.now() + 1.5
            while self.clock.now() < t_end:
                self._check_safety(allow_stop)
                if self.link.latest().nav_state == NAV_TRAJ:
                    started = True
                    break
                self.clock.sleep(0.1)
            if started:
                break
            self.mlog.event("TRAJECTORY_RETRY", attempt=attempt)
        if not started:
            if not must_start:
                return False
            raise MissionAbort("drone did not start the trajectory")
        self._traj_t0, self._traj_dt_s = self.clock.now(), dt_ms / 1000
        while ((self.link.latest().nav_state == NAV_TRAJ
                or (self._all_found_t is not None and not returning))
               and self.clock.now() < deadline):
            self._check_safety(allow_stop)
            if watch and xy is not None and self._all_found_t is None:
                idx = min(len(xy) - 1,
                          int((self.clock.now() - self._traj_t0) / self._traj_dt_s))
                cluster = self._look_candidate(xy, idx)
                if cluster is not None:
                    self.link.send_hold()
                    raise _LookAgain(cluster, (float(xy[idx][0]), float(xy[idx][1])))
            self.clock.sleep(0.1)
        if log_end:
            self.mlog.event("TRAJECTORY_END", tags_found=self._found())
        return True

    # -- look again -----------------------------------------------------------

    def _glimpse_clusters(self) -> list[dict]:
        """Sightings grouped by floor position (merge_radius_m), skipping any near a
        confirmed tag: what the camera saw but could not confirm."""
        r = max(self.cfg.merge_radius_m, 0.1)
        with self._lock:
            confirmed = [(e.x, e.y) for e in self.tracker.confirmed()]
            glimpses = list(self._glimpses)
        clusters: list[dict] = []
        for t, x, y, tid, fms, dx, dy in glimpses:
            if any(math.hypot(x - cx, y - cy) < r for cx, cy in confirmed):
                continue
            c = next((c for c in clusters if math.hypot(x - c["x"], y - c["y"]) < r), None)
            if c is None:
                c = {"x": x, "y": y, "n": 0, "ids": set(), "frames": set(),
                     "last_t": t, "last_drone": (dx, dy)}
                clusters.append(c)
            c["n"] += 1
            c["x"] += (x - c["x"]) / c["n"]
            c["y"] += (y - c["y"]) / c["n"]
            c["ids"].add(tid)
            c["frames"].add(fms)
            if t >= c["last_t"]:
                c["last_t"], c["last_drone"] = t, (dx, dy)
        return clusters

    def _look_candidate(self, xy, idx: int) -> Optional[dict]:
        """A glimpse worth a closer look now: not confirmed, not looked at, not seen for a
        while (or left behind), and the rest of the planned path will not bring it back
        into view (the camera only sees a spot from just short of it in x)."""
        if not self.cfg.look_again or self._looks >= self.cfg.look_max:
            return None
        s = self.link.latest()
        if s is None:
            return None
        now = self.clock.now()
        r = max(self.cfg.merge_radius_m, 0.1)
        rest = xy[idx:]
        for c in self._glimpse_clusters():
            if any(math.hypot(c["x"] - lx, c["y"] - ly) < r for lx, ly in self._looked):
                continue
            quiet = now - c["last_t"] >= self.cfg.look_trigger_after_s
            moved = math.hypot(s.x - c["last_drone"][0],
                               s.y - c["last_drone"][1]) > self.cfg.look_moved_m
            if not (quiet or moved):
                continue
            ahead = ((rest[:, 0] >= c["x"] - self.cfg.traj_slow_before_m)
                     & (rest[:, 0] <= c["x"] + self.cfg.traj_slow_after_m)
                     & (abs(rest[:, 1] - c["y"]) <= 0.2))
            if ahead.any():
                continue                    # the path passes it again: let it look then
            return c
        return None

    def _cluster_confirmed(self, x: float, y: float) -> Optional[TagEstimate]:
        r = max(self.cfg.merge_radius_m, 0.1)
        with self._lock:
            return next((e for e in self.tracker.confirmed()
                         if math.hypot(e.x - x, e.y - y) < r), None)

    def _look(self, c: dict) -> None:
        """Fly (as a trajectory, so the heading and the forward camera stay put) to just
        short of the glimpse in x, then hover so the camera sees over a boxing wall."""
        self._looks += 1
        self._looked.append((c["x"], c["y"]))
        f = self.soft_fence
        lx = min(max(c["x"] - self.cfg.look_offset_m, f.min_x + 0.05), f.max_x - 0.05)
        ly = min(max(c["y"], f.min_y + 0.05), f.max_y - 0.05)
        self.mlog.event("LOOK_AGAIN_START", n=f"{self._looks}/{self.cfg.look_max}",
                        x=round(c["x"], 2), y=round(c["y"], 2),
                        ids=",".join(str(i) for i in sorted(c["ids"])),
                        frames=len(c["frames"]), look_x=round(lx, 2), look_y=round(ly, 2))
        t0 = self.clock.now()
        try:
            s = self.link.latest()
            if math.hypot(s.x - lx, s.y - ly) > 0.05:
                pts, dt_ms, _ = self._make_traj([], (s.x, s.y), (lx, ly))
                ok = self._play_trajectory(pts, dt_ms, (len(pts) - 1) * dt_ms / 1000,
                                           must_start=False, log_end=False)
                if not ok:
                    self.mlog.event("LOOK_AGAIN_MOVE_FAILED",
                                    note="trajectory not started; hovering here")
            t0 = self.clock.now()
            while (self._cluster_confirmed(c["x"], c["y"]) is None
                   and self.clock.now() - t0 < self.cfg.look_hover_s):
                self._check_safety(allow_stop=True)
                self.clock.sleep(0.1)
        finally:                       # also when all tags got found mid-look (normal stop)
            est = self._cluster_confirmed(c["x"], c["y"])
            self.mlog.event("LOOK_AGAIN_RESULT", confirmed=est is not None,
                            id=None if est is None else est.tag_id,
                            frames=None if est is None else est.n_frames,
                            hovered_s=round(self.clock.now() - t0, 1))

    def _resume_trajectory(self, resume_from: tuple[float, float]):
        """New trajectory from where the drone is, rejoining the route at the point it was
        interrupted, then the rest of the route (incl. the squared-off way home)."""
        import numpy as np
        route = self._route
        px, py = resume_from
        best = None
        for k in range(len(route) - 1):
            (ax, ay), (bx, by) = route[k], route[k + 1]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            u = 0.0 if L2 < 1e-12 else min(1.0, max(0.0, ((px - ax) * vx + (py - ay) * vy) / L2))
            qx, qy = ax + u * vx, ay + u * vy
            d = math.hypot(px - qx, py - qy)
            if best is None or d < best[0] - 1e-9:
                best = (d, k, (qx, qy))
        _, k, q = best
        s = self.link.latest()
        cur = (s.x, s.y)
        rest = [q, *route[k + 1:-1]]
        pts, dt_ms, xy = self._make_traj(rest, cur, route[-1])
        self._route = [cur, q, *route[k + 1:]]
        self._traj_xy = xy
        duration = (len(pts) - 1) * dt_ms / 1000
        self.mlog.event("TRAJECTORY_RESUME", points=len(pts), duration_s=round(duration, 1),
                        rejoin_x=round(q[0], 2), rejoin_y=round(q[1], 2))
        return np.asarray(pts), dt_ms

    def _explore(self) -> None:
        if self.cfg.traj_speed_mps or self.cfg.traj_csv:
            self._explore_trajectory()
            return
        total = len(self.plan)
        prev_misses = 0
        for i, (wx, wy) in enumerate(self.plan, start=1):
            self._check_safety(allow_stop=True)
            self.mlog.event("WAYPOINT_SENT", n=f"{i}/{total}",
                            x=round(wx, 2), y=round(wy, 2))
            self.link.send_goto(wx, wy)
            if self._wait_arrival((wx, wy), self._leg_timeout((wx, wy)), True):
                self._reached += 1
                prev_misses = 0
                self.mlog.event("WAYPOINT_REACHED", n=f"{i}/{total}",
                                tags_found=self._found())
                hover_s = self._known_hover.get((round(wx, 6), round(wy, 6)), 0.0)
                if hover_s:                       # known-tags mode: hover over the tag
                    t0 = self.clock.now()
                    while self.clock.now() - t0 < hover_s:
                        self._check_safety(allow_stop=True)
                        self.clock.sleep(0.1)
            else:
                self._missed += 1
                prev_misses += 1
                self.link.send_hold()
                self.mlog.event("WAYPOINT_TIMEOUT", n=f"{i}/{total}")
                if prev_misses >= self.cfg.max_consecutive_misses:
                    raise _StopExploring(f"{prev_misses} consecutive waypoint timeouts")

    def _return_trajectory(self, cur, corner, goal) -> bool:
        """Fly home as a trajectory (heading held), not CMD_GOTO: the firmware's goto
        turns to face the goal first, and spinning ~180-300 deg while flying made the
        optical-flow position slip (11 Nov, 20:02 and 20:14). No slow zones. False if
        it is already there or the drone would not start it (caller falls back to goto)."""
        import numpy as np
        if math.hypot(goal[0] - cur[0], goal[1] - cur[1]) <= self.cfg.arrival_radius_m:
            return False
        dt_ms = round(self.cfg.traj_dt_s * 1000)
        via = [corner] if corner is not None else []
        tr = smooth_path(via, cur, self.cfg.traj_speed_mps or self.cfg.cruise_speed_mps,
                         self.cfg.traj_corner_m, self.cfg.traj_accel, dt=dt_ms / 1000, end=goal)
        xy = tr[:, 1:3]
        pts = np.column_stack([xy - xy[0], np.zeros(len(xy))])
        if len(pts) < 2 or len(pts) > TRAJ_MAX_PTS:
            return False
        duration = (len(pts) - 1) * dt_ms / 1000
        self.mlog.event("RETURN_TRAJECTORY", x=goal[0], y=goal[1], points=len(pts),
                        duration_s=round(duration, 1),
                        via_corner=None if corner is None else (round(float(corner[0]), 2), round(float(corner[1]), 2)))
        return self._play_trajectory(pts, dt_ms, duration, must_start=False,
                                     log_end=False, returning=True)

    def _return_and_land(self, squared: bool = False) -> None:
        goal = self.start_point
        s = self.link.latest()
        corner = None
        if squared and self.cfg.sweep_return == "lateral" and s is not None:
            corner = square_corner(goal, (s.x, s.y), self.cfg.lane_axis)
            if corner is not None and not self.soft_fence.contains(*corner):
                corner = None
        if ((self.cfg.traj_speed_mps or self.cfg.traj_csv) and s is not None
                and self._return_trajectory((s.x, s.y), corner, goal)):
            ok = self._wait_arrival(goal, self.cfg.min_leg_timeout_s, allow_stop=False)
        else:
            if corner is not None:
                self.mlog.event("RETURN_VIA_CORNER", x=round(corner[0], 2), y=round(corner[1], 2))
                self.link.send_goto(*corner)
                if not self._wait_arrival(corner, self._leg_timeout(corner), allow_stop=False):
                    self.mlog.event("RETURN_CORNER_TIMEOUT")
            self.mlog.event("RETURNING_TO_START", x=goal[0], y=goal[1])
            self.link.send_goto(*goal)
            ok = self._wait_arrival(goal, self._leg_timeout(goal), allow_stop=False)
        s = self.link.latest()
        self.mlog.event("START_AREA_REACHED" if ok else "RETURN_TIMEOUT",
                        x=round(s.x, 2), y=round(s.y, 2),
                        dist_to_start_m=round(math.hypot(s.x - goal[0], s.y - goal[1]), 2))
        self.link.send_land(3)
        self._land_sent = True
        self.mlog.event("LANDING_CMD_SENT", x=round(s.x, 2), y=round(s.y, 2))
        self.clock.sleep(self.cfg.land_wait_s)

    def _emergency_land(self, reason: str) -> None:
        self.exit_reason = reason
        self.mlog.event("ABORT", reason=reason)
        if self._started and not self._land_sent:
            try:
                self.link.send_land(3)
                self._land_sent = True
                self.mlog.event("LANDING_CMD_SENT", emergency=True)
            except Exception as exc:                       # noqa: BLE001
                self.mlog.event("LAND_FAILED", error=exc)

    def _detect_only(self) -> None:
        """Walk-through test: send nothing, localise and print tags while the drone is carried."""
        self.mlog.start_clock()                       # mission time zero = start of the walk
        self.mlog.event("DETECT_ONLY_START",
                        note="no commands are sent; carry the drone by hand along the sweep, "
                             "Ctrl+C to finish")
        t_end = (None if self.detect_duration_s is None
                 else self.clock.now() + self.detect_duration_s)
        next_status = self.clock.now()
        stale = False
        try:
            while t_end is None or self.clock.now() < t_end:
                now = self.clock.now()
                s = self.link.latest()
                if s is None or now - s.t > self.cfg.stale_timeout_s:
                    if not stale:
                        stale = True
                        self.mlog.event("TELEMETRY_STALE")
                else:
                    if stale:
                        stale = False
                        self.mlog.event("TELEMETRY_RESUMED")
                    if now >= next_status:
                        next_status = now + DETECT_STATUS_S
                        self.mlog.event("POSITION", x=round(s.x, 2), y=round(s.y, 2),
                                        heading_deg=round(math.degrees(s.heading_rad)),
                                        inside_area=self.fence.contains(s.x, s.y),
                                        tags_found=self._found())
                exp = self.cfg.expected_tags
                if (self.cfg.stop_when_all_found and exp is not None
                        and self._found() >= exp):
                    self.mlog.event("SEARCH_STOPPED", reason=f"all {exp} expected tags found")
                    break
                self.clock.sleep(0.1)
        except KeyboardInterrupt:
            self.mlog.event("DETECT_ONLY_STOPPED", reason="Ctrl+C")

    # -- entry point -------------------------------------------------------

    def _heading_ok(self) -> bool:
        """No compass: PX4's 'forward' is wherever the drone pointed at power-on, so a
        drone turned in the start box flies the whole sweep rotated (11 Nov 20:29: 6 deg
        put it near the left wall). Refuse takeoff unless the heading is near 0."""
        limit = self.cfg.max_start_heading_deg
        s = self.link.latest()
        if not limit or s is None:
            return True
        err = math.degrees(math.atan2(math.sin(s.heading_rad), math.cos(s.heading_rad)))
        if abs(err) <= limit:
            return True
        self.exit_reason = f"heading {err:+.1f} deg at the start, limit {limit:g}"
        self.mlog.event("HEADING_NOT_ZEROED", heading_deg=round(err, 1), limit_deg=limit,
                        fix="power-cycle the drone in the start box facing forward, "
                            "do not move it, then run again")
        return False

    def run(self) -> int:
        code = 0
        try:
            self.link.start()
            self._connect()
            self.mlog.event("PLAN", detail=self.describe_plan())
            if self.detect_only:
                self._detect_only()
                self.mlog.event("DETECT_ONLY_END", tags=self._found())
                return 0
            if not self._heading_ok():
                return 1
            if self._confirm_fn is not None and not self._confirm_fn():
                self.exit_reason = "cancelled before takeoff"
                self.mlog.event("CANCELLED")
                return 0
            if not self._heading_ok():           # re-check: it may have been moved meanwhile
                return 1
            self._takeoff()
            try:
                if self._go_gate is not None:
                    self._hold_for_go()
                self._explore()
            except _StopExploring as stop:
                self._stop_reason = str(stop)
                self.mlog.event("SEARCH_STOPPED", reason=self._stop_reason)
                # All tags found is a normal finish: come home the same squared-off way
                # as the end of the sweep. Soft fence / time limit / timeouts: straight home.
                squared = isinstance(stop, _AllTagsFound)
            else:
                squared = False          # sweep finished: the plan already ends at the corner
            self._return_and_land(squared)
            self.mlog.event("MISSION_END", tags=self._found())
        except MissionAbort as exc:
            code = 1
            self._emergency_land(str(exc))
        except KeyboardInterrupt:
            code = 130
            self._emergency_land("interrupted by operator")
        except Exception as exc:                           # noqa: BLE001
            code = 1
            self._emergency_land(f"{type(exc).__name__}: {exc}")
        finally:
            self._finish(code)
            try:
                self.link.stop()
            except Exception:                              # noqa: BLE001
                pass
        return code

    def _finish(self, code: int) -> None:
        with self._lock:
            ests = self.tracker.confirmed()
            ignored = self.tracker.ignored()
        self.mlog.write_tags(ests)
        events = {e["event"]: e for e in self.mlog.events()}
        errors = truth_errors(ests, self.truth) if self.truth else None
        self.mlog.write_summary({
            "mode": "detect-only" if self.detect_only else "mission",
            "truth_errors": errors,
            "drone_id": self.drone_id, "exit_code": code,
            "exit_reason": self.exit_reason, "search_stopped": self._stop_reason,
            "hover_for_go_s": round(self._hold_s, 1) if self._go_gate else None,
            "waypoints_planned": len(self.plan), "waypoints_reached": self._reached,
            "waypoints_missed": self._missed,
            "takeoff_cmd_time": events.get("TAKEOFF_CMD_SENT", {}).get("time"),
            "landing_cmd_time": events.get("LANDING_CMD_SENT", {}).get("time"),
            "mission_time_to_landing_cmd_s":
                events.get("LANDING_CMD_SENT", {}).get("t_mission_s"),
            "tags": [{"id": e.tag_id, "x": round(e.x, 3), "y": round(e.y, 3),
                      "sightings": e.n_obs, "spread_m": round(e.spread_m, 3)}
                     for e in ests],
            "ignored_ids": [{"id": e.tag_id, "x": round(e.x, 3), "y": round(e.y, 3),
                             "frames": e.n_frames, "reason": why} for e, why in ignored],
        })
        print("\nDetected AprilTags (local frame, origin = takeoff point):", flush=True)
        print(format_tag_table(ests), flush=True)
        if ignored:
            print("\nIgnored IDs (not counted as tags):", flush=True)
            for e, why in ignored:
                print(f"  {e.tag_id:3d} at ({e.x:5.2f}, {e.y:5.2f}): {why}", flush=True)
        if errors:
            print("\nHorizontal error against the reference positions:", flush=True)
            print(format_truth_table(errors), flush=True)
        self.mlog.close()
