"""
Loads tag_mission.yaml into typed, validated settings.

Every tunable parameter of the tag-search mission lives in that file (the brief
asks for parameters in configuration files); nothing here is hard-coded except
defaults used when a key is omitted.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

import math

from sweep_planner import Area, path_length
from tag_localizer import CameraGeometry


@dataclass(frozen=True)
class SimSettings:
    """Used only with --simulate."""
    tags: dict = field(default_factory=dict)   # id -> (x, y), local frame
    noise_m: float = 0.0
    takeoff_s: float = 8.0
    max_range_m: float = 4.0
    boxed: dict = field(default_factory=dict)  # id -> SimLink boxed spec (U-shaped foam block)


@dataclass(frozen=True)
class KnownTag:
    """A tag whose floor position is given in the config (known_tags section)."""
    x: float
    y: float
    tag_id: int | None = None       # None: position known, ID not
    blocked: bool = False           # boxed in by a foam block (walls on 3 sides)
    open_side: str = "+x"           # the block's open side (+x/-x/+y/-y, local frame)


@dataclass(frozen=True)
class MissionConfig:
    area: Area
    lane_spacing_m: float
    edge_margin_m: float
    lane_axis: str
    sweep_entry: str
    sweep_return: str
    lane_offset_m: float
    camera: CameraGeometry
    max_pose_err: float
    max_range_m: float
    min_view_elevation_deg: float
    detection_latency_s: float
    update_threshold_m: float
    expected_tags: int | None
    stop_when_all_found: bool
    settle_s: float
    id_max: int | None
    rescue_margin: float | None
    rescue_max_hamming: int
    min_frames: int
    merge_radius_m: float
    cruise_speed_mps: float
    altitude_m: float
    takeoff_wait_s: float
    arrival_radius_m: float
    min_leg_timeout_s: float
    max_mission_s: float
    geofence_margin_m: float
    stale_timeout_s: float
    soft_fence_margin_m: float
    connect_timeout_s: float
    land_wait_s: float
    max_consecutive_misses: int
    output_dir: str
    sim: SimSettings
    # Smooth-trajectory mode (None = stop-and-go goto waypoints). See traj_gen.py.
    traj_speed_mps: float | None = None
    # Refuse takeoff if PX4's heading is more than this far from 0 (no compass: the
    # frame's 'forward' is wherever the drone pointed at power-on). 0 = no check.
    max_start_heading_deg: float = 3.0
    traj_corner_m: float = 0.35
    traj_accel: float = 0.08
    traj_dt_s: float = 0.1
    traj_csv: str | None = None     # fly this t,x,y,z CSV instead of generating the sweep
    traj_slow_speed_mps: float | None = None   # speed where a tag spot is in camera view
    traj_slow_before_m: float = 0.45           # slow zone starts this far short of a spot (in x)
    traj_slow_after_m: float = 0.05            # ... and ends this far past it
    traj_slow_halfwidth_m: float = 0.3         # ... and this far either side of it (in y)
    tag_spots: tuple[tuple[float, float], ...] = ()   # where tags may be placed (local frame)
    far_end_back_m: float = 0.0                # lanes end this far short of the far edge
    avoid_overfly: bool = False                # detour around tag_spots that lie on a lane
    overfly_before_m: float = 0.20             # in line until this far short of a spot ...
    overfly_after_m: float = 0.25              # ... back on the lane this far past it ...
    overfly_side_m: float = 0.45               # ... stepping this far sideways (towards the centre)
    walls: Area | None = None       # arena walls (local frame); fences are kept inside them
    wall_clearance_m: float = 0.5   # nearest the hard geofence may come to a wall
    # look_again: after a partial sighting (1..min_frames-1 frames) that the rest of the
    # path will not see again, pause, hover just short of it, then resume. See tag_mission.
    look_again: bool = False
    look_trigger_after_s: float = 3.0   # no new sighting of the glimpse for this long ...
    look_moved_m: float = 0.6           # ... or the drone this far from where it last saw it
    look_offset_m: float = 0.25         # look point = glimpse - this in x (camera faces +x)
    look_hover_s: float = 5.0           # hover there this long (ends early once confirmed)
    look_max: int = 2                   # looks per mission
    # known_tags: fly to each listed tag and hover over it (downward camera) instead of
    # the sweep. See plan_known_tags.
    known_tags: tuple[KnownTag, ...] = ()
    known_hover_s: float = 3.0          # hover over each tag this long ...
    known_blocked_hover_s: float = 5.0  # ... or this long over a blocked (boxed) tag
    known_open_side_shift_m: float = 0.0   # hover this far towards a block's open side
    known_order: str = "shortest"       # "shortest" (best order) or "listed"


def _num(section: dict, key: str, default: float, positive: bool = False,
         minimum: float | None = None) -> float:
    v = section.get(key, default)
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"{key} must be a number, got {v!r}") from None
    if positive and v <= 0:
        raise ValueError(f"{key} must be positive, got {v}")
    if minimum is not None and v < minimum:
        raise ValueError(f"{key} must be >= {minimum}, got {v}")
    return v


def parse_config(cfg: dict) -> MissionConfig:
    a = cfg.get("search_area")
    if not isinstance(a, dict):
        raise ValueError("search_area (min_x, max_x, min_y, max_y) is required")
    try:
        area = Area(float(a["min_x"]), float(a["max_x"]),
                    float(a["min_y"]), float(a["max_y"]))
    except KeyError as exc:
        raise ValueError(f"search_area is missing {exc}") from None

    sweep = cfg.get("sweep", {}) or {}
    cam = cfg.get("camera", {}) or {}
    det = cfg.get("detection", {}) or {}
    mis = cfg.get("mission", {}) or {}
    sim = cfg.get("simulation", {}) or {}
    traj = cfg.get("trajectory", {}) or {}
    look = cfg.get("look_again", {}) or {}

    walls = None
    w = cfg.get("arena_walls")
    if w is not None:
        try:
            walls = Area(float(w["min_x"]), float(w["max_x"]),
                         float(w["min_y"]), float(w["max_y"]))
        except (KeyError, TypeError) as exc:
            raise ValueError(f"arena_walls needs min_x, max_x, min_y, max_y ({exc})") from None
        if not (walls.contains(0.0, 0.0) and walls.contains(area.min_x, area.min_y)
                and walls.contains(area.max_x, area.max_y)):
            raise ValueError("arena_walls must contain the start (0, 0) and the search area")

    lane_axis = str(sweep.get("lane_axis", "x"))
    if lane_axis not in ("x", "y"):
        raise ValueError("sweep.lane_axis must be 'x' or 'y'")
    entry = str(sweep.get("entry", "direct"))
    if entry not in ("direct", "lateral", "left", "right"):
        raise ValueError("sweep.entry must be 'direct', 'lateral', 'left' or 'right'")
    ret = str(sweep.get("return_leg", "direct"))
    if ret not in ("direct", "lateral"):
        raise ValueError("sweep.return_leg must be 'direct' or 'lateral'")

    expected = det.get("expected_tags")
    if expected is not None:
        expected = int(expected)
        if expected < 1:
            raise ValueError("detection.expected_tags must be >= 1 or omitted")

    tags = {int(k): (float(v[0]), float(v[1]))
            for k, v in (sim.get("tags", {}) or {}).items()}
    boxed = {int(k): dict(v or {}) for k, v in (sim.get("boxed", {}) or {}).items()}

    known = cfg.get("known_tags", {}) or {}
    known_tags: tuple[KnownTag, ...] = ()
    if known.get("enabled", True) and known.get("tags"):
        kt = []
        for i, t in enumerate(known["tags"]):
            try:
                side = str(t.get("open_side", "+x"))
                if side not in ("+x", "-x", "+y", "-y"):
                    raise ValueError(f"open_side must be +x/-x/+y/-y, got {side!r}")
                k = KnownTag(float(t["x"]), float(t["y"]),
                             None if t.get("id") is None else int(t["id"]),
                             bool(t.get("blocked", False)), side)
            except (KeyError, TypeError, ValueError, AttributeError) as exc:
                raise ValueError(f"known_tags.tags[{i}]: {exc}") from None
            if not area.contains(k.x, k.y):
                raise ValueError(f"known_tags.tags[{i}] ({k.x}, {k.y}) is outside the search area")
            kt.append(k)
        known_tags = tuple(kt)
        if expected is None:
            expected = len(known_tags)
    known_order = str(known.get("order", "shortest"))
    if known_order not in ("shortest", "listed"):
        raise ValueError("known_tags.order must be 'shortest' or 'listed'")

    return MissionConfig(
        area=area,
        lane_spacing_m=_num(sweep, "lane_spacing_m", 1.0, positive=True),
        edge_margin_m=_num(sweep, "edge_margin_m", 0.3, minimum=0.0),
        lane_axis=lane_axis,
        sweep_entry=entry,
        sweep_return=ret,
        lane_offset_m=_num(sweep, "lane_offset_m", 0.0, minimum=0.0),
        far_end_back_m=_num(sweep, "far_end_back_m", 0.0, minimum=0.0),
        avoid_overfly=bool(sweep.get("avoid_overfly", False)),
        overfly_before_m=_num(sweep, "overfly_before_m", 0.20, minimum=0.0),
        overfly_after_m=_num(sweep, "overfly_after_m", 0.25, minimum=0.0),
        overfly_side_m=_num(sweep, "overfly_side_m", 0.45, positive=True),
        camera=CameraGeometry(
            pitch_deg=_num(cam, "pitch_deg", 90.0),
            fwd_offset_m=_num(cam, "fwd_offset_m", 0.0),
            right_offset_m=_num(cam, "right_offset_m", 0.0),
            down_offset_m=_num(cam, "down_offset_m", 0.0),
            hfov_deg=_num(cam, "hfov_deg", 91.0, positive=True),
            vfov_deg=_num(cam, "vfov_deg", 82.0, positive=True),
        ),
        max_pose_err=_num(det, "max_pose_err", 0.5, positive=True),
        max_range_m=_num(det, "max_range_m", 3.0, positive=True),
        min_view_elevation_deg=_num(det, "min_view_elevation_deg", 30.0, minimum=0.0),
        detection_latency_s=_num(det, "latency_s", 0.0, minimum=0.0),
        update_threshold_m=_num(det, "update_threshold_m", 0.2, positive=True),
        expected_tags=expected,
        stop_when_all_found=bool(det.get("stop_when_all_found", False)),
        settle_s=_num(det, "settle_s", 0.0, minimum=0.0),
        id_max=(int(det["id_max"]) if det.get("id_max") is not None else None),
        rescue_margin=(float(det["rescue_margin"]) if det.get("rescue_margin") is not None else None),
        rescue_max_hamming=int(_num(det, "rescue_max_hamming", 3, minimum=0)),
        min_frames=int(_num(det, "min_frames", 3, minimum=1)),
        merge_radius_m=_num(det, "merge_radius_m", 0.5, minimum=0.0),
        cruise_speed_mps=_num(mis, "cruise_speed_mps", 0.20, positive=True),
        altitude_m=_num(mis, "altitude_m", 0.5, positive=True),
        takeoff_wait_s=_num(mis, "takeoff_wait_s", 12.0, minimum=0.0),
        arrival_radius_m=_num(mis, "arrival_radius_m", 0.25, positive=True),
        min_leg_timeout_s=_num(mis, "min_leg_timeout_s", 15.0, positive=True),
        max_mission_s=_num(mis, "max_mission_s", 600.0, positive=True),
        geofence_margin_m=_num(mis, "geofence_margin_m", 0.5, minimum=0.0),
        stale_timeout_s=_num(mis, "stale_timeout_s", 3.0, positive=True),
        soft_fence_margin_m=_num(mis, "soft_fence_margin_m", 0.15, minimum=0.0),
        connect_timeout_s=_num(mis, "connect_timeout_s", 30.0, positive=True),
        land_wait_s=_num(mis, "land_wait_s", 8.0, minimum=0.0),
        max_consecutive_misses=int(_num(mis, "max_consecutive_misses", 3, minimum=1)),
        output_dir=str(mis.get("output_dir", "mission_logs")),
        max_start_heading_deg=_num(mis, "max_start_heading_deg", 3.0, minimum=0.0),
        sim=SimSettings(
            tags=tags,
            noise_m=_num(sim, "noise_m", 0.0, minimum=0.0),
            takeoff_s=_num(sim, "takeoff_s", 8.0, minimum=0.0),
            max_range_m=_num(sim, "max_range_m", 4.0, positive=True),
            boxed=boxed,
        ),
        traj_speed_mps=(_num(traj, "speed_mps", 0.12, positive=True) if traj.get("enabled", False) else None),
        traj_corner_m=_num(traj, "corner_radius_m", 0.35, positive=True),
        traj_accel=_num(traj, "accel_mps2", 0.08, positive=True),
        traj_dt_s=_num(traj, "dt_s", 0.1, positive=True),
        traj_csv=(str(traj["csv"]) if traj.get("csv") else None),
        traj_slow_speed_mps=(_num(traj, "slow_speed_mps", 0.06, positive=True)
                             if traj.get("slow_speed_mps") is not None else None),
        traj_slow_before_m=_num(traj, "slow_before_m", 0.45, minimum=0.0),
        traj_slow_after_m=_num(traj, "slow_after_m", 0.05, minimum=0.0),
        traj_slow_halfwidth_m=_num(traj, "slow_halfwidth_m", 0.3, positive=True),
        tag_spots=tuple((float(p[0]), float(p[1])) for p in (cfg.get("tag_spots") or [])),
        walls=walls,
        wall_clearance_m=_num(mis, "wall_clearance_m", 0.5, minimum=0.0),
        look_again=bool(look.get("enabled", False)) and not known_tags,   # known: hovers anyway
        look_trigger_after_s=_num(look, "trigger_after_s", 3.0, positive=True),
        look_moved_m=_num(look, "moved_m", 0.6, positive=True),
        look_offset_m=_num(look, "offset_m", 0.25, minimum=0.0),
        look_hover_s=_num(look, "hover_s", 5.0, positive=True),
        look_max=int(_num(look, "max_looks", 2, minimum=0)),
        known_tags=known_tags,
        known_hover_s=_num(known, "hover_s", 3.0, minimum=0.0),
        known_blocked_hover_s=_num(known, "blocked_hover_s", 5.0, minimum=0.0),
        known_open_side_shift_m=_num(known, "open_side_shift_m", 0.0, minimum=0.0),
        known_order=known_order,
    )


def fences(cfg: MissionConfig, start: tuple[float, float] = (0.0, 0.0)) -> tuple[Area, Area]:
    """
    (hard, soft) fences, used by both the mission and the live map.

    hard: search area + start, grown by geofence_margin_m, then cut back to stay
          wall_clearance_m inside arena_walls (if given). Outside it: land.
    soft: search area + start, grown by soft_fence_margin_m, kept 0.05 m inside the
          hard fence. Outside it: stop searching, fly back to the start, land.
    """
    base = cfg.area.including(*start)
    hard = base.expanded(cfg.geofence_margin_m)
    if cfg.walls is not None:
        try:
            hard = hard.intersection(cfg.walls.expanded(-cfg.wall_clearance_m))
        except ValueError:
            raise ValueError("wall_clearance_m leaves no room inside arena_walls") from None
    soft_m = min(cfg.soft_fence_margin_m, max(cfg.geofence_margin_m - 0.05, 0.0))
    try:
        soft = base.expanded(soft_m).intersection(hard.expanded(-0.05))
    except ValueError:
        raise ValueError("fences leave no room: check arena_walls / wall_clearance_m") from None
    if not (soft.contains(*start) and soft.contains(cfg.area.min_x, cfg.area.min_y)
            and soft.contains(cfg.area.max_x, cfg.area.max_y)):
        raise ValueError(
            f"search area {cfg.area} or start {start} is closer than wall_clearance_m "
            f"({cfg.wall_clearance_m} m) to arena_walls; shrink the area or the clearance")
    return hard, soft


def _deep_merge(base: dict, over: dict) -> dict:
    """base with over's sections on top. A section (top-level dict) merges key by key;
    each key's value is replaced whole, so e.g. simulation.tags in the layout file
    replaces the base's tags instead of adding to them."""
    out = dict(base)
    for k, v in over.items():
        out[k] = {**out[k], **v} if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def find_config(path: str | Path) -> Path:
    """`path` as given if it exists; a bare relative name is also looked for in laptop/
    and in Downloads, so `--config tag_layout.yaml` works from any folder."""
    p = Path(path).expanduser()
    if p.exists() or p.is_absolute():
        if not p.exists():
            raise FileNotFoundError(f"mission config not found: {p}")
        return p
    cands = [Path(__file__).resolve().parent / p, Path.home() / "Downloads" / p]
    for c in cands:
        if c.exists():
            return c
    raise FileNotFoundError(f"mission config not found: {p} (also looked in "
                            + ", ".join(str(c.parent) for c in cands) + ")")


def load_raw(path: str | Path, _seen: tuple = ()) -> dict:
    """The YAML at `path` as a dict, with its `base:` file (if any) merged underneath.

    A layout file from tag_layout_editor.html is short: `base: tag_mission_demo_traj.yaml`
    plus the sections it changes (known_tags, tag_spots, simulation, ...). A relative base
    is looked up next to the file, then in laptop/ (so a file saved in Downloads works).
    """
    p = find_config(path)
    if p.resolve() in _seen:
        raise ValueError(f"config base loop: {p}")
    data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    base = data.pop("base", None)
    if base:
        bp = Path(base)
        if not bp.is_absolute():
            cands = [p.parent / bp, Path(__file__).resolve().parent / bp]
            bp = next((c for c in cands if c.exists()), cands[0])
        data = _deep_merge(load_raw(bp, _seen + (p.resolve(),)), data)
    return data


def load_config(path: str | Path) -> MissionConfig:
    return parse_config(load_raw(path))


def slow_zones(cfg: MissionConfig) -> list[tuple[float, float, float, float]]:
    """
    Boxes (min_x, max_x, min_y, max_y) where the camera can see a tag spot.

    In trajectory mode the drone keeps its takeoff heading (+x). With the camera looking
    straight down (pitch_deg 90) a spot is in view while the drone is over it, so the
    zone is centred on the spot (slow_before_m = slow_after_m); with the old 45 deg
    forward tilt it was in view only from a little short of it in x. Known-tags mode
    stops over each tag instead, so it has no slow zones.
    """
    if not cfg.traj_slow_speed_mps or cfg.known_tags:
        return []
    return [(sx - cfg.traj_slow_before_m, sx + cfg.traj_slow_after_m,
             sy - cfg.traj_slow_halfwidth_m, sy + cfg.traj_slow_halfwidth_m)
            for sx, sy in cfg.tag_spots]


def mission_trajectory(cfg: MissionConfig, plan, start=(0.0, 0.0), end=None):
    """The (t, x, y) trajectory the mission flies for `plan` (same args everywhere).
    Ends at `end` (default: back at `start`).

    Known-tags mode: comes to rest over each plan point that is a tag hover point and
    hovers there (known_hover_s, or known_blocked_hover_s over a blocked tag), all in
    one trajectory so it is uploaded once."""
    import numpy as np
    from traj_gen import smooth_path
    end = start if end is None else end
    if not cfg.known_tags:
        return smooth_path(plan, start, cfg.traj_speed_mps, cfg.traj_corner_m, cfg.traj_accel,
                           dt=cfg.traj_dt_s, end=end, slow_zones=slow_zones(cfg),
                           slow_speed=cfg.traj_slow_speed_mps)
    hover = {(round(x, 6), round(y, 6)): h for x, y, h, _ in known_stops(cfg, start)}
    dt = cfg.traj_dt_s
    rows = [np.array([[0.0, float(start[0]), float(start[1])]])]
    prev = (float(start[0]), float(start[1]))
    for p in [*plan, end]:
        p = (float(p[0]), float(p[1]))
        t0 = rows[-1][-1, 0]
        if math.hypot(p[0] - prev[0], p[1] - prev[1]) > 1e-6:
            seg = smooth_path([], prev, cfg.traj_speed_mps, cfg.traj_corner_m, cfg.traj_accel,
                              dt=dt, end=p)
            seg = seg[1:].copy()
            seg[:, 0] += t0
            rows.append(seg)
            t0 = seg[-1, 0]
        n = int(round(hover.get((round(p[0], 6), round(p[1], 6)), 0.0) / dt))
        if n and p != tuple(end):
            rows.append(np.column_stack([t0 + dt * np.arange(1, n + 1),
                                         np.full(n, p[0]), np.full(n, p[1])]))
        prev = p
    return np.vstack(rows)


def hover_point(cfg: MissionConfig, tag: KnownTag) -> tuple[float, float]:
    """Where the drone hovers so the lens is over `tag` (heading +x, as in trajectory
    mode), shifted known_open_side_shift_m towards a blocked tag's open side."""
    x = tag.x - cfg.camera.fwd_offset_m
    y = tag.y - cfg.camera.right_offset_m
    if tag.blocked and cfg.known_open_side_shift_m:
        sgn = 1.0 if tag.open_side[0] == "+" else -1.0
        if tag.open_side[1] == "x":
            x += sgn * cfg.known_open_side_shift_m
        else:
            y += sgn * cfg.known_open_side_shift_m
    return x, y


def known_stops(cfg: MissionConfig, start=(0.0, 0.0)):
    """[(x, y, hover_s, KnownTag)] in flying order: every known tag once, then home.
    "shortest" tries every order (up to 8 tags) and keeps the shortest round trip."""
    import itertools
    stops = [(*hover_point(cfg, t),
              cfg.known_blocked_hover_s if t.blocked else cfg.known_hover_s, t)
             for t in cfg.known_tags]
    if cfg.known_order == "listed" or len(stops) < 2:
        return stops

    def cost(order):
        return path_length([(s[0], s[1]) for s in order], start, return_to_start=True)
    if len(stops) <= 8:
        return list(min(itertools.permutations(stops), key=cost))
    left, out, cur = list(stops), [], start     # nearest neighbour for big lists
    while left:
        nxt = min(left, key=lambda s: math.hypot(s[0] - cur[0], s[1] - cur[1]))
        left.remove(nxt)
        out.append(nxt)
        cur = (nxt[0], nxt[1])
    return out


def mission_plan(cfg: MissionConfig, start=(0.0, 0.0)):
    """The waypoints the mission flies (same everywhere: mission, live map, tools):
    the hover point over each known tag in known-tags mode, otherwise the sweep."""
    from sweep_planner import detour_spots, plan_sweep
    if cfg.known_tags:
        return [(x, y) for x, y, _, _ in known_stops(cfg, start)]
    plan = plan_sweep(cfg.area, cfg.lane_spacing_m, cfg.edge_margin_m, start, cfg.lane_axis,
                      cfg.sweep_entry, cfg.sweep_return, lane_offset_m=cfg.lane_offset_m,
                      far_end_back_m=cfg.far_end_back_m)
    if cfg.avoid_overfly and cfg.tag_spots and cfg.lane_axis == "x":
        centre_y = (cfg.area.min_y + cfg.area.max_y) / 2
        plan = detour_spots([start, *plan, start], cfg.tag_spots, cfg.overfly_before_m,
                            cfg.overfly_after_m, cfg.overfly_side_m, centre_y)[1:-1]
    return plan
