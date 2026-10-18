"""
Timestamped mission log for the tag-search demonstration.

The brief requires the Python program to record and display the timestamps of
the takeoff command, every tag detection (ID and estimated position), and the
landing command.  Every event is printed (for the examiners) and written to
the run folder:

    events.csv        one row per event (wall time, seconds since takeoff, details)
    observations.csv  every tag sighting, accepted or not, with all intermediate values
    tags.csv          final fused position per tag
    trajectory.csv    drone position at ~5 Hz (for the .ulg / trajectory plot)
    summary.json      run summary

Mission time starts when start_clock() is called, which the mission does at
the moment the takeoff command is sent.
"""

from __future__ import annotations

import csv
import dataclasses
import json
import logging
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from tag_localizer import Observation, TagEstimate

log = logging.getLogger("tag_mission")

_OBS_FIELDS = [f.name for f in dataclasses.fields(Observation)]


class MissionLog:
    def __init__(self, out_dir: Optional[Path],
                 wall: Callable[[], datetime],
                 now: Callable[[], float]) -> None:
        self._wall = wall
        self._now = now
        self._t0: Optional[float] = None
        self.out_dir = out_dir
        self._events: list[dict] = []
        self._last_traj_t = -1e9
        self._files: dict[str, tuple] = {}
        if out_dir is not None:
            out_dir.mkdir(parents=True, exist_ok=True)
            self._open("events", ["time", "t_mission_s", "event", "details"])
            self._open("observations", _OBS_FIELDS)
            self._open("trajectory",
                       ["time", "t_mission_s", "x", "y", "heading_rad", "nav_state"])

    # -- clock -----------------------------------------------------------

    def start_clock(self) -> None:
        """Mission time zero: the moment the takeoff command is initiated."""
        self._t0 = self._now()

    def mission_time(self) -> Optional[float]:
        return None if self._t0 is None else self._now() - self._t0

    def stamp(self) -> str:
        return self._wall().isoformat(timespec="milliseconds")

    # -- files -----------------------------------------------------------

    def _open(self, name: str, fields: list[str]) -> None:
        fh = open(self.out_dir / f"{name}.csv", "w", newline="", encoding="utf-8")
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        self._files[name] = (fh, w)

    def _row(self, name: str, row: dict) -> None:
        if name in self._files:
            fh, w = self._files[name]
            w.writerow(row)
            fh.flush()

    # -- events ----------------------------------------------------------

    def event(self, name: str, **details) -> None:
        tm = self.mission_time()
        stamp = self.stamp()
        self._events.append({"time": stamp, "t_mission_s": tm,
                             "event": name, "details": details})
        clock = stamp[11:23]
        tstr = "  --.-s" if tm is None else f"{tm:+7.1f}s"
        text = "  ".join(f"{k}={v}" for k, v in details.items())
        log.info("[%s] %s | %-22s %s", clock, tstr, name, text)
        self._row("events", {"time": stamp,
                             "t_mission_s": "" if tm is None else round(tm, 3),
                             "event": name,
                             "details": json.dumps(details, default=str)})

    def events(self) -> list[dict]:
        return list(self._events)

    def observation(self, o: Observation) -> None:
        self._row("observations", dataclasses.asdict(o))

    def trajectory(self, x: float, y: float, heading: float,
                   nav_state: str, min_period_s: float = 0.2) -> None:
        t = self._now()
        if t - self._last_traj_t < min_period_s:
            return
        self._last_traj_t = t
        tm = self.mission_time()
        self._row("trajectory", {"time": self.stamp(),
                                 "t_mission_s": "" if tm is None else round(tm, 3),
                                 "x": round(x, 4), "y": round(y, 4),
                                 "heading_rad": round(heading, 4),
                                 "nav_state": nav_state})

    # -- final outputs -----------------------------------------------------

    def write_tags(self, estimates: list[TagEstimate]) -> None:
        if self.out_dir is None:
            return
        with open(self.out_dir / "tags.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["tag_id", "x_m", "y_m", "n_obs", "spread_m",
                        "first_seen_s", "last_seen_s", "best_range_m"])
            for e in estimates:
                w.writerow([e.tag_id, round(e.x, 3), round(e.y, 3), e.n_obs,
                            round(e.spread_m, 3), _r(e.first_seen_s),
                            _r(e.last_seen_s), round(e.best_range_m, 3)])

    def write_summary(self, summary: dict) -> None:
        if self.out_dir is None:
            return
        (self.out_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, default=str), encoding="utf-8")

    def close(self) -> None:
        for fh, _ in self._files.values():
            fh.close()
        self._files.clear()


def _r(v: Optional[float]):
    return "" if v is None else round(v, 2)


def format_tag_table(estimates: list[TagEstimate]) -> str:
    """Final report of detected tags, printed at the end of the mission."""
    if not estimates:
        return "No AprilTags were localised."
    lines = ["  ID |   x (m, N) |   y (m, E) | sightings | spread (m) | first seen",
             "-----+------------+------------+-----------+------------+-----------"]
    for e in estimates:
        first = "    --" if e.first_seen_s is None else f"{e.first_seen_s:6.1f}s"
        lines.append(f"{e.tag_id:4d} | {e.x:10.2f} | {e.y:10.2f} | {e.n_obs:9d} |"
                     f" {e.spread_m:10.2f} | {first}")
    return "\n".join(lines)
