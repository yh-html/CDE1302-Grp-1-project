"""
Drone link + clocks for the tag-search mission.

The mission talks to a DroneLink and a Clock only, so the same mission code
runs against

  * UdpLink  - the real ESP32 drone over WiFi (telemetry UDP 5005, commands
               UDP 5006, AprilTag detections UDP 5008), and
  * SimLink  - a kinematic simulator with a simulated clock, used for dry runs
               and the tests.  It models the drone's command rules (CMD_GOTO
               only after takeoff, etc.) but NOT PX4, VFH, dynamics or sensor
               noise beyond a Gaussian on the tag translation.
"""

from __future__ import annotations

import logging
import math
import random
import socket
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Callable, Optional

from comms import CommsNode
from protocol import (
    CMD_GOTO, CMD_HOLD, CMD_LAND, CMD_START,
    NAV_ARRIVED, NAV_FLYING, NAV_IDLE, NAV_STATE_NAMES,
    AT_DEBUG_PORT, AtDebugPacket, CommandPacket, TagDetection,
    TelemetryPacket, parse_at_debug,
)
from tag_localizer import CameraGeometry

log = logging.getLogger("tag_mission")


# ---------------------------------------------------------------------------
# Clocks
# ---------------------------------------------------------------------------

class RealClock:
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def wall(self) -> datetime:
        return datetime.now().astimezone()


class SimClock:
    """Simulated time: sleep() advances time and steps every registered listener."""

    def __init__(self, start_wall: datetime | None = None, max_step_s: float = 0.05):
        self._t = 0.0
        self._start = start_wall or datetime(2026, 11, 12, 10, 0, 0)
        self._listeners: list[Callable[[float], None]] = []
        self._max_step = max_step_s

    def on_advance(self, cb: Callable[[float], None]) -> None:
        self._listeners.append(cb)

    def now(self) -> float:
        return self._t

    def wall(self) -> datetime:
        return self._start + timedelta(seconds=self._t)

    def sleep(self, seconds: float) -> None:
        remaining = max(0.0, seconds)
        while remaining > 1e-12:
            dt = min(self._max_step, remaining)
            self._t += dt
            for cb in self._listeners:
                cb(dt)
            remaining -= dt


# ---------------------------------------------------------------------------
# Link interface
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TelemetrySample:
    t: float            # clock.now() when received
    x: float            # local frame, metres (North)
    y: float            # East
    heading_rad: float
    nav_state: int

    @property
    def nav_state_name(self) -> str:
        return NAV_STATE_NAMES.get(self.nav_state, f"UNKNOWN({self.nav_state})")


TelemetryHandler = Callable[[TelemetrySample], None]
DetectionHandler = Callable[[float, AtDebugPacket], None]


class DroneLink(ABC):
    def __init__(self) -> None:
        self._telem_handlers: list[TelemetryHandler] = []
        self._det_handlers: list[DetectionHandler] = []
        self._latest: Optional[TelemetrySample] = None

    def on_telemetry(self, cb: TelemetryHandler) -> None:
        self._telem_handlers.append(cb)

    def on_detection(self, cb: DetectionHandler) -> None:
        self._det_handlers.append(cb)

    def latest(self) -> Optional[TelemetrySample]:
        return self._latest

    def _emit_telemetry(self, s: TelemetrySample) -> None:
        self._latest = s
        for cb in self._telem_handlers:
            cb(s)

    def _emit_detection(self, t: float, pkt: AtDebugPacket) -> None:
        for cb in self._det_handlers:
            cb(t, pkt)

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def wait_for_link(self, timeout_s: float) -> None:
        """Block until telemetry arrives; raise TimeoutError otherwise."""

    @abstractmethod
    def send_start(self) -> None: ...

    @abstractmethod
    def send_goto(self, x: float, y: float) -> None: ...

    @abstractmethod
    def send_hold(self) -> None: ...

    @abstractmethod
    def send_land(self, repeat: int = 3) -> None: ...


# ---------------------------------------------------------------------------
# Real drone over UDP
# ---------------------------------------------------------------------------

class UdpLink(DroneLink):
    def __init__(self, drone_id: int, telem_port: int = 5005,
                 cmd_port: int = 5006, at_port: int = AT_DEBUG_PORT) -> None:
        super().__init__()
        self.drone_id = drone_id
        self._telem_port = telem_port
        self._at_port = at_port
        # No config_path: waypoints are relative to the takeoff point.
        self._comms = CommsNode(listen_port=telem_port, cmd_port=cmd_port,
                                config_path=None)
        self._cond = threading.Condition()
        self._running = False
        self._at_sock: Optional[socket.socket] = None
        self._at_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        # Start offset (0, 0) so goals are relative to takeoff; also clears any
        # offset a previous setup.yaml run left on the drone.
        self._comms.set_nav_tags([], {})
        self._comms.on_telemetry(self._on_packet)
        self._comms.start()
        self._at_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._at_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._at_sock.bind(("", self._at_port))
        except OSError as exc:
            self._comms.stop()
            raise RuntimeError(
                f"Cannot listen for AprilTag detections on UDP {self._at_port}: {exc}") from exc
        self._at_sock.settimeout(0.5)
        self._running = True
        self._at_thread = threading.Thread(target=self._at_loop,
                                           name="tag-mission-at", daemon=True)
        self._at_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._at_thread:
            self._at_thread.join(timeout=2.0)
        if self._at_sock:
            self._at_sock.close()
            self._at_sock = None
        self._comms.stop()

    def _on_packet(self, pkt: TelemetryPacket, source_ip: str) -> None:
        if pkt.drone_id != self.drone_id:
            return
        with self._cond:
            self._emit_telemetry(TelemetrySample(
                time.monotonic(), pkt.ned_x, pkt.ned_y, pkt.heading_rad, pkt.nav_state))
            self._cond.notify_all()

    def _at_loop(self) -> None:
        while self._running:
            try:
                data, _ = self._at_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            pkt = parse_at_debug(data)
            if pkt is not None and pkt.drone_id == self.drone_id:
                self._emit_detection(time.monotonic(), pkt)

    def wait_for_link(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while self._latest is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    heard = sorted(self._comms.known_drones())
                    raise TimeoutError(
                        f"No telemetry from drone {self.drone_id} on UDP "
                        f"{self._telem_port} (heard drones: {heard or 'none'}).")
                self._cond.wait(timeout=min(0.5, remaining))

    def _send(self, cmd: CommandPacket, name: str) -> None:
        if not self._comms.send_command(self.drone_id, cmd):
            raise RuntimeError(f"Failed to send {name}: drone {self.drone_id} IP is unknown.")

    def send_start(self) -> None:
        self._send(CommandPacket(CMD_START), "CMD_START")

    def send_goto(self, x: float, y: float) -> None:
        self._send(CommandPacket(CMD_GOTO, goal_x=x, goal_y=y), "CMD_GOTO")

    def send_hold(self) -> None:
        self._send(CommandPacket(CMD_HOLD), "CMD_HOLD")

    def send_land(self, repeat: int = 3) -> None:
        for i in range(repeat):
            self._send(CommandPacket(CMD_LAND), "CMD_LAND")
            if i < repeat - 1:
                time.sleep(0.1)


# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------

class SimLink(DroneLink):
    """
    Kinematic stand-in for the drone, driven by a SimClock.

    Obeys the firmware's command rules: CMD_START only on the ground, CMD_GOTO
    only after takeoff completes, CMD_LAND at any time after START.  Flies
    straight lines at speed_mps, turning to face the direction of travel, and
    reports tags on the floor that fall inside the camera's field of view.
    """

    def __init__(self, drone_id: int, clock: SimClock, camera: CameraGeometry,
                 tags: dict[int, tuple[float, float]], *,
                 speed_mps: float = 0.10, takeoff_s: float = 8.0,
                 altitude_m: float = 0.5, hfov_deg: float = 91.0,
                 vfov_deg: float = 82.0, max_range_m: float = 4.0,
                 min_elevation_deg: float = 0.0, noise_m: float = 0.0, seed: int = 1,
                 start: tuple[float, float] = (0.0, 0.0),
                 heading_rad: float = 0.0,
                 telem_hz: float = 10.0, packet_hz: float = 10.0) -> None:
        super().__init__()
        self.drone_id = drone_id
        self._clock = clock
        self._cam = camera
        self._tags = dict(tags)
        self._speed = speed_mps
        self._takeoff_s = takeoff_s
        self._alt = altitude_m
        self._hfov = math.radians(hfov_deg)
        self._vfov = math.radians(vfov_deg)
        self._max_range = max_range_m
        self._min_el = math.radians(min_elevation_deg)
        self._noise = noise_m
        self._rng = random.Random(seed)
        self.x, self.y = start
        self.heading = heading_rad
        self.mode = "ground"            # ground | takeoff | hover | fly | landed
        self._takeoff_left = 0.0
        self._goal: Optional[tuple[float, float]] = None
        self._nav = NAV_IDLE
        self._telem_period = 1.0 / telem_hz
        self._packet_period = 1.0 / packet_hz
        self._telem_acc = self._telem_period    # emit immediately
        self._packet_acc = 0.0
        self.commands: list[tuple[float, str]] = []   # (time, name) for tests
        clock.on_advance(self._step)

    # -- DroneLink ---------------------------------------------------------

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def wait_for_link(self, timeout_s: float) -> None:
        self._clock.sleep(0.1)        # first telemetry sample

    def _log_cmd(self, name: str) -> None:
        self.commands.append((self._clock.now(), name))

    def send_start(self) -> None:
        self._log_cmd("START")
        if self.mode == "ground":
            self.mode = "takeoff"
            self._takeoff_left = self._takeoff_s

    def send_goto(self, x: float, y: float) -> None:
        self._log_cmd("GOTO")
        if self.mode in ("hover", "fly"):      # firmware ignores it otherwise
            self._goal = (x, y)
            self.mode = "fly"
            self._nav = NAV_FLYING

    def send_hold(self) -> None:
        self._log_cmd("HOLD")
        if self.mode == "fly":
            self._goal = None
            self.mode = "hover"
            self._nav = NAV_IDLE

    def send_land(self, repeat: int = 3) -> None:
        self._log_cmd("LAND")
        if self.mode != "ground":
            self.mode = "landed"
            self._goal = None
            self._nav = NAV_IDLE

    @property
    def landed(self) -> bool:
        return self.mode == "landed"

    # -- physics -----------------------------------------------------------

    def _step(self, dt: float) -> None:
        if self.mode == "takeoff":
            self._takeoff_left -= dt
            if self._takeoff_left <= 0:
                self.mode = "hover"
        elif self.mode == "fly" and self._goal is not None:
            dx, dy = self._goal[0] - self.x, self._goal[1] - self.y
            dist = math.hypot(dx, dy)
            if dist > 1e-6:
                self.heading = math.atan2(dy, dx)
            move = self._speed * dt
            if dist <= move:
                self.x, self.y = self._goal
                self._goal = None
                self.mode = "hover"
                self._nav = NAV_ARRIVED
            else:
                self.x += dx / dist * move
                self.y += dy / dist * move

        self._telem_acc += dt
        if self._telem_acc >= self._telem_period - 1e-9:
            self._telem_acc = 0.0
            self._emit_telemetry(TelemetrySample(
                self._clock.now(), self.x, self.y, self.heading, self._nav))

        self._packet_acc += dt
        if self._packet_acc >= self._packet_period - 1e-9:
            self._packet_acc = 0.0
            if self.mode in ("hover", "fly"):
                self._emit_detection(self._clock.now(), self._make_packet())

    def _make_packet(self) -> AtDebugPacket:
        c, s = math.cos(self.heading), math.sin(self.heading)
        dets: list[TagDetection] = []
        for tag_id, (tx_w, ty_w) in sorted(self._tags.items()):
            dx, dy = tx_w - self.x, ty_w - self.y
            fwd = dx * c + dy * s
            right = -dx * s + dy * c
            if math.atan2(self._alt, math.hypot(fwd, right)) < self._min_el:
                continue                      # seen too edge-on to decode
            tx, ty, tz = self._cam.body_to_camera(fwd, right, self._alt)
            if tz < 0.1:
                continue
            rng = math.sqrt(tx * tx + ty * ty + tz * tz)
            if (rng > self._max_range
                    or abs(math.atan2(tx, tz)) > self._hfov / 2
                    or abs(math.atan2(ty, tz)) > self._vfov / 2):
                continue
            if self._noise:
                tx += self._rng.gauss(0.0, self._noise)
                ty += self._rng.gauss(0.0, self._noise)
                tz += self._rng.gauss(0.0, self._noise)
            fx = 320.0 / math.tan(self._hfov / 2)
            fy = 240.0 / math.tan(self._vfov / 2)
            dets.append(TagDetection(tag_id, 0, 100.0,
                                     320.0 + fx * tx / tz, 240.0 + fy * ty / tz,
                                     tx, ty, tz, 0.1))
        return AtDebugPacket(self.drone_id, int(self._clock.now() * 1000), 300,
                             -1, len(dets), dets)
