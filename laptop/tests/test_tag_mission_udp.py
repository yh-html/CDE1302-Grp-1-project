"""
Run the tag mission over real UDP sockets on localhost against a fake drone that
speaks the firmware's wire format (main/wifi_task.h).

This exercises UdpLink, CommsNode, build_command / parse_telemetry /
parse_at_debug and the mission together, which the simulator tests bypass.
It uses high ports so it never collides with a real session.
"""

import math
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_link import RealClock, SimClock, SimLink, UdpLink
from mission_config import parse_config
from mission_log import MissionLog
from protocol import (CMD_GOTO, CMD_HOLD, CMD_LAND, CMD_START, PKT_AT_DEBUG,
                      PKT_TELEM, VFH_BINS)
from tag_mission import TagMission

TELEM_PORT, CMD_PORT, AT_PORT = 25005, 25006, 25008
DRONE_ID = 22
TAGS = {4: (0.9, -0.3), 6: (1.0, 0.35)}
_CMD_FMT = "<BBff12b"


class FakeDrone:
    """SimLink physics in real time behind real UDP sockets.

    Speed is kept at 1 m/s: a tag packet can arrive just before its matching
    position sample, so the pose used lags by up to one sample (0.05 m here).
    At 4 m/s that lag is 0.2 m and the test becomes timing-dependent."""

    def __init__(self, camera):
        self.clock = SimClock()
        self.link = SimLink(DRONE_ID, self.clock, camera, TAGS, speed_mps=1.0,
                            takeoff_s=0.3, noise_m=0.0, telem_hz=20, packet_hz=20)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd.bind(("127.0.0.1", CMD_PORT))
        self.cmd.settimeout(0.01)
        self.received: list[int] = []
        self.link.on_telemetry(self._send_telem)
        self.link.on_detection(self._send_at)
        self._run = True
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def _send_telem(self, s):
        pkt = struct.pack("<BBfffBbf32sBH", PKT_TELEM, DRONE_ID, s.x, s.y,
                          s.heading_rad, s.nav_state, -1, 0.0,
                          bytes(VFH_BINS), 0, 0xFFFF)
        self.out.sendto(pkt, ("127.0.0.1", TELEM_PORT))

    def _send_at(self, t, p):
        data = struct.pack("<BBIHbBB", PKT_AT_DEBUG, DRONE_ID, p.frame_ms,
                           p.proc_ms, p.latched_id, p.raw_count, len(p.detections))
        for d in p.detections:
            data += struct.pack("<bB7f", d.id, d.hamming, d.margin, d.cx, d.cy,
                                d.tx, d.ty, d.tz, d.pose_err)
        self.out.sendto(data, ("127.0.0.1", AT_PORT))

    def _handle(self, data):
        if len(data) < struct.calcsize(_CMD_FMT):
            return
        _, cmd, gx, gy, *_ = struct.unpack(_CMD_FMT, data[:struct.calcsize(_CMD_FMT)])
        self.received.append(cmd)
        if cmd == CMD_START:
            self.link.send_start()
        elif cmd == CMD_GOTO:
            self.link.send_goto(gx, gy)
        elif cmd == CMD_HOLD:
            self.link.send_hold()
        elif cmd == CMD_LAND:
            self.link.send_land()

    def _loop(self):
        while self._run:
            try:
                data, _ = self.cmd.recvfrom(2048)
                self._handle(data)
            except socket.timeout:
                pass
            self.clock.sleep(0.02)          # real-time pacing comes from recvfrom timeout

    def start(self):
        self.thread.start()

    def stop(self):
        self._run = False
        self.thread.join(timeout=2)
        self.out.close()
        self.cmd.close()


def test_mission_over_udp_loopback(tmp_path):
    cfg = parse_config({
        "search_area": {"min_x": 0, "max_x": 1.5, "min_y": -0.6, "max_y": 0.6},
        "sweep": {"lane_spacing_m": 1.0, "edge_margin_m": 0.3},
        "camera": {"pitch_deg": 45.0, "fwd_offset_m": 0.054, "down_offset_m": 0.034},
        "mission": {"cruise_speed_mps": 1.0, "takeoff_wait_s": 0.8,
                    "land_wait_s": 0.3, "connect_timeout_s": 5.0,
                    "min_leg_timeout_s": 5.0, "stale_timeout_s": 2.0,
                    "arrival_radius_m": 0.15},
        "detection": {"max_range_m": 4.0},
    })
    drone = FakeDrone(cfg.camera)
    drone.start()
    clock = RealClock()
    link = UdpLink(DRONE_ID, TELEM_PORT, CMD_PORT, AT_PORT)
    mlog = MissionLog(tmp_path / "run", wall=clock.wall, now=clock.now)
    mission = TagMission(cfg, link, clock, mlog, drone_id=DRONE_ID)
    result = {}
    t = threading.Thread(target=lambda: result.update(code=mission.run()), daemon=True)
    started = time.monotonic()
    t.start()
    t.join(timeout=60)
    drone.stop()
    assert not t.is_alive(), "mission did not finish within 60 s"
    assert result["code"] == 0, mlog.events()[-3:]

    # Commands crossed the wire in the right order.
    assert drone.received[0] == CMD_START
    assert CMD_GOTO in drone.received and drone.received[-1] == CMD_LAND
    # Both tags were decoded from real AT-debug packets and localised.
    assert mission.tracker.ids() == [4, 6]
    for e in mission.tracker.estimates():
        assert math.hypot(e.x - TAGS[e.tag_id][0], e.y - TAGS[e.tag_id][1]) < 0.1
    assert drone.link.landed
    assert time.monotonic() - started < 60
