"""
Minimal ULog (PX4 flight log) writer for tests.

Writes the real binary format (header, flag bits, format/subscription
definitions, data messages) so that pyulog reads it exactly as it reads a log
from the flight controller.  Only the topics and fields the analysis uses.
"""

import math
import struct

_FMT_CHARS = {"uint64_t": "Q", "float": "f", "uint8_t": "B", "bool": "?"}

TOPICS = {
    "vehicle_local_position": [("uint64_t", "timestamp"), ("float", "x"), ("float", "y"),
                               ("float", "z"), ("float", "vx"), ("float", "vy"),
                               ("float", "vz"), ("float", "heading")],
    "vehicle_status": [("uint64_t", "timestamp"), ("uint8_t", "arming_state"),
                       ("uint8_t", "nav_state")],
    "vehicle_attitude": [("uint64_t", "timestamp"), ("float[4]", "q")],
    "vehicle_land_detected": [("uint64_t", "timestamp"), ("bool", "landed")],
}


def _pack_fields(fields, values) -> bytes:
    out, vi = b"", 0
    for ftype, _ in fields:
        if ftype == "float[4]":
            out += struct.pack("<4f", *values[vi])
        else:
            out += struct.pack("<" + _FMT_CHARS[ftype], values[vi])
        vi += 1
    return out


class ULogWriter:
    def __init__(self, topics=None):
        self.topics = {k: TOPICS[k] for k in (topics or TOPICS)}
        self.msg_ids = {name: i for i, name in enumerate(self.topics)}
        self.rows = []          # (timestamp_us, msg_id, payload)

    def add(self, topic: str, t_us: int, *values) -> None:
        fields = self.topics[topic]
        self.rows.append((t_us, self.msg_ids[topic],
                          _pack_fields(fields, (t_us,) + tuple(values))))

    def to_bytes(self) -> bytes:
        buf = bytearray(b"\x55\x4c\x6f\x67\x01\x12\x35\x01" + struct.pack("<Q", 0))

        def msg(kind: bytes, payload: bytes) -> None:
            buf.extend(struct.pack("<HB", len(payload), kind[0]) + payload)

        msg(b"B", bytes(40))                                   # flag bits
        for name, fields in self.topics.items():
            text = ";".join(f"{t} {n}" for t, n in fields) + ";"
            msg(b"F", f"{name}:{text}".encode())
        for name, mid in self.msg_ids.items():
            msg(b"A", struct.pack("<BH", 0, mid) + name.encode())
        for _, mid, payload in sorted(self.rows, key=lambda r: r[0]):
            msg(b"D", struct.pack("<H", mid) + payload)
        return bytes(buf)

    def write(self, path) -> None:
        with open(path, "wb") as fh:
            fh.write(self.to_bytes())


def yaw_pitch_roll_quat(yaw: float, pitch: float = 0.0, roll: float = 0.0):
    """Quaternion (w, x, y, z) for body FRD -> NED, aerospace yaw-pitch-roll order."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)
