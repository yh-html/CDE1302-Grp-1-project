import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest

from protocol import PKT_TELEM, VFH_BINS, parse_telemetry

# Current wire format: main/wifi_task.h wifi_telem_pkt_t (packed, 55 bytes).
_CURRENT_FMT = f"<BBfffBbf{VFH_BINS}sBH"


def _current_packet(**over) -> bytes:
    vals = dict(pkt_type=PKT_TELEM, drone_id=22, x=1.0, y=2.0, heading=0.3,
                nav_state=2, tag_id=-1, tag_dist=0.5, vfh=bytes([1, 0] * (VFH_BINS // 2)),
                is_stuck=1, reloc=0xFFFF)
    vals.update(over)
    return struct.pack(_CURRENT_FMT, *vals.values())


def test_parse_telemetry_current_firmware_format() -> None:
    pkt = parse_telemetry(_current_packet())
    assert pkt is not None
    assert pkt.drone_id == 22
    assert (pkt.ned_x, pkt.ned_y) == (1.0, 2.0)
    assert pkt.nav_state == 2
    assert pkt.tag_id == -1
    assert pkt.is_stuck is True
    assert pkt.reloc_age_s == 0xFFFF
    assert pkt.vfh_blocked[:2] == [True, False]
    assert len(pkt.vfh_blocked) == VFH_BINS


def test_parse_telemetry_rejects_bad_packets() -> None:
    assert parse_telemetry(_current_packet()[:-1]) is None   # truncated
    assert parse_telemetry(_current_packet(pkt_type=0x02)) is None   # wrong type


@pytest.mark.skip(reason="Stale since the initial commit: neither main/wifi_task.h "
                         "nor laptop/protocol.py has PX4 status fields. Re-enable "
                         "if the telemetry packet is extended.")
def test_parse_telemetry_includes_px4_status_fields() -> None:
    raw = struct.pack(
        "<BBfffBbf32sBHHHBBB",
        0x01,
        2,
        1.0,
        2.0,
        0.3,
        0,
        -1,
        0.5,
        b"\x00" * 32,
        0,
        0,
        0,
        0,
        1,
        6,
        0,
    )

    pkt = parse_telemetry(raw)
    assert pkt is not None
    assert pkt.px4_link_ok is True
    assert pkt.px4_hb_age_ms == 0
    assert pkt.px4_armed == 1
    assert pkt.px4_custom_main_mode == 6
