"""
Offline tests of live_map.CameraFeed over loopback: frames arrive, keepalives keep
going, and the feed survives bad data and a decoder crash (the 10 Nov stalls).
"""

import socket
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from test_camera_stream import datagrams          # noqa: E402  firmware-shaped packets

import live_map                                   # noqa: E402


def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def jpeg(value: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", np.full((240, 320), value, np.uint8))
    assert ok
    return buf.tobytes()


def wait_for(pred, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def feed():
    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)      # stands in for the drone's 5006
    cmd.bind(("127.0.0.1", 0))
    cmd.settimeout(2.0)
    stream_port = free_port()
    f = live_map.CameraFeed("127.0.0.1", 6, 45, stream_port=stream_port,
                            command_port=cmd.getsockname()[1])
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(frame_id, data):
        for d in datagrams(frame_id, data):
            tx.sendto(d, ("127.0.0.1", stream_port))

    yield f, cmd, send
    f.close()
    tx.close()
    cmd.close()


def test_frames_are_decoded_and_shown(feed):
    f, _, send = feed
    send(1, jpeg(100))
    assert wait_for(lambda: f.frames >= 1)
    img, info = f.latest()
    assert img is not None and img.shape == (240, 320) and abs(int(img.mean()) - 100) < 3
    assert info.startswith("#1 ")


def test_keepalives_keep_coming_at_about_2_hz(feed):
    f, cmd, _ = feed
    got = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.6:
        try:
            got.append(cmd.recv(64))
        except socket.timeout:
            break
    assert len(got) >= 3                                   # 0.5 s period -> 3-4 in 1.6 s
    assert all(g[1] == 0x07 and g[2] == 1 for g in got)    # CMD_CAMERA_STREAM, enable


def test_garbage_jpeg_is_counted_not_fatal(feed):
    f, _, send = feed
    send(1, b"\xff\xd8 not really a jpeg" * 50)
    assert wait_for(lambda: f.errors >= 1)
    send(2, jpeg(50))
    assert wait_for(lambda: f.frames >= 1)
    assert f._thread.is_alive()


def test_decoder_crash_does_not_kill_the_feed(feed, monkeypatch):
    f, _, send = feed
    real = f._decode
    calls = {"n": 0}

    def boom(data):
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("decoder blew up")
        return real(data)

    monkeypatch.setattr(f, "_decode", boom)
    send(1, jpeg(80))
    assert wait_for(lambda: f.errors >= 1)
    send(2, jpeg(90))
    assert wait_for(lambda: f.frames >= 1)
    assert f._thread.is_alive() and "decoder blew up" in f.last_error


def test_gap_is_reported_then_recovers(feed, monkeypatch):
    f, _, send = feed
    monkeypatch.setattr(live_map.CameraFeed, "GAP_S", 0.3)
    send(1, jpeg(60))
    assert wait_for(lambda: f.frames >= 1)
    assert wait_for(lambda: f.gaps >= 1)
    _, info = f.latest()
    assert "no video for" in info and "still asking" in info
    send(2, jpeg(70))
    assert wait_for(lambda: f.frames >= 2)
    _, info = f.latest()
    assert "no video" not in info


def test_dead_thread_is_restarted(feed):
    f, _, send = feed
    f._running = False
    f._thread.join(timeout=1.0)
    f._running = True
    f.latest()                                             # notices and restarts
    assert f.restarts == 1 and f._thread.is_alive()
    send(1, jpeg(40))
    assert wait_for(lambda: f.frames >= 1)
