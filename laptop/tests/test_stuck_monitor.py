"""StuckMonitor progress detection.

Progress is measured from an anchor (the position where progress was last
recorded), not from the previous telemetry packet.  Telemetry arrives at
10 Hz, so a drone flying at cruise speed moves far less than the threshold
between packets and used to be reported stuck after stuck_timeout_s.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import stuck_monitor
from stuck_monitor import StuckMonitor

DT = 0.1  # telemetry period (s)


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(stuck_monitor.time, "monotonic", c)
    return c


@pytest.fixture
def monitor(tmp_path, clock):
    cfg = tmp_path / "setup.yaml"
    cfg.write_text(
        "stuck:\n"
        "  progress_threshold_m: 0.5\n"
        "  stuck_timeout_s: 10.0\n"
        "  stuck_zone_radius: 0.5\n"
        "  stuck_zone_ttl_s: 60.0\n"
        "  retreat_dist_m: 1.5\n"
    )
    director = SimpleNamespace(_min_x=0.0, _max_x=14.0, _min_y=-5.0, _max_y=15.0)
    store = SimpleNamespace(get_trail=lambda drone_id: [])
    return StuckMonitor(director, store, str(cfg))


def run(monitor, clock, seconds, pos_at):
    """Feed telemetry at 10 Hz; return the time of the first stuck report, or None."""
    steps = int(seconds / DT)
    for i in range(steps):
        clock.t += DT
        if monitor.update(1, *pos_at(i * DT), 0.0):
            return i * DT
    return None


def test_moving_drone_is_never_stuck(monitor, clock):
    # 0.5 m/s is 0.05 m per packet, well under the 0.5 m threshold.
    hit = run(monitor, clock, 30.0, lambda t: (0.5 * t, 0.0))
    assert hit is None
    assert not monitor.is_stuck(1)


def test_stationary_drone_is_stuck_after_timeout(monitor, clock):
    hit = run(monitor, clock, 15.0, lambda t: (3.0, 3.0))
    assert hit is not None
    assert 10.0 <= hit <= 10.3
    assert monitor.is_stuck(1)
    assert len(monitor.stuck_zones()) == 1


def test_stuck_is_reported_once(monitor, clock):
    first = run(monitor, clock, 15.0, lambda t: (3.0, 3.0))
    assert first is not None
    # Further packets while still stuck must not re-trigger.
    assert run(monitor, clock, 5.0, lambda t: (3.0, 3.0)) is None


def test_small_oscillation_is_stuck(monitor, clock):
    # Wobbling within 0.2 m never moves 0.5 m from the anchor.
    hit = run(monitor, clock, 15.0, lambda t: (3.0 + 0.2 * (int(t * 10) % 2), 3.0))
    assert hit is not None


def test_slow_drift_beyond_threshold_counts_as_progress(monitor, clock):
    # 0.06 m/s drift passes 0.5 m every ~8.3 s, so the timeout never expires.
    assert run(monitor, clock, 60.0, lambda t: (0.06 * t, 0.0)) is None


def test_clear_stuck_resets_the_timer(monitor, clock):
    assert run(monitor, clock, 15.0, lambda t: (3.0, 3.0)) is not None
    monitor.clear_stuck(1)
    assert not monitor.is_stuck(1)
    # Still sitting there after the escape goal: a fresh timeout applies.
    assert run(monitor, clock, 5.0, lambda t: (3.0, 3.0)) is None
    assert run(monitor, clock, 8.0, lambda t: (3.0, 3.0)) is not None


def test_drones_are_tracked_independently(monitor, clock):
    for i in range(int(15.0 / DT)):
        clock.t += DT
        monitor.update(1, 0.5 * i * DT, 0.0, 0.0)   # moving
        monitor.update(2, 5.0, 5.0, 0.0)            # parked
    assert monitor.is_stuck(2)
    assert not monitor.is_stuck(1)
