"""Known-tags mode: layout files from tag_layout_editor.html, hover path, simulated flight."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from mission_config import known_stops, load_config, mission_plan, mission_trajectory  # noqa: E402
import run_tag_mission  # noqa: E402

LAYOUT = """\
base: {base}
known_tags:
  hover_s: 3.0
  blocked_hover_s: 5.0
  tags:
    - {{id: 6, x: 4.05, y: 0.9}}
    - {{id: 2, x: 1.35, y: -1.35, blocked: true, open_side: "+x"}}
    - {{id: null, x: 2.7, y: 0.9}}
detection:
  expected_tags: 3
simulation:
  tags:
    2: [1.35, -1.35]
    6: [4.05, 0.9]
  boxed:
    2: {{open_side: "+x", tag_half: 0.05}}
"""


@pytest.fixture
def layout(tmp_path):
    p = tmp_path / "tag_layout.yaml"           # not next to the base: found in laptop/
    p.write_text(LAYOUT.format(base="tag_mission_demo_traj.yaml"), encoding="utf-8")
    return p


def test_layout_merges_onto_base(layout):
    cfg = load_config(layout)
    assert cfg.camera.pitch_deg == 90.0                       # from the base file
    assert len(cfg.known_tags) == 3 and cfg.expected_tags == 3
    assert cfg.known_tags[2].tag_id is None
    assert cfg.sim.tags == {2: (1.35, -1.35), 6: (4.05, 0.9)}  # replaced, not merged with 4
    assert cfg.sim.boxed[2]["open_side"] == "+x"
    assert not cfg.look_again


def test_plan_visits_every_tag_once_in_shortest_order(layout):
    cfg = load_config(layout)
    plan = mission_plan(cfg)
    lens = [(x + cfg.camera.fwd_offset_m, y) for x, y in plan]   # lens over each tag
    want = {(t.x, t.y) for t in cfg.known_tags}
    assert {(round(x, 6), round(y, 6)) for x, y in lens} == want
    assert lens[0] == pytest.approx((1.35, -1.35))       # left tag first, then 6/4 by distance


def test_trajectory_stops_and_hovers_over_each_tag(layout):
    cfg = load_config(layout)
    tr = mission_trajectory(cfg, mission_plan(cfg))
    xy = tr[:, 1:3]
    for x, y, hover_s, t in known_stops(cfg):
        still = np.all(np.isclose(xy, [x, y], atol=1e-9), axis=1).sum()
        assert still * cfg.traj_dt_s >= hover_s - 1e-9
    assert np.allclose(xy[0], 0) and np.allclose(xy[-1], 0)
    step = np.hypot(*np.diff(xy, axis=0).T) / cfg.traj_dt_s
    assert step.max() <= cfg.traj_speed_mps + 1e-6


def test_bad_layout_is_rejected(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("base: tag_mission_demo_traj.yaml\nknown_tags:\n  tags:\n"
                 "    - {id: 1, x: 9.0, y: 0.0}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="outside the search area"):
        load_config(p)


def test_simulated_flight_finds_the_tags(layout, tmp_path):
    rc = run_tag_mission.main(["--drone-id", "1", "--config", str(layout), "--simulate",
                               "--out-dir", str(tmp_path / "runs")])
    assert rc == 0
    run = next((tmp_path / "runs").iterdir())
    assert "LANDING_CMD_SENT" in (run / "events.csv").read_text(encoding="utf-8")
    # both simulated tags (incl. the boxed tag 2, seen from straight above) are found
    found = (run / "tags.csv").read_text(encoding="utf-8").splitlines()[1:]
    assert sorted(int(line.split(",")[0]) for line in found) == [2, 6]
