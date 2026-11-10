#!/usr/bin/env python3
"""
Bench test of the trajectory upload, props on or off, motors never armed.

Uploads the exact trajectory run_tag_mission.py would fly, a few times, and
sends NO takeoff, start or trajectory-start command: the drone only stores the
chunks. (Even a trajectory start would be ignored while it is not flying.)
Watch the ESP32 serial log (firmware with the upload log lines):

    Trajectory upload id=N: receiving 1346 pts
    Trajectory upload id=N complete: 1346 pts     <- all chunks arrived

A "receiving" line with no matching "complete" means a chunk was lost.

    python laptop/traj_upload_test.py --drone-id 1 --config laptop/tag_mission_demo_traj.yaml

Tries the new 60 ms chunk gap and the old 25 ms one (5 uploads each), reads the
ESP32 log over USB (--serial COM17) and prints e.g. "gap 60 ms: 5/5 uploads
complete". Results + full log are saved to laptop/mission_logs/traj_upload_test.txt.
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import drone_link                                    # noqa: E402
from comms import CommsNode                           # noqa: E402
from mission_config import load_config, mission_trajectory, mission_plan   # noqa: E402
from protocol import build_traj_packets               # noqa: E402
from sweep_planner import plan_sweep                  # noqa: E402
from traj_gen import smooth_path                      # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--drone-id", type=int, required=True)
    p.add_argument("--config", default=str(HERE / "tag_mission_demo_traj.yaml"))
    p.add_argument("--repeats", type=int, default=5, help="Uploads per gap (default 5).")
    p.add_argument("--gaps-ms", default="60,25",
                   help="Comma-separated chunk gaps to try (default 60,25: new vs old).")
    p.add_argument("--serial", default="COM17",
                   help="ESP32 USB port to record and check the log ('' = do not read it).")
    p.add_argument("--out", default=str(HERE / "mission_logs" / "traj_upload_test.txt"),
                   help="Where to save the results and the ESP32 log.")
    a = p.parse_args()
    gaps = [float(g) for g in a.gaps_ms.split(",") if g.strip()]

    cfg = load_config(a.config)
    if not cfg.traj_speed_mps:
        raise SystemExit("config has trajectory.enabled false; nothing to upload")
    plan = mission_plan(cfg)
    dt_ms = round(cfg.traj_dt_s * 1000)
    tr = mission_trajectory(dataclasses.replace(cfg, traj_dt_s=dt_ms / 1000), plan)
    xy = tr[:, 1:3] - tr[0, 1:3]
    pts = [(float(x), float(y), 0.0) for x, y in xy]

    # Record the ESP32 log in the background (opening the port does not reset it).
    log_buf = bytearray()
    stop = threading.Event()
    ser = None
    if a.serial:
        import serial                                      # pyserial
        try:
            ser = serial.Serial(a.serial, 115200, timeout=0.2)
        except Exception as exc:                           # noqa: BLE001
            raise SystemExit(f"cannot open {a.serial} ({exc}); close any serial monitor / "
                             "QGroundControl, or pass --serial '' to skip the log")

        def reader() -> None:
            while not stop.is_set():
                log_buf.extend(ser.read(4096))
        threading.Thread(target=reader, daemon=True).start()

    sent: list[tuple[int, float]] = []                     # (upload id, gap ms)
    comms = CommsNode(config_path=None)      # same as the mission link
    comms.start()
    try:
        print(f"Waiting for drone {a.drone_id} telemetry...")
        t0 = time.time()
        while a.drone_id not in comms.known_drones():
            if time.time() - t0 > 20:
                raise SystemExit("no telemetry from the drone in 20 s (WiFi / host IP?)")
            time.sleep(0.2)
        traj_id = 200
        for gap in gaps:
            for r in range(1, a.repeats + 1):
                traj_id += 1
                pkts = build_traj_packets(traj_id, pts)
                for pk in pkts:
                    comms.send_raw(a.drone_id, pk)
                    time.sleep(gap / 1000)
                sent.append((traj_id, gap))
                print(f"gap {gap:.0f} ms, upload {r}/{a.repeats}: id={traj_id}, "
                      f"{len(pts)} pts in {len(pkts)} chunks")
                time.sleep(1.5)
    finally:
        comms.stop()
        stop.set()
        time.sleep(0.4)
        if ser is not None:
            ser.close()

    text = log_buf.decode("utf-8", "replace")
    lines = [f"traj_upload_test {time.strftime('%Y-%m-%d %H:%M:%S')}  drone {a.drone_id}  "
             f"{len(pts)} pts  config {Path(a.config).name}"]
    if ser is None:
        lines.append("ESP32 log not read (--serial ''): check it by hand.")
    elif "Trajectory upload id=" not in text:
        lines.append("ESP32 log has no 'Trajectory upload' lines: is the new firmware flashed?")
    else:
        for gap in gaps:
            ids = [i for i, g in sent if g == gap]
            ok = sum(f"Trajectory upload id={i} complete" in text for i in ids)
            lines.append(f"gap {gap:4.0f} ms: {ok}/{len(ids)} uploads complete"
                         + ("" if ok == len(ids) else "   <-- chunks lost"))
    lines.append("No takeoff or start command was sent.")
    summary = "\n".join(lines)
    print("\n" + summary)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(summary + "\n\n--- ESP32 log ---\n" + text, encoding="utf-8")
    print(f"Saved to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
