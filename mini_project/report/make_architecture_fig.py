"""Draws the system architecture / data-flow figure for the report (fig_architecture.png)."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

plt.rcParams["font.family"] = "Times New Roman"
fig, ax = plt.subplots(figsize=(7.2, 3.0))
ax.set_xlim(-1.5, 101.5)
ax.set_ylim(0, 42)
ax.axis("off")


def box(x, y, w, h, title, lines, fc):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.2",
                                fc=fc, ec="black", lw=1.0))
    ax.text(x + w / 2, y + h - 2.2, title, ha="center", va="top", fontsize=9, weight="bold")
    ax.text(x + w / 2, y + h - 6.6, "\n".join(lines), ha="center", va="top", fontsize=7.4,
            linespacing=1.25)


def arrow(x0, y0, x1, y1, label, dy=0.8, both=False):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle="<|-|>" if both else "-|>", lw=1.1, color="black"))
    ax.text((x0 + x1) / 2, y0 + dy, label, ha="center", va="bottom", fontsize=6.8)


box(0, 6, 25, 32, "Laptop (Python)",
    ["run_tag_mission.py", "lawn-mower sweep planner", "tag localiser", "mission log, safety checks",
     "analyze_flight.py", "(post-flight, .ulg)"], "#dbeafe")
box(44, 6, 25, 32, "Companion: ESP32-S3",
    ["wifi_task  10 Hz", "nav_task  20 Hz", "mission_task (takeoff/land)", "at_detect  ~2 Hz",
     "AprilTag tag16h5 + pose"], "#dcfce7")
box(81, 6, 19, 32, "PX4 controller",
    ["MicoAir743 v2", "offboard position", "EKF: optical flow", "(MTF-02P) + IMU",
     "writes .ulg log"], "#fef3c7")

arrow(25.6, 31, 43.4, 31, "commands\nSTART / GOTO / LAND\nUDP 5006")
arrow(43.4, 22, 25.6, 22, "telemetry x, y, heading\n10 Hz   UDP 5005")
arrow(43.4, 13, 25.6, 13, "tag detections\n(id, camera-frame position)\nUDP 5008")
arrow(69.6, 22, 80.4, 22, "MAVLink\nUART", both=True)
ax.text(50, 2.0, "The camera sits on the ESP32 and only reports tags; it never affects flight.",
        ha="center", fontsize=7.4, style="italic")
fig.savefig("fig_architecture.png", dpi=200, bbox_inches="tight")
print("wrote fig_architecture.png")
