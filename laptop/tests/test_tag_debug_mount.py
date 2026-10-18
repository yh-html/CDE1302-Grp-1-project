"""Camera-to-body geometry for the Astra under-nose AprilTag camera."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tag_debug import (  # noqa: E402
    CAMERA_DETENTS_DEG,
    CAMERA_MOUNTS,
    CAM_DOWN_OFFSET_M,
    CAM_FWD_OFFSET_M,
    cam_to_body,
)
from apriltag_host import FIRMWARE_INTRINSICS  # noqa: E402


@pytest.mark.parametrize("pitch_deg", CAMERA_DETENTS_DEG)
def test_synthetic_tag_at_known_range_and_angle_for_each_detent(pitch_deg):
    """A tag 2 m away and 10 deg image-down rotates to pitch + 10 deg."""
    tag_range_m = 2.0
    image_down_angle_deg = 10.0
    angle = math.radians(image_down_angle_deg)
    ty = tag_range_m * math.sin(angle)
    tz = tag_range_m * math.cos(angle)

    fwd, right, down, body_dist = cam_to_body(
        0.0, ty, tz, pitch_deg=pitch_deg, mount_name="astra_under_nose")

    body_angle = math.radians(pitch_deg + image_down_angle_deg)
    expected_fwd = CAM_FWD_OFFSET_M + tag_range_m * math.cos(body_angle)
    expected_down = CAM_DOWN_OFFSET_M + tag_range_m * math.sin(body_angle)
    assert fwd == pytest.approx(expected_fwd, abs=1e-12)
    assert right == 0.0
    assert down == pytest.approx(expected_down, abs=1e-12)
    assert body_dist == pytest.approx(math.hypot(expected_fwd, expected_down),
                                     abs=1e-12)


def test_positive_image_down_is_not_mistaken_for_forward():
    level = cam_to_body(0.0, 0.0, 1.0, pitch_deg=45.0)
    image_down = cam_to_body(0.0, 0.2, 1.0, pitch_deg=45.0)
    assert image_down[0] < level[0]
    assert image_down[2] > level[2]


def test_nominal_astra_and_assumed_stock_mounts_are_switchable():
    astra = CAMERA_MOUNTS["astra_under_nose"]
    stock = CAMERA_MOUNTS["stock"]
    assert (astra.fwd_offset_m, astra.down_offset_m) == (0.054, 0.034)
    assert (stock.fwd_offset_m, stock.down_offset_m) == (0.020, 0.0)
    assert cam_to_body(0.0, 0.0, 1.0, mount_name="stock") != \
        cam_to_body(0.0, 0.0, 1.0, mount_name="astra_under_nose")


def test_non_detent_pitch_is_rejected():
    with pytest.raises(ValueError, match="detent"):
        cam_to_body(0.0, 0.0, 1.0, pitch_deg=50.0)


def test_91_by_82_fov_is_a_conservative_asymmetric_intrinsics_envelope():
    """The CAD FOV is for clearance; pose uses the calibrated intrinsics."""
    k = FIRMWARE_INTRINSICS
    left = math.degrees(math.atan(k.cx / k.fx))
    right = math.degrees(math.atan((320.0 - k.cx) / k.fx))
    top = math.degrees(math.atan(k.cy / k.fy))
    bottom = math.degrees(math.atan((240.0 - k.cy) / k.fy))
    assert max(left, right) * 2 == pytest.approx(90.8, abs=0.1)
    assert max(top, bottom) * 2 == pytest.approx(81.9, abs=0.1)
    assert left != pytest.approx(right)
    assert top != pytest.approx(bottom)
