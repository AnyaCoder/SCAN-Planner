from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from vlm_pixel_shadow import (  # noqa: E402
    VlmPixelShadow,
    normalize_sport_turn_deg,
    result_has_control,
    snap_to_nearest_gray_floor,
    topdown_pixel_to_world,
    world_xyz_to_topdown_pixel,
)


def test_world_point_projects_to_topdown_pixel():
    pixel = world_xyz_to_topdown_pixel(
        np.asarray([1.0, 0.0, 0.0]),
        (101, 101),
        {
            "range_m": 5.0,
            "map_orientation": "world_fixed",
            "map_origin_world_xyz_m": [0.0, 0.0, 0.0],
        },
    )

    assert pixel == (60, 50)


def test_topdown_pixel_converts_to_world_floor_point():
    metadata = {
        "range_m": 5.0,
        "map_orientation": "world_fixed",
        "map_origin_world_xyz_m": [0.0, 0.0, 0.0],
        "robot_pose": {"x": 0.0, "y": 0.0, "z": -0.1},
        "ground_z_m": -0.3,
    }

    target = topdown_pixel_to_world(0.6, 0.45, metadata)

    assert np.allclose(target, [1.0, 0.5, -0.4])
    assert world_xyz_to_topdown_pixel(target, (101, 101), metadata) == (
        60,
        45,
    )


def test_topdown_point_snaps_to_nearby_gray_floor():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    image[50, 49] = 160

    snapped = snap_to_nearest_gray_floor(image, 0.5, 0.5, {"range_m": 5.0})

    assert snapped["x"] == 0.494949
    assert snapped["y"] == 0.505051


def test_sport_turn_normalizes_signed_angle():
    assert normalize_sport_turn_deg(-72.0) == -72.0
    assert normalize_sport_turn_deg(185.0) == -175.0
    assert normalize_sport_turn_deg(180.0) == 180.0


def test_sport_turn_rejects_zero_angle():
    try:
        normalize_sport_turn_deg(2.0)
    except RuntimeError as exc:
        assert "too small" in str(exc)
    else:
        raise AssertionError("a no-op VLM turn must be rejected")


def test_sport_turn_result_is_a_control_action():
    result = {
        "control_mode": "sport_turn",
        "turn_delta_deg": -72.0,
        "target_xyz": None,
    }
    assert result_has_control(result)


def test_topdown_turn_is_not_converted_to_a_scan_goal():
    node = VlmPixelShadow.__new__(VlmPixelShadow)
    node.args = SimpleNamespace(distance_frame="base")
    result = node.topdown_decision_to_turn_result(
        {"turn_deg": -72.0},
        None,
        12.0,
        navigation_metadata={
            "robot_pose": {
                "x": 1.0,
                "y": 2.0,
                "z": 0.1,
                "yaw_rad": 0.25,
            }
        },
    )

    assert result["control_mode"] == "sport_turn"
    assert result["turn_delta_deg"] == -72.0
    assert result["recommended_action"] == "turn_right"
    assert result["target_xyz"] is None
