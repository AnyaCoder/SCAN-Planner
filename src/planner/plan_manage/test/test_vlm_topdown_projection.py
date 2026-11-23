from pathlib import Path
import sys

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from vlm_pixel_shadow import (  # noqa: E402
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
