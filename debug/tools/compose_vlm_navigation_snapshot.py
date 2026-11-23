#!/usr/bin/env python3
"""Prepare diagnostic camera/top-down VLM inputs and one waypoint record."""

import argparse
import json
import os
from datetime import datetime
from pathlib import Path

from PIL import Image


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--annotated-map", type=Path, required=True)
    parser.add_argument("--camera", type=Path, required=True)
    parser.add_argument("--map-metadata", type=Path, required=True)
    parser.add_argument("--clean-map-output", type=Path, required=True)
    parser.add_argument("--metadata-output", type=Path, required=True)
    parser.add_argument("--snapshot-sequence", type=int, required=True)
    return parser.parse_args()


def atomic_write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def main():
    args = parse_args()
    metadata = json.loads(args.map_metadata.read_text(encoding="utf-8"))
    pose = metadata.get("robot_pose")
    if pose is None:
        raise SystemExit(
            "Snapshot has no /LIO/odom_vehicle pose; refusing to append an "
            "unlocated waypoint"
        )

    annotated = Image.open(args.annotated_map).convert("RGB")
    box = metadata["map_panel_box"]
    if len(box) != 4:
        raise SystemExit("map_panel_box must contain four image coordinates")
    clean_map = annotated.crop(tuple(int(value) for value in box))
    args.clean_map_output.parent.mkdir(parents=True, exist_ok=True)
    # These are short-lived on-demand VLM inputs. Favor capture latency over
    # PNG compression; the model request is re-encoded as JPEG immediately.
    clean_map.save(args.clean_map_output, compress_level=0)

    captured_at = datetime.now().astimezone().isoformat(timespec="seconds")
    record = {
        "sequence": args.snapshot_sequence,
        "captured_at": captured_at,
        "robot_pose": pose,
        "pose_available": pose is not None,
        "map_frame": metadata.get("frame"),
        "map_range_m": [
            float(metadata.get("range_m", 0.0)) * 2.0,
            float(metadata.get("range_m", 0.0)) * 2.0,
        ],
        "map_orientation": metadata.get(
            "map_orientation", "current_heading"
        ),
        "map_origin_world_xyz_m": metadata.get(
            "map_origin_world_xyz_m"
        ),
        "camera_image": str(args.camera.resolve()),
        "clean_map_image": str(args.clean_map_output.resolve()),
    }

    snapshot_metadata = dict(metadata)
    snapshot_metadata["snapshot_sequence"] = args.snapshot_sequence
    snapshot_metadata["snapshot"] = record
    atomic_write_json(args.metadata_output, snapshot_metadata)

    print(
        "waypoint %d: x=%.3f y=%.3f z=%.3f yaw=%.1f deg"
        % (
            args.snapshot_sequence,
            pose["x"],
            pose["y"],
            pose["z"],
            pose["yaw_deg"],
        )
    )
    print("camera:", args.camera)
    print("top-down:", args.clean_map_output)


if __name__ == "__main__":
    main()
