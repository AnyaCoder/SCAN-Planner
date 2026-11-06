#!/usr/bin/env python3
"""Apply the FAST-LIO bridge world alignment to a saved PCD."""

import argparse
from pathlib import Path

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BRIDGE_CONFIG = (
    REPO_ROOT / "src" / "planner" / "lio_bridge" / "config" / "fastlio_bridge.yaml"
)


def read_pcd(path):
    raw = Path(path).read_bytes()
    marker = b"DATA binary\n"
    offset = raw.find(marker)
    if offset < 0:
        raise RuntimeError("only binary PCD files are supported")
    header_text = raw[: offset + len(marker)].decode("ascii", "replace")
    points = int(
        next(
            line.split()[1]
            for line in header_text.splitlines()
            if line.startswith("POINTS ")
        )
    )
    fields = next(
        line.split()[1:]
        for line in header_text.splitlines()
        if line.startswith("FIELDS ")
    )
    data = np.frombuffer(
        raw,
        dtype="<f4",
        count=points * len(fields),
        offset=offset + len(marker),
    ).reshape(points, len(fields)).copy()
    return header_text, fields, data


def write_pcd(path, fields, data):
    path = Path(path)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(fields)}\n"
        f"SIZE {' '.join(['4'] * len(fields))}\n"
        f"TYPE {' '.join(['F'] * len(fields))}\n"
        f"COUNT {' '.join(['1'] * len(fields))}\n"
        f"WIDTH {len(data)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(data)}\n"
        "DATA binary\n"
    )
    path.write_bytes(header.encode("ascii") + data.astype("<f4").tobytes())


def quat_rotate(q, vectors):
    q_xyz = q[:3]
    q_w = q[3]
    twice_cross = 2.0 * np.cross(q_xyz, vectors)
    return vectors + q_w * twice_cross + np.cross(q_xyz, twice_cross)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_pcd")
    parser.add_argument("output_pcd")
    parser.add_argument(
        "--bridge-config",
        default=str(DEFAULT_BRIDGE_CONFIG),
    )
    parser.add_argument("--z-min", type=float)
    parser.add_argument("--z-max", type=float)
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.bridge_config).read_text())["lio_bridge"][
        "ros__parameters"
    ]
    quaternion = np.array(
        [
            config["world_alignment_x"],
            config["world_alignment_y"],
            config["world_alignment_z"],
            config["world_alignment_w"],
        ],
        dtype=float,
    )
    quaternion /= np.linalg.norm(quaternion)

    _, fields, data = read_pcd(args.input_pcd)
    xyz_indices = [fields.index(name) for name in ("x", "y", "z")]
    data[:, xyz_indices] = quat_rotate(quaternion, data[:, xyz_indices])

    normal_indices = [
        fields.index(name)
        for name in ("normal_x", "normal_y", "normal_z")
        if name in fields
    ]
    if len(normal_indices) == 3:
        data[:, normal_indices] = quat_rotate(quaternion, data[:, normal_indices])

    finite = np.isfinite(data[:, xyz_indices]).all(axis=1)
    z = data[:, fields.index("z")]
    keep = finite
    if args.z_min is not None:
        keep &= z >= args.z_min
    if args.z_max is not None:
        keep &= z <= args.z_max
    output = data[keep]
    write_pcd(args.output_pcd, fields, output)
    print(
        f"wrote {len(output)} / {len(data)} points to {args.output_pcd}; "
        f"quaternion={quaternion.tolist()}"
    )


if __name__ == "__main__":
    main()
