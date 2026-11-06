#!/usr/bin/python3
"""Virtual first-pass exploration and topology extraction on a saved PCD."""

import argparse
import heapq
import json
import math
import time
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import (
    binary_closing,
    binary_dilation,
    convolve,
    distance_transform_edt,
    label,
)
from scipy.spatial import cKDTree


REPO_ROOT = Path(__file__).resolve().parents[1]


def latest_bundle_map():
    candidates = sorted(
        (REPO_ROOT / "maps" / "bundles").glob("*/map_world.pcd")
    )
    return str(candidates[-1]) if candidates else ""


def default_waypoints(map_path):
    candidates = []
    if map_path:
        candidates.append(Path(map_path).parent / "clicked_waypoints.yaml")
    candidates.extend(
        [
            REPO_ROOT / "maps" / "clicked_waypoints_temp.yaml",
            REPO_ROOT / "maps" / "clicked_waypoints.yaml",
        ]
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return ""


NEIGHBOR_OFFSETS = (
    (-1, -1),
    (0, -1),
    (1, -1),
    (-1, 0),
    (1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
)

NODE_COLORS = {
    "START": (0, 170, 0),
    "TARGET": (245, 110, 0),
    "TURN": (220, 35, 35),
    "T_JUNCTION": (35, 80, 220),
    "CROSS": (150, 50, 210),
    "PORTAL": (245, 205, 0),
    "DEAD_END": (10, 10, 10),
    "PASS": (100, 100, 100),
}

NODE_ABBREVIATIONS = {
    "START": "S",
    "TARGET": "G",
    "TURN": "L",
    "T_JUNCTION": "T",
    "CROSS": "X",
    "PORTAL": "P",
    "DEAD_END": "D",
    "PASS": ".",
}


def read_pcd(path):
    raw = Path(path).read_bytes()
    marker = b"DATA binary\n"
    offset = raw.find(marker)
    if offset < 0:
        raise RuntimeError("only binary PCD files are supported")
    header = raw[: offset + len(marker)].decode("ascii", "replace")
    fields = next(
        line.split()[1:]
        for line in header.splitlines()
        if line.startswith("FIELDS ")
    )
    points = int(
        next(
            line.split()[1]
            for line in header.splitlines()
            if line.startswith("POINTS ")
        )
    )
    data = np.frombuffer(
        raw,
        dtype="<f4",
        count=points * len(fields),
        offset=offset + len(marker),
    ).reshape(points, len(fields))
    columns = {name: index for index, name in enumerate(fields)}
    xyz = data[:, [columns["x"], columns["y"], columns["z"]]].astype(float)
    return xyz[np.isfinite(xyz).all(axis=1)]


def read_waypoints(path):
    config = yaml.safe_load(Path(path).read_text())
    values = config["scan_planner_node"]["ros__parameters"]["fsm.waypoints"]
    points = np.asarray(values, dtype=float)
    if points.size % 3:
        raise ValueError("waypoint file must contain xyz triples")
    return points.reshape(-1, 3)


def disk(radius):
    size = int(math.ceil(radius))
    axis = np.arange(-size, size + 1)
    xx, yy = np.meshgrid(axis, axis)
    return (xx * xx + yy * yy) <= radius * radius


def build_grid(
    xyz,
    waypoints,
    resolution,
    obstacle_z_max,
    robot_radius,
    margin,
    floor_z_min=-0.38,
    floor_z_max=-0.06,
    floor_support_radius=0.25,
    obstacle_closing_radius=0.10,
):
    x_min = float(waypoints[:, 0].min() - margin)
    x_max = float(waypoints[:, 0].max() + margin)
    y_min = float(waypoints[:, 1].min() - margin)
    y_max = float(waypoints[:, 1].max() + margin)
    width = int(round((x_max - x_min) / resolution))
    height = int(round((y_max - y_min) / resolution))

    inside = xyz[
        (xyz[:, 0] >= x_min)
        & (xyz[:, 0] < x_max)
        & (xyz[:, 1] >= y_min)
        & (xyz[:, 1] < y_max)
    ]
    selected = inside[
        (inside[:, 2] >= -0.10)
        & (inside[:, 2] <= obstacle_z_max)
    ]
    columns = np.clip(
        ((selected[:, 0] - x_min) / resolution).astype(np.int32),
        0,
        width - 1,
    )
    rows = np.clip(
        ((selected[:, 1] - y_min) / resolution).astype(np.int32),
        0,
        height - 1,
    )
    occupied = np.zeros((height, width), dtype=bool)
    occupied[rows, columns] = True
    if obstacle_closing_radius > 0.0:
        occupied = binary_closing(
            occupied,
            structure=disk(obstacle_closing_radius / resolution),
        )

    floor_points = inside[
        (inside[:, 2] >= floor_z_min)
        & (inside[:, 2] <= floor_z_max)
    ]
    floor = np.zeros((height, width), dtype=bool)
    floor_columns = np.clip(
        ((floor_points[:, 0] - x_min) / resolution).astype(np.int32),
        0,
        width - 1,
    )
    floor_rows = np.clip(
        ((floor_points[:, 1] - y_min) / resolution).astype(np.int32),
        0,
        height - 1,
    )
    floor[floor_rows, floor_columns] = True
    footprint = binary_dilation(
        floor,
        structure=disk(floor_support_radius / resolution),
    )
    obstacles = occupied | ~footprint
    distance = distance_transform_edt(~obstacles) * resolution
    free = (~obstacles) & (distance >= robot_radius)
    blocked = ~free
    return {
        "resolution": resolution,
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "width": width,
        "height": height,
        "occupied": occupied,
        "footprint": footprint,
        "blocked": blocked,
        "free": free,
        "distance": distance,
    }


def cell_for_point(grid, point):
    return (
        int((point[0] - grid["x_min"]) / grid["resolution"]),
        int((point[1] - grid["y_min"]) / grid["resolution"]),
    )


def point_for_cell(grid, cell):
    return (
        (cell[0] + 0.5) * grid["resolution"] + grid["x_min"],
        (cell[1] + 0.5) * grid["resolution"] + grid["y_min"],
    )


def local_window_bounds(grid, pose, size_x, size_y):
    columns = min(
        grid["width"],
        max(1, int(round(size_x / grid["resolution"]))),
    )
    rows = min(
        grid["height"],
        max(1, int(round(size_y / grid["resolution"]))),
    )
    first_column = min(
        max(0, pose[0] - columns // 2),
        grid["width"] - columns,
    )
    first_row = min(
        max(0, pose[1] - rows // 2),
        grid["height"] - rows,
    )
    return (
        first_column,
        first_row,
        first_column + columns,
        first_row + rows,
    )


def window_contains(window, cell):
    return (
        window[0] <= cell[0] < window[2]
        and window[1] <= cell[1] < window[3]
    )


def clip_to_window_runs(cells, window):
    runs = []
    current = []
    for cell in cells:
        if window_contains(window, cell):
            current.append(cell)
        elif current:
            if len(current) >= 2:
                runs.append(current)
            current = []
    if len(current) >= 2:
        runs.append(current)
    return runs


def window_world_bounds(grid, window):
    first_column, first_row, last_column, last_row = window
    return {
        "min_x": round(
            grid["x_min"] + first_column * grid["resolution"], 3
        ),
        "max_x": round(
            grid["x_min"] + last_column * grid["resolution"], 3
        ),
        "min_y": round(
            grid["y_min"] + first_row * grid["resolution"], 3
        ),
        "max_y": round(
            grid["y_min"] + last_row * grid["resolution"], 3
        ),
    }


def nearest_free(grid, cell, max_radius):
    column, row = cell
    best = None
    for candidate_row in range(
        max(0, row - max_radius), min(grid["height"], row + max_radius + 1)
    ):
        for candidate_column in range(
            max(0, column - max_radius),
            min(grid["width"], column + max_radius + 1),
        ):
            if not grid["free"][candidate_row, candidate_column]:
                continue
            distance = (candidate_row - row) ** 2 + (
                candidate_column - column
            ) ** 2
            if best is None or distance < best[0]:
                best = (distance, candidate_column, candidate_row)
    if best is None:
        raise RuntimeError("no free cell near requested point")
    return best[1], best[2]


def bfs(mask, start):
    width = mask.shape[1]
    distance = np.full(mask.shape, -1, dtype=np.int32)
    parents = {}
    start_row, start_column = start[1], start[0]
    if not mask[start_row, start_column]:
        return distance, parents
    distance[start_row, start_column] = 0
    queue = deque([start])
    moves = (
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    )
    while queue:
        column, row = queue.popleft()
        current = distance[row, column]
        for delta_row, delta_column in moves:
            next_row = row + delta_row
            next_column = column + delta_column
            if not (
                0 <= next_row < mask.shape[0]
                and 0 <= next_column < width
                and mask[next_row, next_column]
            ):
                continue
            if delta_row and delta_column:
                if not mask[row, next_column] or not mask[next_row, column]:
                    continue
            if distance[next_row, next_column] >= 0:
                continue
            distance[next_row, next_column] = current + 1
            parents[(next_column, next_row)] = (column, row)
            queue.append((next_column, next_row))
    return distance, parents


def reconstruct(parents, start, goal):
    current = goal
    path = [current]
    while current != start:
        if current not in parents:
            return None
        current = parents[current]
        path.append(current)
    return path[::-1]


def reveal(
    grid,
    known_free,
    known_occupied,
    pose,
    max_range,
    rays,
    window=None,
):
    x0, y0 = point_for_cell(grid, pose)
    step = grid["resolution"]
    blocked = grid["blocked"]
    for ray in range(rays):
        angle = 2.0 * math.pi * ray / rays
        cosine = math.cos(angle)
        sine = math.sin(angle)
        distance = 0.0
        while distance <= max_range:
            column = int(
                (x0 + cosine * distance - grid["x_min"]) / grid["resolution"]
            )
            row = int(
                (y0 + sine * distance - grid["y_min"]) / grid["resolution"]
            )
            if not (
                0 <= row < grid["height"]
                and 0 <= column < grid["width"]
            ):
                break
            if window is not None and not window_contains(
                window, (column, row)
            ):
                break
            if blocked[row, column]:
                known_occupied[row, column] = True
                break
            known_free[row, column] = True
            distance += step
    known_free[pose[1], pose[0]] = True


def frontier_mask(known_free, unknown):
    structure = np.ones((3, 3), dtype=bool)
    unknown_next_to_free = binary_dilation(unknown, structure=structure)
    return known_free & unknown_next_to_free


def frontier_score(grid, distances, frontier, target, target_weight):
    candidates = np.argwhere(frontier)
    if len(candidates) == 0:
        return None
    rows = candidates[:, 0]
    columns = candidates[:, 1]
    reachable = distances[rows, columns] >= 0
    if not reachable.any():
        return None
    rows = rows[reachable]
    columns = columns[reachable]
    path_cells = distances[rows, columns].astype(float)
    x = (columns + 0.5) * grid["resolution"] + grid["x_min"]
    y = (rows + 0.5) * grid["resolution"] + grid["y_min"]
    target_x, target_y = point_for_cell(grid, target)
    target_distance = np.hypot(x - target_x, y - target_y)
    score = path_cells * grid["resolution"] + target_weight * target_distance
    best = int(np.argmin(score))
    return int(columns[best]), int(rows[best])


def point_distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def path_distance(grid, path):
    return sum(point_distance(a, b) for a, b in zip(path, path[1:])) * grid[
        "resolution"
    ]


def heading_change(previous, current, following):
    incoming = np.asarray(current) - np.asarray(previous)
    outgoing = np.asarray(following) - np.asarray(current)
    if np.linalg.norm(incoming) == 0.0 or np.linalg.norm(outgoing) == 0.0:
        return 0.0
    cosine = float(
        np.dot(incoming, outgoing)
        / (np.linalg.norm(incoming) * np.linalg.norm(outgoing))
    )
    return math.degrees(math.acos(np.clip(cosine, -1.0, 1.0)))


def explore(grid, start_point, target_point, args):
    start = nearest_free(grid, cell_for_point(grid, start_point), 40)
    target = nearest_free(grid, cell_for_point(grid, target_point), 40)
    known_free = np.zeros_like(grid["free"])
    known_occupied = np.zeros_like(grid["free"])
    pose = start
    trajectory = [start]
    frontier_choices = []
    snapshots = []
    stop_reason = "max_steps"
    target_found = False

    def sense():
        window = local_window_bounds(
            grid,
            pose,
            args.local_window_size_x,
            args.local_window_size_y,
        )
        reveal(
            grid,
            known_free,
            known_occupied,
            pose,
            args.sensor_range,
            args.rays,
            window,
        )
        return window

    def capture(frontier=None, frontier_path=None, target_reached=False):
        window = local_window_bounds(
            grid,
            pose,
            args.local_window_size_x,
            args.local_window_size_y,
        )
        first_column, first_row, last_column, last_row = window
        snapshots.append(
            {
                "step": len(snapshots),
                "pose": (int(pose[0]), int(pose[1])),
                "trajectory_count": len(trajectory),
                "trajectory_length": round(path_distance(grid, trajectory), 3),
                "known_free": known_free.copy(),
                "known_occupied": known_occupied.copy(),
                "window": window,
                "window_size": [
                    args.local_window_size_x,
                    args.local_window_size_y,
                ],
                "sensor_range": args.sensor_range,
                "rays": args.rays,
                "local_known_free": known_free[
                    first_row:last_row, first_column:last_column
                ].copy(),
                "local_known_occupied": known_occupied[
                    first_row:last_row, first_column:last_column
                ].copy(),
                "frontier": frontier,
                "frontier_path": list(frontier_path or []),
                "target_reached": bool(target_reached),
                "coverage": round(
                    known_free.sum() / max(1, int(grid["free"].sum())),
                    4,
                ),
            }
        )

    sense()
    capture()
    for _ in range(args.max_steps):
        distances, parents = bfs(known_free, pose)
        target_distance = distances[target[1], target[0]]
        if target_distance >= 0:
            path = reconstruct(parents, pose, target)
            if path is not None:
                trajectory.extend(path[1:])
                pose = target
                sense()
                capture(target_reached=True)
                stop_reason = "target_reached"
                target_found = True
                break

        unknown = ~(known_free | known_occupied)
        frontier = frontier_mask(known_free, unknown)
        chosen = frontier_score(
            grid, distances, frontier, target, args.target_weight
        )
        if chosen is None:
            stop_reason = "no_reachable_frontier"
            break
        path = reconstruct(parents, pose, chosen)
        if path is None or len(path) < 2:
            stop_reason = "frontier_path_missing"
            break
        frontier_choices.append(point_for_cell(grid, chosen))
        move_count = min(len(path) - 1, args.move_cells)
        for next_pose in path[1 : move_count + 1]:
            pose = next_pose
            trajectory.append(pose)
        sense()
        capture(frontier=chosen, frontier_path=path)

    if (
        not snapshots
        or snapshots[-1]["pose"] != (int(pose[0]), int(pose[1]))
        or snapshots[-1]["target_reached"] != target_found
    ):
        capture(target_reached=target_found)

    coverage = known_free.sum() / max(1, int(grid["free"].sum()))
    target_x, target_y = point_for_cell(grid, target)
    pose_x, pose_y = point_for_cell(grid, pose)
    result = {
        "start": [int(start[0]), int(start[1])],
        "target": [int(target[0]), int(target[1])],
        "target_pose": [round(target_x, 3), round(target_y, 3)],
        "target_reached": target_found,
        "stop_reason": stop_reason,
        "steps": len(trajectory) - 1,
        "trajectory_length": round(path_distance(grid, trajectory), 3),
        "final_pose": [round(pose_x, 3), round(pose_y, 3)],
        "distance_to_target": round(
            math.hypot(pose_x - target_x, pose_y - target_y), 3
        ),
        "known_free_cells": int(known_free.sum()),
        "coverage": round(float(coverage), 4),
        "frontier_choices": [
            [round(x, 3), round(y, 3)] for x, y in frontier_choices
        ],
        "trajectory": [[int(column), int(row)] for column, row in trajectory],
        "known_free": known_free,
        "known_occupied": known_occupied,
    }
    return result, snapshots


def mask_neighbors(cell, mask):
    column, row = cell
    for delta_column, delta_row in NEIGHBOR_OFFSETS:
        next_column = column + delta_column
        next_row = row + delta_row
        if (
            0 <= next_row < mask.shape[0]
            and 0 <= next_column < mask.shape[1]
            and mask[next_row, next_column]
        ):
            yield next_column, next_row


def mask_degrees(mask):
    kernel = np.ones((3, 3), dtype=np.int16)
    kernel[1, 1] = 0
    return convolve(mask.astype(np.int16), kernel, mode="constant", cval=0)


def prune_skeleton_spurs(skeleton, min_length):
    skeleton = skeleton.copy()
    for _ in range(5):
        degrees = mask_degrees(skeleton)
        endpoints = list(zip(*np.where(skeleton & (degrees == 1))))
        removed = 0
        for row, column in endpoints:
            if not skeleton[row, column]:
                continue
            path = [(int(column), int(row))]
            previous = None
            current = path[0]
            while True:
                candidates = [
                    point
                    for point in mask_neighbors(current, skeleton)
                    if point != previous
                ]
                if len(candidates) != 1:
                    break
                following = candidates[0]
                if degrees[following[1], following[0]] != 2:
                    break
                path.append(following)
                previous, current = current, following
                if len(path) >= min_length:
                    break
            if len(path) < min_length:
                for column, row in path:
                    skeleton[row, column] = False
                removed += len(path)
        if removed == 0:
            break
    return skeleton


def skeleton_edges(edges):
    result = defaultdict(list)
    for edge_id, edge in edges.items():
        result[edge["a"]].append(edge_id)
        result[edge["b"]].append(edge_id)
    return result


def path_cells_length(path, resolution):
    return sum(point_distance(a, b) for a, b in zip(path, path[1:])) * resolution


def min_path_clearance(grid, path):
    return min(float(grid["distance"][row, column]) for column, row in path)


def add_graph_edge(edges, nodes, grid, first, second, path):
    if first == second or len(path) < 2:
        return
    clean = [path[0]]
    for point in path[1:]:
        if point != clean[-1]:
            clean.append(point)
    clean[0] = nodes[first]["cell"]
    clean[-1] = nodes[second]["cell"]
    if len(clean) < 2 or clean[0] == clean[-1]:
        return
    length = path_cells_length(clean, grid["resolution"])
    if length <= 0.0:
        return
    key = (min(first, second), max(first, second), round(length, 2))
    for edge in edges.values():
        edge_key = (
            min(edge["a"], edge["b"]),
            max(edge["a"], edge["b"]),
            round(edge["length"], 2),
        )
        if edge_key == key:
            return
    edge_id = max(edges.keys(), default=0) + 1
    edges[edge_id] = {
        "id": edge_id,
        "a": first,
        "b": second,
        "path": clean,
        "length": round(length, 3),
        "min_clearance": round(min_path_clearance(grid, clean), 3),
    }


def find_parent(parents, node):
    while parents[node] != node:
        parents[node] = parents[parents[node]]
        node = parents[node]
    return node


def union_nodes(parents, first, second):
    first_root = find_parent(parents, first)
    second_root = find_parent(parents, second)
    if first_root != second_root:
        parents[second_root] = first_root


def merge_close_nodes(nodes, edges, grid, merge_distance):
    if not nodes:
        return nodes, edges
    parents = {node_id: node_id for node_id in nodes}
    for edge in edges.values():
        if edge["length"] <= merge_distance:
            union_nodes(parents, edge["a"], edge["b"])

    groups = defaultdict(list)
    for node_id in nodes:
        groups[find_parent(parents, node_id)].append(node_id)
    representative = {}
    merged_nodes = {}
    for group_index, group in enumerate(groups.values(), start=1):
        best = max(group, key=lambda node_id: nodes[node_id]["clearance"])
        merged_nodes[group_index] = {
            "id": group_index,
            "cell": nodes[best]["cell"],
            "clearance": nodes[best]["clearance"],
            "cluster_size": sum(
                nodes[node_id]["cluster_size"] for node_id in group
            ),
            "source_nodes": group,
        }
        for node_id in group:
            representative[node_id] = group_index

    merged_edges = {}
    for edge in edges.values():
        first = representative[edge["a"]]
        second = representative[edge["b"]]
        if first == second:
            continue
        path = list(edge["path"])
        path[0] = merged_nodes[first]["cell"]
        path[-1] = merged_nodes[second]["cell"]
        if len(path) < 2 or path[0] == path[-1]:
            continue
        length = path_cells_length(path, grid["resolution"])
        key = (min(first, second), max(first, second), round(length, 2))
        duplicate = False
        for existing in merged_edges.values():
            existing_key = (
                min(existing["a"], existing["b"]),
                max(existing["a"], existing["b"]),
                round(existing["length"], 2),
            )
            if key == existing_key:
                duplicate = True
                break
        if duplicate:
            continue
        edge_id = max(merged_edges.keys(), default=0) + 1
        merged_edges[edge_id] = {
            "id": edge_id,
            "a": first,
            "b": second,
            "path": path,
            "length": round(length, 3),
            "min_clearance": round(min_path_clearance(grid, path), 3),
        }
    return merged_nodes, merged_edges


def prune_short_leaves(
    nodes, edges, min_length, grid=None, protected_points=()
):
    while True:
        adjacency = skeleton_edges(edges)
        removed = False
        for node_id in list(nodes):
            incident = adjacency.get(node_id, [])
            if len(incident) == 0:
                del nodes[node_id]
                removed = True
                break
            if len(incident) != 1:
                continue
            edge = edges[incident[0]]
            if edge["length"] >= min_length:
                continue
            if nodes[node_id].get("type") in ("START", "TARGET"):
                continue
            if grid is not None and protected_points:
                node_x, node_y = point_for_cell(grid, nodes[node_id]["cell"])
                if any(
                    math.hypot(node_x - point[0], node_y - point[1])
                    <= max(1.20, 1.5 * min_length)
                    for point in protected_points
                ):
                    continue
            del edges[incident[0]]
            del nodes[node_id]
            removed = True
            break
        if not removed:
            break
    return nodes, edges


def oriented_edge_path(edge, node_id):
    if edge["a"] == node_id:
        return list(edge["path"])
    if edge["b"] == node_id:
        return list(reversed(edge["path"]))
    raise ValueError("node is not incident to edge")


def suppress_degree_two(nodes, edges, grid):
    while True:
        adjacency = skeleton_edges(edges)
        selected = None
        for node_id, incident in adjacency.items():
            if len(incident) != 2:
                continue
            first_edge = edges[incident[0]]
            second_edge = edges[incident[1]]
            first_other = (
                first_edge["b"]
                if first_edge["a"] == node_id
                else first_edge["a"]
            )
            second_other = (
                second_edge["b"]
                if second_edge["a"] == node_id
                else second_edge["a"]
            )
            if first_other == second_other:
                continue
            selected = (
                node_id,
                incident[0],
                incident[1],
                first_other,
                second_other,
            )
            break
        if selected is None:
            break
        node_id, first_id, second_id, first_other, second_other = selected
        first_path = oriented_edge_path(edges[first_id], node_id)
        second_path = oriented_edge_path(edges[second_id], node_id)
        combined = list(reversed(first_path)) + second_path[1:]
        length = path_cells_length(combined, grid["resolution"])
        key = (
            min(first_other, second_other),
            max(first_other, second_other),
            round(length, 2),
        )
        duplicate = False
        for edge_id, edge in edges.items():
            if edge_id in (first_id, second_id):
                continue
            edge_key = (
                min(edge["a"], edge["b"]),
                max(edge["a"], edge["b"]),
                round(edge["length"], 2),
            )
            if edge_key == key:
                duplicate = True
                break
        del edges[first_id]
        del edges[second_id]
        del nodes[node_id]
        if not duplicate:
            edge_id = max(edges.keys(), default=0) + 1
            edges[edge_id] = {
                "id": edge_id,
                "a": first_other,
                "b": second_other,
                "path": combined,
                "length": round(length, 3),
                "min_clearance": round(min_path_clearance(grid, combined), 3),
            }
    return nodes, edges


def collapse_degree_two_node(nodes, edges, grid, node_id):
    adjacency = skeleton_edges(edges)
    incident = adjacency.get(node_id, [])
    if len(incident) != 2:
        return False
    first_edge = edges[incident[0]]
    second_edge = edges[incident[1]]
    first_other = (
        first_edge["b"] if first_edge["a"] == node_id else first_edge["a"]
    )
    second_other = (
        second_edge["b"] if second_edge["a"] == node_id else second_edge["a"]
    )
    if first_other == second_other:
        return False
    first_path = oriented_edge_path(first_edge, node_id)
    second_path = oriented_edge_path(second_edge, node_id)
    combined = list(reversed(first_path)) + second_path[1:]
    del edges[incident[0]]
    del edges[incident[1]]
    del nodes[node_id]
    add_graph_edge(
        edges, nodes, grid, first_other, second_other, combined
    )
    return True


def absorb_turns_at_junctions(nodes, edges, grid, max_distance):
    while True:
        adjacency = skeleton_edges(edges)
        selected = None
        for node_id, node in nodes.items():
            if node.get("type") != "TURN":
                continue
            incident = adjacency.get(node_id, [])
            if len(incident) != 2:
                continue
            for edge_id in incident:
                edge = edges[edge_id]
                other = edge["b"] if edge["a"] == node_id else edge["a"]
                if nodes[other].get("type") not in (
                    "T_JUNCTION",
                    "CROSS",
                ):
                    continue
                if edge["length"] <= max_distance:
                    selected = node_id
                    break
            if selected is not None:
                break
        if selected is None or not collapse_degree_two_node(
            nodes, edges, grid, selected
        ):
            break


def graph_distances(edges, source, limit):
    adjacency = defaultdict(list)
    for edge in edges.values():
        adjacency[edge["a"]].append((edge["b"], edge["length"]))
        adjacency[edge["b"]].append((edge["a"], edge["length"]))
    distances = {source: 0.0}
    queue = [(0.0, source)]
    while queue:
        distance, node_id = heapq.heappop(queue)
        if distance > distances.get(node_id, math.inf):
            continue
        if distance > limit:
            continue
        for neighbor, length in adjacency.get(node_id, []):
            candidate = distance + length
            if candidate > limit:
                continue
            if candidate >= distances.get(neighbor, math.inf):
                continue
            distances[neighbor] = candidate
            heapq.heappush(queue, (candidate, neighbor))
    return distances


def cluster_junctions(nodes, edges, grid, args):
    adjacency = skeleton_edges(edges)
    candidates = [
        node_id
        for node_id in nodes
        if len(adjacency.get(node_id, [])) >= 3
    ]
    parents = {node_id: node_id for node_id in nodes}
    for source in candidates:
        distances = graph_distances(
            edges, source, args.junction_cluster_path
        )
        source_x, source_y = point_for_cell(grid, nodes[source]["cell"])
        for target in candidates:
            if target <= source or target not in distances:
                continue
            target_x, target_y = point_for_cell(grid, nodes[target]["cell"])
            if (
                math.hypot(target_x - source_x, target_y - source_y)
                <= args.junction_cluster_radius
            ):
                union_nodes(parents, source, target)

    groups = defaultdict(list)
    for node_id in nodes:
        groups[find_parent(parents, node_id)].append(node_id)

    mapping = {}
    cluster_nodes = {}
    for group_index, group in enumerate(groups.values(), start=1):
        cells = np.asarray([nodes[node_id]["cell"] for node_id in group])
        mean_cell = cells.mean(axis=0)
        representative = min(
            group,
            key=lambda node_id: (
                float(
                    (nodes[node_id]["cell"][0] - mean_cell[0]) ** 2
                    + (nodes[node_id]["cell"][1] - mean_cell[1]) ** 2
                ),
                -nodes[node_id]["clearance"],
            ),
        )
        cluster_node = {
            "id": group_index,
            "cell": nodes[representative]["cell"],
            "clearance": max(
                nodes[node_id]["clearance"] for node_id in group
            ),
            "cluster_size": sum(
                nodes[node_id]["cluster_size"] for node_id in group
            ),
            "source_nodes": group,
        }
        if len(group) == 1 and "type" in nodes[group[0]]:
            cluster_node["type"] = nodes[group[0]]["type"]
        cluster_nodes[group_index] = cluster_node
        for node_id in group:
            mapping[node_id] = group_index

    cluster_edges = {}
    pair_edges = {}
    for edge in edges.values():
        first = mapping[edge["a"]]
        second = mapping[edge["b"]]
        if first == second:
            continue
        path = list(edge["path"])
        path[0] = cluster_nodes[first]["cell"]
        path[-1] = cluster_nodes[second]["cell"]
        if len(path) < 2 or path[0] == path[-1]:
            continue
        length = path_cells_length(path, grid["resolution"])
        pair = (min(first, second), max(first, second))
        if pair in pair_edges:
            previous = cluster_edges[pair_edges[pair]]
            if previous["length"] <= length:
                continue
            del cluster_edges[previous["id"]]
        edge_id = max(cluster_edges.keys(), default=0) + 1
        cluster_edges[edge_id] = {
            "id": edge_id,
            "a": first,
            "b": second,
            "path": path,
            "length": round(length, 3),
            "min_clearance": round(min_path_clearance(grid, path), 3),
        }
        pair_edges[pair] = edge_id
    return cluster_nodes, cluster_edges


def reindex_nodes(nodes, edges):
    ordered = sorted(nodes)
    mapping = {node_id: index for index, node_id in enumerate(ordered, start=1)}
    new_nodes = {}
    for node_id in ordered:
        node = dict(nodes[node_id])
        node["id"] = mapping[node_id]
        new_nodes[mapping[node_id]] = node
    new_edges = {}
    for index, edge in enumerate(edges.values(), start=1):
        new_edges[index] = {
            "id": index,
            "a": mapping[edge["a"]],
            "b": mapping[edge["b"]],
            "path": edge["path"],
            "length": edge["length"],
            "min_clearance": edge["min_clearance"],
        }
    return new_nodes, new_edges


def raw_skeleton_graph(known_free, grid, min_spur_length):
    free_mask = known_free & grid["free"]
    if not free_mask.any():
        return {}, {}
    if not hasattr(cv2, "ximgproc") or not hasattr(cv2.ximgproc, "thinning"):
        raise RuntimeError("opencv-contrib-python is required for skeletonization")
    skeleton = cv2.ximgproc.thinning(free_mask.astype(np.uint8) * 255) // 255
    minimum_cells = max(2, int(round(min_spur_length / grid["resolution"])))
    skeleton = prune_skeleton_spurs(skeleton, minimum_cells)
    degrees = mask_degrees(skeleton)
    special = skeleton & (degrees != 2)
    special_labels, special_count = label(
        special, structure=np.ones((3, 3), dtype=int)
    )

    nodes = {}
    for cluster_id in range(1, special_count + 1):
        rows, columns = np.where(special_labels == cluster_id)
        if len(rows) == 0:
            continue
        clearance = grid["distance"][rows, columns]
        best = int(np.argmax(clearance))
        nodes[cluster_id] = {
            "id": cluster_id,
            "cell": (int(columns[best]), int(rows[best])),
            "clearance": round(float(clearance[best]), 3),
            "cluster_size": int(len(rows)),
        }

    edges = {}
    path_mask = skeleton & ~special
    path_labels, path_count = label(
        path_mask, structure=np.ones((3, 3), dtype=int)
    )

    def adjacent_clusters(point):
        return {
            int(special_labels[row, column])
            for column, row in mask_neighbors(point, special)
            if special_labels[row, column] > 0
        }

    for path_id in range(1, path_count + 1):
        rows, columns = np.where(path_labels == path_id)
        points = [(int(column), int(row)) for row, column in zip(rows, columns)]
        point_set = set(points)
        ends = [
            point
            for point in points
            if sum(
                neighbor in point_set
                for neighbor in mask_neighbors(point, skeleton)
            )
            <= 1
        ]
        if not ends:
            continue
        start = ends[0]
        order = [start]
        visited = {start}
        current = start
        while True:
            candidates = [
                point
                for point in mask_neighbors(current, skeleton)
                if point in point_set and point not in visited
            ]
            if not candidates:
                break
            following = candidates[0]
            order.append(following)
            visited.add(following)
            current = following
        first_clusters = adjacent_clusters(order[0])
        second_clusters = adjacent_clusters(order[-1])
        if not first_clusters or not second_clusters:
            continue
        for first in first_clusters:
            for second in second_clusters:
                if first not in nodes or second not in nodes:
                    continue
                path = [nodes[first]["cell"]] + order + [nodes[second]["cell"]]
                add_graph_edge(edges, nodes, grid, first, second, path)

    direct_pairs = set()
    for row, column in zip(*np.where(special)):
        first = int(special_labels[row, column])
        for next_column, next_row in mask_neighbors((column, row), special):
            second = int(special_labels[next_row, next_column])
            if (
                first <= 0
                or second <= 0
                or first == second
                or first not in nodes
                or second not in nodes
            ):
                continue
            pair = (min(first, second), max(first, second))
            if pair in direct_pairs:
                continue
            direct_pairs.add(pair)
            path = [nodes[first]["cell"], nodes[second]["cell"]]
            add_graph_edge(edges, nodes, grid, first, second, path)
    return nodes, edges


def marker_candidates(edge, grid, args):
    path = edge["path"]
    if len(path) < 4:
        return []
    points = np.asarray([point_for_cell(grid, cell) for cell in path], dtype=float)
    cumulative = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    )
    total = float(cumulative[-1])
    if total < 0.8:
        return []

    turns = []
    for index in range(1, len(path) - 1):
        if cumulative[index] < args.corner_window + 0.05:
            continue
        if total - cumulative[index] < args.corner_window + 0.05:
            continue
        left = int(
            np.searchsorted(
                cumulative, cumulative[index] - args.corner_window
            )
        )
        right = int(
            np.searchsorted(
                cumulative,
                cumulative[index] + args.corner_window,
                side="right",
            )
            - 1
        )
        left = max(0, min(left, index - 1))
        right = max(index + 1, min(right, len(path) - 1))
        angle = heading_change(points[left], points[index], points[right])
        if angle >= args.corner_angle:
            turns.append((float(angle), index))

    selected_turns = []
    for angle, index in sorted(turns, reverse=True):
        if all(
            abs(cumulative[index] - cumulative[existing]) >= args.turn_spacing
            for _, existing in selected_turns
        ):
            selected_turns.append((angle, index))
    selected_turns.sort(key=lambda item: item[1])

    portals = []
    clearances = np.asarray(
        [grid["distance"][row, column] for column, row in path], dtype=float
    )
    for index in range(2, len(path) - 2):
        if cumulative[index] < args.portal_window / 2.0:
            continue
        if total - cumulative[index] < args.portal_window / 2.0:
            continue
        if clearances[index] > args.portal_max_width / 2.0:
            continue
        left = int(
            np.searchsorted(
                cumulative, max(0.0, cumulative[index] - args.portal_window)
            )
        )
        right = int(
            np.searchsorted(
                cumulative,
                min(total, cumulative[index] + args.portal_window),
                side="right",
            )
        )
        left = max(0, min(left, index - 1))
        right = max(index + 1, min(right, len(path)))
        left_max = float(clearances[left:index].max())
        right_max = float(clearances[index + 1 : right].max())
        if (
            min(left_max, right_max) - clearances[index]
            >= args.portal_shoulder
        ):
            portals.append((clearances[index], index))

    selected_portals = []
    for clearance, index in sorted(portals):
        if all(
            abs(cumulative[index] - cumulative[existing])
            >= args.portal_spacing
            for _, existing in selected_portals
        ):
            selected_portals.append((clearance, index))
    selected_portals.sort(key=lambda item: item[1])

    markers = [
        {
            "index": index,
            "type": "PORTAL",
            "width": round(2.0 * clearance, 3),
        }
        for clearance, index in selected_portals
    ]
    for angle, index in selected_turns:
        if any(
            abs(cumulative[index] - cumulative[marker["index"]])
            < args.turn_spacing * 0.6
            for marker in markers
        ):
            continue
        markers.append(
            {
                "index": index,
                "type": "TURN",
                "heading_change_deg": round(angle, 1),
            }
        )
    markers.sort(key=lambda marker: marker["index"])
    return markers


def split_edges_at_markers(nodes, edges, grid, args):
    split_nodes = dict(nodes)
    split_edges = {}
    for edge in edges.values():
        path = edge["path"]
        markers = marker_candidates(edge, grid, args)
        if not markers:
            new_id = max(split_edges.keys(), default=0) + 1
            split_edges[new_id] = {
                "id": new_id,
                "a": edge["a"],
                "b": edge["b"],
                "path": path,
                "length": edge["length"],
                "min_clearance": edge["min_clearance"],
            }
            continue
        cuts = [{"index": 0, "node": edge["a"], "type": None}]
        for marker in markers:
            node_id = max(split_nodes.keys(), default=0) + 1
            split_nodes[node_id] = {
                "id": node_id,
                "cell": path[marker["index"]],
                "clearance": round(
                    float(
                        grid["distance"][
                            path[marker["index"]][1], path[marker["index"]][0]
                        ]
                    ),
                    3,
                ),
                "cluster_size": 1,
                "type": marker["type"],
            }
            for key, value in marker.items():
                if key not in ("index", "type"):
                    split_nodes[node_id][key] = value
            cuts.append(
                {
                    "index": marker["index"],
                    "node": node_id,
                    "type": marker["type"],
                }
            )
        cuts.append({"index": len(path) - 1, "node": edge["b"], "type": None})
        for first, second in zip(cuts[:-1], cuts[1:]):
            if second["index"] <= first["index"]:
                continue
            segment = path[first["index"] : second["index"] + 1]
            if len(segment) < 2:
                continue
            add_graph_edge(
                split_edges,
                split_nodes,
                grid,
                first["node"],
                second["node"],
                segment,
            )
    return split_nodes, split_edges


def nearest_graph_node(nodes, grid, point):
    point_x, point_y = point
    best = None
    for node_id, node in nodes.items():
        x, y = point_for_cell(grid, node["cell"])
        distance = math.hypot(x - point_x, y - point_y)
        if best is None or distance < best[0]:
            best = (distance, node_id)
    return best


def insert_point_node(
    nodes, edges, grid, point, node_type, point_snap_distance
):
    distance_to_node, closest_node = nearest_graph_node(nodes, grid, point)
    if distance_to_node <= point_snap_distance:
        nodes[closest_node]["type"] = node_type
        nodes[closest_node]["requested_pose"] = [
            round(point[0], 3),
            round(point[1], 3),
        ]
        return closest_node

    best_edge = None
    best_index = None
    best_distance = None
    for edge in edges.values():
        for index, cell in enumerate(edge["path"]):
            x, y = point_for_cell(grid, cell)
            distance = math.hypot(x - point[0], y - point[1])
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_edge = edge
                best_index = index
    if best_edge is None:
        raise RuntimeError("no topology edge available for waypoint")

    if best_index == 0:
        endpoint = best_edge["a"]
        nodes[endpoint]["type"] = node_type
        nodes[endpoint]["requested_pose"] = [
            round(point[0], 3),
            round(point[1], 3),
        ]
        return endpoint
    if best_index == len(best_edge["path"]) - 1:
        endpoint = best_edge["b"]
        nodes[endpoint]["type"] = node_type
        nodes[endpoint]["requested_pose"] = [
            round(point[0], 3),
            round(point[1], 3),
        ]
        return endpoint

    node_id = max(nodes.keys(), default=0) + 1
    cell = best_edge["path"][best_index]
    nodes[node_id] = {
        "id": node_id,
        "cell": cell,
        "clearance": round(float(grid["distance"][cell[1], cell[0]]), 3),
        "cluster_size": 1,
        "type": node_type,
        "requested_pose": [round(point[0], 3), round(point[1], 3)],
    }
    removed_id = best_edge["id"]
    first_path = best_edge["path"][: best_index + 1]
    second_path = best_edge["path"][best_index:]
    del edges[removed_id]
    if best_index > 0:
        add_graph_edge(
            edges, nodes, grid, best_edge["a"], node_id, first_path
        )
    if best_index < len(best_edge["path"]) - 1:
        add_graph_edge(
            edges, nodes, grid, node_id, best_edge["b"], second_path
        )
    return node_id


def bearing_difference(first, second):
    return abs(((first - second + 180.0) % 360.0) - 180.0)


def edge_bearing(edge, node_id, nodes, grid, sample_distance):
    path = oriented_edge_path(edge, node_id)
    start_x, start_y = point_for_cell(grid, nodes[node_id]["cell"])
    target_x, target_y = start_x, start_y
    for cell in path[1:]:
        target_x, target_y = point_for_cell(grid, cell)
        if math.hypot(target_x - start_x, target_y - start_y) >= sample_distance:
            break
    return math.degrees(math.atan2(target_y - start_y, target_x - start_x))


def unique_directions(directions, merge_angle):
    unique = []
    for direction in directions:
        if all(
            bearing_difference(direction, existing) > merge_angle
            for existing in unique
        ):
            unique.append(direction)
    return unique


def classify_decision_nodes(
    nodes, edges, grid, known_free, known_occupied, args
):
    adjacency = skeleton_edges(edges)
    unknown = ~(known_free | known_occupied)
    near_unknown = binary_dilation(
        unknown, structure=disk(args.frontier_radius / grid["resolution"])
    )
    for node_id, node in nodes.items():
        incident = adjacency.get(node_id, [])
        node["degree"] = len(incident)
        if node.get("type") in ("START", "TARGET", "TURN", "PORTAL"):
            continue
        if len(incident) <= 1:
            node["type"] = "DEAD_END"
            row, column = node["cell"][1], node["cell"][0]
            node["frontier"] = bool(near_unknown[row, column])
            continue
        directions = unique_directions(
            [
                edge_bearing(
                    edges[edge_id],
                    node_id,
                    nodes,
                    grid,
                    args.direction_sample_distance,
                )
                for edge_id in incident
            ],
            args.direction_merge_angle,
        )
        node["unique_directions"] = len(directions)
        if len(directions) >= 4:
            node["type"] = "CROSS"
        elif len(directions) == 3:
            node["type"] = "T_JUNCTION"
        elif len(directions) == 2:
            angle = bearing_difference(directions[0], directions[1])
            if len(incident) >= 3:
                node["type"] = "T_JUNCTION"
            elif angle >= 150.0:
                node["type"] = "PASS"
            else:
                node["type"] = "TURN"
        else:
            node["type"] = "DEAD_END"


def collapse_pass_nodes(nodes, edges, grid):
    while True:
        adjacency = skeleton_edges(edges)
        pass_node = next(
            (
                node_id
                for node_id, node in nodes.items()
                if node.get("type") == "PASS"
            ),
            None,
        )
        if pass_node is None:
            break
        incident = adjacency.get(pass_node, [])
        if len(incident) != 2:
            if not incident:
                del nodes[pass_node]
            else:
                nodes[pass_node]["type"] = (
                    "T_JUNCTION" if len(incident) >= 3 else "DEAD_END"
                )
            continue
        first_edge = edges[incident[0]]
        second_edge = edges[incident[1]]
        first_other = (
            first_edge["b"] if first_edge["a"] == pass_node else first_edge["a"]
        )
        second_other = (
            second_edge["b"]
            if second_edge["a"] == pass_node
            else second_edge["a"]
        )
        if first_other == second_other:
            del edges[incident[0]]
            del edges[incident[1]]
            del nodes[pass_node]
            continue
        first_path = oriented_edge_path(first_edge, pass_node)
        second_path = oriented_edge_path(second_edge, pass_node)
        combined = list(reversed(first_path)) + second_path[1:]
        del edges[incident[0]]
        del edges[incident[1]]
        del nodes[pass_node]
        add_graph_edge(
            edges, nodes, grid, first_other, second_other, combined
        )


def verify_topology(nodes, edges, grid, trajectory, args):
    trajectory_world = np.asarray(
        [point_for_cell(grid, cell) for cell in trajectory], dtype=float
    )
    tree = cKDTree(trajectory_world)
    for edge in edges.values():
        edge_world = np.asarray(
            [point_for_cell(grid, cell) for cell in edge["path"]], dtype=float
        )
        distances, _ = tree.query(edge_world)
        support = float(np.mean(distances <= args.verify_distance))
        edge["trajectory_support"] = round(support, 3)
        edge["verified"] = support >= args.verify_fraction
    verified_nodes = set()
    for edge in edges.values():
        if edge["verified"]:
            verified_nodes.add(edge["a"])
            verified_nodes.add(edge["b"])
    for node_id, node in nodes.items():
        node["verified"] = node_id in verified_nodes
        if node.get("type") == "PORTAL":
            node["state"] = "OPEN" if node["verified"] else "UNKNOWN"


def dijkstra(nodes, edges, start_id, target_id, verified_only):
    adjacency = defaultdict(list)
    for edge in edges.values():
        if verified_only and not edge.get("verified", False):
            continue
        adjacency[edge["a"]].append((edge["b"], edge["length"], edge["id"]))
        adjacency[edge["b"]].append((edge["a"], edge["length"], edge["id"]))
    if start_id not in nodes or target_id not in nodes:
        return {"found": False, "node_ids": [], "edge_ids": [], "length": None}
    distances = {start_id: 0.0}
    parents = {}
    queue = [(0.0, start_id)]
    while queue:
        distance, node_id = heapq.heappop(queue)
        if distance > distances.get(node_id, math.inf):
            continue
        if node_id == target_id:
            break
        for neighbor, length, edge_id in adjacency.get(node_id, []):
            candidate = distance + length
            if candidate >= distances.get(neighbor, math.inf):
                continue
            distances[neighbor] = candidate
            parents[neighbor] = (node_id, edge_id)
            heapq.heappush(queue, (candidate, neighbor))
    if target_id not in distances:
        return {"found": False, "node_ids": [], "edge_ids": [], "length": None}
    node_ids = [target_id]
    edge_ids = []
    current = target_id
    while current != start_id:
        previous, edge_id = parents[current]
        edge_ids.append(edge_id)
        node_ids.append(previous)
        current = previous
    return {
        "found": True,
        "node_ids": list(reversed(node_ids)),
        "edge_ids": list(reversed(edge_ids)),
        "length": round(distances[target_id], 3),
    }


def topology_record(nodes, edges):
    node_records = []
    for node in sorted(nodes.values(), key=lambda item: item["id"]):
        record = {
            "id": node["id"],
            "type": node.get("type", "PASS"),
            "cell": [int(node["cell"][0]), int(node["cell"][1])],
            "clearance": node["clearance"],
            "degree": node.get("degree", 0),
            "verified": bool(node.get("verified", False)),
            "first_visible_step": int(node.get("first_visible_step", 0)),
        }
        if "requested_pose" in node:
            record["requested_pose"] = node["requested_pose"]
        if "heading_change_deg" in node:
            record["heading_change_deg"] = node["heading_change_deg"]
        if "width" in node:
            record["width"] = node["width"]
        if "state" in node:
            record["state"] = node["state"]
        if "frontier" in node:
            record["frontier"] = bool(node["frontier"])
        if "unique_directions" in node:
            record["unique_directions"] = int(node["unique_directions"])
        node_records.append(record)
    edge_records = []
    for edge in sorted(edges.values(), key=lambda item: item["id"]):
        edge_records.append(
            {
                "id": edge["id"],
                "from": edge["a"],
                "to": edge["b"],
                "length": edge["length"],
                "min_clearance": edge["min_clearance"],
                "verified": bool(edge.get("verified", False)),
                "trajectory_support": edge.get("trajectory_support", 0.0),
                "first_visible_step": int(
                    edge.get("first_visible_step", 0)
                ),
            }
        )
    return {
        "nodes": node_records,
        "edges": edge_records,
        "node_types": dict(
            sorted(
                (
                    node_type,
                    sum(
                        1
                        for node in node_records
                        if node["type"] == node_type
                    ),
                )
                for node_type in {node["type"] for node in node_records}
            )
        ),
    }


def image_point(grid, cell, scale):
    return (
        int(round(cell[0] * scale)),
        int(round((grid["height"] - 1 - cell[1]) * scale)),
    )


def draw_node_shape(draw, x, y, radius, node_type, frontier=False):
    color = NODE_COLORS.get(node_type, (100, 100, 100))
    if node_type == "T_JUNCTION":
        draw.polygon(
            [
                (x, y - radius),
                (x - radius, y + radius),
                (x + radius, y + radius),
            ],
            fill=color,
            outline=(255, 255, 255),
        )
    elif node_type == "CROSS":
        draw.rectangle(
            (x - radius, y - radius, x + radius, y + radius),
            fill=color,
            outline=(255, 255, 255),
        )
    elif node_type == "DEAD_END":
        if frontier:
            color = (205, 205, 205)
        draw.polygon(
            [
                (x, y - radius),
                (x + radius, y),
                (x, y + radius),
                (x - radius, y),
            ],
            fill=color,
            outline=(255, 255, 255),
        )
    else:
        draw.ellipse(
            (x - radius, y - radius, x + radius, y + radius),
            fill=color,
            outline=(255, 255, 255),
        )


def render(
    grid,
    result,
    nodes,
    edges,
    waypoints,
    route,
    destination,
    window=None,
):
    base = np.empty((grid["height"], grid["width"], 3), dtype=np.uint8)
    base[:] = (90, 90, 90)
    base[result["known_free"]] = (235, 235, 235)
    base[result["known_occupied"]] = (25, 25, 25)
    for column, row in result["trajectory"]:
        base[row, column] = (45, 100, 230)

    scale = 4
    picture = Image.fromarray(np.flipud(base)).resize(
        (grid["width"] * scale, grid["height"] * scale),
        Image.NEAREST,
    )
    draw = ImageDraw.Draw(picture)
    route_edges = set(route.get("edge_ids", []))
    for edge in edges.values():
        points = [image_point(grid, cell, scale) for cell in edge["path"]]
        if edge["id"] in route_edges:
            color = (0, 220, 220)
            width = max(2, scale)
        elif edge.get("verified", False):
            color = (20, 155, 75)
            width = max(1, scale // 2)
        else:
            color = (165, 165, 165)
            width = 1
        draw.line(points, fill=color, width=width)

    for node in nodes.values():
        x, y = image_point(grid, node["cell"], scale)
        radius = 2 * scale
        node_type = node.get("type", "PASS")
        draw_node_shape(
            draw,
            x,
            y,
            radius,
            node_type,
            node.get("frontier", False),
        )
        abbreviation = NODE_ABBREVIATIONS.get(node_type, "?")
        label = f"{abbreviation}{node['id']}"
        if node_type == "DEAD_END" and node.get("frontier", False):
            label += "?"
        draw.text(
            (x + radius + 2, y - radius),
            label,
            fill=(0, 0, 0),
        )

    for index, waypoint in enumerate(waypoints):
        cell = cell_for_point(grid, waypoint)
        x, y = image_point(grid, cell, scale)
        draw.ellipse(
            (x - 3 * scale, y - 3 * scale, x + 3 * scale, y + 3 * scale),
            outline=(235, 140, 30),
            width=scale,
        )
        draw.text((x + 4 * scale, y - 3 * scale), f"W{index}", fill=(130, 65, 0))

    if window is not None:
        left = window[0] * scale
        top = (grid["height"] - window[3]) * scale
        right = max(left, (window[2] - 1) * scale)
        bottom = max(top, (grid["height"] - 1 - window[1]) * scale)
        draw.rectangle(
            (left, top, right, bottom),
            outline=(0, 190, 255),
            width=max(2, scale // 2),
        )

    legend = [
        ("L TURN", NODE_COLORS["TURN"]),
        ("T JUNCTION", NODE_COLORS["T_JUNCTION"]),
        ("X CROSS", NODE_COLORS["CROSS"]),
        ("D DEAD_END", NODE_COLORS["DEAD_END"]),
        ("D? FRONTIER END", (205, 205, 205)),
        ("P PORTAL", NODE_COLORS["PORTAL"]),
        ("S START", NODE_COLORS["START"]),
        ("G TARGET", NODE_COLORS["TARGET"]),
        ("LOCAL WINDOW", (0, 190, 255)),
        ("VERIFIED EDGE", (20, 155, 75)),
        ("VERIFIED ROUTE", (0, 220, 220)),
    ]
    box_width = 200
    box_height = (len(legend) + 1) * 16 + 10
    draw.rectangle((6, 6, 6 + box_width, 6 + box_height), fill=(250, 250, 250))
    draw.text((12, 10), "first-pass exploration", fill=(0, 0, 0))
    for index, (label, color) in enumerate(legend, start=1):
        y = 10 + index * 16
        draw.rectangle((12, y, 24, y + 10), fill=color)
        draw.text((30, y - 2), label, fill=(0, 0, 0))
    picture.save(destination)


def compute_visibility_steps(nodes, edges, snapshots):
    if not snapshots:
        return
    last_step = snapshots[-1]["step"]
    for node in nodes.values():
        column, row = node["cell"]
        node["first_visible_step"] = last_step
        for snapshot in snapshots:
            if snapshot["known_free"][row, column]:
                node["first_visible_step"] = snapshot["step"]
                break
    for edge in edges.values():
        endpoint_step = max(
            nodes[edge["a"]].get("first_visible_step", last_step),
            nodes[edge["b"]].get("first_visible_step", last_step),
        )
        edge["first_visible_step"] = last_step
        path = edge["path"]
        for snapshot in snapshots:
            if snapshot["step"] < endpoint_step:
                continue
            visible = sum(
                snapshot["known_free"][row, column]
                for column, row in path
            )
            if visible / max(1, len(path)) >= 0.65:
                edge["first_visible_step"] = snapshot["step"]
                break


def load_font(size):
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    if path.exists():
        return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def animation_frame(
    grid,
    snapshot,
    trajectory,
    nodes,
    edges,
    route,
    waypoints,
    target_waypoint,
    last_step,
    scale,
    fonts,
):
    base = np.empty((grid["height"], grid["width"], 3), dtype=np.uint8)
    base[:] = (75, 75, 75)
    window = snapshot.get("window")
    if "local_known_free" in snapshot:
        first_column, first_row, last_column, last_row = window
        local_base = base[first_row:last_row, first_column:last_column]
        local_base[snapshot["local_known_free"]] = (235, 235, 235)
        local_base[snapshot["local_known_occupied"]] = (25, 25, 25)
    else:
        base[snapshot["known_free"]] = (235, 235, 235)
        base[snapshot["known_occupied"]] = (25, 25, 25)
    picture = Image.fromarray(np.flipud(base)).resize(
        (grid["width"] * scale, grid["height"] * scale),
        Image.NEAREST,
    )
    draw = ImageDraw.Draw(picture)
    step = snapshot["step"]
    route_edges = set(route.get("edge_ids", []))
    show_route = snapshot["target_reached"] or step == last_step

    for edge in edges.values():
        if edge.get("first_visible_step", last_step) > step:
            continue
        if show_route and edge["id"] in route_edges:
            color = (0, 220, 220)
            width = max(2, scale)
        elif edge.get("verified", False):
            color = (20, 155, 75)
            width = max(1, scale // 2)
        else:
            color = (165, 165, 165)
            width = 1
        for visible_path in clip_to_window_runs(edge["path"], window):
            draw.line(
                [image_point(grid, cell, scale) for cell in visible_path],
                fill=color,
                width=width,
            )

    frontier_path = snapshot.get("frontier_path", [])
    for visible_path in clip_to_window_runs(frontier_path, window):
        draw.line(
            [image_point(grid, cell, scale) for cell in visible_path],
            fill=(245, 195, 20),
            width=max(1, scale // 2),
        )

    trajectory_part = trajectory[: snapshot["trajectory_count"]]
    for visible_path in clip_to_window_runs(trajectory_part, window):
        draw.line(
            [image_point(grid, cell, scale) for cell in visible_path],
            fill=(45, 100, 230),
            width=max(2, scale),
        )
    pose_x, pose_y = image_point(grid, snapshot["pose"], scale)
    draw.ellipse(
        (
            pose_x - 2 * scale,
            pose_y - 2 * scale,
            pose_x + 2 * scale,
            pose_y + 2 * scale,
        ),
        fill=(0, 220, 255),
        outline=(255, 255, 255),
        width=2,
    )

    visible_nodes = [
        node
        for node in nodes.values()
        if node.get("first_visible_step", last_step) <= step
        and node.get("type") != "PASS"
        and window_contains(window, node["cell"])
    ]
    new_nodes = [
        node
        for node in visible_nodes
        if node.get("first_visible_step", last_step) == step
    ]
    for node in visible_nodes:
        x, y = image_point(grid, node["cell"], scale)
        radius = 2 * scale
        if node in new_nodes:
            draw.ellipse(
                (x - 4 * scale, y - 4 * scale, x + 4 * scale, y + 4 * scale),
                outline=(0, 255, 255),
                width=max(2, scale // 2),
            )
        node_type = node.get("type", "PASS")
        draw_node_shape(
            draw,
            x,
            y,
            radius,
            node_type,
            node.get("frontier", False),
        )
        abbreviation = NODE_ABBREVIATIONS.get(node_type, "?")
        suffix = "?" if node.get("frontier", False) else ""
        draw.text(
            (x + radius + 2, y - radius),
            f"{abbreviation}{node['id']}{suffix}",
            fill=(0, 0, 0),
            font=fonts["small"],
        )

    if snapshot.get("frontier") is not None:
        frontier_x, frontier_y = image_point(
            grid, snapshot["frontier"], scale
        )
        radius = 3 * scale
        draw.ellipse(
            (
                frontier_x - radius,
                frontier_y - radius,
                frontier_x + radius,
                frontier_y + radius,
            ),
            outline=(245, 195, 20),
            width=max(2, scale // 2),
        )

    if window is not None:
        left = window[0] * scale
        top = (grid["height"] - window[3]) * scale
        right = max(left, (window[2] - 1) * scale)
        bottom = max(top, (grid["height"] - 1 - window[1]) * scale)
        draw.rectangle(
            (left, top, right, bottom),
            outline=(0, 190, 255),
            width=max(2, scale // 2),
        )

    target_cell = cell_for_point(grid, waypoints[target_waypoint])
    target_x, target_y = image_point(grid, target_cell, scale)
    target_radius = 3 * scale
    draw.line(
        (
            target_x - target_radius,
            target_y - target_radius,
            target_x + target_radius,
            target_y + target_radius,
        ),
        fill=(245, 110, 0),
        width=2,
    )
    draw.line(
        (
            target_x - target_radius,
            target_y + target_radius,
            target_x + target_radius,
            target_y - target_radius,
        ),
        fill=(245, 110, 0),
        width=2,
    )

    counts = defaultdict(int)
    for node in visible_nodes:
        counts[node.get("type", "PASS")] += 1
    lines = [
        f"step {step:03d}/{last_step:03d}",
        f"known free {snapshot['coverage'] * 100:.1f}%",
        (
            f"window {snapshot['window_size'][0]:.1f}"
            f"x{snapshot['window_size'][1]:.1f} m, "
            f"ray {snapshot['sensor_range']:.1f} m"
        ),
        (
            f"L {counts['TURN']}  T {counts['T_JUNCTION']}  "
            f"X {counts['CROSS']}  D {counts['DEAD_END']}"
        ),
        f"P {counts['PORTAL']}  path {snapshot['trajectory_length']:.2f} m",
    ]
    if snapshot.get("frontier") is not None:
        frontier_x, frontier_y = point_for_cell(grid, snapshot["frontier"])
        lines.append(f"frontier goal {frontier_x:.2f}, {frontier_y:.2f}")
    else:
        lines.append("frontier goal -")
    if new_nodes:
        new_names = ", ".join(
            f"{node.get('type', 'PASS')} {node['id']}" for node in new_nodes[:3]
        )
        lines.append(f"new: {new_names}")
    else:
        lines.append("new: -")

    hud_height = 12 + len(lines) * 15 + 58
    draw.rectangle((8, 8, 310, 8 + hud_height), fill=(250, 250, 250))
    for index, line in enumerate(lines):
        draw.text((14, 12 + index * 15), line, fill=(0, 0, 0), font=fonts["hud"])
    legend_start = 12 + len(lines) * 15 + 6
    legend_types = (
        "TURN",
        "T_JUNCTION",
        "CROSS",
        "DEAD_END",
        "PORTAL",
        "START",
        "TARGET",
    )
    for index, node_type in enumerate(legend_types):
        column = index % 3
        row = index // 3
        x = 14 + column * 88
        y = legend_start + row * 16
        draw.rectangle((x, y, x + 9, y + 9), fill=NODE_COLORS[node_type])
        draw.text(
            (x + 13, y - 3),
            NODE_ABBREVIATIONS[node_type] + " " + node_type.title(),
            fill=(0, 0, 0),
            font=fonts["small"],
        )

    progress = step / max(1, last_step)
    bar_width = grid["width"] * scale - 16
    draw.rectangle((8, picture.height - 12, 8 + bar_width, picture.height - 7),
                   outline=(255, 255, 255))
    draw.rectangle(
        (8, picture.height - 12, 8 + int(bar_width * progress), picture.height - 7),
        fill=(0, 220, 255),
    )
    return picture


def render_animation(
    grid,
    snapshots,
    trajectory,
    nodes,
    edges,
    route,
    waypoints,
    args,
    output,
):
    if not snapshots:
        return {}
    stride = max(1, args.animation_stride)
    selected = snapshots[::stride]
    if selected[-1]["step"] != snapshots[-1]["step"]:
        selected.append(snapshots[-1])
    scale = max(1, args.animation_scale)
    fonts = {
        "hud": load_font(11),
        "small": load_font(9),
    }
    last_step = snapshots[-1]["step"]
    frames = [
        animation_frame(
            grid,
            snapshot,
            trajectory,
            nodes,
            edges,
            route,
            waypoints,
            args.target_waypoint,
            last_step,
            scale,
            fonts,
        )
        for snapshot in selected
    ]
    gif_path = output / "exploration.gif"
    palette_frames = [
        frame.convert("P", palette=Image.ADAPTIVE, colors=128)
        for frame in frames
    ]
    palette_frames[0].save(
        gif_path,
        save_all=True,
        append_images=palette_frames[1:],
        duration=args.animation_frame_ms,
        loop=0,
        optimize=True,
        disposal=2,
    )

    contact_count = max(1, min(args.animation_contact_count, len(frames)))
    contact_indices = sorted(
        set(
            int(index)
            for index in np.linspace(0, len(frames) - 1, contact_count)
        )
    )
    tile_width = max(320, frames[0].width * 2 // 3)
    tile_height = int(tile_width * frames[0].height / frames[0].width)
    columns = 2
    rows = int(math.ceil(len(contact_indices) / columns))
    gap = 4
    sheet = Image.new(
        "RGB",
        (
            columns * tile_width + (columns - 1) * gap,
            rows * tile_height + (rows - 1) * gap,
        ),
        (255, 255, 255),
    )
    for index, frame_index in enumerate(contact_indices):
        tile = frames[frame_index].resize(
            (tile_width, tile_height), Image.LANCZOS
        )
        column = index % columns
        row = index // columns
        sheet.paste(
            tile,
            (
                column * (tile_width + gap),
                row * (tile_height + gap),
            ),
        )
    sequence_path = output / "exploration_sequence.png"
    sheet.save(sequence_path)
    return {
        "gif": str(gif_path),
        "sequence": str(sequence_path),
        "frames": len(frames),
        "frame_duration_ms": args.animation_frame_ms,
    }


def voxel_downsample(points, leaf_size):
    if len(points) == 0 or leaf_size <= 0.0:
        return points
    keys = np.floor(points / leaf_size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def make_point_cloud(points, frame_id, stamp):
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    return point_cloud2.create_cloud_xyz32(
        header,
        np.asarray(points, dtype=np.float32).reshape(-1, 3).tolist(),
    )


def local_occupied_points(xyz, grid, snapshot):
    window = snapshot["window"]
    first_column, first_row, last_column, last_row = window
    min_x = grid["x_min"] + first_column * grid["resolution"]
    max_x = grid["x_min"] + last_column * grid["resolution"]
    min_y = grid["y_min"] + first_row * grid["resolution"]
    max_y = grid["y_min"] + last_row * grid["resolution"]
    inside = xyz[
        (xyz[:, 0] >= min_x)
        & (xyz[:, 0] < max_x)
        & (xyz[:, 1] >= min_y)
        & (xyz[:, 1] < max_y)
    ]
    if len(inside) == 0:
        return inside
    columns = np.floor(
        (inside[:, 0] - grid["x_min"]) / grid["resolution"]
    ).astype(np.int32) - first_column
    rows = np.floor(
        (inside[:, 1] - grid["y_min"]) / grid["resolution"]
    ).astype(np.int32) - first_row
    local_occupied = snapshot["local_known_occupied"]
    valid = (
        (columns >= 0)
        & (columns < local_occupied.shape[1])
        & (rows >= 0)
        & (rows < local_occupied.shape[0])
    )
    selected = np.flatnonzero(valid)
    if len(selected) == 0:
        return inside[:0]
    occupied = local_occupied[rows[selected], columns[selected]]
    return inside[selected[occupied]]


def route_cells(nodes, edges, route):
    if not route.get("found", False):
        return []
    cells = []
    for edge_id, first, second in zip(
        route["edge_ids"], route["node_ids"], route["node_ids"][1:]
    ):
        path = oriented_edge_path(edges[edge_id], first)
        if cells:
            path = path[1:]
        cells.extend(path)
    return cells


def build_rviz_markers(
    grid,
    snapshot,
    trajectory,
    nodes,
    edges,
    route,
    waypoints,
    target_waypoint,
    last_step,
    stamp,
    frame_id,
):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    window = snapshot["window"]
    step = snapshot["step"]
    show_route = snapshot["target_reached"] or step == last_step
    route_edges = set(route.get("edge_ids", []))
    markers = MarkerArray()
    delete = Marker()
    delete.action = Marker.DELETEALL
    markers.markers.append(delete)
    next_id = 0

    def base_marker(marker_type, namespace):
        nonlocal next_id
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = next_id
        next_id += 1
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def marker_color(color, alpha=1.0):
        return (
            color[0] / 255.0,
            color[1] / 255.0,
            color[2] / 255.0,
            alpha,
        )

    def world_point(cell, z):
        x, y = point_for_cell(grid, cell)
        return Point(x=x, y=y, z=z)

    def add_line(namespace, cells, color, width, z, clip=True):
        runs = (
            clip_to_window_runs(cells, window)
            if clip
            else ([cells] if len(cells) >= 2 else [])
        )
        for run in runs:
            if len(run) < 2:
                continue
            marker = base_marker(Marker.LINE_STRIP, namespace)
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                marker_color(color)
            )
            marker.points = [world_point(cell, z) for cell in run]
            markers.markers.append(marker)

    add_line(
        "trajectory",
        trajectory[: snapshot["trajectory_count"]],
        (45, 100, 230),
        0.035,
        0.035,
    )
    add_line(
        "frontier_path",
        snapshot.get("frontier_path", []),
        (245, 195, 20),
        0.025,
        0.045,
    )
    for edge in edges.values():
        if edge.get("first_visible_step", last_step) > step:
            continue
        if show_route and edge["id"] in route_edges:
            color = (0, 220, 220)
            width = 0.055
        elif edge.get("verified", False):
            color = (20, 155, 75)
            width = 0.030
        else:
            color = (165, 165, 165)
            width = 0.018
        add_line("edges", edge["path"], color, width, 0.055)

    if show_route:
        add_line(
            "route",
            route_cells(nodes, edges, route),
            (0, 220, 220),
            0.070,
            0.070,
            clip=False,
        )

    for node in nodes.values():
        if node.get("first_visible_step", last_step) > step:
            continue
        if not window_contains(window, node["cell"]):
            continue
        node_type = node.get("type", "PASS")
        if node_type == "PASS":
            continue
        marker = base_marker(
            Marker.CUBE
            if node_type in ("T_JUNCTION", "CROSS", "DEAD_END")
            else Marker.SPHERE,
            "nodes",
        )
        marker.pose.position = world_point(node["cell"], 0.22)
        marker.scale.x = 0.24
        marker.scale.y = 0.24
        marker.scale.z = 0.24
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            marker_color(
                NODE_COLORS.get(node_type, (100, 100, 100)),
                0.55 if node.get("frontier", False) else 1.0,
            )
        )
        markers.markers.append(marker)

        label = base_marker(Marker.TEXT_VIEW_FACING, "node_labels")
        label.pose.position = world_point(node["cell"], 0.60)
        label.scale.z = 0.28
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        suffix = "?" if node.get("frontier", False) else ""
        label.text = (
            f"{NODE_ABBREVIATIONS.get(node_type, '?')}{node['id']}{suffix}"
        )
        markers.markers.append(label)

    window_marker = base_marker(Marker.LINE_STRIP, "window")
    window_marker.scale.x = 0.045
    window_marker.color.r = 0.0
    window_marker.color.g = 0.75
    window_marker.color.b = 1.0
    window_marker.color.a = 1.0
    first_column, first_row, last_column, last_row = window
    window_cells = [
        (first_column, first_row),
        (last_column - 1, first_row),
        (last_column - 1, last_row - 1),
        (first_column, last_row - 1),
        (first_column, first_row),
    ]
    window_marker.points = [
        world_point(cell, 0.08) for cell in window_cells
    ]
    markers.markers.append(window_marker)

    robot = base_marker(Marker.CYLINDER, "robot")
    robot.pose.position = world_point(snapshot["pose"], 0.28)
    robot.scale.x = 0.32
    robot.scale.y = 0.32
    robot.scale.z = 0.52
    robot.color.r = 0.0
    robot.color.g = 0.85
    robot.color.b = 1.0
    robot.color.a = 1.0
    markers.markers.append(robot)

    if snapshot.get("frontier") is not None:
        frontier = base_marker(Marker.SPHERE, "frontier")
        frontier.pose.position = world_point(snapshot["frontier"], 0.30)
        frontier.scale.x = 0.22
        frontier.scale.y = 0.22
        frontier.scale.z = 0.22
        frontier.color.r = 0.96
        frontier.color.g = 0.76
        frontier.color.b = 0.08
        frontier.color.a = 1.0
        markers.markers.append(frontier)

    target = base_marker(Marker.CYLINDER, "target")
    target_cell = cell_for_point(
        grid, waypoints[target_waypoint]
    )
    target.pose.position = world_point(target_cell, 0.30)
    target.scale.x = 0.28
    target.scale.y = 0.28
    target.scale.z = 0.60
    target.color.r = 0.96
    target.color.g = 0.43
    target.color.b = 0.0
    target.color.a = 1.0
    markers.markers.append(target)

    return markers


def publish_rviz_replay(
    grid,
    xyz,
    snapshots,
    trajectory,
    nodes,
    edges,
    route,
    waypoints,
    args,
):
    if not snapshots:
        print("RViz replay skipped: no snapshots")
        return
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import PointCloud2
        from visualization_msgs.msg import MarkerArray
    except ImportError as error:
        raise RuntimeError(
            "RViz replay requires ROS 2 rclpy; source /opt/ros/foxy/setup.bash first"
        ) from error

    rclpy.init(args=None)
    node = Node("virtual_scan_replay")
    frame_id = args.rviz_frame
    qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    world_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    map_pub = node.create_publisher(
        OccupancyGrid, "/virtual_scan/local_map", qos
    )
    local_points_pub = node.create_publisher(
        PointCloud2,
        "/virtual_scan/local_points",
        qos,
    )
    world_points_pub = node.create_publisher(
        PointCloud2,
        "/virtual_scan/world_cloud",
        world_qos,
    )
    marker_pub = node.create_publisher(
        MarkerArray,
        "/virtual_scan/markers",
        qos,
    )

    world_points = None
    if args.rviz_world_cloud:
        world_points = voxel_downsample(xyz, 0.10)
        print(
            f"RViz world cloud: {len(world_points)} points",
            flush=True,
        )

    rate = max(0.2, args.rviz_rate)
    period = 1.0 / rate
    last_step = snapshots[-1]["step"]
    print(
        "RViz topics: /virtual_scan/local_map /virtual_scan/local_points "
        "/virtual_scan/world_cloud /virtual_scan/markers",
        flush=True,
    )
    try:
        while rclpy.ok():
            for snapshot in snapshots:
                stamp = node.get_clock().now().to_msg()
                if world_points is not None:
                    world_points_pub.publish(
                        make_point_cloud(world_points, frame_id, stamp)
                    )
                first_column, first_row, last_column, last_row = snapshot[
                    "window"
                ]
                local_map = OccupancyGrid()
                local_map.header.frame_id = frame_id
                local_map.header.stamp = stamp
                local_map.info.resolution = grid["resolution"]
                local_map.info.width = last_column - first_column
                local_map.info.height = last_row - first_row
                local_map.info.origin.position.x = (
                    grid["x_min"] + first_column * grid["resolution"]
                )
                local_map.info.origin.position.y = (
                    grid["y_min"] + first_row * grid["resolution"]
                )
                local_map.info.origin.position.z = 0.0
                local_map.info.origin.orientation.w = 1.0
                data = np.full(
                    (local_map.info.height, local_map.info.width),
                    -1,
                    dtype=np.int8,
                )
                data[snapshot["local_known_free"]] = 0
                data[snapshot["local_known_occupied"]] = 100
                local_map.data = data.reshape(-1).tolist()
                map_pub.publish(local_map)

                points = voxel_downsample(
                    local_occupied_points(xyz, grid, snapshot),
                    0.05,
                )
                local_points_pub.publish(
                    make_point_cloud(points, frame_id, stamp)
                )
                marker_pub.publish(
                    build_rviz_markers(
                        grid,
                        snapshot,
                        trajectory,
                        nodes,
                        edges,
                        route,
                        waypoints,
                        args.target_waypoint,
                        last_step,
                        stamp,
                        frame_id,
                    )
                )
                step = snapshot["step"]
                if step == 0 or step == snapshots[-1]["step"]:
                    print(
                        f"RViz frame {step}/{snapshots[-1]['step']}",
                        flush=True,
                    )
                deadline = time.monotonic() + period
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(
                        node,
                        timeout_sec=min(0.05, deadline - time.monotonic()),
                    )
            if not args.rviz_loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    default_map = latest_bundle_map()
    parser.add_argument(
        "--pcd",
        default=default_map,
        help="saved world-aligned map; defaults to the newest maps/bundles/*/map_world.pcd",
    )
    parser.add_argument(
        "--waypoints",
        default="",
        help="SCAN waypoint YAML; defaults to the map bundle or maps/clicked_waypoints*.yaml",
    )
    parser.add_argument("--start-waypoint", type=int, default=0)
    parser.add_argument("--target-waypoint", type=int, default=2)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--robot-radius", type=float, default=0.10)
    parser.add_argument("--obstacle-z-max", type=float, default=0.50)
    parser.add_argument("--floor-z-min", type=float, default=-0.38)
    parser.add_argument("--floor-z-max", type=float, default=-0.06)
    parser.add_argument("--floor-support-radius", type=float, default=0.25)
    parser.add_argument("--obstacle-closing-radius", type=float, default=0.10)
    parser.add_argument("--margin", type=float, default=3.0)
    parser.add_argument("--sensor-range", type=float, default=5.0)
    parser.add_argument("--rays", type=int, default=720)
    parser.add_argument("--local-window-size-x", type=float, default=10.0)
    parser.add_argument("--local-window-size-y", type=float, default=10.0)
    parser.add_argument("--move-cells", type=int, default=8)
    parser.add_argument("--max-steps", type=int, default=400)
    parser.add_argument("--target-weight", type=float, default=0.5)
    parser.add_argument("--min-spur-length", type=float, default=0.50)
    parser.add_argument("--node-merge-distance", type=float, default=0.25)
    parser.add_argument("--min-leaf-length", type=float, default=0.80)
    parser.add_argument("--point-snap-distance", type=float, default=0.35)
    parser.add_argument("--junction-cluster-radius", type=float, default=1.00)
    parser.add_argument("--junction-cluster-path", type=float, default=1.60)
    parser.add_argument(
        "--turn-junction-merge-distance", type=float, default=1.00
    )
    parser.add_argument("--direction-sample-distance", type=float, default=0.45)
    parser.add_argument("--direction-merge-angle", type=float, default=25.0)
    parser.add_argument("--corner-angle", type=float, default=30.0)
    parser.add_argument("--corner-window", type=float, default=0.40)
    parser.add_argument("--turn-spacing", type=float, default=0.80)
    parser.add_argument("--portal-max-width", type=float, default=1.20)
    parser.add_argument("--portal-shoulder", type=float, default=0.15)
    parser.add_argument("--portal-window", type=float, default=1.00)
    parser.add_argument("--portal-spacing", type=float, default=1.00)
    parser.add_argument("--frontier-radius", type=float, default=0.40)
    parser.add_argument("--verify-distance", type=float, default=0.60)
    parser.add_argument("--verify-fraction", type=float, default=0.55)
    parser.add_argument("--no-animation", action="store_true")
    parser.add_argument("--animation-scale", type=int, default=3)
    parser.add_argument("--animation-frame-ms", type=int, default=300)
    parser.add_argument("--animation-stride", type=int, default=1)
    parser.add_argument("--animation-contact-count", type=int, default=6)
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--rviz-rate", type=float, default=4.0)
    parser.add_argument("--rviz-loop", action="store_true")
    parser.add_argument(
        "--rviz-world-cloud",
        action="store_true",
        default=True,
        help="publish the saved PCD as a 3D world cloud",
    )
    parser.add_argument(
        "--no-rviz-world-cloud",
        dest="rviz_world_cloud",
        action="store_false",
    )
    parser.add_argument("--rviz-frame", default="map")
    parser.add_argument("--outdir", default="/tmp/pcd_virtual_exploration")
    args = parser.parse_args()

    if not args.pcd:
        parser.error("no map found; pass --pcd or create maps/bundles/*/map_world.pcd")
    if not args.waypoints:
        args.waypoints = default_waypoints(args.pcd)
    if not args.waypoints:
        parser.error("no waypoint file found; pass --waypoints")

    output = Path(args.outdir)
    output.mkdir(parents=True, exist_ok=True)
    xyz = read_pcd(args.pcd)
    waypoints = read_waypoints(args.waypoints)
    if not 0 <= args.start_waypoint < len(waypoints):
        raise ValueError("start waypoint index is out of range")
    if not 0 <= args.target_waypoint < len(waypoints):
        raise ValueError("target waypoint index is out of range")
    start_point = waypoints[args.start_waypoint, :2]
    target_point = waypoints[args.target_waypoint, :2]

    grid = build_grid(
        xyz,
        waypoints,
        args.resolution,
        args.obstacle_z_max,
        args.robot_radius,
        args.margin,
        args.floor_z_min,
        args.floor_z_max,
        args.floor_support_radius,
        args.obstacle_closing_radius,
    )
    result, snapshots = explore(grid, start_point, target_point, args)
    trajectory = [tuple(cell) for cell in result.pop("trajectory")]
    known_free = result.pop("known_free")
    known_occupied = result.pop("known_occupied")

    _, parents = bfs(grid["free"], tuple(result["start"]))
    shortest_path = reconstruct(
        parents, tuple(result["start"]), tuple(result["target"])
    )
    result["shortest_free_path_length"] = (
        round(path_distance(grid, shortest_path), 3)
        if shortest_path is not None
        else None
    )
    if result["shortest_free_path_length"]:
        result["route_efficiency"] = round(
            result["shortest_free_path_length"]
            / max(result["trajectory_length"], 1e-6),
            3,
        )

    nodes, edges = raw_skeleton_graph(
        known_free, grid, args.min_spur_length
    )
    nodes, edges = merge_close_nodes(
        nodes, edges, grid, args.node_merge_distance
    )
    nodes, edges = prune_short_leaves(
        nodes,
        edges,
        args.min_leaf_length,
        grid,
        (start_point, target_point),
    )
    nodes, edges = suppress_degree_two(nodes, edges, grid)
    nodes, edges = reindex_nodes(nodes, edges)
    nodes, edges = cluster_junctions(nodes, edges, grid, args)
    nodes, edges = suppress_degree_two(nodes, edges, grid)
    nodes, edges = reindex_nodes(nodes, edges)
    nodes, edges = split_edges_at_markers(nodes, edges, grid, args)
    nodes, edges = prune_short_leaves(
        nodes, edges, args.min_leaf_length
    )
    target_id = insert_point_node(
        nodes,
        edges,
        grid,
        result["target_pose"],
        "TARGET",
        args.point_snap_distance,
    )
    start_id = insert_point_node(
        nodes,
        edges,
        grid,
        point_for_cell(grid, tuple(result["start"])),
        "START",
        args.point_snap_distance,
    )
    nodes, edges = prune_short_leaves(
        nodes, edges, args.min_leaf_length
    )
    for _ in range(2):
        classify_decision_nodes(
            nodes, edges, grid, known_free, known_occupied, args
        )
        collapse_pass_nodes(nodes, edges, grid)
    absorb_turns_at_junctions(
        nodes,
        edges,
        grid,
        args.turn_junction_merge_distance,
    )
    for _ in range(2):
        classify_decision_nodes(
            nodes, edges, grid, known_free, known_occupied, args
        )
        collapse_pass_nodes(nodes, edges, grid)
    verify_topology(nodes, edges, grid, trajectory, args)
    compute_visibility_steps(nodes, edges, snapshots)
    route = dijkstra(nodes, edges, start_id, target_id, verified_only=True)
    route["poses"] = {
        str(node_id): [
            round(point_for_cell(grid, nodes[node_id]["cell"])[0], 3),
            round(point_for_cell(grid, nodes[node_id]["cell"])[1], 3),
        ]
        for node_id in route["node_ids"]
    }
    final_cell = cell_for_point(grid, tuple(result["final_pose"]))
    final_window = local_window_bounds(
        grid,
        final_cell,
        args.local_window_size_x,
        args.local_window_size_y,
    )
    result["perception"] = {
        "local_window_size": [
            args.local_window_size_x,
            args.local_window_size_y,
        ],
        "local_window_cells": list(final_window),
        "local_window_world": window_world_bounds(grid, final_window),
        "sensor_range": args.sensor_range,
        "rays": args.rays,
        "mode": "sliding_local_window_centered_on_pose",
        "global_memory": "separate_for_topology",
    }
    render(
        grid,
        {
            "known_free": known_free,
            "known_occupied": known_occupied,
            "trajectory": trajectory,
        },
        nodes,
        edges,
        waypoints,
        route,
        output / "exploration.png",
        final_window,
    )
    if not args.no_animation:
        result["animation"] = render_animation(
            grid,
            snapshots,
            trajectory,
            nodes,
            edges,
            route,
            waypoints,
            args,
            output,
        )
    result["topology"] = topology_record(nodes, edges)
    result["verified_route"] = route
    result["parameters"] = vars(args)
    (output / "result.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    )

    summary = {
        "target_reached": result["target_reached"],
        "stop_reason": result["stop_reason"],
        "trajectory_length": result["trajectory_length"],
        "shortest_free_path_length": result["shortest_free_path_length"],
        "route_efficiency": result.get("route_efficiency"),
        "coverage": result["coverage"],
        "frontier_steps": len(result["frontier_choices"]),
        "node_types": result["topology"]["node_types"],
        "verified_route_found": route["found"],
        "verified_route_length": route["length"],
        "animation": result.get("animation", {}).get("gif"),
        "sequence": result.get("animation", {}).get("sequence"),
        "output": str(output),
    }
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    if args.rviz:
        publish_rviz_replay(
            grid,
            xyz,
            snapshots,
            trajectory,
            nodes,
            edges,
            route,
            waypoints,
            args,
        )


if __name__ == "__main__":
    main()
