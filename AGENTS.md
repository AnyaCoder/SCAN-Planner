# Repository Instructions

## Start Here

Before changing this repository, read:

1. `CURRENT_CONSTRUCTION.md`
2. `git status --short`
3. The relevant source and tests for the files being changed

`CURRENT_CONSTRUCTION.md` is the current construction plan and progress record.
Keep it updated when a milestone, interface, or acceptance criterion changes.

## Project Boundaries

- SCAN-Planner owns collision checking, trajectory planning, and physical
  execution. Do not replace it with a VLM planner.
- The VLM chooses one normalized floor point. Normal-forward requests use the
  front camera and the existing camera-ray/LiDAR back-projection.
- If the camera is too close to a wall/glass surface and no floor is visible,
  VLM returns `topdown`. A second request receives the camera and top-down map
  and chooses one neutral-gray floor point from the local map. The program
  converts that pixel into a world target and validates gray floor within
  0.20 m. Top-down fallback never enumerates routes or candidates.
- The VLM does not decide whether the robot can physically pass.
- Preserve the existing 10 m x 10 m, 5 cm local dual-LiDAR map.
- Images belong only in the current VLM request. Historical VLM context must
  contain compact text and approved world points.

## Working Rules

- Preserve all existing uncommitted changes unless they are directly part of
  the requested change.
- Use `apply_patch` for source and documentation edits.
- Test ROS-independent logic without requiring the robot or a ROS graph.
- Real-robot tests must start from the repository scripts and use the existing
  USB Wi-Fi or hotspot networking path.
- Do not silently reset, discard, or overwrite navigation history, maps, or
  source changes.
