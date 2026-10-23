#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/fastlio_ws}"
SRC="$WS/src/FAST_LIO"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/patches"
PATCHES=(
  "$PATCH_DIR/fast_lio_foxy_service_callback.patch"
  "$PATCH_DIR/fast_lio_tf_remap.patch"
  "$PATCH_DIR/fast_lio_gravity_alignment.patch"
)
UPSTREAM="https://github.com/hku-mars/FAST_LIO.git"
COMMIT="a4743b095409588842a5b30ddfa27e29d2f99164"
IKD_COMMIT="e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4"

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo "ROS 2 Foxy is required: /opt/ros/foxy/setup.bash not found" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
if [[ -f /unitree/module/graph_pid_ws/install/local_setup.bash ]]; then
  # shellcheck disable=SC1091
  source /unitree/module/graph_pid_ws/install/local_setup.bash
fi

mkdir -p "$WS/src"
if [[ ! -d "$SRC" ]]; then
  git clone --recursive "$UPSTREAM" "$SRC"
  git -C "$SRC" checkout "$COMMIT"
  git -C "$SRC" submodule update --init --recursive
elif [[ ! -d "$SRC/.git" ]]; then
  echo "$SRC exists but is not a Git checkout; remove it or choose another workspace" >&2
  exit 1
else
  git -C "$SRC" fetch origin "$COMMIT"
  git -C "$SRC" checkout "$COMMIT"
  git -C "$SRC" submodule update --init --recursive
fi

if [[ -d "$SRC/ikd-Tree/.git" ]]; then
  git -C "$SRC/ikd-Tree" checkout "$IKD_COMMIT"
fi

for patch in "${PATCHES[@]}"; do
  if git -C "$SRC" apply --check "$patch"; then
    git -C "$SRC" apply "$patch"
  elif git -C "$SRC" apply --reverse --check "$patch"; then
    echo "FAST-LIO patch already applied: $(basename "$patch")"
  else
    echo "FAST-LIO source does not match pinned revision or patch context: $patch" >&2
    exit 1
  fi
done

cd "$WS"
colcon build --symlink-install --packages-select fast_lio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "FAST-LIO2 built in $WS"
echo "source $WS/install/local_setup.bash"
