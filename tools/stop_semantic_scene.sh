#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=tools/stack_lib.sh
source "$SCRIPT_DIR/stack_lib.sh"

stop_local_pgids "Semantic segmentation" \
  '(semantic_scene|cpu_service|tensorrt_service)\.py'
ssh -o ConnectTimeout=5 "$GO2_HOST" \
  "pkill -TERM -f '[t]ensorrt_service\.py' 2>/dev/null || true" \
  2>/dev/null || true
