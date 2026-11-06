#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
engine="${1:-${root}/engines/yolov8n_fp16.engine}"
clip_model="${2:-mobileclip}"
image_url="${SCAN_CAMERA_URL:-http://127.0.0.1:8765/frame.jpg}"
data_dir="${root}/data"

mkdir -p "${data_dir}"
cd "${root}/yolo_trt"
./build/yolo_trt \
  --engine "${engine}" \
  --url "${image_url}" \
  --image "${data_dir}/latest_detected.jpg" \
  --json "${data_dir}/latest_detections.json" \
  --frames 1 \
  --interval 0

case "${clip_model}" in
  mobileclip)
    clip_engine="${root}/engines/mobileclip2_s0_fp16.engine"
    label_dir="${root}/models/mobileclip2_s0"
    ;;
  clip)
    clip_engine="${root}/engines/clip_vit_b32_fp16.engine"
    label_dir="${root}/models/clip_vit_b32"
    ;;
  *)
    echo "Unknown CLIP model: ${clip_model}" >&2
    echo "Use mobileclip or clip." >&2
    exit 2
    ;;
esac

cd "${root}/clip_trt"
./build/clip_trt \
  --model "${clip_model}" \
  --engine "${clip_engine}" \
  --image "${image_url}" \
  --labels "${label_dir}/labels.txt" \
  --embeddings "${label_dir}/text_embeddings.bin" \
  --json "${data_dir}/latest_scene.json"
cat "${data_dir}/latest_scene.json"
echo
