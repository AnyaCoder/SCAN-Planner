# SegFormer semantic scene

This directory keeps the semantic inference tools separate from the YOLO
world-model files.

- `tensorrt_service.py`: resident TensorRT service used by the Go2 stack.
- `cpu_service.py`: slower PyTorch/Transformers service used as a reference.
- `service_common.py`: shared HTTP API, camera polling, overlays, and statistics.
- `export_onnx.py`: export the cached SegFormer-B0 checkpoint to ONNX.
- `compare_backends.py`: compare CPU and TensorRT segmentation output.

`tools/start_semantic_scene.sh` synchronizes `tensorrt_service.py` and
`service_common.py` to the Go2 and starts the service on port `8768`. The model
engine and metadata remain local, ignored runtime assets; they are not source
files.

Typical export command:

```bash
perception/.venv/bin/python perception/semantic/export_onnx.py
```
