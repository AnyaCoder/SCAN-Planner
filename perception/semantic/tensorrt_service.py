#!/usr/bin/env python3
"""Resident SegFormer-B0 TensorRT service for the Go2 camera."""

import argparse
import ctypes
import json
import time
from pathlib import Path

import cv2
import numpy as np
import tensorrt as trt

from service_common import (
    LatestFrame,
    channel_softmax,
    color_mask,
    serve_camera,
    summarize,
)


class CudaRuntime:
    def __init__(self):
        self.lib = ctypes.CDLL("libcudart.so")
        self.lib.cudaMalloc.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_size_t,
        ]
        self.lib.cudaMalloc.restype = ctypes.c_int
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaFree.restype = ctypes.c_int
        self.lib.cudaMemcpyAsync.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        self.lib.cudaMemcpyAsync.restype = ctypes.c_int
        self.lib.cudaStreamCreate.argtypes = [
            ctypes.POINTER(ctypes.c_void_p)
        ]
        self.lib.cudaStreamCreate.restype = ctypes.c_int
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamSynchronize.restype = ctypes.c_int
        self.lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamDestroy.restype = ctypes.c_int

    def check(self, result, operation):
        if result != 0:
            raise RuntimeError("%s failed with CUDA error %d" % (operation, result))

    def malloc(self, size):
        pointer = ctypes.c_void_p()
        self.check(self.lib.cudaMalloc(ctypes.byref(pointer), size), "cudaMalloc")
        return pointer

    def free(self, pointer):
        if pointer:
            self.lib.cudaFree(pointer)

    def stream(self):
        stream = ctypes.c_void_p()
        self.check(self.lib.cudaStreamCreate(ctypes.byref(stream)), "cudaStreamCreate")
        return stream

    def copy(self, destination, source, size, kind, stream):
        self.check(
            self.lib.cudaMemcpyAsync(
                destination,
                source,
                size,
                kind,
                stream,
            ),
            "cudaMemcpyAsync",
        )

    def synchronize(self, stream):
        self.check(
            self.lib.cudaStreamSynchronize(stream),
            "cudaStreamSynchronize",
        )

    def destroy_stream(self, stream):
        if stream:
            self.lib.cudaStreamDestroy(stream)


class SegformerTensorRT:
    def __init__(self, engine_path, metadata_path):
        metadata_file = Path(metadata_path)
        self.metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
        self.id_to_label = {
            int(key): str(value)
            for key, value in self.metadata["id2label"].items()
        }
        self.input_size = int(self.metadata["input_size"])
        self.mean = np.asarray(self.metadata["image_mean"], dtype=np.float32)
        self.std = np.asarray(self.metadata["image_std"], dtype=np.float32)
        self.scale = float(self.metadata["rescale_factor"])

        logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(logger, "")
        runtime = trt.Runtime(logger)
        engine_bytes = Path(engine_path).read_bytes()
        self.engine = runtime.deserialize_cuda_engine(engine_bytes)
        if self.engine is None:
            raise RuntimeError("TensorRT engine deserialization failed")
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("TensorRT execution context creation failed")

        self.input_index = self.engine.get_binding_index("pixel_values")
        self.output_index = self.engine.get_binding_index("logits")
        if self.input_index < 0 or self.output_index < 0:
            raise RuntimeError("engine must expose pixel_values and logits")

        self.input_shape = tuple(
            int(value) for value in self.engine.get_binding_shape(self.input_index)
        )
        self.output_shape = tuple(
            int(value) for value in self.engine.get_binding_shape(self.output_index)
        )
        if min(self.input_shape) <= 0 or min(self.output_shape) <= 0:
            raise RuntimeError("engine bindings must have static shapes")
        if self.input_shape != (1, 3, self.input_size, self.input_size):
            raise RuntimeError("unexpected input shape: %s" % (self.input_shape,))

        self.input_dtype = trt.nptype(
            self.engine.get_binding_dtype(self.input_index)
        )
        self.output_dtype = trt.nptype(
            self.engine.get_binding_dtype(self.output_index)
        )
        self.cuda = CudaRuntime()
        self.stream_handle = self.cuda.stream()
        self.input_bytes = int(np.prod(self.input_shape)) * np.dtype(
            self.input_dtype
        ).itemsize
        self.output_bytes = int(np.prod(self.output_shape)) * np.dtype(
            self.output_dtype
        ).itemsize
        self.device_input = self.cuda.malloc(self.input_bytes)
        self.device_output = self.cuda.malloc(self.output_bytes)
        self.host_output = np.empty(self.output_shape, dtype=self.output_dtype)

    def preprocess(self, image):
        resized = cv2.resize(
            image,
            (self.input_size, self.input_size),
            interpolation=cv2.INTER_LINEAR,
        )
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32)
        tensor *= self.scale
        tensor = (tensor - self.mean) / self.std
        return np.ascontiguousarray(
            tensor.transpose(2, 0, 1)[None, ...],
            dtype=self.input_dtype,
        )

    def infer(self, image):
        input_tensor = self.preprocess(image)
        started = time.monotonic()
        self.cuda.copy(
            self.device_input,
            ctypes.c_void_p(input_tensor.ctypes.data),
            self.input_bytes,
            1,
            self.stream_handle,
        )
        bindings = [0] * self.engine.num_bindings
        bindings[self.input_index] = int(self.device_input.value)
        bindings[self.output_index] = int(self.device_output.value)
        if not self.context.execute_async_v2(
            bindings=bindings,
            stream_handle=int(self.stream_handle.value),
        ):
            raise RuntimeError("TensorRT execute_async_v2 failed")
        self.cuda.copy(
            ctypes.c_void_p(self.host_output.ctypes.data),
            self.device_output,
            self.output_bytes,
            2,
            self.stream_handle,
        )
        self.cuda.synchronize(self.stream_handle)
        inference_ms = (time.monotonic() - started) * 1000.0
        return self.host_output[0], inference_ms

    def close(self):
        self.cuda.destroy_stream(self.stream_handle)
        self.stream_handle = None
        self.cuda.free(self.device_input)
        self.device_input = None
        self.cuda.free(self.device_output)
        self.device_output = None


def decode_logits(logits):
    return logits.argmax(axis=0)


def process_image(model, image, source_sequence=None):
    logits, inference_ms = model.infer(image)
    prediction = decode_logits(logits)
    probabilities = channel_softmax(logits)
    started = time.monotonic()
    classes, lower_floor, lower_confidence, floor_columns = summarize(
        prediction, probabilities, model.id_to_label
    )
    overlay_jpeg, mask_png = color_mask(prediction, image)
    postprocess_ms = (time.monotonic() - started) * 1000.0
    payload = {
        "ready": True,
        "backend": "tensorrt-fp16",
        "sequence": source_sequence,
        "unix_time": time.time(),
        "model": model.metadata["model"],
        "input_size": model.input_size,
        "inference_ms": round(inference_ms, 2),
        "postprocess_ms": round(postprocess_ms, 2),
        "floor_lower_center_fraction": round(lower_floor, 4),
        "floor_lower_by_column": [round(value, 4) for value in floor_columns],
        "lower_center_confidence": round(lower_confidence, 4),
        "classes": classes,
    }
    return payload, overlay_jpeg, mask_png


def run(args):
    if args.image:
        model = SegformerTensorRT(args.engine, args.metadata)
        try:
            image = cv2.imread(args.image, cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError("cannot decode image: %s" % args.image)
            payload, overlay_jpeg, mask_png = process_image(model, image)
            output_dir = Path(args.output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / "latest.json").write_text(
                json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
            (output_dir / "overlay.jpg").write_bytes(overlay_jpeg)
            (output_dir / "mask.png").write_bytes(mask_png)
            print(json.dumps(payload, separators=(",", ":")))
            return
        finally:
            model.close()

    latest = LatestFrame()
    model = SegformerTensorRT(args.engine, args.metadata)
    for _ in range(max(1, args.warmup)):
        model.infer(np.zeros((720, 1280, 3), dtype=np.uint8))

    print(
        "semantic TensorRT ready: %s, %dx%d, http://%s:%d/latest.json"
        % (args.engine, model.input_size, model.input_size, args.bind, args.port),
        flush=True,
    )
    try:
        serve_camera(
            args,
            latest,
            lambda image, sequence: process_image(model, image, sequence),
            "tensorrt-fp16",
        )
    finally:
        model.close()


def main():
    cv2.setNumThreads(1)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--url", default="http://127.0.0.1:8765/frame.jpg"
    )
    parser.add_argument(
        "--engine",
        default=(
            "/home/unitree/SCAN-Planner/perception/engines/"
            "segformer_b0_ade_512_fp16.engine"
        ),
    )
    parser.add_argument(
        "--metadata",
        default=(
            "/home/unitree/SCAN-Planner/perception/engines/"
            "segformer_b0_ade_512.json"
        ),
    )
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--image")
    parser.add_argument("--output-dir", default="/tmp/semantic-trt")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
