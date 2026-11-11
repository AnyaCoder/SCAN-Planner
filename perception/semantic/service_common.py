#!/usr/bin/env python3
"""Shared HTTP, camera, and post-processing helpers for SegFormer services."""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import urlopen

import cv2
import numpy as np


FLOOR_CLASS_ID = 3
MAX_JPEG_BYTES = 4 * 1024 * 1024
PALETTE = np.asarray(
    [
        (125, 125, 125),  # wall
        (20, 20, 20),
        (170, 100, 20),
        (40, 180, 70),  # floor
        (30, 120, 220),
        (210, 210, 210),  # ceiling
        (220, 40, 40),
        (160, 80, 200),
        (30, 200, 220),
        (200, 180, 30),
        (90, 180, 100),
        (180, 100, 100),
        (80, 60, 160),
        (100, 200, 50),
        (240, 130, 20),  # door
    ],
    dtype=np.uint8,
)


class LatestFrame:
    def __init__(self):
        self.lock = threading.Lock()
        self.json_bytes = json.dumps(
            {"ready": False, "error": "waiting for first frame"}
        ).encode("utf-8")
        self.overlay_jpeg = b""
        self.mask_png = b""

    def update(self, payload, overlay_jpeg, mask_png):
        with self.lock:
            self.json_bytes = json.dumps(
                payload, separators=(",", ":"), ensure_ascii=False
            ).encode("utf-8")
            self.overlay_jpeg = overlay_jpeg
            self.mask_png = mask_png

    def snapshot(self):
        with self.lock:
            return self.json_bytes, self.overlay_jpeg, self.mask_png


class SemanticServer:
    def __init__(self, host, port, latest):
        self.host = host
        self.port = port
        self.latest = latest
        self.httpd = None
        self.thread = None

    def start(self):
        latest = self.latest

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                json_bytes, overlay_jpeg, mask_png = latest.snapshot()
                if self.path in ("/", "/latest", "/latest.json"):
                    self._send(json_bytes, "application/json")
                elif self.path in ("/overlay", "/overlay.jpg"):
                    self._send(overlay_jpeg, "image/jpeg")
                elif self.path in ("/mask", "/mask.png"):
                    self._send(mask_png, "image/png")
                elif self.path == "/health":
                    self._send(b'{"ok":true}', "application/json")
                else:
                    self.send_error(404)

            def _send(self, body, content_type):
                if not body:
                    self.send_error(503, "no semantic frame available")
                    return
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format_string, *args):
                return

        self.httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self.thread = threading.Thread(
            target=self.httpd.serve_forever,
            name="semantic-http",
            daemon=True,
        )
        self.thread.start()

    def stop(self):
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd.server_close()


def read_camera_frame(url):
    with urlopen(url, timeout=2.0) as response:
        sequence = response.headers.get("X-Frame-Sequence")
        encoded_frame = response.read(MAX_JPEG_BYTES)
    image = cv2.imdecode(
        np.frombuffer(encoded_frame, dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    if image is None:
        raise RuntimeError("camera JPEG decode failed")
    return sequence, image


def color_mask(prediction, image):
    height, width = image.shape[:2]
    resized_prediction = cv2.resize(
        prediction.astype(np.uint8),
        (width, height),
        interpolation=cv2.INTER_NEAREST,
    )
    mask = PALETTE[resized_prediction.astype(np.int64) % len(PALETTE)]
    overlay = cv2.addWeighted(image, 0.55, mask, 0.45, 0.0)
    overlay_jpeg = cv2.imencode(
        ".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 82]
    )[1].tobytes()
    mask_png = cv2.imencode(".png", resized_prediction)[1].tobytes()
    return overlay_jpeg, mask_png


def channel_softmax(logits):
    shifted = np.asarray(logits, dtype=np.float32)
    shifted -= shifted.max(axis=0, keepdims=True)
    probabilities = np.exp(shifted)
    probabilities /= probabilities.sum(axis=0, keepdims=True)
    return probabilities


def summarize(prediction, probabilities, id_to_label):
    flat = prediction.reshape(-1)
    counts = np.bincount(flat, minlength=max(id_to_label) + 1)
    total = max(1, flat.size)
    top_indices = np.argsort(counts)[-10:][::-1]
    classes = [
        {
            "id": int(index),
            "name": id_to_label.get(int(index), str(index)),
            "fraction": round(float(counts[index]) / total, 4),
        }
        for index in top_indices
        if counts[index] > 0
    ]
    height, width = prediction.shape
    lower = prediction[
        int(height * 0.62) :,
        int(width * 0.25) : int(width * 0.75),
    ]
    lower_floor = float(np.mean(lower == FLOOR_CLASS_ID))
    lower_rows = prediction[int(height * 0.45) :, :]
    floor_columns = [
        float(np.mean(part == FLOOR_CLASS_ID))
        for part in np.array_split(lower_rows, 12, axis=1)
    ]
    lower_confidence = float(
        np.mean(
            probabilities[
                :,
                int(height * 0.62) :,
                int(width * 0.25) : int(width * 0.75),
            ].max(axis=0)
        )
    )
    return classes, lower_floor, lower_confidence, floor_columns


def serve_camera(args, latest, process_frame, backend):
    server = SemanticServer(args.bind, args.port, latest)
    server.start()
    last_sequence = None
    try:
        while True:
            loop_started = time.monotonic()
            try:
                sequence, image = read_camera_frame(args.url)
                if sequence is not None and sequence == last_sequence:
                    time.sleep(0.01)
                    continue
                payload, overlay_jpeg, mask_png = process_frame(
                    image, sequence
                )
                payload["loop_ms"] = round(
                    (time.monotonic() - loop_started) * 1000.0, 2
                )
                latest.update(payload, overlay_jpeg, mask_png)
                last_sequence = sequence
                print(
                    json.dumps(payload, separators=(",", ":")),
                    flush=True,
                )
            except Exception as exc:
                error = {
                    "ready": False,
                    "backend": backend,
                    "error": str(exc),
                    "unix_time": time.time(),
                }
                latest.update(error, b"", b"")
                print(json.dumps(error, separators=(",", ":")), flush=True)
                time.sleep(1.0)
                continue

            remaining = 1.0 / max(args.fps, 0.1) - (
                time.monotonic() - loop_started
            )
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        server.stop()
