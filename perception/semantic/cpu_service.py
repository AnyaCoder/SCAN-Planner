#!/usr/bin/env python3
"""Low-rate indoor semantic segmentation service for the Go2 camera."""

import argparse
import json
import os
import time

import cv2
import numpy as np
import torch
from transformers import AutoImageProcessor, SegformerForSemanticSegmentation

from service_common import LatestFrame, color_mask, serve_camera, summarize


def run(args):
    latest = LatestFrame()
    torch.set_num_threads(max(1, args.torch_threads))
    processor = AutoImageProcessor.from_pretrained(args.model)
    model = SegformerForSemanticSegmentation.from_pretrained(args.model).eval()
    id_to_label = model.config.id2label

    print(
        "semantic model ready: %s, %dx%d, http://%s:%d/latest.json"
        % (args.model, args.size, args.size, args.bind, args.port),
        flush=True,
    )

    def process_frame(image, sequence):
        rgb = cv2.cvtColor(
            cv2.resize(image, (args.size, args.size)),
            cv2.COLOR_BGR2RGB,
        )
        inputs = processor(images=rgb, return_tensors="pt")
        with torch.inference_mode():
            inference_started = time.monotonic()
            logits = model(**inputs).logits
            inference_ms = (time.monotonic() - inference_started) * 1000.0
        probabilities = torch.softmax(logits, dim=1)[0].numpy()
        prediction = logits.argmax(1)[0].numpy()
        classes, lower_floor, lower_confidence, floor_columns = summarize(
            prediction, probabilities, id_to_label
        )
        overlay_jpeg, mask_png = color_mask(prediction, image)
        payload = {
            "ready": True,
            "backend": "cpu-pytorch",
            "sequence": sequence,
            "unix_time": time.time(),
            "model": args.model,
            "input_size": args.size,
            "inference_ms": round(inference_ms, 1),
            "floor_lower_center_fraction": round(lower_floor, 4),
            "floor_lower_by_column": [
                round(value, 4) for value in floor_columns
            ],
            "lower_center_confidence": round(lower_confidence, 4),
            "classes": classes,
        }
        return payload, overlay_jpeg, mask_png

    serve_camera(args, latest, process_frame, "cpu-pytorch")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--url",
        default="http://%s:8765/frame.jpg"
        % os.environ.get("GO2_WIFI_HOST", "192.168.77.18"),
    )
    parser.add_argument(
        "--model",
        default="nvidia/segformer-b0-finetuned-ade-512-512",
    )
    parser.add_argument("--size", type=int, default=512)
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--torch-threads", type=int, default=3)
    args = parser.parse_args()
    if args.size < 128 or args.size % 32:
        parser.error("--size must be >=128 and divisible by 32")
    run(args)


if __name__ == "__main__":
    main()
