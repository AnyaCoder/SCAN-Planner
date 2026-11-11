#!/usr/bin/env python3
"""Compare a CPU SegFormer prediction with a TensorRT mask."""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from transformers import AutoImageProcessor, SegformerForSemanticSegmentation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--trt-mask", required=True)
    parser.add_argument("--trt-json", required=True)
    parser.add_argument(
        "--model",
        default="nvidia/segformer-b0-finetuned-ade-512-512",
    )
    args = parser.parse_args()

    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot decode image")
    trt_mask = cv2.imread(args.trt_mask, cv2.IMREAD_GRAYSCALE)
    if trt_mask is None:
        raise RuntimeError("cannot decode TensorRT mask")
    if trt_mask.shape != image.shape[:2]:
        raise RuntimeError(
            "mask shape %s does not match image %s"
            % (trt_mask.shape, image.shape[:2])
        )
    trt_payload = json.loads(
        Path(args.trt_json).read_text(encoding="utf-8")
    )

    processor = AutoImageProcessor.from_pretrained(
        args.model, local_files_only=True
    )
    model = SegformerForSemanticSegmentation.from_pretrained(
        args.model, local_files_only=True
    ).eval()
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    inputs = processor(images=rgb, return_tensors="pt")
    started = time.monotonic()
    with torch.inference_mode():
        logits = model(**inputs).logits
    cpu_inference_ms = (time.monotonic() - started) * 1000.0
    probabilities = torch.softmax(logits, dim=1)[0].numpy()
    prediction = logits.argmax(1)[0].numpy()
    cpu_mask = cv2.resize(
        prediction.astype(np.uint8),
        (image.shape[1], image.shape[0]),
        interpolation=cv2.INTER_NEAREST,
    )

    equal = cpu_mask == trt_mask
    pixel_accuracy = float(np.mean(equal))
    classes = np.union1d(np.unique(cpu_mask), np.unique(trt_mask))
    ious = {}
    for class_id in classes:
        class_id = int(class_id)
        expected = cpu_mask == class_id
        actual = trt_mask == class_id
        union = np.count_nonzero(expected | actual)
        if union:
            ious[class_id] = float(
                np.count_nonzero(expected & actual) / union
            )

    height, width = prediction.shape
    lower = probabilities[
        int(height * 0.62) :,
        int(width * 0.25) : int(width * 0.75),
    ]
    cpu_lower_confidence = float(np.mean(lower.max(axis=0)))
    output = {
        "cpu_inference_ms": round(cpu_inference_ms, 2),
        "trt_inference_ms": trt_payload.get("inference_ms"),
        "pixel_accuracy": round(pixel_accuracy, 5),
        "disagreement_fraction": round(1.0 - pixel_accuracy, 5),
        "mean_iou_present_classes": round(float(np.mean(list(ious.values()))), 5),
        "wall_iou": round(ious.get(0, 0.0), 5),
        "floor_iou": round(ious.get(3, 0.0), 5),
        "cpu_floor_fraction": round(float(np.mean(cpu_mask == 3)), 5),
        "trt_floor_fraction": round(float(np.mean(trt_mask == 3)), 5),
        "cpu_lower_confidence": round(cpu_lower_confidence, 5),
        "trt_lower_confidence": trt_payload.get(
            "lower_center_confidence"
        ),
        "classes_in_comparison": len(ious),
    }
    print(json.dumps(output, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
