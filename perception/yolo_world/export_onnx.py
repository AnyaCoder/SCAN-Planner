#!/usr/bin/env python3
"""Export a fixed-vocabulary YOLO-World model to ONNX."""

import argparse
import json
import shutil
from pathlib import Path

from ultralytics import YOLO, YOLOWorld


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--weights",
        default=str(root / "perception/models/yolo_world/yolov8s-worldv2.pt"),
    )
    parser.add_argument(
        "--classes",
        default=str(Path(__file__).with_name("classes.txt")),
    )
    parser.add_argument(
        "--output",
        default=str(root / "perception/models/yolo_world"),
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--opset", type=int, default=12)
    return parser.parse_args()


def load_classes(path):
    classes = [
        line.strip()
        for line in Path(path).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not classes:
        raise ValueError("class vocabulary is empty")
    return classes


def main():
    args = parse_args()
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    classes = load_classes(args.classes)

    model = YOLOWorld(args.weights)
    model.set_classes(classes)
    specialized_path = output_dir / "yolov8s_worldv2_indoor.pt"
    model.save(str(specialized_path))

    export_model = YOLO(str(specialized_path))
    exported = Path(
        export_model.export(
            format="onnx",
            imgsz=args.imgsz,
            opset=args.opset,
            simplify=True,
            dynamic=False,
            batch=1,
            device="cpu",
            nms=False,
        )
    )
    onnx_path = output_dir / "yolov8s_worldv2_indoor.onnx"
    if exported.resolve() != onnx_path.resolve():
        shutil.copy2(exported, onnx_path)

    metadata = {
        "weights": str(Path(args.weights).resolve()),
        "specialized_weights": str(specialized_path.resolve()),
        "onnx": str(onnx_path.resolve()),
        "imgsz": args.imgsz,
        "opset": args.opset,
        "classes": classes,
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print("ONNX: {}".format(onnx_path))
    print("classes: {}".format(len(classes)))


if __name__ == "__main__":
    main()
