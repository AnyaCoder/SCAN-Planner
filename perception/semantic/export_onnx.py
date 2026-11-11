#!/usr/bin/env python3
"""Export the cached SegFormer-B0 model to a fixed-size ONNX graph."""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from transformers import AutoImageProcessor, SegformerForSemanticSegmentation


class LogitsOnly(torch.nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, pixel_values):
        return self.model(pixel_values=pixel_values).logits


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default="nvidia/segformer-b0-finetuned-ade-512-512",
    )
    parser.add_argument(
        "--output",
        default="perception/models/segformer_b0_ade_512.onnx",
    )
    parser.add_argument(
        "--metadata",
        default="perception/models/segformer_b0_ade_512.json",
    )
    parser.add_argument("--size", type=int, default=512)
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()

    output = Path(args.output).resolve()
    metadata_path = Path(args.metadata).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.parent.mkdir(parents=True, exist_ok=True)

    processor = AutoImageProcessor.from_pretrained(
        args.model, local_files_only=True
    )
    model = SegformerForSemanticSegmentation.from_pretrained(
        args.model, local_files_only=True
    ).eval()
    wrapper = LogitsOnly(model).eval()

    dummy_rgb = np.zeros((args.size, args.size, 3), dtype=np.uint8)
    inputs = processor(images=dummy_rgb, return_tensors="pt")
    pixel_values = inputs["pixel_values"].contiguous()

    with torch.inference_mode():
        reference = wrapper(pixel_values)

    torch.onnx.export(
        wrapper,
        (pixel_values,),
        str(output),
        input_names=["pixel_values"],
        output_names=["logits"],
        opset_version=args.opset,
        do_constant_folding=True,
        export_params=True,
        dynamic_axes=None,
    )

    metadata = {
        "model": args.model,
        "input_size": args.size,
        "input_name": "pixel_values",
        "output_name": "logits",
        "num_labels": int(model.config.num_labels),
        "id2label": {
            str(key): str(value)
            for key, value in model.config.id2label.items()
        },
        "image_mean": list(processor.image_mean),
        "image_std": list(processor.image_std),
        "rescale_factor": float(processor.rescale_factor),
        "reference_shape": list(reference.shape),
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print("ONNX:", output)
    print("metadata:", metadata_path)
    print("reference logits:", tuple(reference.shape))


if __name__ == "__main__":
    main()
