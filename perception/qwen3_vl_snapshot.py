#!/usr/bin/env python3

import argparse
import io
import json
import os
import time
import urllib.request
import warnings
from pathlib import Path

os.environ.setdefault("TRANSFORMERS_VERBOSITY", "error")
os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
warnings.filterwarnings("ignore", message="Failed to load image Python extension.*")

import torch
from PIL import Image
from transformers import AutoProcessor, Qwen3VLForConditionalGeneration


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = os.environ.get(
    "SCAN_QWEN3_VL_MODEL", str(REPO_ROOT / "perception" / "models" / "qwen3_vl_2b")
)
DEFAULT_IMAGE = os.environ.get("SCAN_CAMERA_URL", "http://127.0.0.1:8765/frame.jpg")
DEFAULT_PROMPT = (
    "用一句话描述这个室内场景，并指出正前方是否可通行、"
    "最明显的障碍物是什么。"
)


def load_image(source):
    if source.startswith(("http://", "https://")):
        with urllib.request.urlopen(source, timeout=10) as response:
            return Image.open(io.BytesIO(response.read())).convert("RGB")
    return Image.open(source).convert("RGB")


def move_inputs(inputs, device):
    return {
        key: value.to(device) if torch.is_tensor(value) else value
        for key, value in inputs.items()
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run one Qwen3-VL semantic query against a Go2 camera frame."
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--image", default=DEFAULT_IMAGE)
    parser.add_argument("--prompt", default=DEFAULT_PROMPT)
    parser.add_argument("--max-new-tokens", type=int, default=32)
    parser.add_argument("--min-pixels", type=int, default=None)
    parser.add_argument("--max-pixels", type=int, default=None)
    parser.add_argument("--dtype", choices=("float16", "bfloat16"), default="float16")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.runs < 1:
        raise SystemExit("--runs must be at least 1")
    if not torch.cuda.is_available():
        raise SystemExit("CUDA is not available")

    dtype = torch.float16 if args.dtype == "float16" else torch.bfloat16
    started = time.perf_counter()
    processor = AutoProcessor.from_pretrained(args.model, use_fast=True)
    model = Qwen3VLForConditionalGeneration.from_pretrained(
        args.model,
        dtype=dtype,
        attn_implementation="sdpa",
    ).to("cuda")
    model.eval()
    torch.cuda.synchronize()
    load_seconds = time.perf_counter() - started

    if args.min_pixels is not None:
        processor.image_processor.min_pixels = args.min_pixels
    if args.max_pixels is not None:
        processor.image_processor.max_pixels = args.max_pixels

    image = load_image(args.image)
    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image", "image": image},
                {"type": "text", "text": args.prompt},
            ],
        }
    ]

    torch.cuda.synchronize()
    preprocess_started = time.perf_counter()
    inputs = processor.apply_chat_template(
        messages,
        tokenize=True,
        add_generation_prompt=True,
        return_dict=True,
        return_tensors="pt",
    )
    inputs = move_inputs(inputs, "cuda")
    torch.cuda.synchronize()
    preprocess_seconds = time.perf_counter() - preprocess_started

    model.generation_config.temperature = None
    model.generation_config.top_p = None
    model.generation_config.top_k = None

    results = []
    for index in range(args.runs):
        torch.cuda.reset_peak_memory_stats()
        torch.cuda.synchronize()
        run_started = time.perf_counter()
        with torch.inference_mode():
            output = model.generate(
                **inputs,
                max_new_tokens=args.max_new_tokens,
                do_sample=False,
            )
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - run_started
        new_tokens = output[0, inputs["input_ids"].shape[1] :]
        text = processor.tokenizer.decode(
            new_tokens, skip_special_tokens=True
        ).strip()
        token_count = int(new_tokens.shape[-1])
        results.append(
            {
                "run": index + 1,
                "seconds": round(elapsed, 3),
                "tokens": token_count,
                "tokens_per_second": round(token_count / elapsed, 2),
                "peak_cuda_gb": round(
                    torch.cuda.max_memory_allocated() / 1e9, 3
                ),
                "text": text,
            }
        )

    report = {
        "model": args.model,
        "image": args.image,
        "dtype": args.dtype,
        "input_tokens": int(inputs["input_ids"].shape[-1]),
        "load_seconds": round(load_seconds, 3),
        "preprocess_seconds": round(preprocess_seconds, 3),
        "results": results,
    }

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return

    print(
        f"load {load_seconds:.2f}s | preprocess {preprocess_seconds:.3f}s | "
        f"input {report['input_tokens']} tokens"
    )
    for result in results:
        print(
            f"run {result['run']}: {result['seconds']:.2f}s, "
            f"{result['tokens']} tokens, "
            f"{result['tokens_per_second']:.2f} tok/s, "
            f"peak {result['peak_cuda_gb']:.3f} GB"
        )
        print(result["text"])


if __name__ == "__main__":
    main()
