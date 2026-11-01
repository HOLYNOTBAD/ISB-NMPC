#!/usr/bin/env python3
"""Convert a YOLOv5 .pt checkpoint to the .wts format used by tensorrtx."""

import argparse
import struct
from pathlib import Path

import torch


def load_state_dict(pt_path: Path):
    checkpoint = torch.load(str(pt_path), map_location="cpu", weights_only=False)

    if isinstance(checkpoint, dict):
        model = checkpoint.get("ema") or checkpoint.get("model")
        if model is None and "state_dict" in checkpoint:
            return checkpoint["state_dict"]
    else:
        model = checkpoint

    if model is None:
        raise RuntimeError(f"could not find model in {pt_path}")

    model.float().eval()

    detect_layer = model.model[-1]
    if hasattr(detect_layer, "anchors") and hasattr(detect_layer, "stride"):
        anchor_grid = detect_layer.anchors * detect_layer.stride[..., None, None]
        if hasattr(detect_layer, "anchor_grid"):
            delattr(detect_layer, "anchor_grid")
        detect_layer.register_buffer("anchor_grid", anchor_grid)
        detect_layer.register_buffer("strides", detect_layer.stride)

    return model.state_dict()


def write_wts(state_dict, output_path: Path):
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w") as f:
        f.write(f"{len(state_dict.keys())}\n")
        for key, value in state_dict.items():
            flat = value.reshape(-1).cpu().tolist()
            f.write(f"{key} {len(flat)}")
            for item in flat:
                f.write(" " + struct.pack(">f", float(item)).hex())
            f.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pt", type=Path)
    parser.add_argument("-o", "--output", type=Path)
    args = parser.parse_args()

    pt_path = args.pt.resolve()
    output_path = (args.output or pt_path.with_suffix(".wts")).resolve()

    state_dict = load_state_dict(pt_path)
    write_wts(state_dict, output_path)
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
