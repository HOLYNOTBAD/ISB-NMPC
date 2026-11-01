#!/usr/bin/env python3
"""Plot per-trial and aggregate target-position heatmaps in the UAV FPV.

The input may be a session directory, a controller directory, or one trial
directory.  Pixel positions are reconstructed from ``states.csv`` and the
camera calibration stored in each trial's ``metadata.yaml``.
"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import numpy as np
import yaml


REQUIRED_COLUMNS = {
    "uav_x", "uav_y", "uav_z", "uav_qw", "uav_qx", "uav_qy", "uav_qz",
    "target_x", "target_y", "target_z",
}


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", type=Path,
        help="session, controller, or trial directory containing metadata.yaml",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        help="output directory (default: INPUT/fpv_heatmaps, or beside a trial)",
    )
    # Retained as hidden no-ops so older command lines continue to work.
    parser.add_argument("--bins", type=int, default=48, help=argparse.SUPPRESS)
    parser.add_argument(
        "--include-offscreen", action="store_true",
        help="retain front-facing samples outside the image (axes remain image-sized)",
    )
    parser.add_argument(
        "--scatter", action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def quat_rotate(q, vectors):
    """Rotate Nx3 vectors with unit quaternion q=[w,x,y,z]."""
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    xyz = q[1:]
    vectors = np.asarray(vectors, dtype=float)
    cross = np.cross(np.broadcast_to(xyz, vectors.shape), vectors)
    return vectors + 2.0 * (q[0] * cross + np.cross(xyz, cross))


def conjugate(q):
    return np.asarray((q[0], -q[1], -q[2], -q[3]), dtype=float)


def load_trial(trial_dir, include_offscreen=False):
    metadata_path = trial_dir / "metadata.yaml"
    states_path = trial_dir / "states.csv"
    with metadata_path.open() as handle:
        metadata = yaml.safe_load(handle) or {}
    camera = metadata.get("camera", {})
    required_camera = {
        "img_width", "img_height", "img_f", "p_B_C", "q_B_C"
    }
    missing_camera = sorted(required_camera - set(camera))
    if missing_camera:
        raise ValueError("missing camera fields: " + ", ".join(missing_camera))

    with states_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        missing_columns = sorted(REQUIRED_COLUMNS - set(reader.fieldnames or ()))
        if missing_columns:
            raise ValueError("missing state columns: " + ", ".join(missing_columns))
        rows = list(reader)
    if not rows:
        raise ValueError("states.csv has no samples")

    values = np.asarray([
        [float(row[name]) for name in (
            "uav_x", "uav_y", "uav_z", "uav_qw", "uav_qx", "uav_qy",
            "uav_qz", "target_x", "target_y", "target_z"
        )]
        for row in rows
    ])
    finite = np.all(np.isfinite(values), axis=1)
    values = values[finite]
    if not len(values):
        raise ValueError("states.csv has no finite samples")

    width, height = float(camera["img_width"]), float(camera["img_height"])
    focal = float(camera["img_f"])
    cx, cy = float(camera.get("cx", width / 2.0)), float(camera.get("cy", height / 2.0))
    p_b_c = np.asarray(camera["p_B_C"], dtype=float)
    q_b_c = np.asarray(camera["q_B_C"], dtype=float)
    if focal <= 0 or np.linalg.norm(q_b_c) < 1e-12:
        raise ValueError("invalid camera calibration")

    pixels = []
    front_count = 0
    for row in values:
        uav_p, q_w_b, target_p = row[0:3], row[3:7], row[7:10]
        if np.linalg.norm(q_w_b) < 1e-12:
            continue
        camera_center = uav_p + quat_rotate(q_w_b, p_b_c[None, :])[0]
        p_body = quat_rotate(conjugate(q_w_b), (target_p - camera_center)[None, :])
        p_camera = quat_rotate(conjugate(q_b_c), p_body)[0]
        if p_camera[2] <= 1e-9:
            continue
        front_count += 1
        u = cx + focal * p_camera[0] / p_camera[2]
        v = cy + focal * p_camera[1] / p_camera[2]
        if include_offscreen or (0.0 <= u <= width and 0.0 <= v <= height):
            pixels.append((u, v))
    if not pixels:
        raise ValueError("no projected samples inside the FPV image")

    result = metadata.get("result", {})
    info = {
        "trial": trial_dir.name,
        "controller": str(metadata.get("controller", trial_dir.parent.name)),
        "success": bool(result.get("success", False)),
        "reason": str(result.get("reason", "")),
        "total_samples": len(values),
        "front_samples": front_count,
        "plotted_samples": len(pixels),
    }
    return np.asarray(pixels), (width, height, cx, cy), info


def latex_number(value, _position):
    return r"${:g}$".format(value)


def plot_heatmap(path, points, camera, dpi):
    plt.rcParams["mathtext.fontset"] = "cm"
    width, height, cx, cy = camera
    fig_width = 8.0
    fig, axis = plt.subplots(figsize=(fig_width, fig_width * height / width + 1.0))
    # Alpha compositing is the density encoding: every dot is a real sample,
    # and repeated samples naturally build up a darker red without smoothing
    # or inventing density between measurements.
    axis.scatter(points[:, 0], points[:, 1], s=7, color="#d62728", alpha=0.10,
                 linewidths=0)
    axis.axvline(cx, color="black", linestyle="--", linewidth=1.2)
    axis.axhline(cy, color="black", linestyle="--", linewidth=1.2)
    axis.set_xlim(0, width)
    axis.set_ylim(height, 0)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel(r"$u$ [px]")
    axis.set_ylabel(r"$v$ [px]")
    formatter = FuncFormatter(latex_number)
    axis.xaxis.set_major_formatter(formatter)
    axis.yaxis.set_major_formatter(formatter)
    axis.set_axisbelow(True)
    axis.grid(True, color="black", alpha=0.16, linewidth=0.6, linestyle="-")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def find_trials(input_path):
    input_path = input_path.expanduser().resolve()
    if (input_path / "metadata.yaml").is_file():
        candidates = [input_path]
    else:
        candidates = sorted(path.parent for path in input_path.rglob("metadata.yaml"))
    return [path for path in candidates if (path / "states.csv").is_file()]


def main():
    args = arguments()
    trials = find_trials(args.input)
    if not trials:
        raise SystemExit("no trial containing metadata.yaml and states.csv found")
    input_path = args.input.expanduser().resolve()
    default_root = (input_path.parent if (input_path / "metadata.yaml").is_file()
                    else input_path) / "fpv_heatmaps"
    output_root = args.output_dir.expanduser().resolve() if args.output_dir else default_root

    loaded, skipped = [], []
    for trial in trials:
        try:
            points, camera, info = load_trial(trial, args.include_offscreen)
        except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
            skipped.append((trial, str(error)))
            continue
        loaded.append((trial, points, camera, info))

    if not loaded:
        details = "; ".join("{}: {}".format(p, e) for p, e in skipped[:4])
        raise SystemExit("no trial had plottable samples: " + details)

    # Mixing pixel coordinates from different camera calibrations would make a
    # single aggregate image misleading, so fail explicitly in that case.
    cameras = {camera for _, _, camera, _ in loaded}
    if len(cameras) != 1:
        raise SystemExit(
            "trials use different camera calibrations; cannot combine them safely")
    points = np.vstack([item[1] for item in loaded])
    output = output_root / "all_trials_fpv_heatmap.png"
    plot_heatmap(output, points, loaded[0][2], args.dpi)
    print("Wrote {} ({} trials, {} samples)".format(
        output, len(loaded), len(points)))

    summary_path = output_root / "summary.csv"
    with summary_path.open("w", newline="") as handle:
        fields = ("controller", "trial", "success", "reason", "total_samples",
                  "front_samples", "plotted_samples")
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(item[3] for item in loaded)
    print("Wrote {}".format(summary_path))
    for path, reason in skipped:
        print("Skipped {}: {}".format(path, reason))


if __name__ == "__main__":
    main()
