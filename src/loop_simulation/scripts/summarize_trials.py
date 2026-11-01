#!/usr/bin/env python3
"""Print measured outcomes from existing loop-simulation recordings.

Usage: summarize_trials.py SESSION_OR_PARENT [SESSION_OR_PARENT ...]
This is offline only: it never starts or modifies ROS/PX4/Gazebo.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import yaml


def trial_paths(root):
    if (root / "metadata.yaml").is_file():
        return [root]
    return sorted(p.parent for p in root.rglob("metadata.yaml")
                  if (p.parent / "states.csv").is_file())


def summarize(trial):
    with (trial / "metadata.yaml").open() as handle:
        metadata = yaml.safe_load(handle)
    peak_speed = 0.0
    peak_abs_roll = 0.0
    peak_abs_pitch = 0.0
    peak_tilt = 0.0
    peak_tilt_time = 0.0
    peak_tilt_speed = 0.0
    roll_at_peak_tilt = 0.0
    pitch_at_peak_tilt = 0.0
    samples = 0
    with (trial / "states.csv").open(newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                vx, vy, vz = (float(row[key]) for key in
                              ("uav_vx", "uav_vy", "uav_vz"))
                qw, qx, qy, qz = (float(row[key]) for key in
                                  ("uav_qw", "uav_qx", "uav_qy", "uav_qz"))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("invalid state row in {}: {}".format(trial, exc))
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            roll = math.degrees(math.atan2(
                2.0 * (qw * qx + qy * qz),
                1.0 - 2.0 * (qx * qx + qy * qy)))
            pitch_sine = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
            pitch = math.degrees(math.asin(pitch_sine))
            tilt_cosine = max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))
            tilt = math.degrees(math.acos(tilt_cosine))
            if not all(math.isfinite(value) for value in
                       (speed, roll, pitch, tilt)):
                raise ValueError("nonfinite state in {}".format(trial))
            peak_speed = max(peak_speed, speed)
            peak_abs_roll = max(peak_abs_roll, abs(roll))
            peak_abs_pitch = max(peak_abs_pitch, abs(pitch))
            if tilt > peak_tilt:
                peak_tilt = tilt
                peak_tilt_time = float(row["elapsed_sec"])
                peak_tilt_speed = speed
                roll_at_peak_tilt = roll
                pitch_at_peak_tilt = pitch
            samples += 1
    result = metadata["result"]
    if samples == 0:
        return {
            "trial": str(trial),
            "success": result["success"],
            "reason": result["reason"],
            "elapsed_sec": "{:.2f}".format(float(result["elapsed_sec"])),
            "samples": 0,
            "peak_speed_mps": "",
            "peak_abs_roll_deg": "",
            "peak_abs_pitch_deg": "",
            "peak_tilt_deg": "",
            "peak_tilt_time_sec": "",
            "speed_at_peak_tilt_mps": "",
            "roll_at_peak_tilt_deg": "",
            "pitch_at_peak_tilt_deg": "",
        }
    return {
        "trial": str(trial),
        "success": result["success"],
        "reason": result["reason"],
        "elapsed_sec": "{:.2f}".format(float(result["elapsed_sec"])),
        "samples": samples,
        "peak_speed_mps": "{:.2f}".format(peak_speed),
        "peak_abs_roll_deg": "{:.2f}".format(peak_abs_roll),
        "peak_abs_pitch_deg": "{:.2f}".format(peak_abs_pitch),
        "peak_tilt_deg": "{:.2f}".format(peak_tilt),
        "peak_tilt_time_sec": "{:.2f}".format(peak_tilt_time),
        "speed_at_peak_tilt_mps": "{:.2f}".format(peak_tilt_speed),
        "roll_at_peak_tilt_deg": "{:.2f}".format(roll_at_peak_tilt),
        "pitch_at_peak_tilt_deg": "{:.2f}".format(pitch_at_peak_tilt),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path)
    args = parser.parse_args()
    fields = ("trial", "success", "reason", "elapsed_sec", "samples",
              "peak_speed_mps", "peak_abs_roll_deg", "peak_abs_pitch_deg",
              "peak_tilt_deg", "peak_tilt_time_sec",
              "speed_at_peak_tilt_mps", "roll_at_peak_tilt_deg",
              "pitch_at_peak_tilt_deg")
    writer = csv.DictWriter(sys.stdout, fieldnames=fields)
    writer.writeheader()
    count = 0
    for root in args.paths:
        if not root.exists():
            parser.error("path not found: {}".format(root))
        for trial in trial_paths(root):
            writer.writerow(summarize(trial))
            count += 1
    if count == 0:
        parser.error("no recorded trials found")


if __name__ == "__main__":
    main()
