#!/usr/bin/env python3

"""Offline ISB-NMPC diagnostics for controller tuning from debug CSV logs."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from rosbag_fpv_strip import add_fpv_strip, extract_fpv_frames, print_alignment_summary


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PARAM_FILE = "src/isb_nmpc/config/holydrone/simulation/isb_nmpc_sim_outdoor_5ms_0degree.yaml"
DEFAULT_OUTPUT_SUBDIR = "tuning_plots"


def resolve_path(path):
    path = Path(path).expanduser()
    if path.is_absolute() or path.exists():
        return path
    return WORKSPACE_ROOT / path


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Generate offline ISB-NMPC tracking, control, prediction, and "
            "solver diagnostics from debug CSV logs."
        )
    )
    parser.add_argument("csv_files", nargs="+", help="CSV logs to analyze.")
    parser.add_argument(
        "--param-file",
        default=DEFAULT_PARAM_FILE,
        help="Controller YAML used for the run (limits and tuning context).",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help=(
            "Output directory for all inputs. By default, each CSV writes to "
            "a tuning_plots subdirectory beside that CSV."
        ),
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=3000,
        help="Maximum plotted samples per signal (summary uses all samples).",
    )
    parser.add_argument("--dpi", type=int, default=160, help="Output PNG DPI.")
    parser.add_argument(
        "--bag-file",
        default=None,
        help="Optional ROS1 bag providing time-aligned FPV debug frames.",
    )
    parser.add_argument(
        "--fpv-interval",
        type=float,
        default=3.0,
        help="Seconds between FPV thumbnails when --bag-file is used.",
    )
    parser.add_argument(
        "--fpv-topic",
        default="/camera/fpv_debug",
        help="Image topic used for FPV thumbnails.",
    )
    return parser.parse_args()


def load_params(path):
    path = resolve_path(path)
    with open(path, "r") as handle:
        params = yaml.safe_load(handle) or {}
    params["_path"] = str(path)
    return params


def read_csv_columns(path):
    rows = []
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header")
        for row in reader:
            rows.append(row)
    if not rows:
        raise ValueError("CSV has no data rows")

    columns = {}
    for name in reader.fieldnames:
        values = []
        for row in rows:
            try:
                values.append(float(row[name]))
            except (KeyError, TypeError, ValueError):
                values.append(float("nan"))
        columns[name] = np.asarray(values, dtype=float)
    return columns


def require_columns(data, names):
    missing = [name for name in names if name not in data]
    if missing:
        raise ValueError("missing required columns: {}".format(", ".join(missing)))


def col(data, name, default=np.nan):
    if name in data:
        return data[name]
    count = len(next(iter(data.values())))
    return np.full(count, default, dtype=float)


def param_float(params, name, default=np.nan):
    try:
        return float(params.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def safe_stat(values, operation, default=np.nan):
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return float(default)
    return float(operation(finite))


def percentile(values, q):
    return safe_stat(values, lambda finite: np.percentile(finite, q))


def rms(values):
    return safe_stat(values, lambda finite: np.sqrt(np.mean(finite * finite)))


def fraction(mask, valid=None):
    mask = np.asarray(mask, dtype=bool)
    if valid is not None:
        valid = np.asarray(valid, dtype=bool)
        mask = mask[valid]
    if mask.size == 0:
        return float("nan")
    return float(np.mean(mask))


def vector_norm(data, names):
    values = np.column_stack([col(data, name) for name in names])
    valid = np.all(np.isfinite(values), axis=1)
    result = np.full(len(values), np.nan)
    result[valid] = np.linalg.norm(values[valid], axis=1)
    return result


def relative_closing_speed(data):
    uav_p = np.column_stack([col(data, "uav_p" + axis) for axis in "xyz"])
    target_p = np.column_stack([col(data, "target_p" + axis) for axis in "xyz"])
    uav_v = np.column_stack([col(data, "uav_v" + axis) for axis in "xyz"])
    target_v = np.column_stack([col(data, "target_v" + axis) for axis in "xyz"])
    line = target_p - uav_p
    line_norm = np.linalg.norm(line, axis=1)
    valid = (
        np.all(np.isfinite(line), axis=1)
        & np.all(np.isfinite(uav_v), axis=1)
        & np.all(np.isfinite(target_v), axis=1)
        & (line_norm > 1e-6)
    )
    closing = np.full(len(line), np.nan)
    direction = line[valid] / line_norm[valid, None]
    closing[valid] = np.sum((uav_v[valid] - target_v[valid]) * direction, axis=1)
    return closing


def make_time(data):
    require_columns(data, ["stamp"])
    stamp = col(data, "stamp")
    finite = np.flatnonzero(np.isfinite(stamp))
    if finite.size == 0:
        raise ValueError("stamp column contains no finite values")
    relative = stamp - stamp[finite[0]]
    # Some legacy logs were recorded while ROS time was frozen.  Keep those
    # logs usable by reconstructing the controller timeline from sequence
    # order (the SO3 bridge runs at 50 Hz).
    valid = relative[finite]
    if valid.size > 1 and np.nanmax(valid) - np.nanmin(valid) < 1e-9:
        return np.arange(len(stamp), dtype=float) / 50.0
    return relative


def decimation_indices(count, max_points):
    if count <= max_points:
        return np.arange(count)
    step = int(math.ceil(float(count) / float(max_points)))
    return np.arange(0, count, step)


def add_limit_lines(axis, lower=None, upper=None, color="0.35"):
    if lower is not None and np.isfinite(lower):
        axis.axhline(lower, color=color, linestyle=":", linewidth=1.0)
    if upper is not None and np.isfinite(upper):
        axis.axhline(upper, color=color, linestyle=":", linewidth=1.0)


def finish_figure(
    fig,
    title,
    output_path,
    dpi,
    reference_axis=None,
    fpv_frames=None,
    time_limits=None,
):
    fig.suptitle(title, fontsize=13)
    if fpv_frames is not None:
        add_fpv_strip(fig, reference_axis, fpv_frames, time_limits)
    else:
        fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(str(output_path), dpi=dpi)
    plt.close(fig)


def plot_tracking(csv_path, data, time, index, output_dir, dpi, fpv_frames=None):
    t = time[index]
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)

    axes[0].plot(t, col(data, "range")[index], label=r"$r$")
    axes[0].plot(t, col(data, "x1_range")[index], "--", label=r"$r_1$")
    axes[0].plot(t, col(data, "xN_range")[index], ":", label=r"$r_N$")
    axes[0].set_ylabel("range [m]")
    axes[0].legend(ncol=3)

    axes[1].plot(
        t, col(data, "current_intercept_speed")[index], label=r"$s$"
    )
    axes[1].plot(
        t,
        col(data, "scheduled_intercept_speed")[index],
        "--",
        label=r"$s_{\mathrm{scheduled}}$",
    )
    axes[1].plot(
        t, relative_closing_speed(data)[index], ":", label=r"$v_{\mathrm{closing}}$"
    )
    axes[1].set_ylabel("speed [m/s]")
    axes[1].legend(ncol=3)

    axes[2].plot(t, col(data, "img_err_u")[index], label=r"$e_u$")
    axes[2].plot(t, col(data, "img_err_v")[index], label=r"$e_v$")
    add_limit_lines(axes[2], -1.0, 1.0)
    axes[2].set_ylabel("normalized image error")
    axes[2].legend(ncol=2)

    axes[3].step(
        t, col(data, "has_target", 0.0)[index], where="post", label="target state"
    )
    axes[3].step(
        t,
        col(data, "target_in_main_view", 0.0)[index],
        where="post",
        label="main view",
    )
    axes[3].set_ylim(-0.1, 1.1)
    axes[3].set_ylabel("availability")
    axes[3].set_xlabel("time [s]")
    axes[3].legend(ncol=2)

    for axis in axes:
        axis.grid(True, alpha=0.3)
    output_path = output_dir / "{}_tracking.png".format(csv_path.stem)
    finish_figure(
        fig,
        "{} tracking diagnostics".format(csv_path.stem),
        output_path,
        dpi,
        axes[-1],
        fpv_frames,
        (float(np.nanmin(time)), float(np.nanmax(time))),
    )
    return output_path


def plot_controls(
    csv_path, data, params, time, index, output_dir, dpi, fpv_frames=None
):
    t = time[index]
    fig, axes = plt.subplots(5, 1, figsize=(13, 14), sharex=True)

    axes[0].plot(t, col(data, "u0_T")[index], label=r"$T_0$")
    axes[0].plot(t, col(data, "cmd_T")[index], "--", label=r"$T_{\mathrm{cmd}}$")
    add_limit_lines(
        axes[0],
        param_float(params, "min_thrust"),
        param_float(params, "max_thrust"),
    )
    hover = param_float(params, "hover_acceleration")
    if np.isfinite(hover):
        axes[0].axhline(hover, color="tab:green", linestyle=":", label=r"$g_{\mathrm{hover}}$")
    axes[0].set_ylabel(r"thrust [m/s$^2$]")
    axes[0].legend(ncol=3)

    rate_colors = {"x": "tab:red", "y": "tab:green", "z": "tab:blue"}
    for axis_name in "xyz":
        axes[1].plot(
            t,
            col(data, "cmd_w" + axis_name)[index],
            color=rate_colors[axis_name],
            label=r"$\omega_{{{},\mathrm{{cmd}}}}$".format(axis_name),
        )
    axes[1].set_ylabel("command rate [rad/s]")
    axes[1].legend(ncol=3)

    motion_colors = {"x": "tab:red", "y": "tab:green", "z": "tab:blue"}
    for axis_name in "xyz":
        velocity = col(data, "uav_v" + axis_name)
        axes[2].plot(
            t,
            velocity[index],
            color=motion_colors[axis_name],
            label=r"$v_{}$".format(axis_name),
        )
        acceleration = np.gradient(velocity, time)
        axes[3].plot(
            t,
            acceleration[index],
            color=motion_colors[axis_name],
            label=r"$a_{}$".format(axis_name),
        )
    axes[2].set_ylabel("UAV velocity [m/s]")
    axes[2].legend(ncol=3)
    axes[3].set_ylabel(r"UAV acceleration [m/s$^2$]" + "\n(derived)")
    axes[3].legend(ncol=3)

    axes[4].plot(t, col(data, "u0_ds")[index], label=r"$\dot{s}_0$")
    delta_limit = col(data, "max_delta_intercept_speed")
    if np.any(np.isfinite(delta_limit)):
        limit = safe_stat(delta_limit, np.nanmedian)
        add_limit_lines(axes[4], -limit, limit)
    axes[4].set_ylabel(r"$\dot{s}$ [m/s$^2$]")
    axes[4].set_xlabel("time [s]")
    slack_axis = axes[4].twinx()
    slack_axis.plot(t, col(data, "u0_sv1")[index], "--", label=r"$s_{v,u}$")
    slack_axis.plot(t, col(data, "u0_sv2")[index], ":", label=r"$s_{v,v}$")
    slack_axis.set_ylabel("FOV slack")
    lines_left, labels_left = axes[4].get_legend_handles_labels()
    lines_right, labels_right = slack_axis.get_legend_handles_labels()
    axes[4].legend(lines_left + lines_right, labels_left + labels_right, ncol=3)

    for axis in axes:
        axis.grid(True, alpha=0.3)
    output_path = output_dir / "{}_controls.png".format(csv_path.stem)
    finish_figure(
        fig,
        "{} control diagnostics".format(csv_path.stem),
        output_path,
        dpi,
        axes[-1],
        fpv_frames,
        (float(np.nanmin(time)), float(np.nanmax(time))),
    )
    return output_path


def plot_solver(csv_path, data, time, index, output_dir, dpi, fpv_frames=None):
    t = time[index]
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)

    kkt = col(data, "kkt")[index]
    axes[0].semilogy(t, np.maximum(np.abs(kkt), 1e-12), label="KKT residual")
    axes[0].set_ylabel("KKT")
    axes[0].legend()

    axes[1].step(
        t, col(data, "feedback_status")[index], where="post", label="feedback status"
    )
    axes[1].axhline(0.0, color="0.35", linestyle=":")
    axes[1].set_ylabel("status (0=OK)")
    axes[1].legend()

    axes[2].step(
        t,
        col(data, "preparation_ok", 0.0)[index],
        where="post",
        label="preparation OK",
    )
    axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_ylabel("prepared")
    axes[2].legend()

    dt_ms = np.full(len(time), np.nan)
    dt_ms[1:] = np.diff(time) * 1000.0
    axes[3].plot(t, dt_ms[index], label=r"$\Delta t$")
    axes[3].set_ylabel("log interval [ms]")
    axes[3].set_xlabel("time [s]")
    axes[3].legend()

    for axis in axes:
        axis.grid(True, alpha=0.3)
    output_path = output_dir / "{}_solver.png".format(csv_path.stem)
    finish_figure(
        fig,
        "{} solver diagnostics".format(csv_path.stem),
        output_path,
        dpi,
        axes[-1],
        fpv_frames,
        (float(np.nanmin(time)), float(np.nanmax(time))),
    )
    return output_path


def plot_prediction(csv_path, data, time, index, output_dir, dpi, fpv_frames=None):
    t = time[index]
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)

    for name, style, label in [
        ("range", "-", r"$r$"),
        ("x1_range", "--", r"$r_1$"),
        ("xN_range", ":", r"$r_N$"),
    ]:
        axes[0].plot(t, col(data, name)[index], style, label=label)
    axes[0].set_ylabel("predicted range [m]")
    axes[0].legend(ncol=3)

    for name, style, label in [
        ("current_intercept_speed", "-", r"$s$"),
        ("x1_intercept_speed", "--", r"$s_1$"),
        ("xN_intercept_speed", ":", r"$s_N$"),
    ]:
        axes[1].plot(t, col(data, name)[index], style, label=label)
    axes[1].set_ylabel("predicted speed [m/s]")
    axes[1].legend(ncol=3)

    axes[2].plot(t, col(data, "img_err_u")[index], label=r"$e_u$")
    axes[2].plot(t, col(data, "img_err_v")[index], label=r"$e_v$")
    axes[2].plot(t, col(data, "xN_img_err_u")[index], "--", label=r"$e_{u,N}$")
    axes[2].plot(t, col(data, "xN_img_err_v")[index], "--", label=r"$e_{v,N}$")
    add_limit_lines(axes[2], -1.0, 1.0)
    axes[2].set_ylabel("image error")
    axes[2].legend(ncol=4)

    terminal_offset = np.column_stack(
        [col(data, "xN_p" + axis) - col(data, "uav_p" + axis) for axis in "xyz"]
    )
    for component, axis_name in enumerate("xyz"):
        axes[3].plot(
            t,
            terminal_offset[index, component],
            label=r"$p_{{N,{}}}-p_{{0,{}}}$".format(axis_name, axis_name),
        )
    axes[3].set_ylabel("terminal displacement [m]")
    axes[3].set_xlabel("time [s]")
    axes[3].legend(ncol=3)

    for axis in axes:
        axis.grid(True, alpha=0.3)
    output_path = output_dir / "{}_prediction.png".format(csv_path.stem)
    finish_figure(
        fig,
        "{} horizon diagnostics".format(csv_path.stem),
        output_path,
        dpi,
        axes[-1],
        fpv_frames,
        (float(np.nanmin(time)), float(np.nanmax(time))),
    )
    return output_path


def symmetric_saturation_fraction(values, limit, threshold=0.98):
    if not np.isfinite(limit) or limit <= 0.0:
        return float("nan")
    values = np.asarray(values, dtype=float)
    valid = np.isfinite(values)
    return fraction(np.abs(values) >= threshold * limit, valid)


def build_summary(csv_path, data, params, time):
    dt = np.diff(time)
    valid_dt = dt[np.isfinite(dt) & (dt > 0.0)]
    img_u = col(data, "img_err_u")
    img_v = col(data, "img_err_v")
    img_norm = np.sqrt(img_u * img_u + img_v * img_v)
    current_speed = col(data, "current_intercept_speed")
    scheduled_speed = col(data, "scheduled_intercept_speed")
    target = col(data, "has_target", 0.0) > 0.5
    view = col(data, "target_in_main_view", 0.0) > 0.5
    nominal_rate = param_float(params, "control_rate")

    summary = {
        "file": csv_path.name,
        "param_file": params["_path"],
        "samples": len(time),
        "duration_s": safe_stat(time, lambda x: np.max(x) - np.min(x)),
        "sample_rate_hz_median": (
            1.0 / np.median(valid_dt) if valid_dt.size else float("nan")
        ),
        "sample_interval_ms_p95": (
            1000.0 * np.percentile(valid_dt, 95) if valid_dt.size else float("nan")
        ),
        "nominal_control_rate_hz": nominal_rate,
        "feedback_success_pct": 100.0 * fraction(col(data, "feedback_status") == 0),
        "preparation_success_pct": 100.0 * fraction(col(data, "preparation_ok") > 0.5),
        "kkt_median": percentile(np.abs(col(data, "kkt")), 50),
        "kkt_p95": percentile(np.abs(col(data, "kkt")), 95),
        "kkt_max": safe_stat(np.abs(col(data, "kkt")), np.max),
        "target_available_pct": 100.0 * fraction(target),
        "main_view_pct": 100.0 * fraction(view),
        "main_view_pct_when_target": 100.0 * fraction(view, target),
        "range_start_m": safe_stat(col(data, "range")[:1], np.mean),
        "range_min_m": safe_stat(col(data, "range"), np.min),
        "range_end_m": safe_stat(col(data, "range")[-1:], np.mean),
        "image_u_rms": rms(img_u),
        "image_v_rms": rms(img_v),
        "image_norm_p95": percentile(img_norm, 95),
        "image_outside_normalized_fov_pct": 100.0
        * fraction((np.abs(img_u) > 1.0) | (np.abs(img_v) > 1.0)),
        "speed_tracking_rmse_mps": rms(current_speed - scheduled_speed),
        "closing_speed_rmse_mps": rms(current_speed - relative_closing_speed(data)),
        "slack_u_active_pct": 100.0 * fraction(col(data, "u0_sv1") > 1e-6),
        "slack_v_active_pct": 100.0 * fraction(col(data, "u0_sv2") > 1e-6),
        "slack_u_max": safe_stat(col(data, "u0_sv1"), np.max),
        "slack_v_max": safe_stat(col(data, "u0_sv2"), np.max),
        "thrust_min": safe_stat(col(data, "u0_T"), np.min),
        "thrust_max": safe_stat(col(data, "u0_T"), np.max),
    }

    for axis_name in "xyz":
        limit = param_float(params, "max_bodyrate_" + axis_name)
        summary["bodyrate_{}_saturation_pct".format(axis_name)] = 100.0 * (
            symmetric_saturation_fraction(col(data, "u0_w" + axis_name), limit)
        )
    delta_limit = safe_stat(col(data, "max_delta_intercept_speed"), np.nanmedian)
    summary["speed_delta_saturation_pct"] = 100.0 * symmetric_saturation_fraction(
        col(data, "u0_ds"), delta_limit
    )
    return summary


def tuning_cues(summary):
    cues = []
    if summary["feedback_success_pct"] < 100.0 or summary["preparation_success_pct"] < 100.0:
        cues.append(
            "Solver failures are present: inspect the failure times in the solver plot "
            "before changing tracking weights."
        )
    nominal_rate = summary["nominal_control_rate_hz"]
    nominal_interval_ms = 1000.0 / nominal_rate if nominal_rate > 0.0 else float("nan")
    if (
        np.isfinite(nominal_interval_ms)
        and summary["sample_interval_ms_p95"] > 1.5 * nominal_interval_ms
    ):
        cues.append(
            "The upper-tail log interval is high for the configured control rate; "
            "check CPU load and solver timing before increasing aggressiveness."
        )
    if summary["kkt_p95"] > 1.0 or summary["kkt_max"] > 100.0:
        cues.append(
            "The KKT residual has a large upper tail: correlate its peaks with FOV "
            "slack, rate limits, and rapid reference changes; also check problem scaling."
        )
    if summary["target_available_pct"] < 1.0 and summary["main_view_pct"] > 1.0:
        cues.append(
            "This is effectively a bearing-only segment: range is the solver's nominal "
            "range and measured relative closing speed is unavailable, so do not tune "
            "position/range behavior from those curves."
        )
    if summary["image_outside_normalized_fov_pct"] > 1.0:
        cues.append(
            "Image error leaves the normalized FOV: inspect Q_perception_u/v, "
            "R_slack_u/v, LOS speed scheduling, and intercept speed together."
        )
    if summary["slack_u_active_pct"] > 1.0 or summary["slack_v_active_pct"] > 1.0:
        cues.append(
            "FOV slack is active: increasing slack penalties may improve visibility but "
            "can also reduce feasibility; correlate with KKT and rate saturation."
        )
    saturated_axes = [
        axis_name
        for axis_name in "xyz"
        if summary["bodyrate_{}_saturation_pct".format(axis_name)] > 5.0
    ]
    if saturated_axes:
        cues.append(
            "Body-rate saturation is frequent on {}: inspect R_roll/R_pitch/R_yaw, "
            "perception weights, and trajectory aggressiveness before changing limits."
            .format(", ".join(saturated_axes))
        )
    if summary["speed_delta_saturation_pct"] > 5.0:
        cues.append(
            "The intercept-speed rate is frequently constrained: compare Q_speed with "
            "R_speed_delta and max_delta_intercept_speed."
        )
    if summary["speed_tracking_rmse_mps"] > 1.0:
        cues.append(
            "Optimized intercept speed differs materially from its schedule: compare "
            "Q_speed against Q_velocity, perception costs, and R_speed_delta."
        )
    if not cues:
        cues.append(
            "No dominant threshold violation was detected; use the plots and compare "
            "the same metrics across parameter sweeps."
        )
    return cues


def write_report(csv_path, summary, output_dir):
    output_path = output_dir / "{}_tuning_report.txt".format(csv_path.stem)
    lines = [
        "ISB-NMPC offline tuning report",
        "CSV: {}".format(csv_path),
        "Parameters: {}".format(summary["param_file"]),
        "",
        "Key metrics",
        "  duration: {:.3f} s, samples: {}, median rate: {:.2f} Hz".format(
            summary["duration_s"], summary["samples"], summary["sample_rate_hz_median"]
        ),
        "  solver feedback/preparation success: {:.2f}% / {:.2f}%".format(
            summary["feedback_success_pct"], summary["preparation_success_pct"]
        ),
        "  KKT median/p95/max: {:.6g} / {:.6g} / {:.6g}".format(
            summary["kkt_median"], summary["kkt_p95"], summary["kkt_max"]
        ),
        "  3D target available / target in main view: {:.2f}% / {:.2f}%".format(
            summary["target_available_pct"], summary["main_view_pct"]
        ),
        "  image RMS u/v, norm p95: {:.4f} / {:.4f} / {:.4f}".format(
            summary["image_u_rms"], summary["image_v_rms"], summary["image_norm_p95"]
        ),
        "  outside normalized FOV: {:.2f}%".format(
            summary["image_outside_normalized_fov_pct"]
        ),
        "  speed schedule RMSE: {:.4f} m/s".format(summary["speed_tracking_rmse_mps"]),
        "  FOV slack active u/v: {:.2f}% / {:.2f}%".format(
            summary["slack_u_active_pct"], summary["slack_v_active_pct"]
        ),
        "  body-rate saturation x/y/z: {:.2f}% / {:.2f}% / {:.2f}%".format(
            summary["bodyrate_x_saturation_pct"],
            summary["bodyrate_y_saturation_pct"],
            summary["bodyrate_z_saturation_pct"],
        ),
        "  speed-rate saturation: {:.2f}%".format(summary["speed_delta_saturation_pct"]),
        "",
        "Tuning cues (diagnostic, not automatic parameter changes)",
    ]
    lines.extend("  - " + cue for cue in tuning_cues(summary))
    output_path.write_text("\n".join(lines) + "\n")
    return output_path


def write_summaries(summaries, output_dir):
    output_path = output_dir / "tuning_summary.csv"
    fieldnames = list(summaries[0].keys())
    with open(output_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summaries)
    return output_path


def analyze(
    csv_path,
    params,
    output_dir,
    max_points,
    dpi,
    bag_file=None,
    fpv_interval=3.0,
    fpv_topic="/camera/fpv_debug",
):
    data = read_csv_columns(csv_path)
    require_columns(
        data,
        [
            "stamp",
            "feedback_status",
            "preparation_ok",
            "kkt",
            "range",
            "current_intercept_speed",
            "scheduled_intercept_speed",
            "img_err_u",
            "img_err_v",
            "uav_vx",
            "uav_vy",
            "uav_vz",
            "u0_T",
            "u0_wx",
            "u0_wy",
            "u0_wz",
            "u0_ds",
            "u0_sv1",
            "u0_sv2",
            "xN_range",
            "xN_intercept_speed",
            "xN_img_err_u",
            "xN_img_err_v",
        ],
    )
    time = make_time(data)
    index = decimation_indices(len(time), max_points)
    output_dir.mkdir(parents=True, exist_ok=True)
    fpv_frames = None
    if bag_file is not None:
        finite_stamps = col(data, "stamp")
        finite_stamps = finite_stamps[np.isfinite(finite_stamps)]
        fpv_frames = extract_fpv_frames(
            bag_file,
            float(finite_stamps[0]),
            float(np.nanmax(time)),
            interval=fpv_interval,
            fpv_topic=fpv_topic,
        )
        print_alignment_summary(fpv_frames)

    outputs = [
        plot_tracking(csv_path, data, time, index, output_dir, dpi, fpv_frames),
        plot_controls(
            csv_path, data, params, time, index, output_dir, dpi, fpv_frames
        ),
        plot_solver(csv_path, data, time, index, output_dir, dpi, fpv_frames),
        plot_prediction(csv_path, data, time, index, output_dir, dpi, fpv_frames),
    ]
    summary = build_summary(csv_path, data, params, time)
    outputs.append(write_report(csv_path, summary, output_dir))
    return summary, outputs


def main():
    args = parse_args()
    params = load_params(args.param_file)
    requested_output_dir = resolve_path(args.output_dir) if args.output_dir else None
    summaries_by_dir = {}

    for raw_path in args.csv_files:
        csv_path = resolve_path(raw_path)
        output_dir = requested_output_dir or csv_path.parent / DEFAULT_OUTPUT_SUBDIR
        try:
            summary, outputs = analyze(
                csv_path,
                params,
                output_dir,
                args.max_points,
                args.dpi,
                resolve_path(args.bag_file) if args.bag_file else None,
                args.fpv_interval,
                args.fpv_topic,
            )
            summaries_by_dir.setdefault(output_dir, []).append(summary)
            for output in outputs:
                print("Wrote {}".format(output))
        except Exception as exc:
            print("Skipping {}: {}".format(csv_path, exc))

    if not summaries_by_dir:
        raise SystemExit("No CSV logs were analyzed successfully.")
    for output_dir, summaries in summaries_by_dir.items():
        print("Wrote {}".format(write_summaries(summaries, output_dir)))


if __name__ == "__main__":
    main()
