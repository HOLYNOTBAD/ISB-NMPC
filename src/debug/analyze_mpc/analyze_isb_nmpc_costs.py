#!/usr/bin/env python3

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


DEFAULT_PARAM_FILE = "src/isb_nmpc/config/holydrone/simulation/isb_nmpc_sim_outdoor_5ms_0degree.yaml"
DEFAULT_LOG_DIR = "log/interception"
DEFAULT_OUTPUT_SUBDIR = "cost_plots"
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]

COST_PLOT_LABELS = {
    "L_velocity": r"$L_{\mathrm{velocity}}$",
    "L_speed": r"$L_{\mathrm{speed}}$",
    "L_vis": r"$L_{\mathrm{vis}}$",
    "L_reg": r"$L_{\mathrm{reg}}$",
    "L_N": r"$L_N$",
    "total_first_step_est": r"$\widehat{L}_{\mathrm{total}}$",
}


def resolve_path(path):
    path = Path(path)
    if path.is_absolute() or path.exists():
        return path
    return WORKSPACE_ROOT / path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline ISB-NMPC interception cost decomposition from debug CSV logs."
    )
    parser.add_argument(
        "csv_files",
        nargs="*",
        help="CSV log files to process. Defaults to every non-empty CSV in log/interception.",
    )
    parser.add_argument(
        "--log-dir",
        default=DEFAULT_LOG_DIR,
        help="Directory scanned when csv_files is empty.",
    )
    parser.add_argument(
        "--param-file",
        default=DEFAULT_PARAM_FILE,
        help="ISB-NMPC yaml containing Q/R weights.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help=(
            "Output directory for all generated files. By default, each CSV "
            "writes to a cost_plots subdirectory beside that CSV."
        ),
    )
    parser.add_argument(
        "--latest",
        action="store_true",
        help="Only process the newest CSV from --log-dir.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show figures interactively after saving.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=2500,
        help="Maximum plotted points per curve after decimation.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=160,
        help="PNG output DPI.",
    )
    parser.add_argument(
        "--no-cost-csv",
        action="store_true",
        help="Do not write per-sample *_costs.csv files.",
    )
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
    parser.add_argument(
        "--use-logged-target-velocity",
        action="store_true",
        help="Use target_v* columns in interception velocity costs. By default this mirrors the current controller, which sends zero target velocity to the solver.",
    )
    return parser.parse_args()


def load_weights(path):
    path = resolve_path(path)
    with open(path, "r") as handle:
        params = yaml.safe_load(handle) or {}

    def value(name, default=0.0):
        raw = params.get(name, default)
        try:
            return float(raw)
        except (TypeError, ValueError):
            return float(default)

    return {
        "Q_velocity": value("Q_velocity"),
        "Q_perception_u": value("Q_perception_u"),
        "Q_perception_v": value("Q_perception_v"),
        "Q_image_rate_u": value("Q_image_rate_u"),
        "Q_image_rate_v": value("Q_image_rate_v"),
        "Q_image_rate_z": value(
            "Q_image_rate_z",
            0.5 * (value("Q_image_rate_u") + value("Q_image_rate_v")),
        ),
        "Q_speed": value("Q_speed"),
        "Q_terminal_line": value("Q_terminal_line"),
        # Terminal LSQ weights in so3_quadrotor_model.cpp.  These are
        # intentionally separate from the running image penalties.
        "Q_terminal_vis_u": value("Q_terminal_vis_u", 20.0),
        "Q_terminal_vis_v": value("Q_terminal_vis_v", 20.0),
        "hover_acceleration": value("hover_acceleration", 9.8066),
        "R_thrust": value("R_thrust"),
        "R_roll": value("R_roll", value("R_pitchroll")),
        "R_pitch": value("R_pitch", value("R_pitchroll")),
        "R_yaw": value("R_yaw"),
        "R_speed_delta": value("R_speed_delta"),
        "R_slack_u": value("R_slack_u"),
        "R_slack_v": value("R_slack_v"),
        "q_B_C": params.get("q_B_C", [1.0, 0.0, 0.0, 0.0]),
    }


def discover_csv_files(args):
    if args.csv_files:
        files = [resolve_path(p) for p in args.csv_files]
    else:
        log_dir = resolve_path(args.log_dir)
        files = sorted(
            # The broader pattern also keeps existing isb_nmpc_level2_*.csv
            # logs readable after the logger naming cleanup.
            log_dir.glob("isb_nmpc_*.csv"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        if args.latest:
            files = files[:1]
    return [p for p in files if p.is_file() and p.stat().st_size > 600]


def read_csv_columns(path):
    rows = []
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)
    if not rows:
        raise ValueError("no data rows")

    columns = {}
    for key in rows[0].keys():
        values = []
        for row in rows:
            try:
                values.append(float(row[key]))
            except (TypeError, ValueError):
                values.append(float("nan"))
        columns[key] = np.asarray(values, dtype=float)
    return columns


def col(data, name, default=0.0):
    if name in data:
        return data[name]
    length = len(next(iter(data.values())))
    return np.full(length, default, dtype=float)


def require_columns(data, names):
    missing = [name for name in names if name not in data]
    if missing:
        raise ValueError("missing required columns: {}".format(", ".join(missing)))


def finite_gradient(y, t):
    y_safe = np.asarray(y, dtype=float).copy()
    t_safe = np.asarray(t, dtype=float).copy()
    valid = np.isfinite(y_safe) & np.isfinite(t_safe)
    if valid.sum() < 3:
        return np.zeros_like(y_safe)
    y_safe[~valid] = np.interp(t_safe[~valid], t_safe[valid], y_safe[valid])
    dt = np.gradient(t_safe)
    dt[np.abs(dt) < 1e-6] = np.nan
    dy = np.gradient(y_safe)
    grad = dy / dt
    grad[~np.isfinite(grad)] = 0.0
    return grad


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    norm = np.sqrt(np.sum(q * q, axis=-1, keepdims=True))
    norm = np.where(norm > 1e-9, norm, 1.0)
    return q / norm


def quat_inverse(q):
    inv = np.asarray(q, dtype=float).copy()
    inv[..., 1:] *= -1.0
    return inv


def quat_rotate(q, v):
    q = quat_normalize(q)
    v = np.asarray(v, dtype=float)
    w = q[..., 0]
    xyz = q[..., 1:]
    t = 2.0 * np.cross(xyz, v)
    return v + w[..., None] * t + np.cross(xyz, t)


def finite_or_default(values, default=0.0):
    values = np.asarray(values, dtype=float)
    return np.where(np.isfinite(values), values, default)


def bearing_from_focus(w, x, y, z):
    n_t = np.column_stack([
        2.0 * w * y + 2.0 * x * z,
        2.0 * y * z - 2.0 * w * x,
        1.0 - 2.0 * x * x - 2.0 * y * y,
    ])
    n_norm = np.linalg.norm(n_t, axis=1)
    return n_t / np.where(n_norm > 1e-9, n_norm, 1.0)[:, None]


def compute_costs(data, weights, use_logged_target_velocity=False):
    require_columns(data, [
        "stamp",
        "img_err_u", "img_err_v",
        "current_intercept_speed", "scheduled_intercept_speed",
        "uav_px", "uav_py", "uav_pz",
        "uav_qw", "uav_qx", "uav_qy", "uav_qz",
        "uav_vx", "uav_vy", "uav_vz",
        "target_vx", "target_vy", "target_vz",
        "target_dir_x", "target_dir_y", "target_dir_z",
        "focus_w", "focus_x", "focus_y", "focus_z",
        "u0_T", "u0_wx", "u0_wy", "u0_wz", "u0_ds", "u0_sv1", "u0_sv2",
        "xN_px", "xN_py", "xN_pz",
    ])
    stamp = col(data, "stamp")
    time = stamp - stamp[0]
    # Legacy real-flight logs can contain a frozen ROS timestamp.  Rebuild a
    # monotonic 50 Hz controller timeline so plots and derivatives remain
    # meaningful instead of collapsing onto t=0.
    finite_time = time[np.isfinite(time)]
    if finite_time.size > 1 and np.nanmax(finite_time) - np.nanmin(finite_time) < 1e-9:
        time = np.arange(len(stamp), dtype=float) / 50.0

    img_err_u = col(data, "img_err_u")
    img_err_v = col(data, "img_err_v")
    current_speed = col(data, "current_intercept_speed")
    scheduled_speed = col(data, "scheduled_intercept_speed")
    uav_v = np.column_stack([
        finite_or_default(col(data, "uav_vx")),
        finite_or_default(col(data, "uav_vy")),
        finite_or_default(col(data, "uav_vz")),
    ])
    logged_target_v = np.column_stack([
        finite_or_default(col(data, "target_vx")),
        finite_or_default(col(data, "target_vy")),
        finite_or_default(col(data, "target_vz")),
    ])
    solver_target_v = logged_target_v if use_logged_target_velocity else np.zeros_like(logged_target_v)
    q_world_body = np.column_stack([
        finite_or_default(col(data, "uav_qw"), 1.0),
        finite_or_default(col(data, "uav_qx")),
        finite_or_default(col(data, "uav_qy")),
        finite_or_default(col(data, "uav_qz")),
    ])
    q_body_camera = np.asarray(weights["q_B_C"], dtype=float)
    if q_body_camera.shape != (4,):
        q_body_camera = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=float)
    q_body_camera = quat_normalize(q_body_camera)

    u0_t = col(data, "u0_T")
    u0_wx = col(data, "u0_wx")
    u0_wy = col(data, "u0_wy")
    u0_wz = col(data, "u0_wz")
    u0_ds = col(data, "u0_ds")
    u0_sv1 = col(data, "u0_sv1")
    u0_sv2 = col(data, "u0_sv2")

    focus_w = finite_or_default(col(data, "focus_w"), 1.0)
    focus_x = finite_or_default(col(data, "focus_x"))
    focus_y = finite_or_default(col(data, "focus_y"))
    focus_z = finite_or_default(col(data, "focus_z"))
    n_t = bearing_from_focus(focus_w, focus_x, focus_y, focus_z)
    n_dot = np.column_stack([
        finite_gradient(n_t[:, 0], time),
        finite_gradient(n_t[:, 1], time),
        finite_gradient(n_t[:, 2], time),
    ])
    e_jit = np.cross(n_t, n_dot)

    rel_v_world = uav_v - solver_target_v
    q_body_camera_batch = np.repeat(q_body_camera[None, :], len(time), axis=0)
    bearing_body = quat_rotate(q_body_camera_batch, n_t)
    bearing_world = quat_rotate(q_world_body, bearing_body)
    bearing_norm = np.linalg.norm(bearing_world, axis=1)
    bearing_world = bearing_world / np.where(bearing_norm > 1e-9, bearing_norm, 1.0)[:, None]
    e_int = rel_v_world - current_speed[:, None] * bearing_world
    velocity_cost = weights["Q_velocity"] * np.sum(e_int * e_int, axis=1)
    speed_cost = weights["Q_speed"] * (current_speed - scheduled_speed) ** 2

    los_cost = (
        weights["Q_perception_u"] * img_err_u ** 2
        + weights["Q_perception_v"] * img_err_v ** 2
    )
    jitter_cost = (
        weights["Q_image_rate_v"] * e_jit[:, 0] ** 2
        + weights["Q_image_rate_u"] * e_jit[:, 1] ** 2
        + weights["Q_image_rate_z"] * e_jit[:, 2] ** 2
    )
    vis_u_cost = weights["Q_perception_u"] * img_err_u ** 2
    vis_v_cost = weights["Q_perception_v"] * img_err_v ** 2
    jit_x_cost = weights["Q_image_rate_v"] * e_jit[:, 0] ** 2
    jit_y_cost = weights["Q_image_rate_u"] * e_jit[:, 1] ** 2
    jit_z_cost = weights["Q_image_rate_z"] * e_jit[:, 2] ** 2
    regularization_cost = (
        weights["R_thrust"] * (u0_t - weights["hover_acceleration"]) ** 2
        + weights["R_roll"] * u0_wx ** 2
        + weights["R_pitch"] * u0_wy ** 2
        + weights["R_yaw"] * u0_wz ** 2
        + weights["R_speed_delta"] * u0_ds ** 2
        + weights["R_slack_u"] * u0_sv1 ** 2
        + weights["R_slack_v"] * u0_sv2 ** 2
    )
    xN_position = np.column_stack([
        finite_or_default(col(data, "xN_px")),
        finite_or_default(col(data, "xN_py")),
        finite_or_default(col(data, "xN_pz")),
    ])
    uav_position = np.column_stack([
        finite_or_default(col(data, "uav_px")),
        finite_or_default(col(data, "uav_py")),
        finite_or_default(col(data, "uav_pz")),
    ])
    target_direction = np.column_stack([
        finite_or_default(col(data, "target_dir_x")),
        finite_or_default(col(data, "target_dir_y")),
        finite_or_default(col(data, "target_dir_z")),
    ])
    line_norm = np.linalg.norm(target_direction, axis=1)
    line_direction = target_direction / np.where(line_norm > 1e-9, line_norm, 1.0)[:, None]
    line_direction = np.where(np.isfinite(line_direction), line_direction, 0.0)
    line_direction[line_norm <= 1e-9] = np.array([1.0, 0.0, 0.0])
    terminal_line_dist = np.linalg.norm(
        np.cross(xN_position - uav_position, line_direction), axis=1)

    terminal_line_cost = weights["Q_terminal_line"] * terminal_line_dist ** 2
    terminal_vis_u_cost = (
        weights["Q_terminal_vis_u"] * col(data, "xN_img_err_u") ** 2)
    terminal_vis_v_cost = (
        weights["Q_terminal_vis_v"] * col(data, "xN_img_err_v") ** 2)
    terminal_cost = (
        terminal_line_cost + terminal_vis_u_cost + terminal_vis_v_cost)

    costs = {
        "L_velocity": velocity_cost,
        "L_speed": speed_cost,
        "L_vis_proj_u": vis_u_cost,
        "L_vis_proj_v": vis_v_cost,
        "L_jit_x": jit_x_cost,
        "L_jit_y": jit_y_cost,
        "L_jit_z": jit_z_cost,
        "L_los": los_cost,
        "L_jit": jitter_cost,
        "L_terminal_line": terminal_line_cost,
        "L_terminal_vis_u": terminal_vis_u_cost,
        "L_terminal_vis_v": terminal_vis_v_cost,
        "L_int": velocity_cost + speed_cost,
        "L_vis": los_cost + jitter_cost,
        "L_reg": regularization_cost,
        "L_N": terminal_cost,
    }

    total = np.zeros_like(time)
    for name, values in costs.items():
        if name not in ("L_velocity", "L_speed", "L_vis", "L_reg", "L_N"):
            continue
        total += np.nan_to_num(values, nan=0.0, posinf=0.0, neginf=0.0)
    costs["total_first_step_est"] = total
    return time, costs


def decimate(time, series, max_points):
    if len(time) <= max_points:
        return time, series
    step = int(math.ceil(float(len(time)) / float(max_points)))
    return time[::step], {name: values[::step] for name, values in series.items()}


def summarize_costs(csv_path, time, costs):
    row = {"file": csv_path.name, "samples": len(time), "duration_s": float(time[-1] - time[0])}
    for name, values in costs.items():
        finite = values[np.isfinite(values)]
        if finite.size == 0:
            row[name + "_mean"] = 0.0
            row[name + "_max"] = 0.0
            row[name + "_integral"] = 0.0
            continue
        row[name + "_mean"] = float(np.mean(finite))
        row[name + "_max"] = float(np.max(finite))
        row[name + "_integral"] = float(np.trapz(np.nan_to_num(values), time))
    return row


def plot_costs(
    csv_path, time, costs, output_dir, max_points, dpi, show, fpv_frames=None
):
    output_dir.mkdir(parents=True, exist_ok=True)
    plot_time, plot_costs = decimate(time, costs, max_points)

    cost_names = ["L_velocity", "L_speed", "L_vis", "L_reg", "L_N"]

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.suptitle("{} cost decomposition".format(csv_path.stem), fontsize=13)

    for name in cost_names:
        axes[0].plot(
            plot_time,
            plot_costs[name],
            label=COST_PLOT_LABELS[name],
            linewidth=1.5,
        )
    axes[0].set_title("First-step approximate grouped costs")
    axes[0].set_ylabel("cost")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", ncol=4, fontsize=8)

    stack_names = cost_names
    stack_values = [np.nan_to_num(plot_costs[name]) for name in stack_names]
    stack_labels = [COST_PLOT_LABELS[name] for name in stack_names]
    axes[1].stackplot(plot_time, stack_values, labels=stack_labels, alpha=0.85)
    axes[1].plot(
        plot_time,
        plot_costs["total_first_step_est"],
        color="black",
        linewidth=1.0,
        label=COST_PLOT_LABELS["total_first_step_est"],
    )
    axes[1].set_title("Stacked first-step approximate cost")
    axes[1].set_ylabel("cost")
    axes[1].set_xlabel("time [s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right", ncol=5, fontsize=8)

    finite_time = time[np.isfinite(time)]
    time_limits = (float(np.min(finite_time)), float(np.max(finite_time)))
    if fpv_frames is not None:
        add_fpv_strip(fig, axes[-1], fpv_frames, time_limits)
    else:
        fig.tight_layout(rect=[0, 0, 1, 0.96])

    output_path = output_dir / "{}_costs.png".format(csv_path.stem)
    fig.savefig(str(output_path), dpi=dpi)
    if show:
        plt.show()
    plt.close(fig)
    return output_path


def write_cost_csv(csv_path, time, costs, output_dir):
    output_path = output_dir / "{}_costs.csv".format(csv_path.stem)
    fieldnames = ["time"]
    fieldnames.extend(costs.keys())
    with open(output_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, t in enumerate(time):
            row = {"time": "{:.9g}".format(t)}
            for name, values in costs.items():
                value = values[index]
                row[name] = "{:.9g}".format(value) if np.isfinite(value) else "nan"
            writer.writerow(row)
    return output_path


def write_summary(rows, output_dir):
    if not rows:
        return None
    output_path = output_dir / "cost_summary.csv"
    fieldnames = list(rows[0].keys())
    with open(output_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return output_path


def output_dir_for(csv_path, requested_output_dir=None):
    if requested_output_dir is not None:
        return requested_output_dir
    return csv_path.parent / DEFAULT_OUTPUT_SUBDIR


def main():
    args = parse_args()
    weights = load_weights(args.param_file)
    csv_files = discover_csv_files(args)
    if not csv_files:
        raise SystemExit("No usable CSV logs found.")

    requested_output_dir = resolve_path(args.output_dir) if args.output_dir else None

    summaries_by_output_dir = {}
    for csv_path in csv_files:
        try:
            output_dir = output_dir_for(csv_path, requested_output_dir)
            data = read_csv_columns(csv_path)
            time, costs = compute_costs(
                data, weights, args.use_logged_target_velocity
            )
            fpv_frames = None
            if args.bag_file:
                finite_stamps = col(data, "stamp")
                finite_stamps = finite_stamps[np.isfinite(finite_stamps)]
                finite_time = time[np.isfinite(time)]
                fpv_frames = extract_fpv_frames(
                    resolve_path(args.bag_file),
                    float(finite_stamps[0]),
                    float(np.max(finite_time)),
                    interval=args.fpv_interval,
                    fpv_topic=args.fpv_topic,
                )
                print_alignment_summary(fpv_frames)
            output_path = plot_costs(
                csv_path,
                time,
                costs,
                output_dir,
                args.max_points,
                args.dpi,
                args.show,
                fpv_frames,
            )
            if not args.no_cost_csv:
                cost_csv_path = write_cost_csv(csv_path, time, costs, output_dir)
                print("Wrote {}".format(cost_csv_path))
            summaries_by_output_dir.setdefault(output_dir, []).append(
                summarize_costs(csv_path, time, costs)
            )
            print("Wrote {}".format(output_path))
        except Exception as exc:
            print("Skipping {}: {}".format(csv_path, exc))

    for output_dir, summaries in summaries_by_output_dir.items():
        summary_path = write_summary(summaries, output_dir)
        if summary_path is not None:
            print("Wrote {}".format(summary_path))


if __name__ == "__main__":
    main()
