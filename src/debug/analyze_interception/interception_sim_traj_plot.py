#!/usr/bin/env python3
"""Create speed, time, and FPV-pixel interception plots from a ROS1 bag."""

import argparse
from collections import defaultdict
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[3]


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag")
    p.add_argument("--output-dir")
    p.add_argument("--model-states-topic", default="/gazebo/model_states")
    p.add_argument("--uav-pose-topic", default="/mavros/local_position/pose")
    p.add_argument(
        "--uav-velocity-topic", default="/mavros/local_position/velocity_local"
    )
    p.add_argument("--balloon-topic", default="/balloons/masspoint")
    p.add_argument("--pixel-topic", default="/nmpc/p_image")
    p.add_argument("--fsm-topic", default="/fsm/fsm_state")
    p.add_argument("--fsm-state", default="EXEC_INTERCEPTION")
    p.add_argument("--uav-model", default="holydrone")
    p.add_argument("--balloon-prefix", default="balloon_")
    p.add_argument("--img-width", type=float, default=2592.0)
    p.add_argument("--img-height", type=float, default=1944.0)
    p.add_argument("--img-f", type=float, default=1495.7243877115116)
    p.add_argument("--img-cx", type=float)
    p.add_argument("--img-cy", type=float)
    p.add_argument("--dpi", type=int, default=180)
    p.add_argument("--title")
    return p.parse_args()


def resolve(value):
    path = Path(value).expanduser()
    return path if path.is_absolute() or path.exists() else ROOT / path


def stamp(message, bag_time):
    value = message.header.stamp.to_sec() if hasattr(message, "header") else 0.0
    return value if value > 0.0 else bag_time.to_sec()


def append_position(track, time_value, position, velocity=None):
    velocity_values = (
        (velocity.x, velocity.y, velocity.z)
        if velocity is not None else (np.nan, np.nan, np.nan)
    )
    row = (time_value, position.x, position.y, position.z) + velocity_values
    if np.all(np.isfinite(row[:4])) and (not track or time_value > track[-1][0]):
        track.append(row)


def load_data(args, bag_path):
    gazebo_uav, local_uav, mavros_velocity, pixels = [], [], [], []
    fsm_samples = []
    gazebo_balloons, message_balloons = defaultdict(list), defaultdict(list)
    topics = [args.model_states_topic, args.uav_pose_topic, args.uav_velocity_topic,
              args.balloon_topic, args.pixel_topic, args.fsm_topic]
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=topics):
            time_value = stamp(msg, bag_time)
            if topic == args.model_states_topic:
                for index, (name, pose) in enumerate(zip(msg.name, msg.pose)):
                    velocity = msg.twist[index].linear if index < len(msg.twist) else None
                    if name == args.uav_model:
                        append_position(gazebo_uav, time_value, pose.position, velocity)
                    elif name.startswith(args.balloon_prefix):
                        append_position(
                            gazebo_balloons[name], time_value, pose.position, velocity
                        )
            elif topic == args.uav_pose_topic:
                append_position(local_uav, time_value, msg.pose.position)
            elif topic == args.uav_velocity_topic:
                linear = msg.twist.linear
                mavros_velocity.append(
                    (time_value, linear.x, linear.y, linear.z)
                )
            elif topic == args.balloon_topic:
                for index, point in enumerate(msg.points):
                    target_id = int(getattr(point, "volume", index))
                    append_position(message_balloons[f"balloon_{target_id}"],
                                    time_value, point.position, point.velocity)
            elif topic == args.pixel_topic and msg.point.z > 1e-9:
                cx = args.img_width / 2 if args.img_cx is None else args.img_cx
                cy = args.img_height / 2 if args.img_cy is None else args.img_cy
                u = cx + args.img_f * msg.point.x / msg.point.z
                v = cy + args.img_f * msg.point.y / msg.point.z
                if np.isfinite(u) and np.isfinite(v):
                    pixels.append((time_value, u, v))
            elif topic == args.fsm_topic:
                fsm_samples.append((time_value, msg.state_name))
    uav = gazebo_uav or local_uav
    # masspoint carries the balloon node's commanded physical velocity, whereas
    # Gazebo twist may remain zero when SetModelState is used for motion.
    balloons = message_balloons if message_balloons else gazebo_balloons
    if not uav or not any(balloons.values()):
        raise RuntimeError("UAV or balloon trajectory is missing from the bag")
    if len(pixels) < 2:
        raise RuntimeError(f"fewer than two valid samples on {args.pixel_topic}")
    if len(mavros_velocity) < 2:
        raise RuntimeError(f"insufficient UAV velocity on {args.uav_velocity_topic}")
    uav = attach_velocity(uav, np.asarray(mavros_velocity))
    sources = (
        "{} position + MAVROS velocity".format("Gazebo" if gazebo_uav else "MAVROS"),
        "masspoint" if message_balloons else "Gazebo",
    )
    intervals = state_intervals(fsm_samples, args.fsm_state)
    if not intervals:
        raise RuntimeError(f"no {args.fsm_state} interval on {args.fsm_topic}")
    uav = filter_intervals(np.asarray(uav), intervals)
    balloons = {
        name: filter_intervals(np.asarray(track), intervals)
        for name, track in balloons.items() if track
    }
    balloons = {name: track for name, track in balloons.items() if len(track) > 1}
    pixels = filter_intervals(np.asarray(pixels), intervals)
    if len(uav) < 2 or not balloons or len(pixels) < 2:
        raise RuntimeError(f"insufficient samples inside {args.fsm_state}")
    return uav, balloons, pixels, sources, intervals


def attach_velocity(position_track, velocity_track):
    """Interpolate a timestamped [t,vx,vy,vz] stream onto a position track."""
    position_track = np.asarray(position_track, dtype=float).copy()
    for axis in range(3):
        position_track[:, 4 + axis] = np.interp(
            position_track[:, 0], velocity_track[:, 0], velocity_track[:, 1 + axis]
        )
    return position_track


def state_intervals(samples, selected_state):
    intervals, active_start = [], None
    for time_value, state_name in samples:
        if state_name == selected_state and active_start is None:
            active_start = time_value
        elif state_name != selected_state and active_start is not None:
            intervals.append((active_start, time_value))
            active_start = None
    if active_start is not None and samples:
        intervals.append((active_start, samples[-1][0]))
    return intervals


def filter_intervals(samples, intervals):
    mask = np.zeros(len(samples), dtype=bool)
    for start, end in intervals:
        mask |= (samples[:, 0] >= start) & (samples[:, 0] <= end)
    return samples[mask]


def segment3(track):
    return np.stack((track[:-1, 1:4], track[1:, 1:4]), axis=1)


def speed(track):
    if track.shape[1] >= 7:
        velocity_speed = np.linalg.norm(track[:, 4:7], axis=1)
        segment_speed = 0.5 * (velocity_speed[:-1] + velocity_speed[1:])
        if np.all(np.isfinite(segment_speed)):
            return segment_speed
    dt = np.diff(track[:, 0])
    ds = np.linalg.norm(np.diff(track[:, 1:4], axis=0), axis=1)
    raw = np.divide(ds, dt, out=np.zeros_like(ds), where=dt > 1e-6)
    # Suppress asynchronous position-update spikes when no measured velocity
    # is available. Use an approximately 0.1 s centered median window.
    median_dt = np.median(dt[dt > 1e-6]) if np.any(dt > 1e-6) else 0.1
    half_window = max(1, int(round(0.05 / median_dt)))
    return np.asarray([
        np.median(raw[max(0, i - half_window):min(len(raw), i + half_window + 1)])
        for i in range(len(raw))
    ])


def gradient3(axis, track, values, norm, cmap, label, linestyle):
    linewidths = np.full(len(values), 2.5)
    if linestyle == "dashed":
        segment_length = np.linalg.norm(np.diff(track[:, 1:4], axis=0), axis=1)
        segment_midpoint = np.cumsum(segment_length) - 0.5 * segment_length
        # Build dashes from physical arc length. Applying a normal dashed style
        # to a Line3DCollection restarts the dash pattern on every tiny segment.
        dash_period_m = 1.0
        linewidths[(segment_midpoint % dash_period_m) >= 0.65] = 0.0
    lines = Line3DCollection(
        segment3(track), cmap=cmap, norm=norm, linewidths=linewidths,
        linestyles="solid"
    )
    lines.set_array(values)
    axis.add_collection3d(lines)
    axis.plot(
        [], [], [], color=cmap(norm(float(np.median(values)))),
        linestyle=linestyle, linewidth=2.5, label=label
    )
    axis.scatter(*track[0, 1:4], color=cmap(norm(values[0])), s=30)
    return lines


def setup_3d(axis, tracks, title):
    xyz = np.vstack([track[:, 1:4] for track in tracks])
    low, high = np.min(xyz, axis=0), np.max(xyz, axis=0)
    center, radius = (low + high) / 2, max(0.5, np.max(high - low) / 2)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)
    axis.set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]", title=title)
    axis.grid(True, alpha=0.3)
    axis.legend()


def plot_3d(output, heading, tracks, values, norm, cmap, colorbar_label, dpi):
    fig = plt.figure(figsize=(11, 9))
    axis = fig.add_subplot(111, projection="3d")
    mappable = None
    for (label, track), color_values in zip(tracks, values):
        if len(track) > 1:
            linestyle = "solid" if label == "UAV" else "dashed"
            mappable = gradient3(
                axis, track, color_values, norm, cmap, label, linestyle
            )
    setup_3d(axis, [track for _, track in tracks], heading)
    fig.colorbar(mappable, ax=axis, pad=0.1, shrink=0.72, label=colorbar_label)
    fig.savefig(str(output), dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def plot_pixels(output, heading, pixels, args, t0, time_norm, cmap):
    points = pixels[:, 1:3]
    segments = np.stack((points[:-1], points[1:]), axis=1)
    times = (pixels[:-1, 0] + pixels[1:, 0]) / 2 - t0
    lines = LineCollection(segments, cmap=cmap, norm=time_norm, linewidth=2.3)
    lines.set_array(times)
    fig, axis = plt.subplots(figsize=(11, 8))
    axis.add_collection(lines)
    axis.scatter(*points[0], color=cmap(time_norm(times[0])), s=35)
    axis.set_xlim(0, args.img_width)
    axis.set_ylim(args.img_height, 0)
    axis.set_aspect("equal", adjustable="box")
    axis.set(xlabel="u [px]", ylabel="v [px]", title=heading)
    axis.grid(True, alpha=0.3)
    fig.colorbar(lines, ax=axis, label="time [s]")
    fig.savefig(str(output), dpi=args.dpi, bbox_inches="tight")
    plt.close(fig)


def combine_plots(output, image_paths, dpi):
    """Place the three detailed plots in one horizontal overview image."""
    fig, axes = plt.subplots(1, 3, figsize=(24, 7.5))
    panel_titles = (
        "(a) 3D trajectory — speed",
        "(b) 3D trajectory — time",
        "(c) FPV pixel trajectory — time",
    )
    for axis, image_path, panel_title in zip(axes, image_paths, panel_titles):
        axis.imshow(plt.imread(str(image_path)))
        axis.set_title(panel_title, fontsize=14)
        axis.axis("off")
    fig.tight_layout(pad=0.6)
    fig.savefig(str(output), dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def main():
    args = arguments()
    bag_path = resolve(args.bag)
    if not bag_path.is_file():
        raise SystemExit(f"bag does not exist: {bag_path}")
    output_dir = resolve(args.output_dir) if args.output_dir else bag_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    uav, balloons, pixels, sources, intervals = load_data(args, bag_path)
    tracks = [("UAV", uav)] + sorted(balloons.items())
    all_times = np.concatenate([pixels[:, 0]] + [track[:, 0] for _, track in tracks])
    t0, duration = np.min(all_times), max(1e-6, np.ptp(all_times))
    time_norm, time_cmap = Normalize(0, duration), plt.get_cmap("viridis")
    speed_values = [speed(track) for _, track in tracks]
    speed_max = max(1e-6, np.percentile(np.concatenate(speed_values), 99))
    speed_norm, speed_cmap = Normalize(0, speed_max), plt.get_cmap("plasma")
    prefix = output_dir / bag_path.stem
    outputs = [Path(f"{prefix}_traj_speed.png"), Path(f"{prefix}_traj_time.png"),
               Path(f"{prefix}_fpv_pixel_time.png")]
    heading = args.title or f"Interception simulation: {bag_path.stem}"
    plot_3d(outputs[0], f"{heading}\n3D trajectories colored by speed", tracks,
            [np.clip(v, 0, speed_max) for v in speed_values], speed_norm,
            speed_cmap, "speed [m/s]", args.dpi)
    time_values = [(track[:-1, 0] + track[1:, 0]) / 2 - t0 for _, track in tracks]
    plot_3d(outputs[1], f"{heading}\n3D trajectories colored by time", tracks,
            time_values, time_norm, time_cmap, "time [s]", args.dpi)
    plot_pixels(outputs[2], f"{heading}\nBalloon position in UAV FPV",
                pixels, args, t0, time_norm, time_cmap)
    combined_output = Path(f"{prefix}_trajectory_analysis.png")
    combine_plots(combined_output, outputs, args.dpi)
    for output in outputs:
        print(f"Wrote {output}")
    print(f"Wrote {combined_output}")
    print(f"Sources: UAV={sources[0]}, balloons={sources[1]}, pixels={args.pixel_topic}")
    print(f"State filter: {args.fsm_state}, intervals={intervals}")


if __name__ == "__main__":
    main()
