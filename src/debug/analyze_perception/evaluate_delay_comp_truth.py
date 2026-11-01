#!/usr/bin/env python3
"""Evaluate raw and delay-compensated bounding boxes against simulation truth.

The script intentionally reads a bag offline: it does not need a running ROS
master.  Source the workspace first so rosbag can resolve swarm_msgs.
"""

import argparse
import csv
import json
import math
import os
import sys
from bisect import bisect_left

import numpy as np

try:
    import rosbag
except ImportError as exc:
    raise SystemExit("rosbag is unavailable; source /opt/ros/noetic/setup.bash first") from exc


DEFAULT_TOPICS = {
    "raw": "/tracker/bbox_raw",
    "filtered": "/tracker/bbox",
    "pose": "/mavros/local_position/pose",
    "truth_pos": "/balloons/masspoint",
    "truth_hsv": "/tracker/p_image_truth_hsv",
    "origin": "/local_frame/origin",
    "gazebo_pose": "/gazebo/model_states",
    "image": "/camera/image_raw",
}
IMAGE_TOPIC_CANDIDATES = (
    DEFAULT_TOPICS["image"],
    "/holydrone_cam/usb_cam/image_raw",
    "/camera/left",
    "/camera/fpv_debug",
)


def stamp_sec(stamp):
    return stamp.to_sec() if stamp is not None else 0.0


def message_time(msg, bag_time, image=False):
    if image and hasattr(msg, "image_header"):
        value = stamp_sec(msg.image_header.stamp)
        if value > 0.0:
            return value
    if hasattr(msg, "header"):
        value = stamp_sec(msg.header.stamp)
        if value > 0.0:
            return value
    return bag_time.to_sec()


def box_center(msg):
    if not msg.bounding_boxes:
        return None
    box = max(msg.bounding_boxes, key=lambda item: float(item.probability))
    return np.array([(box.xmin + box.xmax) * 0.5,
                     (box.ymin + box.ymax) * 0.5], dtype=float)


def box_centers(msg):
    return [np.array([(box.xmin + box.xmax) * 0.5,
                      (box.ymin + box.ymax) * 0.5], dtype=float)
            for box in msg.bounding_boxes]


def hsv_centers(image_msg):
    """Run the same red HSV detector as target_detection_hsv offline."""
    try:
        import cv2
        from cv_bridge import CvBridge
        image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    except ImportError as exc:
        raise SystemExit(
            "offline HSV fallback requires opencv-python and cv_bridge") from exc
    except Exception as exc:
        print("warning: could not decode image for offline HSV: %s" % exc,
              file=sys.stderr)
        return []
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    high = cv2.inRange(hsv, np.array([171, 80, 40], dtype=np.uint8),
                       np.array([180, 255, 255], dtype=np.uint8))
    low = cv2.inRange(hsv, np.array([0, 80, 40], dtype=np.uint8),
                      np.array([10, 255, 255], dtype=np.uint8))
    mask = cv2.bitwise_or(high, low)
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    # The online node publishes bounding-box centers (not mask centroids).
    return [np.array([
                stats[index, cv2.CC_STAT_LEFT] + stats[index, cv2.CC_STAT_WIDTH] * 0.5,
                stats[index, cv2.CC_STAT_TOP] + stats[index, cv2.CC_STAT_HEIGHT] * 0.5,
            ], dtype=float)
            for index in range(1, count)
            if stats[index, cv2.CC_STAT_AREA] > 25]


def quat_to_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def normalized_lerp_quat(q0, q1, alpha):
    q0, q1 = np.asarray(q0), np.asarray(q1)
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    q = (1.0 - alpha) * q0 + alpha * q1
    return q / max(np.linalg.norm(q), 1e-12)


def bracket(samples, time_value, max_gap):
    if not samples:
        return None
    times = [item[0] for item in samples]
    index = bisect_left(times, time_value)
    if index == 0:
        return samples[0] if abs(samples[0][0] - time_value) <= max_gap else None
    if index == len(samples):
        return samples[-1] if abs(samples[-1][0] - time_value) <= max_gap else None
    before, after = samples[index - 1], samples[index]
    if time_value - before[0] > max_gap or after[0] - time_value > max_gap:
        return None
    alpha = (time_value - before[0]) / max(after[0] - before[0], 1e-12)
    return before, after, alpha


def interpolate_pose(samples, time_value, max_gap):
    value = bracket(samples, time_value, max_gap)
    if value is None:
        return None
    if len(value) != 3 or not isinstance(value[2], float):
        return value[1], value[2]
    before, after, alpha = value
    position = (1.0 - alpha) * before[1] + alpha * after[1]
    quat = normalized_lerp_quat(before[2], after[2], alpha)
    return position, quat


def interpolate_targets(samples, time_value, max_gap):
    value = bracket(samples, time_value, max_gap)
    if value is None:
        return {}
    if len(value) != 3 or not isinstance(value[2], float):
        return value[1]
    before, after, alpha = value
    result = {}
    for target_id in set(before[1]).intersection(after[1]):
        result[target_id] = ((1.0 - alpha) * before[1][target_id] +
                             alpha * after[1][target_id])
    return result


def project_targets(time_value, poses, targets, intrinsics, r_bc, p_bc,
                    world_from_local_translation, max_gap):
    pose = interpolate_pose(poses, time_value, max_gap)
    points = interpolate_targets(targets, time_value, max_gap)
    if pose is None or not points:
        return {}
    vehicle_position, q_wb = pose
    r_wb = quat_to_matrix(q_wb)
    r_wc = r_wb.dot(r_bc)
    camera_position = (vehicle_position + world_from_local_translation +
                       r_wb.dot(p_bc))
    fx, fy, cx, cy = intrinsics
    projected = {}
    for target_id, point_w in points.items():
        point_c = r_wc.T.dot(point_w - camera_position)
        if point_c[2] <= 1e-4:
            continue
        projected[target_id] = np.array([
            fx * point_c[0] / point_c[2] + cx,
            fy * point_c[1] / point_c[2] + cy,
        ])
    return projected


def nearest_sample(samples, time_value, tolerance):
    if not samples:
        return None
    times = [item[0] for item in samples]
    index = bisect_left(times, time_value)
    candidates = samples[max(0, index - 1):min(len(samples), index + 1)]
    result = min(candidates, key=lambda item: abs(item[0] - time_value))
    return result if abs(result[0] - time_value) <= tolerance else None


def interpolate_center_set(samples, time_value, reference, max_gap):
    """Interpolate the HSV center on the target branch nearest reference."""
    value = bracket(samples, time_value, max_gap)
    if value is None:
        return None
    if len(value) != 3 or not isinstance(value[2], float):
        return min(value[1], key=lambda uv: np.linalg.norm(reference - uv))
    before, after, alpha = value
    center_before = min(
        before[1], key=lambda uv: np.linalg.norm(reference - uv))
    center_after = min(
        after[1], key=lambda uv: np.linalg.norm(reference - uv))
    return (1.0 - alpha) * center_before + alpha * center_after


def metrics(values):
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if not len(array):
        return {"count": 0, "mae_px": None, "rmse_px": None,
                "p50_px": None, "p90_px": None, "p95_px": None}
    return {
        "count": int(len(array)),
        "mae_px": float(np.mean(array)),
        "rmse_px": float(np.sqrt(np.mean(array ** 2))),
        "p50_px": float(np.percentile(array, 50)),
        "p90_px": float(np.percentile(array, 90)),
        "p95_px": float(np.percentile(array, 95)),
    }


def delay_metrics(values):
    result = metrics(values)
    return {key.replace("_px", "_s"): value for key, value in result.items()}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", nargs="?", help="input rosbag (legacy positional form)")
    parser.add_argument("--rosbag", dest="rosbag",
                        help="input rosbag path")
    parser.add_argument("--camera-config", required=True,
                        help="camera YAML containing camera.img_f/R_cam_body")
    parser.add_argument(
        "--out-dir",
        help="output directory (default: log/filter_eval/<bag-name>)")
    parser.add_argument("--match-tolerance", type=float, default=0.08,
                        help="maximum bbox pairing difference in seconds")
    parser.add_argument("--truth-max-gap", type=float, default=0.12)
    parser.add_argument("--raw-topic", default=DEFAULT_TOPICS["raw"])
    parser.add_argument("--filtered-topic", default=DEFAULT_TOPICS["filtered"])
    parser.add_argument("--pose-topic", default=DEFAULT_TOPICS["pose"])
    parser.add_argument("--truth-pos-topic", "--truth-topic",
                        dest="truth_pos_topic",
                        default=DEFAULT_TOPICS["truth_pos"],
                        help="balloon positions used to derive p_image_truth_pos")
    parser.add_argument("--truth-hsv-topic", default=DEFAULT_TOPICS["truth_hsv"],
                        help="recorded p_image_truth_hsv bounding boxes")
    parser.add_argument("--image-topic", default=DEFAULT_TOPICS["image"],
                        help="camera images used when recorded HSV results are absent")
    parser.add_argument("--origin-topic", default=DEFAULT_TOPICS["origin"],
                        help="PointStamped world position of the local origin")
    parser.add_argument(
        "--gazebo-model-states-topic", default=DEFAULT_TOPICS["gazebo_pose"],
        help="Gazebo ModelStates topic used as the preferred world pose")
    parser.add_argument("--gazebo-vehicle-model", default="holydrone",
                        help="vehicle name in Gazebo ModelStates")
    parser.add_argument(
        "--world-from-local-translation", nargs=3, type=float, metavar=("X", "Y", "Z"),
        help="world-frame position of the local-frame origin in metres; "
             "overrides --origin-topic")
    args = parser.parse_args()
    if args.bag and args.rosbag:
        parser.error("specify the input bag either positionally or with --rosbag, not both")
    args.bag = args.rosbag or args.bag
    if not args.bag:
        parser.error("an input bag is required; use --rosbag PATH")
    if not args.out_dir:
        bag_name = os.path.basename(args.bag)
        for suffix in (".bag.active", ".bag"):
            if bag_name.endswith(suffix):
                bag_name = bag_name[:-len(suffix)]
                break
        args.out_dir = os.path.join("log", "filter_eval", bag_name)
    return args


def load_camera(path):
    try:
        import yaml
    except ImportError as exc:
        raise SystemExit("PyYAML is required to load the camera config") from exc
    with open(path, "r", encoding="utf-8") as stream:
        camera = yaml.safe_load(stream)["camera"]
    fx = float(camera.get("fx", camera["img_f"]))
    fy = float(camera.get("fy", camera["img_f"]))
    width, height = float(camera["img_width"]), float(camera["img_height"])
    intrinsics = (fx, fy, float(camera.get("cx", width / 2.0)),
                  float(camera.get("cy", height / 2.0)))
    r_bc = np.asarray(camera["R_cam_body"], dtype=float).reshape(3, 3)
    p_bc = np.asarray(camera.get("position_body_m", [0.0, 0.0, 0.0]),
                      dtype=float)
    if p_bc.shape != (3,):
        raise SystemExit("camera.position_body_m must contain 3 values")
    return intrinsics, r_bc, p_bc


def read_bag(args):
    data = {"raw": [], "filtered": [], "pose": [], "image": [],
            "truth_pos": [], "truth_hsv": [], "origin": [],
            "gazebo_pose": []}
    topic_to_key = {args.raw_topic: "raw", args.filtered_topic: "filtered",
                    args.pose_topic: "pose",
                    args.truth_pos_topic: "truth_pos",
                    args.truth_hsv_topic: "truth_hsv",
                    args.image_topic: "image",
                    args.origin_topic: "origin",
                    args.gazebo_model_states_topic: "gazebo_pose"}
    # Keep compatibility with bags produced by the simulation camera launch.
    for topic in IMAGE_TOPIC_CANDIDATES:
        topic_to_key.setdefault(topic, "image")
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=list(topic_to_key)):
            key = topic_to_key[topic]
            if key in ("raw", "filtered"):
                center = box_center(msg)
                if center is not None:
                    data[key].append((message_time(msg, bag_time, image=True),
                                      message_time(msg, bag_time), center))
            elif key == "truth_hsv":
                centers = box_centers(msg)
                if centers:
                    data[key].append((message_time(msg, bag_time, image=True),
                                      centers))
            elif key == "image":
                data[key].append((message_time(msg, bag_time, image=True), msg))
            elif key == "pose":
                p, q = msg.pose.position, msg.pose.orientation
                data[key].append((message_time(msg, bag_time),
                                  np.array([p.x, p.y, p.z]),
                                  np.array([q.x, q.y, q.z, q.w])))
            elif key == "origin":
                data[key].append(
                    (message_time(msg, bag_time),
                     np.array([msg.point.x, msg.point.y, msg.point.z])))
            elif key == "gazebo_pose":
                try:
                    index = msg.name.index(args.gazebo_vehicle_model)
                except ValueError:
                    continue
                pose = msg.pose[index]
                p, q = pose.position, pose.orientation
                data[key].append(
                    (message_time(msg, bag_time),
                     np.array([p.x, p.y, p.z]),
                     np.array([q.x, q.y, q.z, q.w])))
            else:
                points = {int(item.volume): np.array([item.position.x,
                                                      item.position.y,
                                                      item.position.z])
                          for item in msg.points}
                data[key].append((bag_time.to_sec(), points, None))
    for values in data.values():
        values.sort(key=lambda item: item[0])
    return data


def main():
    args = parse_args()
    intrinsics, r_bc, p_bc = load_camera(args.camera_config)
    data = read_bag(args)
    hsv_source = "recorded:%s" % args.truth_hsv_topic
    if not data["truth_hsv"]:
        if not data["image"]:
            raise SystemExit(
                "no recorded HSV results and no images for offline fallback; "
                "check --truth-hsv-topic or --image-topic")
        data["truth_hsv"] = [
            (time_value, hsv_centers(msg)) for time_value, msg in data["image"]]
        data["truth_hsv"] = [item for item in data["truth_hsv"] if item[1]]
        hsv_source = "offline:%s (same thresholds as target_detection_hsv)" % args.image_topic
        print("warning: %s absent; generated HSV results offline from %d images" %
              (args.truth_hsv_topic, len(data["image"])), file=sys.stderr)
    if not data["raw"] or not data["filtered"]:
        counts = {key: len(value) for key, value in data.items()}
        raise SystemExit("bag misses required usable samples: " + json.dumps(counts))
    has_position_truth = bool(data["truth_pos"])
    projection_poses = []
    world_from_local_translation = np.zeros(3)
    origin_source = "unavailable"
    pose_source = "unavailable"
    if has_position_truth and data["gazebo_pose"]:
        projection_poses = data["gazebo_pose"]
        world_from_local_translation = np.zeros(3)
        origin_source = "gazebo_world_pose"
        pose_source = (args.gazebo_model_states_topic + ":" +
                       args.gazebo_vehicle_model)
    elif has_position_truth and args.world_from_local_translation is not None:
        projection_poses = data["pose"]
        world_from_local_translation = np.asarray(
            args.world_from_local_translation, dtype=float)
        origin_source = "command_line"
        pose_source = args.pose_topic
    elif has_position_truth and data["origin"]:
        projection_poses = data["pose"]
        world_from_local_translation = data["origin"][-1][1]
        origin_source = args.origin_topic
        pose_source = args.pose_topic
    elif has_position_truth:
        projection_poses = data["pose"]
        world_from_local_translation = np.zeros(3)
        origin_source = "assumed_zero"
        pose_source = args.pose_topic
        print(
            "warning: no local-to-world origin was provided or recorded; "
            "position-truth metrics are valid only if local and world origins "
            "coincide. Record /local_frame/origin or pass "
            "--world-from-local-translation X Y Z.",
            file=sys.stderr)

    rows = []
    for image_time, raw_publish_time, raw_uv in data["raw"]:
        filtered = nearest_sample(data["filtered"], image_time, args.match_tolerance)
        if filtered is None:
            continue
        current_time, filtered_uv = filtered[1], filtered[2]
        target_id = -1
        truth_image = truth_current = {}
        if has_position_truth:
            truth_image = project_targets(
                image_time, projection_poses, data["truth_pos"], intrinsics, r_bc,
                p_bc, world_from_local_translation, args.truth_max_gap)
            truth_current = project_targets(
                current_time, projection_poses, data["truth_pos"], intrinsics, r_bc,
                p_bc, world_from_local_translation, args.truth_max_gap)
            common = set(truth_image).intersection(truth_current)
            if not common:
                continue
            target_id = min(common, key=lambda key: np.linalg.norm(raw_uv - truth_image[key]))
            truth_i, truth_c = truth_image[target_id], truth_current[target_id]
        hsv_image_uv = interpolate_center_set(
            data["truth_hsv"], image_time, raw_uv, args.truth_max_gap)
        hsv_current_uv = interpolate_center_set(
            data["truth_hsv"], current_time,
            hsv_image_uv if hsv_image_uv is not None else raw_uv,
            args.truth_max_gap)
        if hsv_image_uv is None or hsv_current_uv is None:
            continue
        row = {
            "image_time_s": image_time, "current_time_s": current_time,
            "measured_delay_s": current_time - image_time, "target_id": target_id,
            "raw_u": float(raw_uv[0]), "raw_v": float(raw_uv[1]),
            "truth_pos_image_u": float(truth_i[0]) if has_position_truth else math.nan,
            "truth_pos_image_v": float(truth_i[1]) if has_position_truth else math.nan,
            "truth_pos_current_u": float(truth_c[0]) if has_position_truth else math.nan,
            "truth_pos_current_v": float(truth_c[1]) if has_position_truth else math.nan,
            "truth_hsv_u":
                (float(hsv_image_uv[0])
                 if hsv_image_uv is not None else math.nan),
            "truth_hsv_v":
                (float(hsv_image_uv[1])
                 if hsv_image_uv is not None else math.nan),
            "truth_hsv_current_u":
                (float(hsv_current_uv[0])
                 if hsv_current_uv is not None else math.nan),
            "truth_hsv_current_v":
                (float(hsv_current_uv[1])
                 if hsv_current_uv is not None else math.nan),
            "filtered_u": float(filtered_uv[0]), "filtered_v": float(filtered_uv[1]),
            "raw_vs_truth_pos_image_px": float(np.linalg.norm(raw_uv - truth_i)) if has_position_truth else math.nan,
            "raw_vs_truth_pos_current_px": float(np.linalg.norm(raw_uv - truth_c)) if has_position_truth else math.nan,
            "filtered_vs_truth_pos_current_px":
                float(np.linalg.norm(filtered_uv - truth_c)) if has_position_truth else math.nan,
            "raw_vs_truth_hsv_px":
                (float(np.linalg.norm(raw_uv - hsv_image_uv))
                 if hsv_image_uv is not None else math.nan),
            "raw_vs_truth_hsv_current_px":
                (float(np.linalg.norm(raw_uv - hsv_current_uv))
                 if hsv_current_uv is not None else math.nan),
            "filtered_vs_truth_hsv_current_px":
                (float(np.linalg.norm(filtered_uv - hsv_current_uv))
                 if hsv_current_uv is not None else math.nan),
        }
        rows.append(row)

    if not rows:
        raise SystemExit("no samples could be synchronized; check timestamps, topics, and camera extrinsics")
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, "delay_comp_samples.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    series = (["raw_vs_truth_pos_image_px", "raw_vs_truth_pos_current_px",
               "filtered_vs_truth_pos_current_px"] if has_position_truth else []) + [
              "raw_vs_truth_hsv_px",
              "raw_vs_truth_hsv_current_px",
              "filtered_vs_truth_hsv_current_px"]
    summary = {name: metrics([row[name] for row in rows]) for name in series}
    summary["matched_samples"] = len(rows)
    summary["hsv_source"] = hsv_source
    summary["position_truth_transform"] = {
        "pose_source": pose_source,
        "origin_source": origin_source,
        "world_from_local_translation_m":
            world_from_local_translation.tolist(),
        "camera_position_body_m": p_bc.tolist(),
    }
    summary["delay_s"] = delay_metrics([row["measured_delay_s"] for row in rows])
    if has_position_truth:
        raw_rmse = summary["raw_vs_truth_pos_current_px"]["rmse_px"]
        final_rmse = summary["filtered_vs_truth_pos_current_px"]["rmse_px"]
        summary["current_rmse_improvement_percent"] = (
            100.0 * (raw_rmse - final_rmse) / raw_rmse if raw_rmse else None)
    raw_hsv_rmse = summary["raw_vs_truth_hsv_current_px"]["rmse_px"]
    final_hsv_rmse = summary[
        "filtered_vs_truth_hsv_current_px"]["rmse_px"]
    summary["hsv_current_rmse_improvement_percent"] = (
        100.0 * (raw_hsv_rmse - final_hsv_rmse) / raw_hsv_rmse
        if raw_hsv_rmse else None)
    json_path = os.path.join(args.out_dir, "delay_comp_summary.json")
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        times = np.array([row["current_time_s"] for row in rows])
        times -= times[0]
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)
        for name in series:
            axes[0].plot(times, [row[name] for row in rows], label=name, alpha=0.8)
        axes[0].set(xlabel="time [s]", ylabel="center error [px]", title="Truth error")
        axes[0].grid(True); axes[0].legend()
        axes[1].boxplot([[row[name] for row in rows if math.isfinite(row[name])]
                         for name in series], labels=series, showfliers=False)
        axes[1].set_ylabel("center error [px]"); axes[1].grid(True, axis="y")
        axes[1].tick_params(axis="x", rotation=12)
        fig.savefig(os.path.join(args.out_dir, "delay_comp_truth.png"), dpi=160)
        plt.close(fig)

        fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True,
                                 constrained_layout=True)
        curve_specs = ([
            ("truth_pos_current", "position truth", "black", 2.0),
        ] if has_position_truth else []) + [
            ("truth_hsv_current", "HSV truth", "tab:green", 1.2),
            ("raw", "raw bbox", "tab:gray", 1.0),
            ("filtered", "final filtered", "tab:red", 1.2),
        ]
        for axis, coordinate in zip(axes, ("u", "v")):
            for prefix, label, color, width in curve_specs:
                values = [row[prefix + "_" + coordinate] for row in rows]
                axis.plot(times, values, label=label, color=color,
                          linewidth=width, alpha=0.9)
            axis.set_ylabel(coordinate + " [px]")
            axis.grid(True, alpha=0.35)
        axes[0].set_title("BBox center and current-time truth")
        axes[0].legend(ncol=4)
        axes[1].set_xlabel("time [s]")
        fig.savefig(os.path.join(args.out_dir, "delay_comp_uv_time.png"), dpi=160)
        plt.close(fig)
    except ImportError:
        print("warning: matplotlib unavailable; CSV and JSON were still written", file=sys.stderr)

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print("wrote", args.out_dir)


if __name__ == "__main__":
    main()
