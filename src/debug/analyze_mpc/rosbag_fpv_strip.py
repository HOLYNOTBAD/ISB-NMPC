#!/usr/bin/env python3

"""Extract time-aligned FPV frames from a ROS1 bag and add them to plots."""

import sys
from pathlib import Path

import numpy as np
from matplotlib.offsetbox import AnnotationBbox, OffsetImage


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


def _prepare_ros_python_path():
    candidates = [
        WORKSPACE_ROOT / "devel/lib/python3/dist-packages",
        WORKSPACE_ROOT / "devel/lib/python2.7/dist-packages",
    ]
    for candidate in candidates:
        if candidate.is_dir() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))


def _message_stamp(msg, bag_stamp):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and stamp.to_sec() > 0.0:
        return stamp.to_sec()
    return bag_stamp.to_sec()


def _decode_image(msg):
    encoding = msg.encoding.lower()
    channel_count = {
        "mono8": 1,
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }.get(encoding)
    if channel_count is None:
        try:
            from cv_bridge import CvBridge

            return CvBridge().imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            raise ValueError(
                "unsupported FPV image encoding: {}".format(msg.encoding)
            ) from exc

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = raw.reshape(msg.height, msg.step)
    pixels = rows[:, : msg.width * channel_count]
    if channel_count == 1:
        return np.repeat(pixels.reshape(msg.height, msg.width, 1), 3, axis=2)

    image = pixels.reshape(msg.height, msg.width, channel_count)
    if encoding in ("bgr8", "bgra8"):
        image = image[..., [2, 1, 0] + ([3] if channel_count == 4 else [])]
    if channel_count == 4:
        image = image[..., :3]
    return image.copy()


def _find_interception_entry(bag, fsm_topic, state_name, csv_start_stamp):
    entries = []
    previous_state = None
    try:
        for _, msg, bag_stamp in bag.read_messages(topics=[fsm_topic]):
            current_state = getattr(msg, "state_name", "")
            if current_state == state_name and previous_state != state_name:
                entries.append(_message_stamp(msg, bag_stamp))
            previous_state = current_state
    except Exception as exc:
        return None, "could not read {}: {}".format(fsm_topic, exc)

    if not entries:
        return None, "{} has no transition into {}".format(fsm_topic, state_name)
    entries_before_csv = [stamp for stamp in entries if stamp <= csv_start_stamp + 1e-6]
    if entries_before_csv:
        return entries_before_csv[-1], None
    return entries[0], None


def extract_fpv_frames(
    bag_path,
    csv_start_stamp,
    duration,
    interval=3.0,
    fpv_topic="/camera/fpv_debug",
    fsm_topic="/fsm/fsm_state",
    interception_state="EXEC_INTERCEPTION",
):
    """Return frames at plot-relative times and alignment metadata.

    Plot time zero is the first CSV stamp. The FSM transition is used to verify
    that the CSV really starts after interception begins, not as the time origin.
    """
    if interval <= 0.0:
        raise ValueError("FPV frame interval must be positive")
    bag_path = Path(bag_path).expanduser().resolve()
    if not bag_path.is_file():
        raise FileNotFoundError("rosbag not found: {}".format(bag_path))

    _prepare_ros_python_path()
    try:
        import genpy
        import rosbag
    except ImportError as exc:
        raise RuntimeError(
            "ROS1 Python modules are unavailable; source devel/setup.bash first"
        ) from exc

    target_times = np.arange(0.0, max(0.0, duration) + 1e-9, interval)
    absolute_targets = csv_start_stamp + target_times
    frames = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        fsm_entry_stamp, fsm_warning = _find_interception_entry(
            bag, fsm_topic, interception_state, csv_start_stamp
        )

        margin = max(1.0, interval)
        start_time = genpy.Time.from_sec(float(absolute_targets[0] - margin))
        end_time = genpy.Time.from_sec(float(absolute_targets[-1] + margin))
        messages = bag.read_messages(
            topics=[fpv_topic], start_time=start_time, end_time=end_time
        )

        target_index = 0
        previous = None
        for _, msg, bag_stamp in messages:
            current = (_message_stamp(msg, bag_stamp), msg)
            while (
                target_index < len(absolute_targets)
                and current[0] >= absolute_targets[target_index]
            ):
                candidates = [current]
                if previous is not None:
                    candidates.append(previous)
                selected = min(
                    candidates,
                    key=lambda item: abs(item[0] - absolute_targets[target_index]),
                )
                frames.append(
                    {
                        "time": float(target_times[target_index]),
                        "image": _decode_image(selected[1]),
                        "source_stamp": float(selected[0]),
                    }
                )
                target_index += 1
            previous = current
            if target_index >= len(absolute_targets):
                break

        while target_index < len(absolute_targets) and previous is not None:
            if abs(previous[0] - absolute_targets[target_index]) > margin:
                break
            frames.append(
                {
                    "time": float(target_times[target_index]),
                    "image": _decode_image(previous[1]),
                    "source_stamp": float(previous[0]),
                }
            )
            target_index += 1

    if not frames:
        raise ValueError("{} contains no usable images on {}".format(bag_path, fpv_topic))

    alignment_offset = (
        csv_start_stamp - fsm_entry_stamp if fsm_entry_stamp is not None else None
    )
    return {
        "bag_path": bag_path,
        "frames": frames,
        "csv_start_stamp": float(csv_start_stamp),
        "fsm_entry_stamp": fsm_entry_stamp,
        "fsm_offset": alignment_offset,
        "fsm_warning": fsm_warning,
        "interval": float(interval),
    }


def print_alignment_summary(frame_set):
    if frame_set is None:
        return
    if frame_set["fsm_entry_stamp"] is not None:
        print(
            "FPV alignment: CSV t=0 is {:.3f} s after {}; extracted {} frames "
            "every {:.3f} s.".format(
                frame_set["fsm_offset"],
                "EXEC_INTERCEPTION",
                len(frame_set["frames"]),
                frame_set["interval"],
            )
        )
    else:
        print(
            "FPV alignment: using the first CSV stamp as t=0; extracted {} frames "
            "every {:.3f} s. Warning: {}".format(
                len(frame_set["frames"]),
                frame_set["interval"],
                frame_set["fsm_warning"],
            )
        )


def add_fpv_strip(fig, reference_axis, frame_set, time_limits, zoom=0.12):
    """Add a thumbnail strip aligned with a plot's x data coordinates."""
    if frame_set is None or not frame_set.get("frames"):
        return None

    fig.tight_layout(rect=[0, 0.18, 1, 0.96])
    reference_position = reference_axis.get_position()
    strip_axis = fig.add_axes(
        [reference_position.x0, 0.055, reference_position.width, 0.095]
    )
    strip_axis.set_xlim(time_limits)
    strip_axis.set_ylim(0.0, 1.0)
    strip_axis.set_yticks([])
    strip_axis.set_ylabel("FPV", rotation=0, labelpad=18, va="center")
    strip_axis.tick_params(axis="x", labelsize=7, length=2, pad=1)
    strip_axis.set_xlabel("FPV frame time [s]", fontsize=8, labelpad=1)

    frame_times = []
    for frame in frame_set["frames"]:
        frame_time = frame["time"]
        if frame_time < time_limits[0] - 1e-6 or frame_time > time_limits[1] + 1e-6:
            continue
        image_box = OffsetImage(frame["image"], zoom=zoom)
        annotation = AnnotationBbox(
            image_box,
            (frame_time, 0.53),
            xycoords=strip_axis.transData,
            frameon=True,
            pad=0.05,
            bboxprops={"edgecolor": "0.35", "linewidth": 0.6},
            annotation_clip=False,
        )
        annotation.set_clip_on(False)
        strip_axis.add_artist(annotation)
        frame_times.append(frame_time)

    strip_axis.set_xticks(frame_times)
    strip_axis.set_xticklabels(["{:.1f}".format(value) for value in frame_times])
    for spine in ("left", "right", "top"):
        strip_axis.spines[spine].set_visible(False)
    return strip_axis
