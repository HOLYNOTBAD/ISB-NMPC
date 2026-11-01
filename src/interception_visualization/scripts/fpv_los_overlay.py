#!/usr/bin/env python3

import math
import threading
import time
from collections import deque

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import FsmState
from sensor_msgs.msg import CompressedImage, Image, RegionOfInterest
from std_msgs.msg import Bool, Float32, String
from swarm_msgs.msg import BoundingBoxes


class FpvLosOverlay:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_q_image = None
        self.latest_q_image_stamp = rospy.Time(0)
        self.q_image_history = deque(maxlen=1000)
        self.latest_bboxes = []
        self.latest_bboxes_stamp = rospy.Time(0)
        self.bbox_history = deque(maxlen=1000)
        self.target_intercepted = False
        self.target_intercepted_history = deque(maxlen=256)
        self.latest_yolo_bboxes = []
        self.latest_yolo_bboxes_stamp = rospy.Time(0)
        self.yolo_bbox_history = deque(maxlen=600)
        self.latest_roi_prediction = None
        self.latest_roi_prediction_receive_stamp = rospy.Time(0)
        self.ekf_prediction_trail = deque()
        self.latest_ekf_prediction_stamp = rospy.Time(0)
        self.latest_odometry = None
        self.odometry_history = deque(maxlen=2000)
        self.latest_detection_mode = "--"
        self.detection_mode_history = deque(maxlen=256)
        self.latest_fsm_state = "--"
        self.latest_fsm_state_stamp = rospy.Time(0)
        self.fsm_state_history = deque(maxlen=4000)
        self.latest_mpc_latency_ms = None
        self.latest_mpc_latency_stamp = rospy.Time(0)
        self.mpc_latency_history = deque(maxlen=1000)
        self.latest_yolo_frequency_hz = None
        self.latest_yolo_sim_frequency_hz = None
        self.latest_yolo_frequency_stamp = rospy.Time(0)
        self.last_yolo_wall_time = None
        self.last_yolo_ros_time = rospy.Time(0)
        self.last_yolo_seq = None
        self.latest_raw_frequency_hz = None
        self.latest_raw_sim_frequency_hz = None
        self.latest_raw_frequency_stamp = rospy.Time(0)
        self.last_raw_wall_time = None
        self.last_raw_ros_time = rospy.Time(0)
        self.last_raw_seq = None
        self.image_fps = None
        self.image_sim_fps = None
        self.last_image_wall_time = None
        self.last_image_ros_time = rospy.Time(0)
        self.use_sim_time = bool(rospy.get_param("/use_sim_time", False))
        self.replay_mode = bool(rospy.get_param("~replay_mode", False))
        environment = str(
            rospy.get_param("~hud_environment", "auto")
        ).strip().upper()
        if not environment or environment == "AUTO":
            environment = "SIM" if self.use_sim_time else "EXP"
        self.hud_environment = environment
        initial_detection_mode = str(rospy.get_param(
            "~initial_detection_mode", "--"
        )).strip() or "--"
        if initial_detection_mode != "--":
            self.latest_detection_mode = initial_detection_mode
            self.detection_mode_history.append(
                (rospy.Time(0), initial_detection_mode)
            )
        self.latest_realtime_factor = None
        self.last_clock_wall_time = None
        self.last_clock_ros_time = rospy.Time(0)
        self.latest_roi = None
        self.latest_roi_stamp = rospy.Time(0)
        # With /use_sim_time, ROS time is still zero until the first /clock
        # message.  Initialize this from the first rendered image instead.
        self.start_time = rospy.Time(0)
        self.last_output_time = rospy.Time(0)

        self.image_topic = rospy.get_param(
            "~image_topic", "/target_detection/fpv_source"
        )
        self.raw_image_topic = rospy.get_param(
            "~raw_image_topic", "/camera/image_raw"
        )
        self.compressed_image_topic = rospy.get_param(
            "~compressed_image_topic", ""
        )
        self.output_topic = rospy.get_param("~output_topic", "/camera/fpv_debug")
        self.max_output_rate_hz = max(
            0.0, float(rospy.get_param("~max_output_rate_hz", 0.0))
        )
        self.output_width = max(0, int(rospy.get_param("~output_width", 640)))
        self.output_height = max(0, int(rospy.get_param("~output_height", 480)))
        self.bbox_topic = rospy.get_param("~bbox_topic", "/tracker/bbox")
        self.yolo_bbox_topic = rospy.get_param(
            "~yolo_bbox_topic", "/tracker/bbox_raw"
        )
        self.roi_prediction_topic = rospy.get_param(
            "~roi_prediction_topic", "/tracker/bbox_roi_prediction"
        )
        self.q_image_topic = rospy.get_param("~q_image_topic", "/nmpc/q_image")
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/nmpc/uavStateEstimate"
        )
        self.detection_mode_topic = rospy.get_param(
            "~detection_mode_topic", "/target_detection/mode"
        )
        self.roi_topic = rospy.get_param(
            "~roi_topic", "/target_detection/roi"
        )
        self.fsm_state_topic = rospy.get_param(
            "~fsm_state_topic", "/fsm/fsm_state"
        )
        self.mpc_latency_topic = rospy.get_param(
            "~mpc_latency_topic", "/isb_nmpc/solve_latency_ms"
        )
        self.show_telemetry_hud = bool(
            rospy.get_param("~show_telemetry_hud", True)
        )
        self.los_timeout = float(rospy.get_param("~los_timeout", 0.3))
        self.bbox_timeout = float(rospy.get_param("~bbox_timeout", 0.3))
        self.roi_timeout = float(rospy.get_param("~roi_timeout", 0.3))
        self.roi_prediction_timeout = float(
            rospy.get_param("~roi_prediction_timeout", 0.15)
        )
        self.ekf_trail_duration = max(
            0.05, float(rospy.get_param("~ekf_trail_duration", 0.5))
        )
        self.ekf_trail_max_points = max(
            2, int(rospy.get_param("~ekf_trail_max_points", 30))
        )
        self.bound_margin = float(rospy.get_param("~bound_margin", 0.3))
        self.alpha = float(rospy.get_param("~alpha", 0.75))
        self.metrics_timeout = max(
            0.1, float(rospy.get_param("~metrics_timeout", 1.0))
        )
        self.last_render_ros_time = rospy.Time(0)

        self.img_width = int(rospy.get_param("/camera/img_width", 640))
        self.img_height = int(rospy.get_param("/camera/img_height", 480))
        self.img_f = float(rospy.get_param("/camera/img_f", 115.0))
        self.cx = float(rospy.get_param("/camera/cx", 0.5 * self.img_width))
        self.cy = float(rospy.get_param("/camera/cy", 0.5 * self.img_height))

        self.img_u_max = self._get_optional_bound(
            "~img_u_max", 0.5 * self.img_width / self.img_f - self.bound_margin
        )
        self.img_u_min = self._get_optional_bound("~img_u_min", -self.img_u_max)
        self.img_v_max = self._get_optional_bound(
            "~img_v_max", 0.5 * self.img_height / self.img_f - self.bound_margin
        )
        self.img_v_min = self._get_optional_bound("~img_v_min", -self.img_v_max)

        # ---- adaptive sizes based on image resolution ----
        base = max(1, self.img_width / 640)
        self.line_thin = max(1, int(base))
        self.line_main = max(2, int(2 * base))
        self.r_point = max(4, int(self.img_width / 160))
        self.r_center = max(3, int(self.img_width / 320))
        self.marker_gap = max(3, int(round(1.25 * self.r_point)))
        self.marker_arm = max(3, int(round(1.0 * self.r_point)))

        # ---- low-saturation BGR palette ----
        self.color_fov_normal = (180, 180, 180)    # light gray
        self.color_fov_warn = (40, 170, 230)        # amber orange
        self.color_current_los = (245, 245, 245)    # bright white LOS marker
        self.color_los_line = (190, 255, 170)       # pale green center-to-LOS line
        self.color_bbox = (60, 220, 80)              # YOLO detection box
        self.draw_virtual_balloon = rospy.get_param(
            "~draw_virtual_balloon", False)
        self.color_roi = (126, 190, 144)             # muted jade green
        self.color_ekf_prediction_dim = (135, 135, 135)
        self.color_ekf_prediction = (235, 235, 235)   # white EKF trail head
        self.color_roi_prediction = (65, 65, 65)       # dark gray future lead
        self.color_oob = (40, 170, 230)             # amber orange (out-of-bounds)
        self.color_hud = (235, 235, 235)
        self.color_hud_dim = (175, 175, 175)
        self.color_hud_accent = (230, 190, 95)

        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1
        )
        self.raw_image_sub = rospy.Subscriber(
            self.raw_image_topic, Image, self.raw_image_callback, queue_size=1
        )
        self.compressed_image_sub = None
        if self.compressed_image_topic:
            self.compressed_image_sub = rospy.Subscriber(
                self.compressed_image_topic, CompressedImage,
                self.compressed_image_callback, queue_size=1
            )
        self.bbox_sub = rospy.Subscriber(
            self.bbox_topic, BoundingBoxes, self.bbox_callback, queue_size=10
        )
        self.yolo_bbox_sub = rospy.Subscriber(
            self.yolo_bbox_topic, BoundingBoxes, self.yolo_bbox_callback,
            queue_size=10
        )
        self.intercepted_sub = rospy.Subscriber(
            "/balloons/intercepted", Bool, self.intercepted_callback,
            queue_size=1
        )
        self.roi_prediction_sub = rospy.Subscriber(
            self.roi_prediction_topic,
            BoundingBoxes,
            self.roi_prediction_callback,
            queue_size=10,
        )
        self.q_image_sub = rospy.Subscriber(
            self.q_image_topic, QuaternionStamped, self.q_image_callback, queue_size=10
        )
        self.odometry_sub = rospy.Subscriber(
            self.odometry_topic, Odometry, self.odometry_callback, queue_size=10
        )
        self.detection_mode_sub = rospy.Subscriber(
            self.detection_mode_topic,
            String,
            self.detection_mode_callback,
            queue_size=10,
        )
        self.roi_sub = rospy.Subscriber(
            self.roi_topic, RegionOfInterest, self.roi_callback, queue_size=1
        )
        self.fsm_state_sub = rospy.Subscriber(
            self.fsm_state_topic, FsmState, self.fsm_state_callback,
            queue_size=10
        )
        self.mpc_latency_sub = rospy.Subscriber(
            self.mpc_latency_topic, Float32, self.mpc_latency_callback,
            queue_size=10
        )

        rospy.loginfo(
            "fpv_los_overlay: image=%s output=%s bbox=%s roi_prediction=%s "
            "q_image=%s "
            "ekf_trail=enabled img=%dx%d f=%.3f bounds u=[%.3f %.3f] "
            "v=[%.3f %.3f] alpha=%.2f",
            self.image_topic,
            self.output_topic,
            self.bbox_topic,
            self.roi_prediction_topic,
            self.q_image_topic,
            self.img_width,
            self.img_height,
            self.img_f,
            self.img_u_min,
            self.img_u_max,
            self.img_v_min,
            self.img_v_max,
            self.alpha,
        )

    # ------------------------------------------------------------------
    #  helpers
    # ------------------------------------------------------------------

    def _get_optional_bound(self, name, default):
        value = rospy.get_param(name, None)
        if value is None:
            return float(default)
        return float(value)

    @staticmethod
    def append_timed(history, stamp, value):
        """Append a sample, starting a new history after a replay seek."""
        if history and stamp < history[-1][0]:
            history.clear()
        history.append((stamp, value))

    @staticmethod
    def sample_at(history, stamp, default=None):
        """Return the newest sample that is not newer than the image."""
        for sample_stamp, value in reversed(history):
            if sample_stamp <= stamp:
                return value, sample_stamp
        return default, rospy.Time(0)

    @staticmethod
    def smooth_frequency(previous, instantaneous):
        if not math.isfinite(instantaneous) or instantaneous <= 0.0:
            return previous
        if previous is None or not math.isfinite(previous):
            return instantaneous
        return 0.85 * previous + 0.15 * instantaneous

    def update_stream_frequencies(self, wall_now, ros_stamp, last_wall,
                                  last_ros, wall_frequency,
                                  ros_frequency, ros_sample_count=1.0):
        """Return EWMA rates in wall time and message/ROS time.

        ros_sample_count compensates for skipped messages when Header.seq is
        available, so replay load does not under-report the recorded source
        rate.
        """
        if last_wall is not None:
            wall_dt = wall_now - last_wall
            if wall_dt > 1e-6:
                wall_frequency = self.smooth_frequency(
                    wall_frequency, 1.0 / wall_dt
                )
        if ros_stamp != rospy.Time(0) and last_ros != rospy.Time(0):
            ros_dt = (ros_stamp - last_ros).to_sec()
            # A negative delta is a new simulation clock epoch. A large
            # positive delta is a stream interruption, not a useful rate.
            if 1e-6 < ros_dt < 5.0:
                ros_frequency = self.smooth_frequency(
                    ros_frequency, ros_sample_count / ros_dt
                )
        return wall_frequency, ros_frequency, wall_now, ros_stamp

    def update_realtime_factor(self, wall_now, ros_now):
        if not self.use_sim_time:
            return
        if (self.last_clock_wall_time is not None and
                self.last_clock_ros_time != rospy.Time(0)):
            wall_dt = wall_now - self.last_clock_wall_time
            ros_dt = (ros_now - self.last_clock_ros_time).to_sec()
            if wall_dt > 1e-6 and 0.0 <= ros_dt < 5.0:
                instantaneous = ros_dt / wall_dt
                if math.isfinite(instantaneous) and instantaneous <= 10.0:
                    self.latest_realtime_factor = self.smooth_frequency(
                        self.latest_realtime_factor, instantaneous
                    )
        self.last_clock_wall_time = wall_now
        self.last_clock_ros_time = ros_now

    @staticmethod
    def clamp_px(value, low, high):
        return max(low, min(high, float(value)))

    @staticmethod
    def quaternion_to_ray(q):
        w, x, y, z = q
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if not math.isfinite(norm) or norm < 1e-9:
            return np.array([0.0, 0.0, 1.0], dtype=np.float64)
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        return np.array(
            [
                2.0 * (x * z + w * y),
                2.0 * (y * z - w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
            dtype=np.float64,
        )

    def ray_to_image_point(self, ray):
        """Convert a 3-D LOS ray to pixel coordinates.

        Returns
        -------
        (u_clamped, v_clamped) : tuple of int
            Pixel position, clamped to image bounds.
        in_image : bool
            True if the true projection lies inside the image.
        """
        if ray is None or len(ray) != 3 or not np.all(np.isfinite(ray)):
            return None, False
        if ray[2] <= 1e-6:
            return None, False
        img_u = float(ray[0] / ray[2])
        img_v = float(ray[1] / ray[2])
        u = self.cx + self.img_f * img_u
        v = self.cy + self.img_f * img_v
        in_image = 0.0 <= u < float(self.img_width) and 0.0 <= v < float(self.img_height)
        u_clamped = self.clamp_px(u, 0, self.img_width - 1)
        v_clamped = self.clamp_px(v, 0, self.img_height - 1)
        return (int(round(u_clamped)), int(round(v_clamped))), in_image

    # ------------------------------------------------------------------
    #  drawing primitives
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    #  HUD elements (all draw onto an overlay)
    # ------------------------------------------------------------------

    def draw_bounds(self, overlay):
        """FOV boundary rectangle."""
        h, w = overlay.shape[:2]
        u_min = int(round(self.clamp_px(
            self.cx + self.img_f * self.img_u_min, 0, w - 1)))
        u_max = int(round(self.clamp_px(
            self.cx + self.img_f * self.img_u_max, 0, w - 1)))
        v_min = int(round(self.clamp_px(
            self.cy + self.img_f * self.img_v_min, 0, h - 1)))
        v_max = int(round(self.clamp_px(
            self.cy + self.img_f * self.img_v_max, 0, h - 1)))
        x1, y1 = min(u_min, u_max), min(v_min, v_max)
        x2, y2 = max(u_min, u_max), max(v_min, v_max)
        cv2.rectangle(overlay, (x1, y1), (x2, y2),
                      self._fov_color, thickness=self.line_thin,
                      lineType=cv2.LINE_AA)

    def draw_los_marker(self, overlay, center, color, compact=False):
        """Compact LOS marker made from a center dot and four corner strokes."""
        cx, cy = center
        gap = max(2, self.marker_gap // 2) if compact else self.marker_gap
        arm = max(2, self.marker_arm // 2) if compact else self.marker_arm
        thickness = self.line_thin

        cv2.circle(
            overlay, center, max(1, self.r_center // 2), color,
            thickness=-1, lineType=cv2.LINE_AA
        )
        for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
            corner = (cx + sx * gap, cy + sy * gap)
            cv2.line(
                overlay, corner, (corner[0] + sx * arm, corner[1]),
                color, thickness=thickness, lineType=cv2.LINE_AA
            )
            cv2.line(
                overlay, corner, (corner[0], corner[1] + sy * arm),
                color, thickness=thickness, lineType=cv2.LINE_AA
            )

    def draw_current_los(self, overlay, point, in_image):
        """Draw the solved LOS from the camera principal point to its projection."""
        center = (int(round(self.cx)), int(round(self.cy)))
        cv2.line(
            overlay, center, point, self.color_los_line,
            thickness=self.line_main, lineType=cv2.LINE_AA
        )
        color = self.color_current_los if in_image else self.color_oob
        self.draw_los_marker(overlay, point, color, compact=not in_image)

    def draw_bboxes(self, overlay, bboxes):
        h, w = overlay.shape[:2]
        for box in bboxes:
            x1 = int(round(self.clamp_px(box.xmin, 0, w - 1)))
            y1 = int(round(self.clamp_px(box.ymin, 0, h - 1)))
            x2 = int(round(self.clamp_px(box.xmax, 0, w - 1)))
            y2 = int(round(self.clamp_px(box.ymax, 0, h - 1)))
            if x2 <= x1 or y2 <= y1:
                continue
            cv2.rectangle(
                overlay, (x1, y1), (x2, y2), self.color_bbox,
                thickness=self.line_main, lineType=cv2.LINE_AA
            )
            if self.draw_virtual_balloon:
                center = ((x1 + x2) // 2, (y1 + y2) // 2)
                radius = max(3, int(round(0.5 * max(x2 - x1, y2 - y1))))
                balloon = overlay.copy()
                cv2.circle(balloon, center, radius, (0, 0, 255), cv2.FILLED,
                           cv2.LINE_AA)
                cv2.addWeighted(balloon, 0.45, overlay, 0.55, 0.0, overlay)
            label = "{:.2f}".format(float(box.probability))
            text_origin = (x1, max(12, y1 - 6))
            cv2.putText(
                overlay, label, text_origin, cv2.FONT_HERSHEY_SIMPLEX,
                0.45, self.color_bbox, thickness=self.line_thin,
                lineType=cv2.LINE_AA
            )

    def draw_roi(self, overlay, roi):
        """Actual image region processed by the detector on its latest frame."""
        h, w = overlay.shape[:2]
        if roi is None or roi.width <= 0 or roi.height <= 0:
            return
        x1 = int(round(self.clamp_px(roi.x_offset, 0, w - 1)))
        y1 = int(round(self.clamp_px(roi.y_offset, 0, h - 1)))
        x2 = int(round(self.clamp_px(roi.x_offset + roi.width - 1, 0, w - 1)))
        y2 = int(round(self.clamp_px(roi.y_offset + roi.height - 1, 0, h - 1)))
        if x2 <= x1 or y2 <= y1:
            return
        cv2.rectangle(
            overlay, (x1, y1), (x2, y2), self.color_roi,
            thickness=self.line_thin, lineType=cv2.LINE_AA
        )

    def draw_ekf_prediction_trail(self, overlay, prediction):
        """Draw a restrained gray-to-white EKF prediction trail."""
        h, w = overlay.shape[:2]
        previous = None
        point_count = max(1, len(prediction))
        for index, (_, u, v) in enumerate(prediction):
            if not (math.isfinite(u) and math.isfinite(v)):
                previous = None
                continue
            in_image = 0.0 <= u < float(w) and 0.0 <= v < float(h)
            if not in_image:
                previous = None
                continue

            point = (int(round(u)), int(round(v)))
            progress = float(index + 1) / point_count
            color = tuple(
                int(round(dim + progress * (bright - dim)))
                for dim, bright in zip(
                    self.color_ekf_prediction_dim,
                    self.color_ekf_prediction,
                )
            )
            if previous is not None:
                cv2.line(
                    overlay, previous, point, color,
                    thickness=self.line_thin, lineType=cv2.LINE_AA
                )
            # Sparse, increasingly bright nodes give the trail depth without
            # turning it into a heavy dotted line.
            if index == point_count - 1 or index % 3 == 0:
                radius = max(1, self.r_center // 2)
                cv2.circle(
                    overlay, point, radius, color, thickness=-1,
                    lineType=cv2.LINE_AA
                )
            previous = point

    def draw_roi_prediction_lead(self, overlay, current_box, prediction_box):
        """Connect the current filtered center to the future ROI prediction."""
        if current_box is None or prediction_box is None:
            return
        h, w = overlay.shape[:2]
        current_u = 0.5 * (float(current_box.xmin) + float(current_box.xmax))
        current_v = 0.5 * (float(current_box.ymin) + float(current_box.ymax))
        future_u = 0.5 * (
            float(prediction_box.xmin) + float(prediction_box.xmax)
        )
        future_v = 0.5 * (
            float(prediction_box.ymin) + float(prediction_box.ymax)
        )
        if not all(math.isfinite(value) for value in
                   (current_u, current_v, future_u, future_v)):
            return
        if not (0.0 <= current_u < w and 0.0 <= current_v < h and
                0.0 <= future_u < w and 0.0 <= future_v < h):
            return
        current = (int(round(current_u)), int(round(current_v)))
        future = (int(round(future_u)), int(round(future_v)))
        cv2.line(
            overlay, current, future, self.color_roi_prediction,
            thickness=self.line_main, lineType=cv2.LINE_AA
        )
        cv2.circle(
            overlay, future, max(2, self.r_center // 2),
            self.color_roi_prediction, thickness=-1, lineType=cv2.LINE_AA
        )

    def draw_telemetry_hud(self, image, odometry, bboxes, bbox_fresh,
                           raw_bboxes, raw_bbox_fresh, los_fresh, current_in,
                           detection_mode, elapsed_s,
                           fsm_state, mpc_latency_ms, yolo_frequency_hz,
                           yolo_sim_frequency_hz, raw_frequency_hz,
                           raw_sim_frequency_hz, image_fps, image_sim_fps,
                           use_sim_time, realtime_factor, environment,
                           replay_mode):
        """Bottom telemetry band inspired by flight-test broadcast graphics."""
        h, w = image.shape[:2]
        scale = max(0.65, min(2.4, w / 1280.0))
        band_h = max(104, int(round(142 * scale)))
        y0 = max(0, h - band_h)

        shade = image.copy()
        cv2.rectangle(shade, (0, y0), (w, h), (18, 20, 23), thickness=-1)
        cv2.addWeighted(shade, 0.68, image, 0.32, 0.0, dst=image)
        cv2.line(image, (0, y0), (w, y0), (130, 130, 130),
                 thickness=max(1, int(round(scale))), lineType=cv2.LINE_AA)

        font = cv2.FONT_HERSHEY_DUPLEX
        label_scale = 0.42 * scale
        value_scale = 0.62 * scale
        thin = max(1, int(round(scale)))
        strong = max(1, int(round(1.4 * scale)))
        pad = int(round(28 * scale))
        line_gap = int(round(30 * scale))
        value_dx = int(round(100 * scale))
        rates_value_dx = int(round(82 * scale))

        speed = None
        altitude = None
        if odometry is not None:
            v = odometry.twist.twist.linear
            speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            altitude = odometry.pose.pose.position.z

        def text(value, point, size, color, thickness=thin):
            cv2.putText(image, value, point, font, size, color, thickness,
                        lineType=cv2.LINE_AA)

        def metric_value(value, unit):
            if value is None or not math.isfinite(value):
                return "--"
            return "{:.1f} {}".format(value, unit)

        # Four-column, three-row grid.
        left_y = y0 + int(round(35 * scale))
        row_y = (left_y, left_y + line_gap, left_y + 2 * line_gap)
        left_x = pad
        center_x = int(round(w * 0.28))
        right_x = int(round(w * 0.59))
        rates_x = int(round(w * 0.79))

        # Column 1: vehicle telemetry and MPC computation time.
        text("SPEED", (left_x, row_y[0]), label_scale, self.color_hud_dim)
        text("--" if speed is None else "{:.1f} M/S".format(speed),
             (left_x + value_dx, row_y[0]), value_scale,
             self.color_hud, strong)
        text("ALT", (left_x, row_y[1]), label_scale, self.color_hud_dim)
        text("--" if altitude is None else "{:.1f} M".format(altitude),
             (left_x + value_dx, row_y[1]), value_scale,
             self.color_hud, strong)
        text("MPC", (left_x, row_y[2]), label_scale, self.color_hud_dim)
        text(metric_value(mpc_latency_ms, "MS"),
             (left_x + value_dx, row_y[2]), value_scale,
             self.color_hud, strong)

        # Columns 2-3 share the first row for the mission clock. Their lower
        # rows contain vision/FSM and target/guidance respectively.
        elapsed_s = max(0.0, elapsed_s)
        minutes = int(elapsed_s // 60.0)
        seconds = elapsed_s - 60.0 * minutes
        if use_sim_time:
            mode = (
                "{} --X".format(environment) if realtime_factor is None
                else "{} {:.2f}X".format(environment, realtime_factor)
            )
        else:
            mode = environment
        clock = "{}  T+{:02d}:{:04.1f}".format(mode, minutes, seconds)
        clock_size = cv2.getTextSize(clock, font, value_scale, strong)[0]
        shared_center_x = w // 2
        clock_x = shared_center_x - clock_size[0] // 2
        text(clock, (clock_x, row_y[0]), value_scale,
             self.color_hud, strong)
        text("VISION", (center_x, row_y[1]), label_scale,
             self.color_hud_dim)
        text(detection_mode or "--", (center_x + value_dx, row_y[1]),
             value_scale, self.color_hud, strong)
        text("FSM", (center_x, row_y[2]), label_scale, self.color_hud_dim)
        text(fsm_state or "--", (center_x + value_dx, row_y[2]),
             value_scale, self.color_hud, strong)

        # Column 3 lower rows: target and LOS health.
        valid_boxes = bboxes if bbox_fresh else []
        valid_raw_boxes = raw_bboxes if raw_bbox_fresh else []
        raw_confidence = max(
            [float(box.probability) for box in valid_raw_boxes], default=None
        )
        filtered_confidence = max(
            [float(box.probability) for box in valid_boxes], default=None
        )
        target_value = "NO LOCK"
        target_color = self.color_oob
        # Detection strategy and target health are independent. A filtered
        # box without a fresh raw detector box is an estimator coast, not a
        # visual lock and not itself a reacquisition state.
        if raw_confidence is not None:
            target_value = "LOCK  {:3.0f}%".format(100.0 * raw_confidence)
            target_color = self.color_hud_accent
        elif filtered_confidence is not None:
            target_value = "COAST {:3.0f}%".format(
                100.0 * filtered_confidence
            )
            target_color = self.color_oob
        los_value = "LOS OK" if los_fresh and current_in else "LOS LOST"
        los_color = self.color_hud if los_fresh and current_in else self.color_oob
        text("TARGET", (right_x, row_y[1]), label_scale, self.color_hud_dim)
        text(target_value, (right_x + value_dx, row_y[1]), value_scale,
             target_color, strong)
        text("GUIDANCE", (right_x, row_y[2]), label_scale,
             self.color_hud_dim)
        text(los_value, (right_x + value_dx, row_y[2]), value_scale,
             los_color, strong)

        def rate_value(wall_rate, sim_rate, unit):
            if wall_rate is None or not math.isfinite(wall_rate):
                return "--"
            if replay_mode:
                if sim_rate is None or not math.isfinite(sim_rate):
                    return "{:.1f}/--".format(wall_rate)
                return "{:.1f}/{:.1f}".format(wall_rate, sim_rate)
            if use_sim_time:
                if sim_rate is None or not math.isfinite(sim_rate):
                    return "{:.1f}W/--S".format(wall_rate)
                return "{:.1f}W/{:.1f}S".format(wall_rate, sim_rate)
            return "{:.1f} {}".format(wall_rate, unit)

        # Column 4: camera, detector, and overlay-render rates. Replay uses
        # PLAY/REC (delivery wall time / recorded message time); simulation
        # uses W/S, and live experiments display wall-clock rate only.
        rate_scale = (0.48 if use_sim_time else 0.62) * scale
        for row, label, wall_value, sim_value, unit in (
                (0, "RAW P/R" if replay_mode else "RAW",
                 raw_frequency_hz, raw_sim_frequency_hz, "HZ"),
                (1, "YOLO P/R" if replay_mode else "YOLO", yolo_frequency_hz,
                 yolo_sim_frequency_hz, "HZ"),
                (2, "HUD P/R" if replay_mode else "HUD",
                 image_fps, image_sim_fps, "FPS")):
            text(label, (rates_x, row_y[row]), label_scale,
                 self.color_hud_dim)
            text(rate_value(wall_value, sim_value, unit),
                 (rates_x + rates_value_dx, row_y[row]), rate_scale,
                 self.color_hud, strong)

    # ------------------------------------------------------------------
    #  ROS callbacks
    # ------------------------------------------------------------------

    def q_image_callback(self, msg):
        stamp = (
            msg.header.stamp
            if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        )
        value = (
            float(msg.quaternion.w),
            float(msg.quaternion.x),
            float(msg.quaternion.y),
            float(msg.quaternion.z),
        )
        with self.lock:
            self.latest_q_image = value
            self.latest_q_image_stamp = stamp
            self.append_timed(self.q_image_history, stamp, value)

    def bbox_callback(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        with self.lock:
            bboxes = list(msg.bounding_boxes)
            self.append_timed(self.bbox_history, stamp, bboxes)
            if bboxes:
                self.latest_bboxes = bboxes
                self.latest_bboxes_stamp = stamp
                box = bboxes[0]
                u = 0.5 * (float(box.xmin) + float(box.xmax))
                v = 0.5 * (float(box.ymin) + float(box.ymax))
                if math.isfinite(u) and math.isfinite(v):
                    if (self.ekf_prediction_trail and
                            stamp < self.ekf_prediction_trail[-1][0]):
                        self.ekf_prediction_trail.clear()
                    self.ekf_prediction_trail.append((stamp, u, v))
                    cutoff = stamp - rospy.Duration(self.ekf_trail_duration)
                    while (self.ekf_prediction_trail and
                           self.ekf_prediction_trail[0][0] < cutoff):
                        self.ekf_prediction_trail.popleft()
                    while (len(self.ekf_prediction_trail) >
                           self.ekf_trail_max_points):
                        self.ekf_prediction_trail.popleft()
                    self.latest_ekf_prediction_stamp = stamp

    def intercepted_callback(self, msg):
        stamp = rospy.Time.now()
        with self.lock:
            self.target_intercepted = bool(msg.data)
            self.append_timed(
                self.target_intercepted_history, stamp,
                self.target_intercepted
            )
            if self.target_intercepted:
                self.latest_bboxes = []
                self.latest_bboxes_stamp = rospy.Time(0)
                self.latest_yolo_bboxes = []
                self.latest_yolo_bboxes_stamp = rospy.Time(0)
                self.append_timed(self.bbox_history, stamp, [])
                self.append_timed(self.yolo_bbox_history, stamp, [])
                self.ekf_prediction_trail.clear()

    def yolo_bbox_callback(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        with self.lock:
            self.latest_yolo_bboxes = list(msg.bounding_boxes)
            self.latest_yolo_bboxes_stamp = stamp
            self.append_timed(
                self.yolo_bbox_history, stamp,
                list(self.latest_yolo_bboxes)
            )
        wall_now = time.monotonic()
        with self.lock:
            seq = int(msg.header.seq)
            ros_sample_count = 1.0
            if (self.last_yolo_seq is not None and
                    seq > self.last_yolo_seq and
                    seq - self.last_yolo_seq < 1000):
                ros_sample_count = float(seq - self.last_yolo_seq)
            self.last_yolo_seq = seq
            (self.latest_yolo_frequency_hz,
             self.latest_yolo_sim_frequency_hz,
             self.last_yolo_wall_time,
             self.last_yolo_ros_time) = self.update_stream_frequencies(
                wall_now, stamp, self.last_yolo_wall_time,
                self.last_yolo_ros_time, self.latest_yolo_frequency_hz,
                self.latest_yolo_sim_frequency_hz, ros_sample_count
            )
            self.latest_yolo_frequency_stamp = rospy.Time.now()

    def roi_prediction_callback(self, msg):
        with self.lock:
            self.latest_roi_prediction = (
                msg.bounding_boxes[0] if msg.bounding_boxes else None
            )
            self.latest_roi_prediction_receive_stamp = rospy.Time.now()

    def odometry_callback(self, msg):
        stamp = (
            msg.header.stamp
            if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        )
        with self.lock:
            self.latest_odometry = msg
            self.append_timed(self.odometry_history, stamp, msg)

    def detection_mode_callback(self, msg):
        stamp = rospy.Time.now()
        value = msg.data.strip() or "--"
        with self.lock:
            self.latest_detection_mode = value
            self.append_timed(self.detection_mode_history, stamp, value)

    def roi_callback(self, msg):
        with self.lock:
            self.latest_roi = msg
            self.latest_roi_stamp = rospy.Time.now()

    def fsm_state_callback(self, msg):
        stamp = (
            msg.header.stamp
            if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        )
        value = msg.state_name.strip() or "--"
        with self.lock:
            self.latest_fsm_state = value
            self.latest_fsm_state_stamp = stamp
            self.append_timed(self.fsm_state_history, stamp, value)

    def mpc_latency_callback(self, msg):
        value = float(msg.data)
        stamp = rospy.Time.now()
        with self.lock:
            self.latest_mpc_latency_ms = value if math.isfinite(value) else None
            self.latest_mpc_latency_stamp = stamp
            self.append_timed(
                self.mpc_latency_history, stamp,
                self.latest_mpc_latency_ms
            )

    def raw_image_callback(self, msg):
        wall_now = time.monotonic()
        stamp = (
            msg.header.stamp
            if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        )
        with self.lock:
            seq = int(msg.header.seq)
            ros_sample_count = 1.0
            if (self.last_raw_seq is not None and
                    seq > self.last_raw_seq and
                    seq - self.last_raw_seq < 1000):
                ros_sample_count = float(seq - self.last_raw_seq)
            self.last_raw_seq = seq
            (self.latest_raw_frequency_hz,
             self.latest_raw_sim_frequency_hz,
             self.last_raw_wall_time,
             self.last_raw_ros_time) = self.update_stream_frequencies(
                wall_now, stamp, self.last_raw_wall_time,
                self.last_raw_ros_time, self.latest_raw_frequency_hz,
                self.latest_raw_sim_frequency_hz, ros_sample_count
            )
            self.latest_raw_frequency_stamp = rospy.Time.now()

    def image_callback(self, msg):
        now = rospy.Time.now()
        image_stamp = (
            msg.header.stamp
            if msg.header.stamp != rospy.Time(0) else now
        )
        # rqt_bag can seek backwards.  Cached detections from the previous
        # playhead then look stale (or future-dated) and suppress the FPV
        # virtual balloon on the next replay.
        with self.lock:
            if (self.last_render_ros_time != rospy.Time(0) and
                    image_stamp < self.last_render_ros_time):
                self.latest_bboxes = []
                self.latest_bboxes_stamp = rospy.Time(0)
                self.latest_yolo_bboxes = []
                self.latest_yolo_bboxes_stamp = rospy.Time(0)
                self.target_intercepted = False
                self.ekf_prediction_trail.clear()
                self.latest_ekf_prediction_stamp = rospy.Time(0)
                self.start_time = image_stamp
            self.last_render_ros_time = image_stamp
            if self.start_time == rospy.Time(0):
                self.start_time = image_stamp
        if self.max_output_rate_hz > 0.0 and self.last_output_time != rospy.Time(0):
            elapsed = (now - self.last_output_time).to_sec()
            if 0.0 <= elapsed < 1.0 / self.max_output_rate_hz:
                return
        self.last_output_time = now

        wall_now = time.monotonic()
        with self.lock:
            (self.image_fps, self.image_sim_fps,
             self.last_image_wall_time,
             self.last_image_ros_time) = self.update_stream_frequencies(
                wall_now, image_stamp, self.last_image_wall_time,
                self.last_image_ros_time, self.image_fps,
                self.image_sim_fps
            )
            self.update_realtime_factor(wall_now, now)

        image = self.ros_image_to_cv(msg)
        if image is None:
            return

        with self.lock:
            q_image, q_stamp = self.sample_at(
                self.q_image_history, image_stamp
            )
            q_age = (
                float("inf")
                if q_stamp == rospy.Time(0)
                else (image_stamp - q_stamp).to_sec()
            )
            selected_bboxes, bbox_stamp = self.sample_at(
                self.bbox_history, image_stamp, []
            )
            bboxes = list(selected_bboxes)
            target_intercepted, _ = self.sample_at(
                self.target_intercepted_history, image_stamp, False
            )
            if target_intercepted:
                bboxes = []
            bbox_age = (
                float("inf")
                if bbox_stamp == rospy.Time(0)
                else (image_stamp - bbox_stamp).to_sec()
            )
            selected_yolo_bboxes, yolo_stamp = self.sample_at(
                self.yolo_bbox_history, image_stamp, []
            )
            raw_bboxes = list(selected_yolo_bboxes)
            yolo_age = (
                float("inf")
                if yolo_stamp == rospy.Time(0)
                else (image_stamp - yolo_stamp).to_sec()
            )
            # HIL publishes synthetic detections on the raw topic and does
            # not run the bbox filter. Fall back to the raw boxes for display.
            if not bboxes or not 0.0 <= bbox_age <= self.bbox_timeout:
                if raw_bboxes and 0.0 <= yolo_age <= self.bbox_timeout:
                    bboxes = list(raw_bboxes)
                    bbox_age = yolo_age
            roi_prediction = self.latest_roi_prediction
            roi_prediction_age = (
                float("inf")
                if self.latest_roi_prediction_receive_stamp == rospy.Time(0)
                else (
                    now - self.latest_roi_prediction_receive_stamp
                ).to_sec()
            )
            ekf_prediction = [
                (index, sample[1], sample[2])
                for index, sample in enumerate(self.ekf_prediction_trail)
            ]
            ekf_prediction_age = (
                float("inf")
                if self.latest_ekf_prediction_stamp == rospy.Time(0)
                else (now - self.latest_ekf_prediction_stamp).to_sec()
            )
            odometry, _ = self.sample_at(
                self.odometry_history, image_stamp
            )
            detection_mode, _ = self.sample_at(
                self.detection_mode_history, image_stamp, "--"
            )
            metrics_timeout = self.metrics_timeout
            fsm_state, _ = self.sample_at(
                self.fsm_state_history, image_stamp, "--"
            )
            selected_mpc_latency, mpc_latency_stamp = self.sample_at(
                self.mpc_latency_history, image_stamp
            )
            mpc_latency_age = (
                float("inf")
                if mpc_latency_stamp == rospy.Time(0)
                else (image_stamp - mpc_latency_stamp).to_sec()
            )
            mpc_latency_ms = (
                selected_mpc_latency
                if 0.0 <= mpc_latency_age <= metrics_timeout
                else None
            )
            yolo_frequency_fresh = (
                self.latest_yolo_frequency_stamp != rospy.Time(0)
                and 0.0 <= (now - self.latest_yolo_frequency_stamp).to_sec()
                <= metrics_timeout
            )
            yolo_frequency_hz = (
                self.latest_yolo_frequency_hz
                if yolo_frequency_fresh else None
            )
            yolo_sim_frequency_hz = (
                self.latest_yolo_sim_frequency_hz
                if yolo_frequency_fresh else None
            )
            raw_frequency_fresh = (
                self.latest_raw_frequency_stamp != rospy.Time(0)
                and 0.0 <= (now - self.latest_raw_frequency_stamp).to_sec()
                <= metrics_timeout
            )
            raw_frequency_hz = (
                self.latest_raw_frequency_hz
                if raw_frequency_fresh else None
            )
            raw_sim_frequency_hz = (
                self.latest_raw_sim_frequency_hz
                if raw_frequency_fresh else None
            )
            image_fps = self.image_fps
            image_sim_fps = self.image_sim_fps
            realtime_factor = self.latest_realtime_factor
            roi = self.latest_roi
            roi_age = (
                float("inf")
                if self.latest_roi_stamp == rospy.Time(0)
                else (now - self.latest_roi_stamp).to_sec()
            )

        # ---- compute pixel positions ----
        current_pt = None
        current_in = False

        if q_image is not None and 0.0 <= q_age <= self.los_timeout:
            current_ray = self.quaternion_to_ray(q_image)
            current_pt, current_in = self.ray_to_image_point(current_ray)

        # ---- FOV colour switches to amber when current LOS is out-of-bounds ----
        self._fov_color = (
            self.color_fov_warn
            if (current_pt is not None and not current_in)
            else self.color_fov_normal
        )

        # ---- draw everything on a clean overlay, then blend ----
        overlay = image.copy()

        if roi is not None and roi_age <= self.roi_timeout:
            self.draw_roi(overlay, roi)

        # During bag replay, the playhead can pause or jump while the last
        # valid detection remains the best visual reference.  Do not hide the
        # virtual balloon merely because the real-time timeout elapsed.
        bbox_visible = (
            bool(bboxes) and
            (0.0 <= bbox_age <= self.bbox_timeout or
             self.draw_virtual_balloon)
        )
        raw_bbox_fresh = (
            bool(raw_bboxes) and 0.0 <= yolo_age <= self.bbox_timeout
        )

        if (ekf_prediction and
                ekf_prediction_age <= self.ekf_trail_duration):
            self.draw_ekf_prediction_trail(overlay, ekf_prediction)

        if (bbox_visible and
                roi_prediction is not None and
                0.0 <= roi_prediction_age <= self.roi_prediction_timeout):
            self.draw_roi_prediction_lead(
                overlay, bboxes[0], roi_prediction
            )

        self.draw_bounds(overlay)

        blended = cv2.addWeighted(image, 1.0 - self.alpha,
                                  overlay, self.alpha, 0.0)
        if self.show_telemetry_hud:
            self.draw_telemetry_hud(
                blended,
                odometry,
                bboxes,
                bbox_visible,
                raw_bboxes,
                raw_bbox_fresh,
                bool(current_pt is not None and
                     0.0 <= q_age <= self.los_timeout),
                current_in,
                detection_mode,
                (image_stamp - self.start_time).to_sec(),
                fsm_state,
                mpc_latency_ms,
                yolo_frequency_hz,
                yolo_sim_frequency_hz,
                raw_frequency_hz,
                raw_sim_frequency_hz,
                image_fps,
                image_sim_fps,
                self.use_sim_time,
                realtime_factor,
                self.hud_environment,
                self.replay_mode,
            )

        # Guidance diagnostics must remain visible even when the target is in
        # the lower part of the image.  Draw them after the telemetry band so
        # the HUD cannot darken or cover a small real-flight detection.
        if bbox_visible:
            self.draw_bboxes(blended, bboxes)
        if current_pt is not None:
            self.draw_current_los(blended, current_pt, current_in)

        out = None
        if self.output_width or self.output_height:
            target_width = self.output_width or int(
                blended.shape[1] * self.output_height / blended.shape[0])
            target_height = self.output_height or int(
                blended.shape[0] * self.output_width / blended.shape[1])
            resized = cv2.resize(blended, (target_width, target_height),
                                 interpolation=cv2.INTER_AREA)
            out = self.cv_to_ros_image(resized, msg)
        if out is None:
            return
        self.image_pub.publish(out)

    def compressed_image_callback(self, msg):
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            rospy.logwarn_throttle(1.0,
                                   "fpv_los_overlay: failed to decode compressed image")
            return
        raw = Image()
        raw.header = msg.header
        raw.height, raw.width = image.shape[:2]
        raw.encoding = "bgr8"
        raw.is_bigendian = False
        raw.step = raw.width * 3
        raw.data = image.tobytes()
        self.image_callback(raw)

    # ------------------------------------------------------------------
    #  ROS <-> numpy image conversion (unchanged logic)
    # ------------------------------------------------------------------

    def ros_image_to_cv(self, msg):
        if msg.encoding not in ("rgb8", "bgr8", "mono8"):
            rospy.logwarn_throttle(
                1.0, "fpv_los_overlay: unsupported image encoding '%s'",
                msg.encoding,
            )
            return None
        channels = 1 if msg.encoding == "mono8" else 3
        expected_min_step = int(msg.width) * channels
        if msg.step < expected_min_step:
            rospy.logwarn_throttle(
                1.0,
                "fpv_los_overlay: invalid image step=%d width=%d channels=%d",
                msg.step, msg.width, channels,
            )
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = int(msg.step) * int(msg.height)
        if raw.size < expected_size:
            rospy.logwarn_throttle(
                1.0,
                "fpv_los_overlay: image data too short, got=%d expected=%d",
                raw.size, expected_size,
            )
            return None
        rows = raw[:expected_size].reshape((int(msg.height), int(msg.step)))
        packed = rows[:, :expected_min_step]
        if channels == 1:
            image = packed.reshape((int(msg.height), int(msg.width))).copy()
            return np.repeat(image[:, :, None], 3, axis=2)
        image = packed.reshape((int(msg.height), int(msg.width), channels)).copy()
        if msg.encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    def cv_to_ros_image(self, image, source_msg):
        if image.ndim != 3 or image.shape[2] != 3:
            rospy.logwarn_throttle(
                1.0, "fpv_los_overlay: output image is not 3-channel")
            return None
        out = Image()
        out.header = source_msg.header
        out.height = int(image.shape[0])
        out.width = int(image.shape[1])
        if source_msg.encoding in ("rgb8", "mono8"):
            out.encoding = "rgb8"
            output_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            out.encoding = "bgr8"
            output_image = image
        out.is_bigendian = source_msg.is_bigendian
        out.step = int(image.shape[1]) * 3
        out.data = output_image.tobytes()
        return out


def main():
    rospy.init_node("fpv_los_overlay")
    FpvLosOverlay()
    rospy.spin()


if __name__ == "__main__":
    main()
