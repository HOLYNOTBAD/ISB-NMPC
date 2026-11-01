#!/usr/bin/env python3
"""实时发布 bbox 中心曲线，并计算滤波与 raw-hold 的滑动窗口 RMSE。

u/v 曲线保持各输入话题原有的发布频率，不做时间同步、插值或平滑。
RMSE 计算则需要把滤波输出与同一时刻的 HSV 真值对齐，因此会短暂缓存
HSV 数据，并只在滤波时刻被相邻两帧 HSV 真值包围后计算误差。
"""

from bisect import bisect_left, bisect_right
from collections import deque

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from swarm_msgs.msg import BoundingBoxes, MassPoints


def stamp_sec(stamp):
    """将 ROS 时间戳转换为秒。"""
    return stamp.to_sec() if stamp is not None else 0.0


def message_time(message, receipt_time, image=False):
    """读取消息时间；原始检测优先使用对应图像的采集时间。"""
    if image and hasattr(message, "image_header"):
        value = stamp_sec(message.image_header.stamp)
        if value > 0.0:
            return value
    if hasattr(message, "header"):
        value = stamp_sec(message.header.stamp)
        if value > 0.0:
            return value
    return receipt_time


def box_center(message):
    """返回置信度最高检测框的中心坐标 [u, v]。"""
    if not message.bounding_boxes:
        return None
    box = max(message.bounding_boxes,
              key=lambda candidate: float(candidate.probability))
    return np.array([(box.xmin + box.xmax) * 0.5,
                     (box.ymin + box.ymax) * 0.5], dtype=float)


def quat_to_matrix(quaternion):
    """把 geometry_msgs/Quaternion 转换为 body 到 world 的旋转矩阵。"""
    value = np.array([quaternion.x, quaternion.y, quaternion.z,
                      quaternion.w], dtype=float)
    norm = np.linalg.norm(value)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = value / norm
    return np.array([
        [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w),
         2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w), 1.0 - 2.0 * (x*x + z*z),
         2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w), 2.0 * (y*z + x*w),
         1.0 - 2.0 * (x*x + y*y)],
    ])


class LiveFilterEvaluator:
    """发布四路异步像素坐标，并实时计算两种中心误差 RMSE。"""

    def __init__(self):
        # 输入话题均可通过 ROS 私有参数覆盖。
        self.raw_topic = rospy.get_param("~raw_topic", "/tracker/bbox_raw")
        self.filtered_topic = rospy.get_param(
            "~filtered_topic", "/tracker/bbox")
        self.truth_topic = rospy.get_param(
            "~truth_topic", "/tracker/p_image_truth_hsv")
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose")
        self.gazebo_model_states_topic = rospy.get_param(
            "~gazebo_model_states_topic", "/gazebo/model_states")
        self.gazebo_vehicle_model = rospy.get_param(
            "~gazebo_vehicle_model", "holydrone")
        self.truth_position_topic = rospy.get_param(
            "~truth_position_topic", "/balloons/masspoint")

        # Intrinsics come from the running camera, not from the source YAML.
        width = float(rospy.get_param("/camera/img_width"))
        height = float(rospy.get_param("/camera/img_height"))
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/camera_info")
        self.fx = self.fy = self.cx = self.cy = 0.0
        self.k1 = self.k2 = self.p1 = self.p2 = self.k3 = 0.0
        self.have_camera_info = False
        r_cam_body = rospy.get_param("/camera/R_cam_body")
        if len(r_cam_body) != 9:
            raise rospy.ROSInitException(
                "/camera/R_cam_body must contain 9 values")
        # 配置定义为 camera RDF -> body FLU，即 R_bc。
        self.r_bc = np.asarray(r_cam_body, dtype=float).reshape(3, 3)
        self.p_bc = np.asarray(
            rospy.get_param("/camera/position_body_m", [0.0, 0.0, 0.0]),
            dtype=float)
        if self.p_bc.shape != (3,):
            raise rospy.ROSInitException(
                "/camera/position_body_m must contain 3 values")
        self.image_width = width
        self.image_height = height
        self.latest_pose = None
        self.have_gazebo_pose = False
        self.latest_targets = {}
        self.latest_reference_center = None

        # RMSE 使用最近 window_s 秒的数据。max_truth_gap_s 限制 HSV 插值
        # 两侧样本与滤波时刻的最大距离，防止跨越长时间丢帧计算错误。
        self.window_s = float(rospy.get_param("~window_s", 10.0))
        self.report_period_s = float(rospy.get_param("~report_period_s", 1.0))
        self.max_truth_gap_s = float(rospy.get_param(
            "~max_truth_gap_s", 0.15))
        self.buffer_s = float(rospy.get_param("~buffer_s", 2.0))

        # 缓存格式：
        # truth/raw: (消息时间, [u, v])
        # pending_filtered: (滤波输出时间, [u, v])
        # errors: (滤波输出时间, 滤波误差, raw-hold 误差)
        self.truth = deque()
        self.raw = deque()
        self.pending_filtered = deque()
        self.errors = deque()

        rospy.Subscriber(self.raw_topic, BoundingBoxes, self.raw_callback,
                         queue_size=100)
        rospy.Subscriber(self.filtered_topic, BoundingBoxes,
                         self.filtered_callback, queue_size=200)
        rospy.Subscriber(self.truth_topic, BoundingBoxes, self.truth_callback,
                         queue_size=100)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback,
                         queue_size=100)
        rospy.Subscriber(self.gazebo_model_states_topic, ModelStates,
                         self.gazebo_model_states_callback, queue_size=20)
        rospy.Subscriber(self.camera_info_topic, CameraInfo,
                         self.camera_info_callback, queue_size=1)
        rospy.Subscriber(self.truth_position_topic, MassPoints,
                         self.truth_position_callback, queue_size=100)

        # u/v 每收到一条源消息就立即发布，因此三路曲线频率互不影响。
        self.coordinate_pubs = {}
        for source in ("truth_hsv", "truth_pos", "filtered", "raw"):
            for coordinate in ("u", "v"):
                name = source + "_" + coordinate
                self.coordinate_pubs[name] = rospy.Publisher(
                    "/filter_eval/" + name, Float64, queue_size=10)

        # 第三张图只需要这两个滑动窗口 RMSE。
        self.filtered_rmse_pub = rospy.Publisher(
            "/filter_eval/filtered_rmse_px", Float64, queue_size=1)
        self.raw_hold_rmse_pub = rospy.Publisher(
            "/filter_eval/raw_hold_rmse_px", Float64, queue_size=1)

        self.report_timer = rospy.Timer(
            rospy.Duration(max(0.2, self.report_period_s)),
            self.report_callback)

        rospy.loginfo(
            "live_filter_eval: raw=%s filtered=%s hsv=%s position=%s "
            "pose=%s gazebo_model=%s window=%.1fs",
            self.raw_topic, self.filtered_topic, self.truth_topic,
            self.truth_position_topic, self.pose_topic,
            self.gazebo_vehicle_model, self.window_s)

    def publish_center(self, source, center):
        """把一个二维中心拆成 rqt_plot 可直接读取的两个 Float64 话题。"""
        self.coordinate_pubs[source + "_u"].publish(
            Float64(data=float(center[0])))
        self.coordinate_pubs[source + "_v"].publish(
            Float64(data=float(center[1])))

    def raw_callback(self, message):
        """直接发布原始框，同时保存它供 raw-hold 基线使用。"""
        receipt_time = rospy.Time.now().to_sec()
        center = box_center(message)
        if center is None:
            return
        capture_time = message_time(message, receipt_time, image=True)
        self.raw.append((capture_time, center))
        self.latest_reference_center = center
        self.publish_center("raw", center)
        self.trim_buffers(receipt_time)

    def filtered_callback(self, message):
        """直接发布滤波中心，并等待同一时刻的 HSV 真值到达后再评分。"""
        receipt_time = rospy.Time.now().to_sec()
        center = box_center(message)
        if center is None:
            return
        current_time = message_time(message, receipt_time)
        self.pending_filtered.append((current_time, center))
        self.publish_center("filtered", center)
        self.process_pending()
        self.trim_buffers(receipt_time)

    def truth_callback(self, message):
        """直接发布 HSV 中心，并按时间顺序缓存用于 RMSE 对齐。"""
        receipt_time = rospy.Time.now().to_sec()
        center = box_center(message)
        if center is None:
            return
        truth_time = message_time(message, receipt_time, image=True)
        # 时间回跳的数据会破坏二分查找，因此丢弃。
        if self.truth and truth_time < self.truth[-1][0]:
            return
        self.truth.append((truth_time, center))
        self.latest_reference_center = center
        self.publish_center("truth_hsv", center)
        self.process_pending()
        self.trim_buffers(receipt_time)

    def pose_callback(self, message):
        """Gazebo世界位姿不可用时，使用MAVROS局部位姿作为后备。"""
        if self.have_gazebo_pose:
            return
        position = message.pose.position
        self.latest_pose = (
            np.array([position.x, position.y, position.z], dtype=float),
            quat_to_matrix(message.pose.orientation))
        self.publish_position_projection()

    def camera_info_callback(self, message):
        """读取实际渲染相机发布的K和D。"""
        if message.width < 2 or message.height < 2:
            return
        self.image_width = float(message.width)
        self.image_height = float(message.height)
        self.fx = float(message.K[0])
        self.fy = float(message.K[4])
        self.cx = float(message.K[2])
        self.cy = float(message.K[5])
        distortion = list(message.D[:5])
        distortion.extend([0.0] * (5 - len(distortion)))
        self.k1, self.k2, self.p1, self.p2, self.k3 = (
            float(value) for value in distortion)
        self.have_camera_info = self.fx > 0.0 and self.fy > 0.0

    def gazebo_model_states_callback(self, message):
        """使用与气球位置同属Gazebo世界坐标系的机体真值位姿。"""
        try:
            index = message.name.index(self.gazebo_vehicle_model)
        except ValueError:
            rospy.logwarn_throttle(
                2.0, "live_filter_eval: Gazebo model '%s' not found",
                self.gazebo_vehicle_model)
            return
        pose = message.pose[index]
        position = pose.position
        self.latest_pose = (
            np.array([position.x, position.y, position.z], dtype=float),
            quat_to_matrix(pose.orientation))
        self.have_gazebo_pose = True
        self.publish_position_projection()

    def truth_position_callback(self, message):
        """缓存所有气球世界坐标；volume 用作仿真目标 ID。"""
        self.latest_targets = {
            int(point.volume): np.array(
                [point.position.x, point.position.y, point.position.z],
                dtype=float)
            for point in message.points
        }
        self.publish_position_projection()

    def publish_position_projection(self):
        """由目标世界位置和无人机位姿反算当前图像中的 u/v。"""
        if (not self.have_camera_info or self.latest_pose is None or
                not self.latest_targets):
            return
        vehicle_position, r_wb = self.latest_pose
        camera_position = vehicle_position + r_wb.dot(self.p_bc)
        r_wc = r_wb.dot(self.r_bc)
        candidates = []
        for target_id, point_w in self.latest_targets.items():
            point_c = r_wc.T.dot(point_w - camera_position)
            if point_c[2] <= 1e-4:
                continue
            x = point_c[0] / point_c[2]
            y = point_c[1] / point_c[2]
            radius2 = x*x + y*y
            radial = (1.0 + self.k1*radius2 +
                      self.k2*radius2*radius2 +
                      self.k3*radius2*radius2*radius2)
            x_distorted = (
                x*radial + 2.0*self.p1*x*y +
                self.p2*(radius2 + 2.0*x*x))
            y_distorted = (
                y*radial + self.p1*(radius2 + 2.0*y*y) +
                2.0*self.p2*x*y)
            center = np.array([
                self.fx*x_distorted + self.cx,
                self.fy*y_distorted + self.cy,
            ])
            if (0.0 <= center[0] < self.image_width and
                    0.0 <= center[1] < self.image_height):
                candidates.append((target_id, center))
        if not candidates:
            return
        if self.latest_reference_center is None:
            selected = min(candidates, key=lambda item: item[0])[1]
        else:
            selected = min(
                candidates,
                key=lambda item: np.linalg.norm(
                    item[1] - self.latest_reference_center))[1]
        self.publish_center("truth_pos", selected)

    def trim_buffers(self, now):
        """删除已经不可能参与当前滑动窗口计算的旧样本。"""
        cutoff = now - max(self.buffer_s, self.window_s + 1.0)
        while len(self.truth) > 2 and self.truth[1][0] < cutoff:
            self.truth.popleft()
        while len(self.raw) > 2 and self.raw[1][0] < cutoff:
            self.raw.popleft()
        while self.errors and self.errors[0][0] < now - self.window_s:
            self.errors.popleft()

    def interpolate_truth(self, time_value):
        """线性插值出滤波输出时刻的 HSV 中心，仅用于误差计算。

        该插值不会发布到 u/v 曲线，所以不会改变 HSV 图线的原始采样点。
        """
        if len(self.truth) < 2:
            return None
        times = [sample[0] for sample in self.truth]
        index = bisect_left(times, time_value)
        if index == 0 or index == len(times):
            return None
        before, after = self.truth[index - 1], self.truth[index]
        left_gap = time_value - before[0]
        right_gap = after[0] - time_value
        if (left_gap < 0.0 or right_gap < 0.0 or
                left_gap > self.max_truth_gap_s or
                right_gap > self.max_truth_gap_s):
            return None
        span = after[0] - before[0]
        if span <= 1e-9:
            return before[1].copy()
        alpha = left_gap / span
        return before[1] + alpha * (after[1] - before[1])

    def raw_hold(self, time_value):
        """返回指定时刻之前最近一次原始检测中心（零阶保持）。"""
        if not self.raw:
            return None
        times = [sample[0] for sample in self.raw]
        index = bisect_right(times, time_value) - 1
        return self.raw[index][1].copy() if index >= 0 else None

    def process_pending(self):
        """处理已经被 HSV 真值时间范围覆盖的滤波样本。"""
        if len(self.truth) < 2:
            return
        newest_truth_time = self.truth[-1][0]
        oldest_truth_time = self.truth[0][0]

        while (self.pending_filtered and
               self.pending_filtered[0][0] <= newest_truth_time):
            current_time, filtered_center = self.pending_filtered.popleft()
            if current_time <= oldest_truth_time:
                continue
            truth_center = self.interpolate_truth(current_time)
            raw_center = self.raw_hold(current_time)
            if truth_center is None or raw_center is None:
                continue
            filtered_error = float(np.linalg.norm(
                filtered_center - truth_center))
            raw_hold_error = float(np.linalg.norm(
                raw_center - truth_center))
            self.errors.append(
                (current_time, filtered_error, raw_hold_error))

    def report_callback(self, _event):
        """每 report_period_s 发布最近 window_s 秒的两种中心 RMSE。"""
        self.process_pending()
        if not self.errors:
            rospy.loginfo_throttle(
                2.0, "filter_eval: waiting for filtered/raw/HSV data")
            return

        latest_time = self.errors[-1][0]
        while (self.errors and
               self.errors[0][0] < latest_time - self.window_s):
            self.errors.popleft()

        filtered_errors = np.asarray(
            [sample[1] for sample in self.errors], dtype=float)
        raw_hold_errors = np.asarray(
            [sample[2] for sample in self.errors], dtype=float)
        filtered_rmse = float(np.sqrt(np.mean(filtered_errors ** 2)))
        raw_hold_rmse = float(np.sqrt(np.mean(raw_hold_errors ** 2)))

        self.filtered_rmse_pub.publish(Float64(data=filtered_rmse))
        self.raw_hold_rmse_pub.publish(Float64(data=raw_hold_rmse))
        rospy.loginfo(
            "filter_eval %.1fs n=%d filtered_RMSE=%.1f px "
            "raw_hold_RMSE=%.1f px",
            self.window_s, len(self.errors), filtered_rmse, raw_hold_rmse)


def main():
    rospy.init_node("evaluate_delay_comp_truth_live")
    LiveFilterEvaluator()
    rospy.spin()


if __name__ == "__main__":
    main()
