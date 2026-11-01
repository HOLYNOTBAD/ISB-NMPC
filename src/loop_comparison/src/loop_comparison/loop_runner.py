#!/usr/bin/env python3
"""``loop_simulation`` 与 ``loop_comparison`` 共用的多轮实验执行器。

普通循环模式保留真实感知链路；控制器对比模式则从 Gazebo 真值合成
统一的目标观测。两种模式都只记录原始状态、拦截事件和必要实验元数据。
两个轻量入口脚本通过 ``loop_comparison`` 参数选择相应流程。
"""

import argparse
import csv
import math
import os
import signal
import shutil
import subprocess
import sys
import traceback
from datetime import datetime

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import FsmState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from swarm_msgs.msg import BoundingBox, BoundingBoxes, InterceptionEvent, MassPoints
from visualization_msgs.msg import Marker, MarkerArray


RAW_STATE_FIELDS = (
    "timestamp", "elapsed_sec", "uav_x", "uav_y", "uav_z",
    "uav_vx", "uav_vy", "uav_vz", "uav_qw", "uav_qx", "uav_qy", "uav_qz",
    "target_id", "target_x", "target_y", "target_z",
    "target_vx", "target_vy", "target_vz",
)
EVENT_FIELDS = ("timestamp", "elapsed_sec", "target_id", "success", "reason")


def time_is_zero(stamp):
    """判断 ROS 时间戳是否为未初始化的零值。"""
    return stamp == rospy.Time()


def boolean_arg(value):
    """将常见命令行布尔写法转换为 bool，并拒绝含糊值。"""
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError("expected true or false")


class LoopRunner:
    """管理 ROS 通信、试次状态机、采样及实验结果持久化。"""

    def __init__(self, args):
        """保存运行参数并建立两种模式所需的 ROS 订阅、发布和服务代理。"""
        self.args = args
        self.comparison_support = None
        if args.loop_comparison:
            from loop_comparison import interception_comparison
            self.comparison_support = interception_comparison
        # 两种模式共用的最新运行状态和基础采样缓存。
        self.fsm_state = ""
        self.motion_trigger = False
        self.latest_event = None
        self.trial_start_time = rospy.Time()
        self.samples = []
        # 两种模式都以 Gazebo world 作为原始状态记录坐标系。
        self.gazebo_uav = None
        self.gazebo_stamp = rospy.Time()
        self.truth_target = None
        self.truth_stamp = rospy.Time()
        self.interception_events = []
        self.comparison_camera = None
        self.map_uav = None
        self.map_uav_stamp = rospy.Time()
        self.truth_feed_enabled = False
        self.manager_target_present = False
        self.last_comparison_bbox = None
        self.comparison_bbox_lost_since = rospy.Time()
        # target_id belongs to loop_comparison's synthetic truth feed.  Plain
        # loop_simulation obtains the intercepted id from /balloons/event and
        # intentionally does not expose that CLI option.
        self.active_target_id = getattr(args, "target_id", None)
        self.controller_process = None
        self.uav_trajectory_pub = None
        self.last_uav_trajectory_publish_time = rospy.Time()
        self.uav_trajectory_publish_index = 0
        self.uav_trajectory_marker_id = 0

        # 公共话题用于判断试次生命周期并记录原始真值。
        rospy.Subscriber("/fsm/fsm_state", FsmState, self._fsm_state_cb, queue_size=10)
        rospy.Subscriber("/fsm/motion_trigger", Bool, self._motion_trigger_cb, queue_size=10)
        rospy.Subscriber("/balloons/event", InterceptionEvent, self._event_cb, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._gazebo_cb, queue_size=20)
        rospy.Subscriber("/balloons/masspoint", MassPoints,
                         self._truth_cb, queue_size=20)
        # Both loop modes must observe and clear target_manager. Otherwise a
        # latched target from the previous perception trial can start EXEC
        # before the newly spawned balloon has produced a fresh detection.
        rospy.Subscriber("/target_manager/active_target", MassPoints,
                         self._manager_target_cb, queue_size=20)
        if self.args.loop_comparison:
            # 对比模式额外用真值合成无噪声目标观测。
            rospy.Subscriber("/nmpc/uavStateEstimate", Odometry,
                             self._map_uav_cb, queue_size=20)
            self.map_truth_pub = rospy.Publisher(
                "/loop_comparison/target_masspoint_map", MassPoints, queue_size=10)
            self.truth_bbox_pub = rospy.Publisher(
                "/tracker/bbox", BoundingBoxes, queue_size=10)
            if self.args.trajectory_publish_rate > 0.0:
                self.uav_trajectory_pub = rospy.Publisher(
                    "/loop_interception/trajectories", MarkerArray,
                    queue_size=1, latch=True)
            rospy.Timer(rospy.Duration(1.0 / 30.0), self._publish_truth_bbox)
        # 每轮复位通过各节点公开的服务完成，避免重启整个 ROS/Gazebo 系统。
        self.reset_fsm = rospy.ServiceProxy("/fsm/reset_trial", Trigger)
        self.reset_target_manager = rospy.ServiceProxy(
            "/target_manager/reset_trial", Trigger)
        self.reset_target_detection = rospy.ServiceProxy(
            "/target_detection_yolo_node/reset_trial", Trigger)
        self.reset_bbox_filter = rospy.ServiceProxy(
            "/bbox_s2_delay_ekf/reset_trial", Trigger)
        self.has_bbox_filter_reset = False
        self.has_target_detection_reset = False
        self.reset_balloons = rospy.ServiceProxy("/balloons/reset_trial", Trigger)
        self.clear_balloons = rospy.ServiceProxy("/balloons/clear_trial", Trigger)

    def _fsm_state_cb(self, msg):
        self.fsm_state = msg.state_name
        if msg.state_name != "EXEC_INTERCEPTION":
            self.last_comparison_bbox = None
            self.comparison_bbox_lost_since = rospy.Time()

    def _motion_trigger_cb(self, msg):
        self.motion_trigger = msg.data

    def _event_cb(self, msg):
        """缓存最新拦截事件，同时过滤上一轮延迟到达的旧事件。"""
        if (not time_is_zero(self.trial_start_time) and
                not time_is_zero(msg.header.stamp) and
                msg.header.stamp < self.trial_start_time):
            return
        self.latest_event = msg
        if not time_is_zero(self.trial_start_time):
            self.interception_events.append((msg, rospy.Time.now()))

    def _successful_interception_event(self, start_time):
        """Return this trial's successful swept-proximity event, if received."""
        candidates = []
        if self.latest_event is not None:
            candidates.append(self.latest_event)
        candidates.extend(event for event, _received_at in self.interception_events)
        for event in reversed(candidates):
            if (event.success and event.reason == "swept_proximity" and
                    (time_is_zero(event.header.stamp) or
                     event.header.stamp >= start_time)):
                return event
        return None

    def _wait_for_successful_interception(self, start_time, grace=None):
        """Allow an in-flight collision event to win over terminal state races."""
        if grace is None:
            grace = self.args.terminal_event_grace
        deadline = rospy.Time.now() + rospy.Duration(
            max(0.0, grace))
        rate = rospy.Rate(200.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            event = self._successful_interception_event(start_time)
            if event is not None:
                return event
            rate.sleep()
        return self._successful_interception_event(start_time)

    def _gazebo_cb(self, msg):
        """从 Gazebo 所有模型状态中提取指定无人机的位姿与速度真值。"""
        try:
            index = msg.name.index(self.args.uav_model)
        except ValueError:
            return
        self.gazebo_uav = (msg.pose[index], msg.twist[index])
        self.gazebo_stamp = rospy.Time.now()

    def _truth_cb(self, msg):
        """从气球真值列表中选择当前试次目标。"""
        if self.active_target_id is None:
            self.truth_target = msg.points[0] if msg.points else None
        else:
            self.truth_target = next((point for point in msg.points
                                      if point.volume == self.active_target_id), None)
        self.truth_stamp = rospy.Time.now()

    def _map_uav_cb(self, msg):
        self.map_uav = msg
        self.map_uav_stamp = rospy.Time.now()

    def _manager_target_cb(self, msg):
        self.manager_target_present = bool(msg.points)

    def _publish_empty_map_truth(self):
        """发布空目标列表，主动清除下游 target_manager 的目标。"""
        if self.args.loop_comparison:
            self.map_truth_pub.publish(MassPoints())

    def disable_truth_feed(self):
        """停止合成检测器，并主动清除下游可能锁存的目标。"""
        self.truth_feed_enabled = False
        self.last_comparison_bbox = None
        self.comparison_bbox_lost_since = rospy.Time()
        if not self.args.loop_comparison:
            return
        # 多发几次可覆盖订阅者刚建立连接的时间窗，避免跨试次残留旧目标。
        for _ in range(3):
            self._publish_empty_map_truth()
            rospy.sleep(0.02)

    def _publish_map_truth(self):
        """把 Gazebo world 中的目标真值转换到 PX4/NMPC 使用的 map 坐标系。"""
        if self.gazebo_uav is None or self.truth_target is None or self.map_uav is None:
            return
        world_pose, _ = self.gazebo_uav
        map_pose = self.map_uav.pose.pose
        qw = world_pose.orientation
        qm = map_pose.orientation
        q_map_body = (qm.w, qm.x, qm.y, qm.z)
        q_map_world = self.comparison_support.quat_multiply(
            q_map_body, (qw.w, -qw.x, -qw.y, -qw.z))
        world_p = self._xyz(world_pose.position)
        map_p = self._xyz(map_pose.position)
        if not all(math.isfinite(value) for value in map_p):
            raise ValueError("non-finite map UAV position")
        target_p = self._xyz(self.truth_target.position)
        target_v = self._xyz(self.truth_target.velocity)
        relative = tuple(target_p[i] - world_p[i] for i in range(3))
        converted_p = self.comparison_support.rotate(q_map_world, relative)
        converted_v = self.comparison_support.rotate(q_map_world, target_v)
        output = MassPoints()
        point = type(self.truth_target)()
        point.volume = self.truth_target.volume
        point.position.x, point.position.y, point.position.z = (
            map_p[i] + converted_p[i] for i in range(3))
        point.velocity.x, point.velocity.y, point.velocity.z = converted_v
        output.points.append(point)
        self.map_truth_pub.publish(output)

    def _publish_truth_bbox(self, _event):
        """以 30 Hz 从真值生成零噪声目标框，并复用既有目标估计链路。

        只有 UAV、目标和 map 状态都足够新鲜时才发布目标；否则同时发布空的
        map 真值，防止下游继续使用过期数据。
        """
        now = rospy.Time.now()
        message = BoundingBoxes()
        message.header.stamp = now
        message.image_header.stamp = now
        published_map_truth = False
        if (self.truth_feed_enabled and self.gazebo_uav is not None and
                self.truth_target is not None and
                self.comparison_camera is not None and
                (now - self.gazebo_stamp).to_sec() <= self.args.truth_max_age and
                (now - self.truth_stamp).to_sec() <= self.args.truth_max_age and
                self.map_uav is not None and
                (now - self.map_uav_stamp).to_sec() <= self.args.truth_max_age):
            try:
                self._publish_map_truth()
                published_map_truth = True
                pose, twist = self.gazebo_uav
                q = pose.orientation
                camera = self.comparison_camera
                g = self.comparison_support.geometry(
                    self._xyz(pose.position), self._xyz(twist.linear),
                    (q.w, q.x, q.y, q.z),
                    self._xyz(self.truth_target.position),
                    self._xyz(self.truth_target.velocity), camera)
                if g["bearing"][2] > 0:
                    cx = camera["cx"]
                    cy = camera["cy"]
                    u = cx + camera["img_f"] * g["image_u"]
                    v = cy + camera["img_f"] * g["image_v"]
                    if 0 <= u < camera["img_width"] and 0 <= v < camera["img_height"]:
                        box = BoundingBox()
                        box.probability = 1.0
                        box.id = self.active_target_id
                        box.Class = "balloon"
                        box.xmin = int(round(u - 5))
                        box.xmax = int(round(u + 5))
                        box.ymin = int(round(v - 5))
                        box.ymax = int(round(v + 5))
                        message.bounding_boxes.append(box)
                        self.last_comparison_bbox = box
                        self.comparison_bbox_lost_since = rospy.Time()
            except (KeyError, ValueError, rospy.ROSException):
                pass
        if (not message.bounding_boxes and self.truth_feed_enabled and
                self.fsm_state == "EXEC_INTERCEPTION" and
                self.last_comparison_bbox is not None):
            # All comparison controllers receive the same zero-order-held LOS
            # briefly after the target leaves the image. A collision can end
            # interception during this window; otherwise normal loss handling
            # resumes when the bounded hold expires.
            if time_is_zero(self.comparison_bbox_lost_since):
                self.comparison_bbox_lost_since = now
            if ((now - self.comparison_bbox_lost_since).to_sec() <=
                    self.args.los_hold_time):
                message.bounding_boxes.append(self.last_comparison_bbox)
        if not published_map_truth:
            self._publish_empty_map_truth()
        self.truth_bbox_pub.publish(message)

    def wait_for_services(self):
        """等待所有参与节点的试次复位服务就绪。"""
        rospy.loginfo("Waiting for loop reset services...")
        rospy.wait_for_service("/fsm/reset_trial", timeout=self.args.service_timeout)
        rospy.wait_for_service("/target_manager/reset_trial",
                               timeout=self.args.service_timeout)
        if not self.args.loop_comparison:
            rospy.wait_for_service("/target_detection_yolo_node/reset_trial",
                                   timeout=self.args.service_timeout)
            self.has_target_detection_reset = True
        try:
            rospy.wait_for_service("/bbox_s2_delay_ekf/reset_trial", timeout=2.0)
            self.has_bbox_filter_reset = True
        except rospy.ROSException:
            # The perception filter is optional; with filtering disabled the
            # detector publishes directly to /tracker/bbox.
            self.has_bbox_filter_reset = False
            rospy.loginfo("Optional bbox filter reset service is absent.")
        rospy.wait_for_service("/balloons/reset_trial", timeout=self.args.service_timeout)
        rospy.wait_for_service("/balloons/clear_trial", timeout=self.args.service_timeout)
        rospy.loginfo("Loop reset services are ready.")

    def wait_until(self, predicate, timeout, label):
        """按 ROS 仿真时间轮询条件，超时时记录带标签的警告。"""
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return True
            rate.sleep()
        rospy.logwarn("Timeout while waiting for %s.", label)
        return False

    def call_trigger(self, service, name):
        """调用 Trigger 服务，并将传输异常和业务失败统一转换为返回值。"""
        try:
            resp = service()
        except rospy.ServiceException as exc:
            return False, "service [{}] unavailable: {}".format(name, exc)
        if not resp.success:
            rospy.logwarn("%s returned failure: %s", name, resp.message)
        return resp.success, resp.message

    @staticmethod
    def infrastructure_failure(reason):
        """Return whether a trial failed because a required service was lost."""
        return reason.startswith("service [") and " unavailable:" in reason

    def wait_for_stable_comparison_start(self, params, stable_duration=0.75):
        """要求 Gazebo 真值和 PX4 侧估计同时稳定一段连续时间。

        该检查限制初始位置、线速度、姿态和角速度，保证各控制器从可比较的
        初始条件出发；任一条件瞬时越界都会重新开始稳定计时。
        """
        deadline = rospy.Time.now() + rospy.Duration(self.args.ready_timeout)
        stable_since = rospy.Time()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            now = rospy.Time.now()
            ready = False
            if (self.gazebo_uav is not None and self.map_uav is not None and
                    (now - self.gazebo_stamp).to_sec() <= self.args.truth_max_age and
                    (now - self.map_uav_stamp).to_sec() <= self.args.truth_max_age):
                pose, twist = self.gazebo_uav
                speed = self.comparison_support.norm(self._xyz(twist.linear))
                q = pose.orientation
                roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                                  1 - 2 * (q.x * q.x + q.y * q.y))
                pitch_term = max(-1.0, min(1.0,
                    2 * (q.w * q.y - q.z * q.x)))
                pitch = math.asin(pitch_term)
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                angular_speed = self.comparison_support.norm(
                    self._xyz(twist.angular))

                estimate = self.map_uav
                estimate_position = self._xyz(estimate.pose.pose.position)
                estimate_velocity = self._xyz(estimate.twist.twist.linear)
                estimate_bodyrates = self._xyz(estimate.twist.twist.angular)
                expected = params["uav_start_map"]
                position_error = self.comparison_support.norm(tuple(
                    estimate_position[i] - expected[i] for i in range(3)))
                estimate_speed = self.comparison_support.norm(estimate_velocity)
                estimate_angular_speed = self.comparison_support.norm(
                    estimate_bodyrates)
                ready = (position_error <= self.args.start_tolerance and
                         speed <= self.args.start_speed_tolerance and
                         abs(roll) <= self.args.start_attitude_tolerance and
                         abs(pitch) <= self.args.start_attitude_tolerance and
                         abs(yaw) <= self.args.start_yaw_tolerance and
                         angular_speed <= self.args.start_angular_speed_tolerance and
                         estimate_speed <= self.args.start_speed_tolerance and
                         estimate_angular_speed <=
                         self.args.start_angular_speed_tolerance)
                if not ready:
                    rospy.loginfo_throttle(
                        2.0,
                        "Waiting for stable comparison start: "
                        "map_pos=%.3f/%.3f m truth_speed=%.3f/%.3f m/s "
                        "rpy=[%.3f %.3f %.3f] angular_speed=%.3f/%.3f rad/s "
                        "estimate_speed=%.3f m/s estimate_angular_speed=%.3f rad/s.",
                        position_error, self.args.start_tolerance,
                        speed, self.args.start_speed_tolerance,
                        roll, pitch, yaw, angular_speed,
                        self.args.start_angular_speed_tolerance,
                        estimate_speed, estimate_angular_speed,
                    )
            if ready:
                if time_is_zero(stable_since):
                    stable_since = now
                elif (now - stable_since).to_sec() >= stable_duration:
                    return True
            else:
                stable_since = rospy.Time()
            rate.sleep()
        rospy.logwarn("Timeout while waiting for stable comparison start.")
        return False

    def prepare_trial(self, trial_id, comparison_params=None):
        """清理上一轮状态，将 FSM、目标管理器和气球准备到下一轮起点。

        comparison 不直接瞬移无人机，而是沿 FSM 的安全返航流程回到起点，
        以免 Gazebo 真值与 PX4 EKF 状态失去同步。
        """
        self.latest_event = None
        self.motion_trigger = False

        if self.args.loop_comparison:
            self.disable_truth_feed()

        # Remove the visible old target before clearing target_manager. In the
        # perception loop, doing this in the opposite order lets the detector
        # immediately repopulate the manager with the previous balloon.
        ok, msg = self.call_trigger(self.clear_balloons, "/balloons/clear_trial")
        if not ok:
            return False, msg
        if self.has_target_detection_reset:
            ok, msg = self.call_trigger(
                self.reset_target_detection, "/target_detection_yolo_node/reset_trial")
            if not ok:
                return False, msg
        if self.has_bbox_filter_reset:
            ok, msg = self.call_trigger(
                self.reset_bbox_filter, "/bbox_s2_delay_ekf/reset_trial")
            if not ok:
                return False, msg
        ok, msg = self.call_trigger(
            self.reset_target_manager, "/target_manager/reset_trial")
        if not ok:
            return False, msg
        target_cleared = self.wait_until(
            lambda: not self.manager_target_present,
            min(2.0, self.args.ready_timeout),
            "target_manager target clear",
        )
        if not target_cleared:
            return False, "target_not_cleared"
        # comparison 使用 FSM 正常返航；PX4 始终掌握控制权，避免瞬移 Gazebo
        # 模型造成估计器与物理世界不同步。
        if self.args.loop_comparison and trial_id > 1:
            if self.fsm_state == "EXEC_INTERCEPTION":
                completed = self.wait_until(
                    lambda: self.fsm_state != "EXEC_INTERCEPTION",
                    min(2.0, self.args.ready_timeout),
                    "FSM interception completion",
                )
                if not completed:
                    return False, "fsm_interception_completion_timeout"
            if self.fsm_state in ("FINISH_HOLD", "RETURN_TO_START"):
                ready = self.wait_until(
                    lambda: self.fsm_state == "WAIT_TARGET",
                    self.args.ready_timeout,
                    "safe FSM return after comparison trial",
                )
                if not ready:
                    return False, "fsm_safe_return_timeout"

        should_reset_fsm = (
            self.fsm_state in ("EXEC_INTERCEPTION", "HOLD", "FINISH_HOLD") or
            (trial_id > 1 and self.fsm_state not in
             ("WAIT_TARGET", "RETURN_TO_START"))
        )
        if should_reset_fsm:
            ok, msg = self.call_trigger(self.reset_fsm, "/fsm/reset_trial")
            if not ok:
                return False, msg
            returning = self.wait_until(
                lambda: self.fsm_state == "RETURN_TO_START",
                min(2.0, self.args.ready_timeout),
                "FSM RETURN_TO_START",
            )
            if not returning:
                return False, "fsm_return_not_observed"

        ready = self.wait_until(
            lambda: self.fsm_state == "WAIT_TARGET",
            self.args.ready_timeout,
            "FSM WAIT_TARGET",
        )
        if not ready:
            return False, "fsm_not_ready"

        if (comparison_params is not None and
                not self.wait_for_stable_comparison_start(comparison_params)):
            return False, "uav_start_not_stable"

        # The balloon node spawns relative to the UAV pose cached when its reset
        # service runs.  Keep the matching local snapshot: using a newer pose
        # after waiting for target truth amplifies tiny yaw changes by the
        # (potentially long) configured target offset.
        spawn_reference_pose = (
            self.gazebo_uav[0]
            if comparison_params is not None and self.gazebo_uav is not None
            else None
        )
        ok, msg = self.call_trigger(self.reset_balloons, "/balloons/reset_trial")
        if not ok:
            return False, msg

        # Validate the comparison target while the FSM is still in WAIT_TARGET.
        # Enabling the synthetic truth feed below lets the FSM enter
        # EXEC_INTERCEPTION, which immediately publishes the balloon motion
        # trigger.  Checking after that transition incorrectly counts the
        # target's first few centimetres of commanded motion as spawn error.
        if comparison_params is not None:
            # Discard any sample cached before reset_trial returned and wait for
            # the balloon node to publish the freshly reset state.
            self.truth_target = None
            target_ready = self.wait_until(
                lambda: (self.truth_target is not None and
                         int(self.truth_target.volume) ==
                         int(comparison_params["target_id"]) and
                         self.gazebo_uav is not None),
                min(2.0, self.args.ready_timeout),
                "reset target truth",
            )
            if not target_ready:
                return False, "target_truth_not_ready"

            pose = spawn_reference_pose
            if pose is None:
                return False, "uav_spawn_reference_not_ready"
            q = pose.orientation
            start_yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z),
            )
            configured_offset = comparison_params["target_start_config"]
            expected_relative = (
                configured_offset[0] * math.cos(start_yaw) -
                configured_offset[1] * math.sin(start_yaw),
                configured_offset[0] * math.sin(start_yaw) +
                configured_offset[1] * math.cos(start_yaw),
                configured_offset[2],
            )
            uav_position = self._xyz(pose.position)
            target_position = self._xyz(self.truth_target.position)
            actual_relative = tuple(
                target_position[i] - uav_position[i] for i in range(3))
            target_start_error = self.comparison_support.norm(tuple(
                actual_relative[i] - expected_relative[i] for i in range(3)))
            if target_start_error > self.args.target_start_tolerance:
                return False, "target_start_error_m={:.3f}".format(
                    target_start_error)

        rospy.loginfo("Trial %d ready: fsm_state=%s", trial_id, self.fsm_state)
        return True, "ready"

    def run_trial(self, trial_id):
        """执行一轮普通感知闭环仿真，只采集原始状态与拦截事件。"""
        self.samples = []
        self.interception_events = []
        self.trial_start_time = rospy.Time()
        prepared, reason = self.prepare_trial(trial_id)
        if not prepared:
            return {"success": False, "reason": reason, "elapsed_sec": 0.0}

        started = self.wait_until(
            lambda: self.motion_trigger or self.fsm_state == "EXEC_INTERCEPTION",
            self.args.ready_timeout,
            "trial motion trigger",
        )
        if not started:
            return {"success": False, "reason": "motion_trigger_timeout", "elapsed_sec": 0.0}

        start_time = rospy.Time.now()
        self.trial_start_time = start_time
        self.latest_event = None
        self.interception_events = []
        deadline = start_time + rospy.Duration(self.args.timeout)
        rate = rospy.Rate(self.args.sample_rate)
        reason = "timeout"
        success = False
        terminal_event_checked = False
        hold_since = rospy.Time()
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.latest_event is not None:
                if (not time_is_zero(self.latest_event.header.stamp) and
                        self.latest_event.header.stamp < start_time):
                    self.latest_event = None
                    rate.sleep()
                    continue
                success = bool(self.latest_event.success)
                reason = self.latest_event.reason or "event"
                break
            if self.record_raw_state(start_time) is None:
                # Gazebo removes a struck balloon before its interception
                # event reaches this subscriber. Allow that event to arrive
                # before classifying the now-missing truth as invalid.
                event = self._wait_for_successful_interception(start_time)
                terminal_event_checked = True
                if event is not None:
                    success = True
                    reason = event.reason
                else:
                    reason = "invalid state"
                break
            if self.fsm_state == "HOLD":
                # The real FSM can reacquire a briefly lost visual target and
                # resume EXEC_INTERCEPTION. Do not end the trial at the first
                # HOLD message; that would turn a later real interception
                # into a false negative in the loop results.
                if time_is_zero(hold_since):
                    hold_since = rospy.Time.now()
                if (rospy.Time.now() - hold_since).to_sec() < \
                        self.args.target_reacquire_grace:
                    rate.sleep()
                    continue
            else:
                hold_since = rospy.Time()
            if self.fsm_state in ("HOLD", "FINISH_HOLD", "RETURN_TO_START"):
                event = self._wait_for_successful_interception(start_time)
                terminal_event_checked = True
                if event is not None:
                    success = True
                    reason = event.reason
                else:
                    reason = "target_lost"
                break
            rate.sleep()

        if not success and not terminal_event_checked:
            event = self._wait_for_successful_interception(start_time)
            if event is not None:
                success = True
                reason = event.reason

        elapsed = max(0.0, (rospy.Time.now() - start_time).to_sec())
        return {"success": success, "reason": reason, "elapsed_sec": elapsed}

    @staticmethod
    def _xyz(value):
        """将 ROS Vector3/Point 类型转换为普通三元组。"""
        return (value.x, value.y, value.z)

    def record_raw_state(self, start_time):
        """从 Gazebo 真值构造一个原始状态样本。

        仅保存时间、UAV 位姿/速度和目标 ID/位置/速度。真值过期时返回
        ``None``，距离、视野及其他评价量由离线工具计算。
        """
        now = rospy.Time.now()
        max_age = self.args.truth_max_age
        if (self.gazebo_uav is None or self.truth_target is None or
                (now - self.gazebo_stamp).to_sec() > max_age or
                (now - self.truth_stamp).to_sec() > max_age):
            return None
        pose, twist = self.gazebo_uav
        p, v = self._xyz(pose.position), self._xyz(twist.linear)
        tp, tv = self._xyz(self.truth_target.position), self._xyz(self.truth_target.velocity)
        q = pose.orientation
        row = dict.fromkeys(RAW_STATE_FIELDS, "")
        row.update({
            "timestamp": now.to_sec(), "elapsed_sec": max(0., (now - start_time).to_sec()),
            "uav_x": p[0], "uav_y": p[1], "uav_z": p[2],
            "uav_vx": v[0], "uav_vy": v[1], "uav_vz": v[2],
            "uav_qw": q.w, "uav_qx": q.x, "uav_qy": q.y, "uav_qz": q.z,
            "target_id": int(self.truth_target.volume),
            "target_x": tp[0], "target_y": tp[1], "target_z": tp[2],
            "target_vx": tv[0], "target_vy": tv[1], "target_vz": tv[2],
        })
        self.samples.append(row)
        self._publish_uav_trajectory(now)
        return row

    def _clear_uav_trajectory(self):
        """清除 RViz 中上一轮 comparison 的无人机轨迹。"""
        self.last_uav_trajectory_publish_time = rospy.Time()
        self.uav_trajectory_publish_index = 0
        self.uav_trajectory_marker_id = 0
        if self.uav_trajectory_pub is None:
            return
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "world"
        marker.action = Marker.DELETEALL
        self.uav_trajectory_pub.publish(MarkerArray(markers=[marker]))

    def _publish_uav_trajectory(self, stamp):
        """低频增量发布无人机真值轨迹，避免重复发送全部历史点。"""
        if self.uav_trajectory_pub is None:
            return
        interval = 1.0 / self.args.trajectory_publish_rate
        if (not time_is_zero(self.last_uav_trajectory_publish_time) and
                (stamp - self.last_uav_trajectory_publish_time).to_sec() < interval):
            return
        self.last_uav_trajectory_publish_time = stamp
        start_index = max(0, self.uav_trajectory_publish_index - 1)
        new_rows = self.samples[start_index:]
        if len(new_rows) < 2:
            return
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "world"
        marker.ns = "uav_trajectory"
        marker.id = self.uav_trajectory_marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [Point(x=row["uav_x"], y=row["uav_y"], z=row["uav_z"])
                         for row in new_rows]
        self.uav_trajectory_pub.publish(MarkerArray(markers=[marker]))
        self.uav_trajectory_publish_index = len(self.samples)
        self.uav_trajectory_marker_id += 1

    def run_comparison_trial(self, trial_id, params):
        """生成运动文件，执行单轮对比并记录原始状态与拦截事件。"""
        self.disable_truth_feed()
        self.samples = []
        self.last_comparison_bbox = None
        self.comparison_bbox_lost_since = rospy.Time()
        self._clear_uav_trajectory()
        self.interception_events = []
        self.trial_start_time = rospy.Time()
        self.comparison_support.write_motion_files(self.args.motion_file, params)
        prepared, prepare_reason = self.prepare_trial(trial_id, params)
        if not prepared:
            return {"success": False, "reason": prepare_reason, "elapsed_sec": 0.0}
        self.truth_feed_enabled = True
        started = self.wait_until(lambda: self.fsm_state == "EXEC_INTERCEPTION",
                                  self.args.ready_timeout, "EXEC_INTERCEPTION")
        if not started:
            self.disable_truth_feed()
            return {"success": False, "reason": "exec_not_reached", "elapsed_sec": 0.0}
        start_time = rospy.Time.now()
        self.trial_start_time = start_time
        self.latest_event = None
        self.interception_events = []
        first = self.record_raw_state(start_time)
        if first is None:
            self.disable_truth_feed()
            return {"success": False, "reason": "missing_world_truth", "elapsed_sec": 0.0}
        map_position = self._xyz(self.map_uav.pose.pose.position)
        start_error = self.comparison_support.norm(tuple(
            map_position[i] - params["uav_start_map"][i] for i in range(3)))
        start_speed = self.comparison_support.norm((
            first["uav_vx"], first["uav_vy"], first["uav_vz"]))
        w, x, y, z = (first[key] for key in
                      ("uav_qw", "uav_qx", "uav_qy", "uav_qz"))
        start_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        if (start_error > self.args.start_tolerance or
                start_speed > self.args.start_speed_tolerance or
                abs(start_yaw) > self.args.start_yaw_tolerance):
            self.disable_truth_feed()
            detail = ("uav_start_error_m={:.3f};speed_mps={:.3f};"
                      "yaw_rad={:.3f}").format(
                          start_error, start_speed, start_yaw)
            return {"success": False, "reason": detail, "elapsed_sec": 0.0}
        deadline = start_time + rospy.Duration(self.args.timeout)
        rate = rospy.Rate(self.args.sample_rate)
        reason = "timeout"
        success = False
        terminal_event_checked = False
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            event = self._successful_interception_event(start_time)
            if event is not None:
                success = True
                reason = event.reason
                break
            row = self.record_raw_state(start_time)
            if row is None:
                event = self._wait_for_successful_interception(start_time)
                terminal_event_checked = True
                if event is not None:
                    success = True
                    reason = event.reason
                else:
                    reason = "invalid state"
                break
            if self.fsm_state in ("HOLD", "FINISH_HOLD", "RETURN_TO_START"):
                # Losing the target stops visual guidance, but the vehicle can
                # still hit it through momentum. Keep the independent Gazebo
                # swept-proximity detector authoritative during this window.
                event = self._wait_for_successful_interception(
                    start_time, self.args.target_lost_grace)
                terminal_event_checked = True
                if event is not None:
                    success = True
                    reason = event.reason
                else:
                    reason = "target lost"
                break
            rate.sleep()
        if not success and not terminal_event_checked:
            event = self._wait_for_successful_interception(start_time)
            if event is not None:
                success = True
                reason = event.reason
        elapsed = max(0.0, (rospy.Time.now() - start_time).to_sec())
        self.disable_truth_feed()
        return {"success": success, "reason": reason, "elapsed_sec": elapsed}

    def finish_comparison(self):
        """结束真值注入并让仿真器在最后一轮后回到安全空闲状态。"""
        self.disable_truth_feed()
        self.wait_until(lambda: not self.manager_target_present, 2.0,
                        "final target_manager target clear")
        self.call_trigger(self.clear_balloons, "/balloons/clear_trial")
        # 初始化阶段报错时 FSM 可能仍在 IDLE；此时没有可返航的试次，不应再等待
        # 完整 ready timeout，以免掩盖原始异常。
        if not self.fsm_state or self.fsm_state == "IDLE":
            return
        # Let target-estimate timeout propagate before inspecting the final
        # state, then explicitly return from any active/hold state. Waiting for
        # RETURN_TO_START to be observed prevents a stale WAIT_TARGET callback
        # from making cleanup finish early.
        rospy.sleep(0.75)
        if self.fsm_state not in ("WAIT_TARGET", "RETURN_TO_START", "IDLE"):
            reset_ok, _ = self.call_trigger(self.reset_fsm, "/fsm/reset_trial")
            if reset_ok:
                self.wait_until(lambda: self.fsm_state == "RETURN_TO_START",
                                2.0, "final FSM RETURN_TO_START")
        if self.fsm_state != "WAIT_TARGET":
            self.wait_until(lambda: self.fsm_state == "WAIT_TARGET",
                            self.args.ready_timeout, "final FSM WAIT_TARGET")

    def run_comparison(self):
        """运行全部 comparison 试次，并保证异常时也执行最终清理。"""
        try:
            self._run_comparison()
        finally:
            try:
                if not rospy.is_shutdown():
                    self.finish_comparison()
            finally:
                self.stop_comparison_controller()

    def _run_comparison(self):
        """实现 comparison 主循环，写入原始状态、拦截事件和必要元数据。"""
        controllers = ([item.strip() for item in self.args.controllers.split(",")
                        if item.strip()] if self.args.controllers else
                       [self.args.controller])
        invalid = [item for item in controllers
                   if item not in self.comparison_support.CONTROLLERS]
        if invalid:
            raise ValueError("comparison controller must be one of: " +
                             ", ".join(self.comparison_support.CONTROLLERS))
        camera = self.camera_parameters()
        self.comparison_camera = camera
        self.wait_for_services()
        uav_start_map = [float(rospy.get_param(
            "/fsm_node/interception/idle/" + axis)) for axis in
            ("start_x", "start_y", "start_z")]
        uav_collision_radius = float(rospy.get_param(
            "/gazebo_balloon_node/uav_collision_radius", 0.34))
        base_params = self.comparison_support.motion_parameters(
            self.args.source_motion_file, uav_start_map,
            uav_collision_radius, self.args.balloon_radius)
        os.makedirs(self.args.output_dir, exist_ok=True)
        session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S_%f")
        session_dir = os.path.join(self.args.output_dir, session_name)
        os.makedirs(session_dir, exist_ok=False)
        rospy.loginfo("Loop comparison results: %s", session_dir)
        for controller_index, controller in enumerate(controllers):
            self.args.controller = controller
            try:
                if self.args.manage_controllers:
                    self.start_comparison_controller(controller)
                self._run_comparison_controller(session_dir, base_params, camera)
            finally:
                try:
                    if controller_index + 1 < len(controllers) and not rospy.is_shutdown():
                        self.finish_comparison()
                finally:
                    if self.args.manage_controllers:
                        self.stop_comparison_controller()

    def _run_comparison_controller(self, session_dir, base_params, camera):
        """Run every seeded trial for the currently selected controller."""
        for trial_id in range(1, self.args.count + 1):
            params = dict(base_params)
            if params["motion_mode"] == "random_walk":
                params["random_walk_seed"] = self.args.trajectory_seed + trial_id - 1
            self.active_target_id = self.args.target_id + trial_id - 1
            if self.active_target_id > 32767:
                raise ValueError("comparison target IDs must fit BoundingBox.id int16")
            params["target_id"] = self.active_target_id
            run_id = "trial_{:03d}".format(trial_id)
            run_dir = os.path.join(session_dir, self.args.controller, run_id)
            os.makedirs(run_dir, exist_ok=False)
            result = self.run_comparison_trial(trial_id, params)
            trajectory_source = os.path.splitext(self.args.motion_file)[0] + "_trajectory.csv"
            for source, filename in ((self.args.motion_file, "motion.yaml"),
                                     (trajectory_source, "target_trajectory.csv")):
                if os.path.isfile(source):
                    shutil.copy2(source, os.path.join(run_dir, filename))
            metadata = {
                "run_id": run_id,
                "trial_id": trial_id,
                "controller": self.args.controller,
                "motion": params,
                "camera": camera,
                "sample_rate_hz": self.args.sample_rate,
                "timeout_sec": self.args.timeout,
                "target_lost_grace_sec": self.args.target_lost_grace,
                "los_hold_time_sec": self.args.los_hold_time,
                "trajectory_hold_time_sec": self.args.trajectory_hold_time,
                "trajectory_publish_rate_hz": self.args.trajectory_publish_rate,
                "collision_radius_m": params["collision_radius_m"],
                "result": result,
                "state_source": "/gazebo/model_states",
                "target_source": "/balloons/masspoint",
                "motion_file": "motion.yaml",
                "trajectory_file": "target_trajectory.csv",
            }
            self.write_raw_recording(run_dir, metadata)
            rospy.loginfo("Comparison %s trial=%d success=%s reason=%s samples=%d events=%d",
                          self.args.controller, trial_id,
                          result["success"], result["reason"], len(self.samples),
                          len(self.interception_events))
            if self.args.trajectory_hold_time > 0.0:
                rospy.loginfo(
                    "Holding UAV and balloon trajectories for %.1f seconds.",
                    self.args.trajectory_hold_time,
                )
                rospy.sleep(self.args.trajectory_hold_time)
            if self.infrastructure_failure(result["reason"]):
                rospy.logerr(
                    "Stopping comparison after trial %d because required "
                    "infrastructure was lost: %s",
                    trial_id, result["reason"],
                )
                break

    def start_comparison_controller(self, controller):
        """Start exactly one controller process for a multi-method suite."""
        self.stop_comparison_controller()
        if controller == "isb_nmpc":
            command = [
                "roslaunch", "isb_nmpc", "so3_mpc_bridge.launch",
                "param_file:=" + self.args.isb_nmpc_param_file,
                "has_target_pos:=false",
                "attitude_target_topic:=" + self.args.attitude_target_topic,
                "enable_log:=false", "respawn:=true",
                "restart_after_interception:=true",
            ]
            expected_node = "/so3_mpc_bridge_node"
        else:
            command = [
                "roslaunch", "interception_baseline", "interception_baseline.launch",
                "config:=" + self.args.baseline_param_file,
                "controller_type:=" + controller,
            ]
            expected_node = "/interception_baseline_node"
        rospy.loginfo("Starting comparison controller %s", controller)
        self.controller_process = subprocess.Popen(command, start_new_session=True)
        try:
            deadline = rospy.Time.now() + rospy.Duration(self.args.service_timeout)
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                if self.controller_process.poll() is not None:
                    raise RuntimeError(
                        "controller {} exited during startup".format(controller))
                nodes = subprocess.check_output(
                    ["rosnode", "list"], text=True).splitlines()
                if expected_node in nodes:
                    rospy.sleep(1.0)
                    return
                rospy.sleep(0.25)
            raise RuntimeError("timeout starting controller {}".format(controller))
        except Exception:
            self.stop_comparison_controller()
            raise

    def stop_comparison_controller(self):
        """Stop the managed roslaunch process and all children before switching."""
        process = self.controller_process
        self.controller_process = None
        if process is None or process.poll() is not None:
            return
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5.0)

    def camera_parameters(self):
        """读取离线重建视野所需的相机内外参。"""
        camera = {key: rospy.get_param("/camera/" + key) for key in
                  ("p_B_C", "q_B_C", "img_width", "img_height", "img_f")}
        camera["cx"] = float(rospy.get_param("/camera/cx", camera["img_width"] / 2))
        camera["cy"] = float(rospy.get_param("/camera/cy", camera["img_height"] / 2))
        camera["image_bound_margin"] = float(rospy.get_param(
            "/so3_mpc_bridge_node/image_bound_margin", self.args.image_bound_margin))
        return camera

    def write_raw_recording(self, run_dir, metadata):
        """将一轮的原始状态、拦截事件和元数据写盘。"""
        import yaml
        with open(os.path.join(run_dir, "states.csv"), "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=RAW_STATE_FIELDS)
            writer.writeheader()
            writer.writerows(self.samples)
        with open(os.path.join(run_dir, "events.csv"), "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=EVENT_FIELDS)
            writer.writeheader()
            for event, received_at in self.interception_events:
                stamp = (event.header.stamp if not time_is_zero(event.header.stamp)
                         else received_at)
                writer.writerow({
                    "timestamp": stamp.to_sec(),
                    "elapsed_sec": max(0.0, (stamp - self.trial_start_time).to_sec()),
                    "target_id": int(event.target_id),
                    "success": bool(event.success),
                    "reason": event.reason,
                })
        with open(os.path.join(run_dir, "metadata.yaml"), "w") as handle:
            yaml.safe_dump(metadata, handle, sort_keys=True)

    def run(self):
        """执行普通 loop_simulation，每轮只保存原始数据。"""
        self.wait_for_services()
        session_dir = ""
        camera = None
        if self.args.write_results:
            camera = self.camera_parameters()
            session_dir = os.path.join(
                self.args.output_dir,
                datetime.now().strftime("session_%Y%m%d_%H%M%S_%f"))
            os.makedirs(session_dir, exist_ok=False)

        for trial_id in range(1, self.args.count + 1):
            if rospy.is_shutdown():
                break
            rospy.loginfo("Starting loop interception trial %d/%d.", trial_id, self.args.count)
            result = self.run_trial(trial_id)
            rospy.loginfo(
                "Trial %d result: success=%s elapsed=%.3fs reason=%s samples=%d events=%d",
                trial_id, result["success"], result["elapsed_sec"], result["reason"],
                len(self.samples), len(self.interception_events),
            )
            if self.args.write_results:
                run_dir = os.path.join(session_dir, "trial_{:03d}".format(trial_id))
                os.makedirs(run_dir, exist_ok=False)
                self.write_raw_recording(run_dir, {
                    "trial_id": trial_id,
                    "controller": self.args.controller,
                    "camera": camera,
                    "sample_rate_hz": self.args.sample_rate,
                    "timeout_sec": self.args.timeout,
                    "target_reacquire_grace_sec": self.args.target_reacquire_grace,
                    "result": result,
                    "state_source": "/gazebo/model_states",
                    "target_source": "/balloons/masspoint",
                })


def parse_args(argv, loop_comparison=False):
    """解析公共参数，并按运行模式追加各自专用选项。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--ready-timeout", type=float,
                        default=180.0)
    parser.add_argument("--service-timeout", type=float,
                        default=120.0 if loop_comparison else 60.0)
    parser.add_argument("--controller", default="isb_nmpc")
    parser.add_argument(
        "--output-dir",
        default=("results/interception_comparison" if loop_comparison
                 else "log/loop_interception"),
    )
    parser.add_argument("--uav-model", default="holydrone")
    parser.add_argument("--sample-rate", type=float, default=30.0)
    parser.add_argument("--truth-max-age", type=float, default=0.2)
    parser.add_argument("--terminal-event-grace", type=float, default=2.0)
    parser.add_argument("--image-bound-margin", type=float, default=0.3)
    if loop_comparison:
        parser.add_argument(
            "--target-lost-grace", type=float, default=1.0,
            help=("seconds to keep accepting swept-proximity success after "
                  "the target is lost (default: 1.0)"),
        )
        parser.add_argument(
            "--los-hold-time", type=float, default=0.3,
            help=("seconds to hold the last valid comparison LOS after the "
                  "synthetic target leaves the image (default: 0.3)"),
        )
        parser.add_argument(
            "--trajectory-hold-time", type=float, default=3.0,
            help=("seconds to keep the frozen UAV and balloon trajectories "
                  "visible after each trial (default: 3.0)"),
        )
        parser.add_argument(
            "--trajectory-publish-rate", type=float, default=2.0,
            help=("UAV trajectory visualization rate in Hz; 0 disables "
                  "trajectory publication (default: 2.0)"),
        )
        parser.add_argument("--source-motion-file", required=True)
        parser.add_argument(
            "--motion-file",
            default="log/interception_comparison/active_motion.yaml")
        parser.add_argument("--target-id", type=int, default=1)
        parser.add_argument("--balloon-radius", type=float)
        parser.add_argument("--controllers", default="")
        parser.add_argument("--manage-controllers", action="store_true")
        parser.add_argument("--baseline-param-file", default="")
        parser.add_argument("--isb_nmpc-param-file", default="")
        parser.add_argument("--attitude-target-topic",
                            default="/mavros/setpoint_raw/attitude")
        parser.add_argument(
            "--trajectory-seed", type=int, default=1000,
            help=("base seed for random-walk comparison trials; trial N uses "
                  "base + N - 1 (default: 1000)"),
        )
        parser.add_argument("--start-tolerance", type=float, default=0.35)
        parser.add_argument("--target-start-tolerance", type=float, default=0.25)
        parser.add_argument("--start-speed-tolerance", type=float, default=0.7)
        parser.add_argument("--start-yaw-tolerance", type=float, default=0.35)
        parser.add_argument("--start-attitude-tolerance", type=float, default=0.2)
        parser.add_argument("--start-angular-speed-tolerance", type=float, default=0.5)
    else:
        parser.add_argument(
            "--target-reacquire-grace", type=float, default=0.5,
            help=("seconds to allow a transient FSM HOLD to reacquire before "
                  "classifying target loss (default: 0.5)"),
        )
        parser.add_argument(
            "--write-results", type=boolean_arg, default=True,
            help="write per-trial CSV files (true or false)",
        )
        parser.add_argument(
            "--no-write-results", dest="write_results", action="store_false",
            help="run trials without writing raw state, event, or metadata files",
        )
    parser.set_defaults(loop_comparison=loop_comparison)
    args = parser.parse_args(argv)
    if not loop_comparison and args.target_reacquire_grace < 0.0:
        parser.error("--target-reacquire-grace must be nonnegative")
    if loop_comparison:
        if args.trajectory_seed < 0:
            parser.error("--trajectory-seed must be nonnegative")
        if args.trajectory_hold_time < 0.0:
            parser.error("--trajectory-hold-time must be nonnegative")
        if args.trajectory_publish_rate < 0.0:
            parser.error("--trajectory-publish-rate must be nonnegative")
        if args.los_hold_time < 0.0:
            parser.error("--los-hold-time must be nonnegative")
        if args.count < 1:
            parser.error("--count must be at least 1")
        if args.trajectory_seed + args.count - 1 > 2147483647:
            parser.error("trajectory seeds must fit a signed 32-bit integer")
        if args.manage_controllers:
            managed_controllers = [
                item.strip() for item in args.controllers.split(",")
                if item.strip()
            ]
            from loop_comparison.interception_comparison import CONTROLLERS
            if not managed_controllers or any(name not in CONTROLLERS for name in managed_controllers):
                parser.error("--controllers must list supported methods: " + ",".join(CONTROLLERS))
            if len(managed_controllers) != len(set(managed_controllers)):
                parser.error("--controllers must not contain duplicate methods")
            required_files = []
            if "isb_nmpc" in managed_controllers:
                required_files.append(
                    ("--isb_nmpc-param-file", args.isb_nmpc_param_file))
            if any(name != "isb_nmpc" for name in managed_controllers):
                required_files.append(
                    ("--baseline-param-file", args.baseline_param_file))
            missing = [name for name, value in required_files if not value]
            if missing:
                parser.error("managed controllers require " + ", ".join(missing))
            for name, path in required_files:
                if not os.path.isfile(path):
                    parser.error(name + " does not exist: " + path)
    return args


def main(argv, loop_comparison=False):
    """初始化 ROS 节点并分派到普通循环或控制器对比流程。"""
    args = parse_args(rospy.myargv(argv=["loop_comparison"] + argv)[1:], loop_comparison=loop_comparison)
    node_name = "loop_comparison" if loop_comparison else "loop_simulation"
    rospy.init_node(node_name, anonymous=False)
    try:
        tester = LoopRunner(args)
        if args.loop_comparison:
            tester.run_comparison()
        else:
            tester.run()
    except Exception:
        rospy.logfatal("%s terminated with an exception:\n%s",
                       node_name, traceback.format_exc())
        raise


if __name__ == "__main__":
    main(sys.argv[1:])
