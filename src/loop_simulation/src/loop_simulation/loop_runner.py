#!/usr/bin/env python3
"""运行保留真实感知链路的多轮拦截仿真，记录状态、事件和实验元数据。"""

import argparse
import csv
import os
import sys
import traceback
from datetime import datetime

import rospy
from gazebo_msgs.msg import ModelStates
from quadrotor_msgs.msg import FsmState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from swarm_msgs.msg import InterceptionEvent, MassPoints


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
        """保存运行参数并建立 ROS 订阅和服务代理。"""
        self.args = args
        # 最新运行状态和基础采样缓存。
        self.fsm_state = ""
        self.motion_trigger = False
        self.latest_event = None
        self.trial_start_time = rospy.Time()
        self.samples = []
        # 以 Gazebo world 作为原始状态记录坐标系。
        self.gazebo_uav = None
        self.gazebo_stamp = rospy.Time()
        self.truth_target = None
        self.truth_stamp = rospy.Time()
        self.interception_events = []
        self.manager_target_present = False
        # 公共话题用于判断试次生命周期并记录原始真值。
        rospy.Subscriber("/fsm/fsm_state", FsmState, self._fsm_state_cb, queue_size=10)
        rospy.Subscriber("/fsm/motion_trigger", Bool, self._motion_trigger_cb, queue_size=10)
        rospy.Subscriber("/balloons/event", InterceptionEvent, self._event_cb, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._gazebo_cb, queue_size=20)
        rospy.Subscriber("/balloons/masspoint", MassPoints,
                         self._truth_cb, queue_size=20)
        # Observe and clear target_manager before starting the next trial. A
        # latched target from the previous perception trial can start EXEC
        # before the newly spawned balloon has produced a fresh detection.
        rospy.Subscriber("/target_manager/active_target", MassPoints,
                         self._manager_target_cb, queue_size=20)
        # 每轮复位通过各节点公开的服务完成，避免重启整个 ROS/Gazebo 系统。
        self.reset_fsm = rospy.ServiceProxy("/fsm/reset_trial", Trigger)
        self.reset_target_manager = rospy.ServiceProxy(
            "/target_manager/reset_trial", Trigger)
        self.reset_target_detection_yolo = rospy.ServiceProxy(
            "/target_detection_yolo_node/reset_trial", Trigger)
        self.reset_bbox_filter = rospy.ServiceProxy(
            "/bbox_s2_delay_ekf/reset_trial", Trigger)
        self.has_bbox_filter_reset = False
        self.has_target_detection_yolo_reset = False
        self.reset_balloons = rospy.ServiceProxy("/balloons/reset_trial", Trigger)
        self.clear_balloons = rospy.ServiceProxy("/balloons/clear_trial", Trigger)

    def _fsm_state_cb(self, msg):
        self.fsm_state = msg.state_name

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
        self.truth_target = msg.points[0] if msg.points else None
        self.truth_stamp = rospy.Time.now()

    def _manager_target_cb(self, msg):
        self.manager_target_present = bool(msg.points)

    def wait_for_services(self):
        """等待所有参与节点的试次复位服务就绪。"""
        rospy.loginfo("Waiting for loop reset services...")
        rospy.wait_for_service("/fsm/reset_trial", timeout=self.args.service_timeout)
        rospy.wait_for_service("/target_manager/reset_trial",
                               timeout=self.args.service_timeout)
        rospy.wait_for_service("/target_detection_yolo_node/reset_trial",
                               timeout=self.args.service_timeout)
        self.has_target_detection_yolo_reset = True
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

    def prepare_trial(self, trial_id):
        """清理上一轮状态，将 FSM、目标管理器和气球准备到下一轮起点。"""
        self.latest_event = None
        self.motion_trigger = False

        # Remove the visible old target before clearing target_manager. In the
        # perception loop, doing this in the opposite order lets the detector
        # immediately repopulate the manager with the previous balloon.
        ok, msg = self.call_trigger(self.clear_balloons, "/balloons/clear_trial")
        if not ok:
            return False, msg
        if self.has_target_detection_yolo_reset:
            ok, msg = self.call_trigger(
                self.reset_target_detection_yolo, "/target_detection_yolo_node/reset_trial")
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


        ok, msg = self.call_trigger(self.reset_balloons, "/balloons/reset_trial")
        if not ok:
            return False, msg

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
        return row

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


def parse_args(argv):
    """解析真实感知链路循环仿真的参数。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--ready-timeout", type=float,
                        default=180.0)
    parser.add_argument("--service-timeout", type=float,
                        default=60.0)
    parser.add_argument("--controller", choices=("isb_nmpc",), default="isb_nmpc")
    parser.add_argument(
        "--output-dir",
        default="log/loop_interception",
    )
    parser.add_argument("--uav-model", default="holydrone")
    parser.add_argument("--sample-rate", type=float, default=30.0)
    parser.add_argument("--truth-max-age", type=float, default=0.2)
    parser.add_argument("--terminal-event-grace", type=float, default=2.0)
    parser.add_argument("--image-bound-margin", type=float, default=0.3)
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
    args = parser.parse_args(argv)
    if args.target_reacquire_grace < 0.0:
        parser.error("--target-reacquire-grace must be nonnegative")
    return args


def main(argv):
    """运行包含相机、检测和估计链路的循环仿真。"""
    args = parse_args(argv)
    rospy.init_node("loop_simulation", anonymous=False)
    try:
        LoopRunner(args).run()
    except Exception:
        rospy.logfatal("loop_simulation terminated with an exception:\n%s",
                      traceback.format_exc())
        raise


if __name__ == "__main__":
    main(sys.argv[1:])
