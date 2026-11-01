#!/usr/bin/env python3

import bisect
import math
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from geometry_msgs.msg import QuaternionStamped
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import ControlCommand


class TimeSeries:
    def __init__(self, maxlen):
        self.maxlen = maxlen
        self.t = deque(maxlen=maxlen)
        self.v = deque(maxlen=maxlen)

    def append(self, t, v):
        self.t.append(t)
        self.v.append(v)


class IsbNmpcLogPlotter:
    def __init__(self):
        self.window_sec = rospy.get_param("~window_sec", 30.0)
        self.refresh_hz = rospy.get_param("~refresh_hz", 10.0)
        self.max_samples = rospy.get_param("~max_samples", 20000)
        self.max_plot_points = rospy.get_param("~max_plot_points", 500)

        self.uav_state_topic = rospy.get_param("~uav_state_topic", "/nmpc/uavStateEstimate")
        self.target_state_topic = rospy.get_param("~target_state_topic", "/nmpc/targetStateEstimate")
        self.q_image_topic = rospy.get_param("~q_image_topic", "/nmpc/q_image")
        self.predicted_path_topic = rospy.get_param(
            "~predicted_path_topic", "/isb_nmpc/predicted_trajectory_vis")
        self.control_command_topic = rospy.get_param(
            "~control_command_topic", "/isb_nmpc/control_command")
        self.attitude_target_topic = rospy.get_param(
            "~attitude_target_topic", "/mavros/setpoint_raw/attitude")

        self.t0 = None
        self.series = {}
        for key in [
            "uav_px", "uav_py", "uav_pz", "target_px", "target_py", "target_pz",
            "pred_px", "pred_py", "pred_pz",
            "uav_vx", "uav_vy", "uav_vz", "target_vx", "target_vy", "target_vz",
            "uav_qw", "uav_qx", "uav_qy", "uav_qz",
            "cmd_qw", "cmd_qx", "cmd_qy", "cmd_qz",
            "img_qw", "img_qx", "img_qy", "img_qz",
            "cmd_thrust", "mavros_thrust",
            "cmd_wx", "cmd_wy", "cmd_wz", "mavros_wx", "mavros_wy", "mavros_wz",
            "err_p", "err_v", "img_axis_err",
        ]:
            self.series[key] = TimeSeries(self.max_samples)

        self.latest_uav_state = None
        self.latest_target_state = None
        self.latest_q_image = None

        rospy.Subscriber(self.uav_state_topic, Odometry,
                         self.uav_state_cb, queue_size=100)
        rospy.Subscriber(self.target_state_topic, Odometry,
                         self.target_state_cb, queue_size=100)
        rospy.Subscriber(self.q_image_topic, QuaternionStamped,
                         self.q_image_cb, queue_size=100)
        rospy.Subscriber(self.predicted_path_topic, Path,
                         self.predicted_path_cb, queue_size=20)
        rospy.Subscriber(self.control_command_topic, ControlCommand,
                         self.control_command_cb, queue_size=100)
        rospy.Subscriber(self.attitude_target_topic, AttitudeTarget,
                         self.attitude_target_cb, queue_size=100)

        self.fig, self.axes = plt.subplots(6, 1, figsize=(14, 16), sharex=True)
        self.fig.canvas.manager.set_window_title("ISB-NMPC Log Plot")
        self.fig.suptitle("ISB-NMPC Interception Debug", fontsize=14)

        self.lines = {}
        self._init_axes()
        self.anim = FuncAnimation(
            self.fig,
            self.redraw,
            interval=max(50, int(1000.0 / max(1.0, self.refresh_hz))),
            blit=False,
        )

    def _init_axes(self):
        colors = {"x": "tab:red", "y": "tab:green", "z": "tab:blue"}

        ax = self.axes[0]
        for axis in ["x", "y", "z"]:
            self.lines[f"target_p{axis}"], = ax.plot(
                [], [], "--", color=colors[axis], label=f"target p{axis}")
            self.lines[f"uav_p{axis}"], = ax.plot(
                [], [], "-", color=colors[axis], label=f"uav p{axis}")
            self.lines[f"pred_p{axis}"], = ax.plot(
                [], [], ":", color=colors[axis], label=f"pred p{axis}")
        ax.set_ylabel("Pos [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", ncol=3)

        ax = self.axes[1]
        for axis in ["x", "y", "z"]:
            self.lines[f"target_v{axis}"], = ax.plot(
                [], [], "--", color=colors[axis], label=f"target v{axis}")
            self.lines[f"uav_v{axis}"], = ax.plot(
                [], [], "-", color=colors[axis], label=f"uav v{axis}")
        ax.set_ylabel("Vel [m/s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", ncol=3)

        ax = self.axes[2]
        quat_colors = {"w": "black", "x": "tab:red", "y": "tab:green", "z": "tab:blue"}
        for axis in ["w", "x", "y", "z"]:
            self.lines[f"uav_q{axis}"], = ax.plot(
                [], [], "-", color=quat_colors[axis], label=f"uav q{axis}")
            self.lines[f"cmd_q{axis}"], = ax.plot(
                [], [], "--", color=quat_colors[axis], label=f"cmd q{axis}")
            self.lines[f"img_q{axis}"], = ax.plot(
                [], [], ":", color=quat_colors[axis], label=f"img q{axis}")
        ax.set_ylabel("Quat [-]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", ncol=4)

        ax = self.axes[3]
        self.lines["cmd_thrust"], = ax.plot(
            [], [], "-", color="tab:orange", label="cmd thrust [m/s^2]")
        self.lines["mavros_thrust"], = ax.plot(
            [], [], "--", color="tab:purple", label="mavros thrust [0-1]")
        ax.set_ylabel("Thrust")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left")

        ax = self.axes[4]
        for axis in ["x", "y", "z"]:
            self.lines[f"cmd_w{axis}"], = ax.plot(
                [], [], "-", color=colors[axis], label=f"cmd w{axis}")
            self.lines[f"mavros_w{axis}"], = ax.plot(
                [], [], "--", color=colors[axis], label=f"mavros w{axis}")
        ax.set_ylabel("Rate [rad/s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", ncol=3)

        ax = self.axes[5]
        self.lines["err_p"], = ax.plot([], [], color="tab:red", label="target distance")
        self.lines["err_v"], = ax.plot([], [], color="tab:blue", label="relative speed")
        self.lines["img_axis_err"], = ax.plot(
            [], [], color="black", label="q_image optical-axis angle")
        ax.set_ylabel("Error")
        ax.set_xlabel("Time [s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left")

    def _relative_time(self, stamp):
        t = stamp.to_sec()
        if t <= 0.0:
            t = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    @staticmethod
    def _quat_tuple(msg_q):
        return (msg_q.w, msg_q.x, msg_q.y, msg_q.z)

    @staticmethod
    def _quat_rotate(q, v):
        w, x, y, z = q
        vx, vy, vz = v
        # q * [0, v] * q^-1 for unit quaternions.
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (
            vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx,
        )

    @staticmethod
    def _angle_between(a, b):
        na = math.sqrt(sum(x * x for x in a))
        nb = math.sqrt(sum(x * x for x in b))
        if na < 1e-6 or nb < 1e-6:
            return 0.0
        dot = sum(x * y for x, y in zip(a, b)) / (na * nb)
        dot = max(-1.0, min(1.0, dot))
        return math.acos(dot)

    def _append_interception_errors(self, t_rel):
        if self.latest_uav_state is None or self.latest_target_state is None:
            return

        uav_p = self.latest_uav_state["p"]
        target_p = self.latest_target_state["p"]
        uav_v = self.latest_uav_state["v"]
        target_v = self.latest_target_state["v"]
        los = tuple(tp - up for up, tp in zip(uav_p, target_p))
        rel_v = tuple(tv - uv for uv, tv in zip(uav_v, target_v))

        self.series["err_p"].append(t_rel, math.sqrt(sum(x * x for x in los)))
        self.series["err_v"].append(t_rel, math.sqrt(sum(x * x for x in rel_v)))

    def uav_state_cb(self, msg):
        t_rel = self._relative_time(msg.header.stamp)
        self.latest_uav_state = {
            "p": (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ),
            "v": (
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ),
            "q": self._quat_tuple(msg.pose.pose.orientation),
        }

        for axis, value in zip(["x", "y", "z"], self.latest_uav_state["p"]):
            self.series[f"uav_p{axis}"].append(t_rel, value)
        for axis, value in zip(["x", "y", "z"], self.latest_uav_state["v"]):
            self.series[f"uav_v{axis}"].append(t_rel, value)
        for axis, value in zip(["w", "x", "y", "z"], self.latest_uav_state["q"]):
            self.series[f"uav_q{axis}"].append(t_rel, value)
        self._append_interception_errors(t_rel)

    def target_state_cb(self, msg):
        t_rel = self._relative_time(msg.header.stamp)
        self.latest_target_state = {
            "p": (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ),
            "v": (
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ),
        }

        for axis, value in zip(["x", "y", "z"], self.latest_target_state["p"]):
            self.series[f"target_p{axis}"].append(t_rel, value)
        for axis, value in zip(["x", "y", "z"], self.latest_target_state["v"]):
            self.series[f"target_v{axis}"].append(t_rel, value)
        self._append_interception_errors(t_rel)

    def q_image_cb(self, msg):
        t_rel = self._relative_time(msg.header.stamp)
        self.latest_q_image = self._quat_tuple(msg.quaternion)
        for axis, value in zip(["w", "x", "y", "z"], self.latest_q_image):
            self.series[f"img_q{axis}"].append(t_rel, value)
        image_ray = self._quat_rotate(self.latest_q_image, (0.0, 0.0, 1.0))
        self.series["img_axis_err"].append(
            t_rel, self._angle_between(image_ray, (0.0, 0.0, 1.0)))
        self._append_interception_errors(t_rel)

    def predicted_path_cb(self, msg):
        if not msg.poses:
            return
        pose = msg.poses[0]
        t_rel = self._relative_time(msg.header.stamp)
        self.series["pred_px"].append(t_rel, pose.pose.position.x)
        self.series["pred_py"].append(t_rel, pose.pose.position.y)
        self.series["pred_pz"].append(t_rel, pose.pose.position.z)

    def control_command_cb(self, msg):
        t_rel = self._relative_time(msg.header.stamp)
        q = self._quat_tuple(msg.orientation)
        for axis, value in zip(["w", "x", "y", "z"], q):
            self.series[f"cmd_q{axis}"].append(t_rel, value)
        self.series["cmd_thrust"].append(t_rel, msg.collective_thrust)
        self.series["cmd_wx"].append(t_rel, msg.bodyrates.x)
        self.series["cmd_wy"].append(t_rel, msg.bodyrates.y)
        self.series["cmd_wz"].append(t_rel, msg.bodyrates.z)

    def attitude_target_cb(self, msg):
        t_rel = self._relative_time(msg.header.stamp)
        self.series["mavros_thrust"].append(t_rel, msg.thrust)
        self.series["mavros_wx"].append(t_rel, msg.body_rate.x)
        self.series["mavros_wy"].append(t_rel, msg.body_rate.y)
        self.series["mavros_wz"].append(t_rel, msg.body_rate.z)

    def _set_line_data(self, key, left, right):
        s = self.series[key]
        if not s.t:
            self.lines[key].set_data([], [])
            return
        t_list = list(s.t)
        v_list = list(s.v)
        margin = 1.0
        lo = bisect.bisect_left(t_list, left - margin)
        hi = bisect.bisect_right(t_list, right + margin)
        t_win = t_list[lo:hi]
        v_win = v_list[lo:hi]
        n = len(t_win)
        if n > self.max_plot_points:
            step = n // self.max_plot_points
            t_win = t_win[::step]
            v_win = v_win[::step]
        self.lines[key].set_data(t_win, v_win)

    def redraw(self, _frame):
        xmax = None
        for key in self.lines:
            s = self.series[key]
            if not s.t:
                continue
            t_last = s.t[-1]
            if xmax is None or t_last > xmax:
                xmax = t_last

        if xmax is None:
            return []

        if xmax < self.window_sec:
            left = 0.0
            right = xmax + max(0.1, 0.05 * max(1.0, xmax))
        else:
            left = xmax - self.window_sec
            right = xmax + 0.1

        for key in self.lines:
            self._set_line_data(key, left, right)

        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(left, right)

        return list(self.lines.values())

    def run(self):
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    rospy.init_node("plot_isb_nmpc_online")
    IsbNmpcLogPlotter().run()
