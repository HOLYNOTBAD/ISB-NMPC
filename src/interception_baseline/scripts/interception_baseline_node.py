#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import ControlCommand, FsmState

from interception_baseline.controllers.attack import (
    BasicAttackController,
    RotateAttackAcceleration2VelController,
    RotateAttackAccelerationController,
    RotateAttackAccelerationController2,
    RotateAttackController,
    RotateHighspeedAttackController,
)
from interception_baseline.controllers.backstepping import BacksteppingController
from interception_baseline.controllers.docking import DockingController
from interception_baseline.controllers.pslos import PSLOSController, PSLOSControllerParams


R_FLU_LEGACY = np.array(
    [
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def vector_from_msg(msg):
    return np.array([msg.x, msg.y, msg.z], dtype=np.float64)


def quat_to_rot(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [w * w + x * x - y * y - z * z, 2.0 * (x * y - w * z),
             2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), w * w - x * x + y * y - z * z,
             2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x),
             w * w - x * x - y * y + z * z],
        ],
        dtype=np.float64,
    )


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def matrix_param(name, default):
    values = rospy.get_param(name, default)
    if values is None or len(values) != 9:
        raise ValueError("{} must contain 9 numbers".format(name))
    return np.array(values, dtype=np.float64).reshape(3, 3)


def vector_param(name, default):
    values = rospy.get_param(name, default)
    if len(values) != 3:
        raise ValueError("{} must contain 3 numbers".format(name))
    return np.array(values, dtype=np.float64)


class InterceptionBaselineNode:
    def __init__(self):
        self.control_rate = float(rospy.get_param("~control_rate",
                                                  rospy.get_param("/control_rate", 50.0)))
        self.attitude_target_topic = rospy.get_param(
            "~attitude_target_topic",
            rospy.get_param("/attitude_target_topic", "/mavros/setpoint_raw/attitude"),
        )
        self.velocity_target_topic = rospy.get_param(
            "~velocity_target_topic",
            rospy.get_param("/velocity_target_topic", "/mavros/setpoint_velocity/cmd_vel"),
        )
        self.acceleration_target_topic = rospy.get_param(
            "~acceleration_target_topic",
            rospy.get_param("/acceleration_target_topic", "/mavros/setpoint_raw/local"),
        )
        self.state_estimate_topic = rospy.get_param(
            "~state_estimate_topic",
            rospy.get_param("/state_estimate_topic", "/nmpc/uavStateEstimate"),
        )
        self.target_state_estimate_topic = rospy.get_param(
            "~target_state_estimate_topic",
            rospy.get_param("/target_state_estimate_topic", "/nmpc/targetStateEstimate"),
        )
        self.p_image_topic = rospy.get_param(
            "~p_image_topic", rospy.get_param("/p_image_topic", "/nmpc/p_image")
        )
        self.fsm_state_topic = rospy.get_param(
            "~fsm_state_topic", rospy.get_param("/fsm_state_topic", "/fsm/fsm_state")
        )

        self.controller_type = rospy.get_param(
            "~controller_type", rospy.get_param("/controller_type", "backstepping")
        )

        self.hover_thrust = float(rospy.get_param("~hover_thrust",
                                                  rospy.get_param("/hover_thrust", 0.72)))
        self.hover_acceleration = float(rospy.get_param(
            "~hover_acceleration", rospy.get_param("/hover_acceleration", 9.81)))
        self.mass = float(rospy.get_param("~mass", rospy.get_param("/mass", 1.55)))
        self.drag_coefficient = float(rospy.get_param(
            "~drag_coefficient", rospy.get_param("/drag_coefficient", 0.073)))
        self.max_body_rate = float(rospy.get_param(
            "~max_body_rate", rospy.get_param("/max_body_rate",
                                              rospy.get_param("/max_rate", 3.0))))
        self.max_body_rate_x = float(rospy.get_param(
            "~max_body_rate_x", rospy.get_param("/max_body_rate_x",
                                                 self.max_body_rate)))
        self.max_body_rate_y = float(rospy.get_param(
            "~max_body_rate_y", rospy.get_param("/max_body_rate_y",
                                                 self.max_body_rate)))
        self.max_body_rate_z = float(rospy.get_param(
            "~max_body_rate_z", rospy.get_param("/max_body_rate_z",
                                                 self.max_body_rate)))
        self.max_acceleration = float(rospy.get_param(
            "~max_acceleration", rospy.get_param("/max_acceleration", 6.0)))
        self.image_timeout = float(rospy.get_param(
            "~image_timeout", rospy.get_param("/image_timeout", 0.5)))
        self.state_timeout = float(rospy.get_param(
            "~state_timeout", rospy.get_param("/state_timeout", 0.5)))
        self.target_timeout = float(rospy.get_param(
            "~target_timeout", rospy.get_param("/target_timeout", 0.5)))
        self.p_image_is_normalized = bool(rospy.get_param(
            "~p_image_is_normalized", rospy.get_param("/p_image_is_normalized", True)))

        img_width = float(rospy.get_param("/camera/img_width", 640.0))
        img_height = float(rospy.get_param("/camera/img_height", 480.0))
        u0 = float(rospy.get_param("/camera/cx", 0.5 * img_width))
        v0 = float(rospy.get_param("/camera/cy", 0.5 * img_height))
        f = float(rospy.get_param("/camera/img_f", 115.0))
        self.u0 = u0
        self.v0 = v0
        self.f = f
        self.img_width = img_width
        self.img_height = img_height
        self.image_center = np.array([u0, v0], dtype=np.float64)

        self.r_cb_flu = matrix_param("/camera/R_cam_body", None)
        self.r_cb_legacy = R_FLU_LEGACY.T.dot(self.r_cb_flu)
        self.n_cc = vector_param("~n_cc", rospy.get_param("/n_cc", [0.0, 0.0, 1.0]))

        self.controller, self.controller_output_mode = self.make_controller(u0, v0, f)

        self.latest_state: Optional[Odometry] = None
        self.latest_target: Optional[Odometry] = None
        self.latest_p_image: Optional[PointStamped] = None
        self.latest_target_acc = np.zeros(3, dtype=np.float64)
        self.last_target_vel: Optional[np.ndarray] = None
        self.last_target_stamp: Optional[rospy.Time] = None
        self.interception_active = False
        self.last_control_time: Optional[rospy.Time] = None

        self.state_sub = rospy.Subscriber(
            self.state_estimate_topic, Odometry, self.state_callback, queue_size=10
        )
        self.target_sub = rospy.Subscriber(
            self.target_state_estimate_topic, Odometry, self.target_callback, queue_size=10
        )
        self.p_image_sub = rospy.Subscriber(
            self.p_image_topic, PointStamped, self.p_image_callback, queue_size=10
        )
        self.fsm_sub = rospy.Subscriber(
            self.fsm_state_topic, FsmState, self.fsm_state_callback, queue_size=10
        )
        self.attitude_pub = rospy.Publisher(
            self.attitude_target_topic, AttitudeTarget, queue_size=10
        )
        self.velocity_pub = rospy.Publisher(
            self.velocity_target_topic, TwistStamped, queue_size=10
        )
        self.acceleration_pub = rospy.Publisher(
            self.acceleration_target_topic, PositionTarget, queue_size=10
        )
        self.control_command_pub = rospy.Publisher(
            "/interception_baseline/control_command", ControlCommand, queue_size=10
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(1.0, self.control_rate)), self.control_loop
        )

        rospy.loginfo(
            "interception_baseline_node initialized: state=%s target=%s p_image=%s "
            "fsm=%s attitude=%s velocity=%s acceleration=%s controller=%s output_mode=%s",
            self.state_estimate_topic,
            self.target_state_estimate_topic,
            self.p_image_topic,
            self.fsm_state_topic,
            self.attitude_target_topic,
            self.velocity_target_topic,
            self.acceleration_target_topic,
            self.controller_type,
            self.controller_output_mode,
        )

    def make_controller(self, u0, v0, f):
        controller_type = self.controller_type
        if controller_type == "backstepping":
            return BacksteppingController(
                u0=u0,
                v0=v0,
                f=f,
                n_cc=self.n_cc,
                image_width=self.img_width,
                image_height=self.img_height,
                image_bound_margin=float(rospy.get_param(
                    "~backstepping_image_bound_margin",
                    rospy.get_param("/backstepping_image_bound_margin", 0.3))),
                mass=self.mass,
                C_d=self.drag_coefficient,
                thrust_hover=self.hover_thrust,
                k_b=float(rospy.get_param("~backstepping_k_b",
                                          rospy.get_param("/backstepping_k_b", -1.0))),
                c_1=float(rospy.get_param("~backstepping_c_1",
                                          rospy.get_param("/backstepping_c_1", 1.0))),
                c_2=float(rospy.get_param("~backstepping_c_2",
                                          rospy.get_param("/backstepping_c_2", 0.8))),
                c_3=float(rospy.get_param("~backstepping_c_3",
                                          rospy.get_param("/backstepping_c_3", 0.3))),
                approach_speed=float(rospy.get_param(
                    "~backstepping_approach_speed",
                    rospy.get_param("/backstepping_approach_speed", 10.0))),
                omega_damping=float(rospy.get_param(
                    "~backstepping_omega_damping",
                    rospy.get_param("/backstepping_omega_damping", 0.1))),
                omega_max=float(rospy.get_param(
                    "~backstepping_omega_max",
                    rospy.get_param("/backstepping_omega_max", 2.5))),
                omega_slew_max=float(rospy.get_param(
                    "~backstepping_omega_slew_max",
                    rospy.get_param("/backstepping_omega_slew_max", 12.0))),
                omega_filter_tau=float(rospy.get_param(
                    "~backstepping_omega_filter_tau",
                    rospy.get_param("/backstepping_omega_filter_tau", 0.08))),
                barrier_gain_max=float(rospy.get_param(
                    "~backstepping_barrier_gain_max",
                    rospy.get_param("/backstepping_barrier_gain_max", 20.0))),
                tilt_max_deg=float(rospy.get_param(
                    "~backstepping_tilt_max_deg",
                    rospy.get_param("/backstepping_tilt_max_deg", 20.0))),
                los_distance_floor=float(rospy.get_param(
                    "~backstepping_los_distance_floor",
                    rospy.get_param("/backstepping_los_distance_floor", 5.0))),
                max_intercept_speed=float(rospy.get_param(
                    "~backstepping_max_intercept_speed",
                    rospy.get_param("/backstepping_max_intercept_speed", 12.0))),
                speed_guard_band=float(rospy.get_param(
                    "~backstepping_speed_guard_band",
                    rospy.get_param("/backstepping_speed_guard_band", 2.0))),
                speed_brake_gain=float(rospy.get_param(
                    "~backstepping_speed_brake_gain",
                    rospy.get_param("/backstepping_speed_brake_gain", 3.0))),
                attitude_gain=float(rospy.get_param(
                    "~backstepping_attitude_gain",
                    rospy.get_param("/backstepping_attitude_gain", 2.5))),
            ), "bodyrate_thrust"
        if controller_type == "pslos":
            def pslos_param(name, default):
                return float(rospy.get_param(
                    "~pslos_{}".format(name),
                    rospy.get_param("/pslos_{}".format(name), default),
                ))

            return PSLOSController(PSLOSControllerParams(
                mass=self.mass,
                g=self.hover_acceleration,
                hover_thrust=self.hover_thrust,
                k1=pslos_param("k1", 0.8),
                k2=pslos_param("k2", 1.4),
                k_los=pslos_param("k_los", 1.5),
                kb=pslos_param("kb", 0.16),
                k_att=pslos_param("k_att", 2.8),
                accel_max=pslos_param("accel_max", 6.0),
                max_intercept_speed=pslos_param("max_intercept_speed", 12.0),
                speed_guard_band=pslos_param("speed_guard_band", 2.0),
                speed_brake_gain=pslos_param("speed_brake_gain", 4.0),
                omega_max=pslos_param("omega_max", 3.0),
                omega_slew_max=pslos_param("omega_slew_max", 8.0),
                thrust_min=pslos_param("thrust_min", 0.0),
                thrust_max=pslos_param("thrust_max", 40.0),
                designed_los_pitch_deg=pslos_param("designed_los_pitch_deg", 0.0),
                los_distance_floor=pslos_param("los_distance_floor", 5.0),
            )), "bodyrate_thrust"
        if controller_type == "basic_attack":
            return BasicAttackController(), "velocity_yawrate"
        if controller_type == "rotate_attack":
            return RotateAttackController(
                u0,
                v0,
                f,
                self.r_cb_legacy,
                self.n_cc,
                forward_ramp=float(rospy.get_param(
                    "~rotate_attack_forward_ramp",
                    rospy.get_param("/rotate_attack_forward_ramp", 0.03))),
                forward_speed_max=float(rospy.get_param(
                    "~rotate_attack_forward_speed_max",
                    rospy.get_param("/rotate_attack_forward_speed_max", 10.0))),
                lateral_gain=float(rospy.get_param(
                    "~rotate_attack_lateral_gain",
                    rospy.get_param("/rotate_attack_lateral_gain", 12.0))),
                vertical_gain=float(rospy.get_param(
                    "~rotate_attack_vertical_gain",
                    rospy.get_param("/rotate_attack_vertical_gain", 10.0))),
                yaw_gain=float(rospy.get_param(
                    "~rotate_attack_yaw_gain",
                    rospy.get_param("/rotate_attack_yaw_gain", 0.002))),
            ), "velocity_yawrate"
        if controller_type == "rotate_highspeed_attack":
            v_norm_d = float(rospy.get_param("~v_norm_d", rospy.get_param("/v_norm_d", 10.0)))
            max_speed = float(rospy.get_param(
                "~rotate_highspeed_max_speed",
                rospy.get_param("/rotate_highspeed_max_speed", 12.0)))
            return RotateHighspeedAttackController(
                u0, v0, f, self.r_cb_legacy, self.n_cc,
                v_norm_d=v_norm_d, max_speed=max_speed
            ), "velocity_yawrate"
        if controller_type == "rotate_attack_accel":
            return RotateAttackAccelerationController(
                u0, v0, f, self.r_cb_legacy, self.n_cc
            ), "acceleration_yawrate"
        if controller_type == "rotate_attack_accel2":
            return RotateAttackAccelerationController2(
                u0, v0, f, self.r_cb_legacy, self.n_cc
            ), "acceleration_yawrate"
        if controller_type == "rotate_attack_accel2vel":
            return RotateAttackAcceleration2VelController(
                u0, v0, f, self.r_cb_legacy, self.n_cc
            ), "velocity_yawrate"
        if controller_type == "docking":
            kvod = float(rospy.get_param("~docking_kvod", rospy.get_param("/docking_kvod", 2.5)))
            v_approach = float(rospy.get_param("~docking_v_approach",
                                               rospy.get_param("/docking_v_approach", 1.0)))
            return DockingController(
                u0, v0, f, self.r_cb_legacy, self.n_cc, kvod=kvod,
                v_approach=v_approach,
            ), "velocity_yawrate"

        raise ValueError(
            "Unsupported controller_type '{}'. Expected one of: backstepping, pslos, "
            "basic_attack, rotate_attack, rotate_highspeed_attack, "
            "rotate_attack_accel, rotate_attack_accel2, rotate_attack_accel2vel, "
            "docking.".format(controller_type)
        )

    def state_callback(self, msg):
        self.latest_state = msg

    def target_callback(self, msg):
        target_vel = vector_from_msg(msg.twist.twist.linear)
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        if self.last_target_vel is not None and self.last_target_stamp is not None:
            dt = (stamp - self.last_target_stamp).to_sec()
            if dt > 1e-4:
                self.latest_target_acc = (target_vel - self.last_target_vel) / dt
        self.last_target_vel = target_vel
        self.last_target_stamp = stamp
        self.latest_target = msg

    def p_image_callback(self, msg):
        self.latest_p_image = msg

    def fsm_state_callback(self, msg):
        was_active = self.interception_active
        self.interception_active = msg.state_name == "EXEC_INTERCEPTION"
        if self.interception_active and not was_active:
            if hasattr(self.controller, "reset"):
                self.controller.reset()
            self.last_control_time = None
            rospy.loginfo("interception_baseline_node: EXEC_INTERCEPTION received.")
        elif was_active and not self.interception_active:
            rospy.loginfo("interception_baseline_node: left EXEC_INTERCEPTION.")

    def msg_is_fresh(self, msg, timeout, now):
        if msg is None:
            return False
        if msg.header.stamp == rospy.Time(0):
            return True
        return (now - msg.header.stamp).to_sec() <= timeout

    def build_pos_info(self):
        state = self.latest_state
        mav_r_flu = quat_to_rot(state.pose.pose.orientation)
        if self.controller_type in ("backstepping", "pslos"):
            mav_r = mav_r_flu
            r_cb = self.r_cb_flu
        else:
            mav_r = mav_r_flu.dot(R_FLU_LEGACY)
            r_cb = self.r_cb_legacy
        pos_info = {
            "mav_pos": vector_from_msg(state.pose.pose.position),
            "mav_vel": vector_from_msg(state.twist.twist.linear),
            "mav_R": mav_r,
            "R_cb": r_cb,
            "mav_yaw": yaw_from_quat(state.pose.pose.orientation),
            "mav_original_angle": [yaw_from_quat(state.pose.pose.orientation), 0.0, 0.0],
        }
        if self.controller_type in ("backstepping", "pslos"):
            target = self.latest_target
            pos_info.update({
                "sphere_pos": vector_from_msg(target.pose.pose.position),
                "sphere_vel": vector_from_msg(target.twist.twist.linear),
                "sphere_acc": self.latest_target_acc,
            })
        return pos_info

    def build_pos_i(self):
        point = self.latest_p_image.point
        if self.p_image_is_normalized:
            pos_i = np.array([
                self.u0 + point.x * self.f,
                self.v0 + point.y * self.f,
                point.z,
            ], dtype=np.float64)
        else:
            pos_i = np.array([point.x, point.y, 0.0], dtype=np.float64)
        return pos_i

    def run_controller(self, pos_info, pos_i, dt, controller_reset):
        if self.controller_type == "backstepping":
            return self.controller.compute(pos_info, pos_i, dt)
        if self.controller_type == "pslos":
            return self.controller.compute(pos_info, pos_i, dt)
        if self.controller_type == "basic_attack":
            return self.controller.compute(pos_info, pos_i, self.image_center)
        if self.controller_type in ("rotate_attack", "rotate_highspeed_attack"):
            return self.controller.compute(pos_info, pos_i, self.image_center, controller_reset)
        if self.controller_type in ("rotate_attack_accel", "rotate_attack_accel2",
                                    "rotate_attack_accel2vel"):
            return self.controller.compute(pos_info, pos_i, controller_reset)
        if self.controller_type == "docking":
            return self.controller.compute(pos_info, pos_i)
        raise ValueError("Unsupported controller_type '{}'".format(self.controller_type))

    def split_controller_command(self, cmd):
        command_vector = np.array(
            [float(np.asarray(cmd[i]).reshape(-1)[0]) for i in range(3)],
            dtype=np.float64,
        )
        yaw_rate = float(np.asarray(cmd[3]).reshape(-1)[0])
        return command_vector, yaw_rate

    def publish_control_command(self, stamp, bodyrates, thrust):
        msg = ControlCommand()
        msg.header.stamp = stamp
        msg.control_mode = ControlCommand.BODY_RATES
        msg.armed = True
        msg.expected_execution_time = stamp
        msg.orientation.w = 1.0
        msg.bodyrates.x = bodyrates[0]
        msg.bodyrates.y = bodyrates[1]
        msg.bodyrates.z = bodyrates[2]
        if self.hover_thrust > 1e-6:
            msg.collective_thrust = thrust / self.hover_thrust * self.hover_acceleration
        else:
            msg.collective_thrust = 0.0
        self.control_command_pub.publish(msg)

    def publish_attitude_target(self, stamp, bodyrates, thrust):
        target = AttitudeTarget()
        target.header.stamp = stamp
        target.header.frame_id = "base_link"
        target.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        target.orientation.w = 1.0
        target.body_rate.x = bodyrates[0]
        target.body_rate.y = bodyrates[1]
        target.body_rate.z = bodyrates[2]
        target.thrust = clamp(thrust, 0.0, 1.0)
        self.attitude_pub.publish(target)

    def publish_velocity_target(self, stamp, velocity, yaw_rate):
        target = TwistStamped()
        target.header.stamp = stamp
        target.header.frame_id = "map"
        target.twist.linear.x = velocity[0]
        target.twist.linear.y = velocity[1]
        target.twist.linear.z = velocity[2]
        target.twist.angular.z = clamp(
            yaw_rate, -self.max_body_rate_z, self.max_body_rate_z)
        self.velocity_pub.publish(target)

    def publish_acceleration_target(self, stamp, acceleration, yaw_rate):
        norm = np.linalg.norm(acceleration)
        if norm > self.max_acceleration:
            acceleration = acceleration * (self.max_acceleration / norm)

        target = PositionTarget()
        target.header.stamp = stamp
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX
            + PositionTarget.IGNORE_PY
            + PositionTarget.IGNORE_PZ
            + PositionTarget.IGNORE_VX
            + PositionTarget.IGNORE_VY
            + PositionTarget.IGNORE_VZ
            + PositionTarget.IGNORE_YAW
        )
        target.acceleration_or_force.x = acceleration[0]
        target.acceleration_or_force.y = acceleration[1]
        target.acceleration_or_force.z = acceleration[2]
        target.yaw_rate = clamp(
            yaw_rate, -self.max_body_rate_z, self.max_body_rate_z)
        self.acceleration_pub.publish(target)

    def publish_controller_output(self, stamp, cmd):
        command_vector, fourth = self.split_controller_command(cmd)

        if self.controller_output_mode == "bodyrate_thrust":
            limits = np.array([
                self.max_body_rate_x,
                self.max_body_rate_y,
                self.max_body_rate_z,
            ], dtype=np.float64)
            bodyrates = np.clip(command_vector, -limits, limits)
            self.publish_control_command(stamp, bodyrates, fourth)
            self.publish_attitude_target(stamp, bodyrates, fourth)
            return
        if self.controller_output_mode == "velocity_yawrate":
            self.publish_velocity_target(stamp, command_vector, fourth)
            return
        if self.controller_output_mode == "acceleration_yawrate":
            self.publish_acceleration_target(stamp, command_vector, fourth)
            return
        raise ValueError("Unsupported controller output mode '{}'".format(
            self.controller_output_mode
        ))

    def publish_safe_empty_command(self, stamp):
        msg = ControlCommand()
        msg.header.stamp = stamp
        msg.control_mode = ControlCommand.NONE
        msg.expected_execution_time = stamp
        msg.orientation.w = 1.0
        self.control_command_pub.publish(msg)

    def control_loop(self, _event):
        now = rospy.Time.now()
        if not self.interception_active:
            self.publish_safe_empty_command(now)
            return

        if not self.msg_is_fresh(self.latest_state, self.state_timeout, now):
            rospy.logwarn_throttle(
                1.0, "interception_baseline_node: waiting for fresh UAV state estimate."
            )
            self.publish_safe_empty_command(now)
            return
        if (self.controller_type in ("backstepping", "pslos") and
                not self.msg_is_fresh(self.latest_target, self.target_timeout, now)):
            rospy.logwarn_throttle(
                1.0, "interception_baseline_node: waiting for fresh target state estimate."
            )
            self.publish_safe_empty_command(now)
            return
        image_fresh = self.msg_is_fresh(self.latest_p_image, self.image_timeout, now)
        if not image_fresh and self.controller_type != "backstepping":
            rospy.logwarn_throttle(
                1.0, "interception_baseline_node: waiting for fresh p_image."
            )
            self.publish_safe_empty_command(now)
            return

        if self.last_control_time is None:
            dt = 1.0 / max(1.0, self.control_rate)
            controller_reset = True
        else:
            dt = max(1e-3, (now - self.last_control_time).to_sec())
            controller_reset = False
        self.last_control_time = now

        try:
            pos_info = self.build_pos_info()
            pos_i = self.build_pos_i() if image_fresh else None
            cmd = self.run_controller(pos_info, pos_i, dt, controller_reset)
            self.publish_controller_output(now, cmd)
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0, "interception_baseline_node: controller failed: %s", exc
            )
            self.publish_safe_empty_command(now)
            return


def main():
    rospy.init_node("interception_baseline_node")
    try:
        InterceptionBaselineNode()
    except Exception as exc:
        rospy.logerr("interception_baseline_node failed to initialize: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
