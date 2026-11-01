"""PSLOS high-speed image-based visual-servoing SO(3) controller.

Adapted from SimpleDroneSimulator/control/high_speed_ibvs_so3_controller.py.
The reference implementation uses NED/FRD coordinates and thrust in newtons;
this adapter accepts this ROS stack's ENU/FLU state and returns FLU body rates
plus normalized MAVROS thrust.
"""

from dataclasses import dataclass

import numpy as np


R_NED_ENU = np.array(
    [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
    dtype=np.float64,
)
R_FRD_FLU = np.diag([1.0, -1.0, -1.0])


def _normalize(vector, eps=1e-9):
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vector))
    return np.zeros(3, dtype=np.float64) if norm < eps else vector / norm


def _skew(vector):
    x, y, z = np.asarray(vector, dtype=np.float64).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def _vex(matrix):
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]], dtype=np.float64)


def _clamp_norm(vector, maximum, eps=1e-12):
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vector))
    if norm <= maximum or norm < eps:
        return vector
    return vector * (maximum / norm)


def _rodrigues(axis_unit, angle):
    skew = _skew(axis_unit)
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew.dot(skew))


@dataclass
class PSLOSControllerParams:
    mass: float = 1.2
    g: float = 10.0
    hover_thrust: float = 0.3
    k1: float = 0.8
    k2: float = 1.4
    k_los: float = 1.5
    kb: float = 0.16
    k_att: float = 2.8
    accel_max: float = 6.0
    max_intercept_speed: float = 12.0
    speed_guard_band: float = 2.0
    speed_brake_gain: float = 4.0
    omega_max: float = 3.0
    omega_slew_max: float = 8.0
    thrust_min: float = 0.0
    thrust_max: float = 40.0
    designed_los_pitch_deg: float = 0.0
    los_distance_floor: float = 5.0
    eps: float = 1e-6


class PSLOSController:
    """Paper-style LOS barrier controller with an SO(3) rate inner loop."""

    def __init__(self, params):
        self.p = params
        self.reset()

    def reset(self):
        self._rd = np.eye(3, dtype=np.float64)
        self._force_des_ned = np.array(
            [0.0, 0.0, -self.p.mass * self.p.g], dtype=np.float64
        )
        self._omega1_frd = np.zeros(3, dtype=np.float64)
        self._omega_cmd_frd = np.zeros(3, dtype=np.float64)

    def _limit_speed_acceleration(self, accel_des_ned, uav_vel_ned):
        """Keep the commanded acceleration from driving speed beyond the limit.

        The governor only changes acceleration parallel to velocity, preserving
        the lateral LOS correction.  It fades propulsion over a guard band and
        adds proportional braking after an overspeed.
        """
        speed = float(np.linalg.norm(uav_vel_ned))
        speed_limit = max(float(self.p.max_intercept_speed), self.p.eps)
        if speed < self.p.eps:
            return accel_des_ned

        velocity_axis = uav_vel_ned / speed
        parallel_accel = float(np.dot(accel_des_ned, velocity_axis))
        if speed >= speed_limit:
            braking = max(
                0.0, float(self.p.speed_brake_gain) * (speed - speed_limit)
            )
            governed_parallel = min(parallel_accel, -braking)
            return accel_des_ned + (
                governed_parallel - parallel_accel
            ) * velocity_axis
        if parallel_accel <= 0.0:
            return accel_des_ned

        guard_band = float(np.clip(self.p.speed_guard_band, 0.0, speed_limit))
        guard_start = speed_limit - guard_band
        if speed < guard_start:
            return accel_des_ned

        if guard_band > self.p.eps and speed < speed_limit:
            allowed_fraction = (speed_limit - speed) / guard_band
            return accel_des_ned - (
                (1.0 - allowed_fraction) * parallel_accel * velocity_axis
            )

        return accel_des_ned - parallel_accel * velocity_axis

    def _update_outer_loop(
            self, r_ned_frd, relative_pos_ned, relative_vel_ned, uav_vel_ned):
        distance = max(float(np.linalg.norm(relative_pos_ned)), self.p.eps)
        target_los_ned = _normalize(-relative_pos_ned, self.p.eps)
        pitch = np.deg2rad(float(self.p.designed_los_pitch_deg))
        designed_los_frd = np.array([np.cos(pitch), 0.0, np.sin(pitch)])
        designed_los_ned = _normalize(r_ned_frd.dot(designed_los_frd), self.p.eps)

        z1_raw = float(1.0 - np.dot(designed_los_ned, target_los_ned))
        kb = max(float(self.p.kb), self.p.eps)
        z1 = float(np.clip(z1_raw, -0.98 * kb, 0.98 * kb))
        barrier = z1 / max(kb * kb - z1 * z1, self.p.eps)

        self._omega1_frd = self.p.k_los * barrier * (
            r_ned_frd.T.dot(np.cross(designed_los_ned, target_los_ned))
        )

        z2 = relative_vel_ned + self.p.k1 * relative_pos_ned
        projection = -np.eye(3) + np.outer(target_los_ned, target_los_ned)
        los_distance = max(
            distance, float(self.p.los_distance_floor), self.p.eps
        )
        accel_des_ned = (
            -self.p.k1 * relative_vel_ned
            - self.p.k2 * z2
            - relative_pos_ned
            + self.p.k_los * barrier * projection.dot(designed_los_ned) / los_distance
        )
        accel_des_ned = _clamp_norm(accel_des_ned, self.p.accel_max)
        accel_des_ned = self._limit_speed_acceleration(accel_des_ned, uav_vel_ned)
        accel_des_ned = _clamp_norm(accel_des_ned, self.p.accel_max)
        self._force_des_ned = self.p.mass * (
            accel_des_ned - np.array([0.0, 0.0, self.p.g])
        )

        thrust_axis_ned = _normalize(
            r_ned_frd.dot(np.array([0.0, 0.0, -1.0])), self.p.eps
        )
        desired_force_axis_ned = _normalize(self._force_des_ned, self.p.eps)
        axis = np.cross(thrust_axis_ned, desired_force_axis_ned)
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < self.p.eps:
            self._rd = r_ned_frd.copy()
            return
        angle = float(np.arccos(np.clip(
            np.dot(thrust_axis_ned, desired_force_axis_ned), -1.0, 1.0
        )))
        self._rd = _rodrigues(axis / axis_norm, angle).dot(r_ned_frd)

    def compute(self, pos_info, _pos_i=None, dt=None):
        r_enu_flu = np.asarray(pos_info["mav_R"], dtype=np.float64).reshape(3, 3)
        r_ned_frd = R_NED_ENU.dot(r_enu_flu).dot(R_FRD_FLU)
        relative_pos_enu = pos_info["mav_pos"] - pos_info["sphere_pos"]
        relative_vel_enu = pos_info["mav_vel"] - pos_info["sphere_vel"]
        relative_pos_ned = R_NED_ENU.dot(relative_pos_enu)
        relative_vel_ned = R_NED_ENU.dot(relative_vel_enu)
        uav_vel_ned = R_NED_ENU.dot(pos_info["mav_vel"])

        self._update_outer_loop(
            r_ned_frd, relative_pos_ned, relative_vel_ned, uav_vel_ned
        )
        attitude_error = self._rd.T.dot(r_ned_frd) - r_ned_frd.T.dot(self._rd)
        omega_frd = self._omega1_frd - self.p.k_att * _vex(attitude_error)
        omega_frd = _clamp_norm(omega_frd, self.p.omega_max)
        if dt is not None and dt > 0.0 and self.p.omega_slew_max > 0.0:
            omega_delta = omega_frd - self._omega_cmd_frd
            omega_delta = _clamp_norm(
                omega_delta, float(self.p.omega_slew_max) * float(dt)
            )
            omega_frd = self._omega_cmd_frd + omega_delta
        self._omega_cmd_frd = omega_frd.copy()

        thrust_axis_ned = _normalize(
            r_ned_frd.dot(np.array([0.0, 0.0, -1.0])), self.p.eps
        )
        thrust_newtons = float(np.clip(
            np.dot(thrust_axis_ned, self._force_des_ned),
            self.p.thrust_min,
            self.p.thrust_max,
        ))
        if thrust_newtons <= self.p.eps:
            vertical_authority = float(np.clip(
                np.dot(thrust_axis_ned, np.array([0.0, 0.0, -1.0])),
                0.2,
                1.0,
            ))
            thrust_newtons = float(np.clip(
                self.p.mass * self.p.g / vertical_authority,
                self.p.thrust_min,
                self.p.thrust_max,
            ))
        normalized_thrust = (
            thrust_newtons / (self.p.mass * self.p.g) * self.p.hover_thrust
            if self.p.mass * self.p.g > self.p.eps else 0.0
        )
        omega_flu = R_FRD_FLU.dot(omega_frd)
        return [omega_flu[0], omega_flu[1], omega_flu[2], normalized_thrust]
