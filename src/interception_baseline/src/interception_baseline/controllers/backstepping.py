import numpy as np

from ..utils.math_utils import skew


class BacksteppingController:
    def __init__(self, u0, v0, f, n_cc, image_width=640.0, image_height=480.0,
                 image_bound_margin=0.3, mass=1.4, C_d=0.073, thrust_hover=0.609,
                 k_b=-1.0, c_1=1.0, c_2=0.8, c_3=0.3,
                 approach_speed=10.0, omega_damping=0.1,
                 omega_max=2.5, omega_slew_max=12.0,
                 omega_filter_tau=0.08, barrier_gain_max=20.0,
                 tilt_max_deg=20.0, los_distance_floor=5.0,
                 max_intercept_speed=12.0, speed_guard_band=2.0,
                 speed_brake_gain=3.0, attitude_gain=2.5):
        self.u0 = u0
        self.v0 = v0
        self.f = f
        self.n_cc = n_cc
        self.m = mass
        self.C_d = C_d
        self.thrust_hover = thrust_hover
        if k_b > 0.0:
            self.k_b = k_b
        else:
            radius_u = 0.5 * float(image_width) / self.f - image_bound_margin
            radius_v = 0.5 * float(image_height) / self.f - image_bound_margin
            safe_radius = max(min(radius_u, radius_v), 1e-3)
            self.k_b = 1.0 - 1.0 / np.sqrt(1.0 + safe_radius * safe_radius)
        self.c_1 = c_1
        self.c_2 = c_2
        self.c_3 = c_3
        self.approach_speed = approach_speed
        self.omega_damping = omega_damping
        self.omega_max = omega_max
        self.omega_slew_max = omega_slew_max
        self.omega_filter_tau = omega_filter_tau
        self.barrier_gain_max = barrier_gain_max
        self.tilt_max_deg = tilt_max_deg
        self.los_distance_floor = los_distance_floor
        self.max_intercept_speed = max_intercept_speed
        self.speed_guard_band = speed_guard_band
        self.speed_brake_gain = speed_brake_gain
        self.attitude_gain = attitude_gain
        self.last_mav_vel = None
        self.last_omega = np.zeros(3, dtype=np.float64)
        self.filtered_omega = np.zeros(3, dtype=np.float64)

    def _limit_speed_force(self, force_des, mav_vel, gravity):
        """Limit acceleration along velocity while retaining LOS authority."""
        speed_limit = max(float(self.max_intercept_speed), 0.0)
        speed = float(np.linalg.norm(mav_vel))
        if speed_limit <= 0.0 or speed < 1e-6:
            return force_des

        velocity_axis = mav_vel / speed
        accel_des = force_des / self.m + gravity
        parallel_accel = float(accel_des.T.dot(velocity_axis).item())

        if speed >= speed_limit:
            braking = max(0.0, float(self.speed_brake_gain) * (speed - speed_limit))
            governed_parallel = min(parallel_accel, -braking)
        else:
            if parallel_accel <= 0.0:
                return force_des
            guard_band = float(np.clip(self.speed_guard_band, 0.0, speed_limit))
            guard_start = speed_limit - guard_band
            if speed < guard_start:
                return force_des
            governed_parallel = parallel_accel * (speed_limit - speed) / max(
                guard_band, 1e-6)

        accel_des += (governed_parallel - parallel_accel) * velocity_axis
        return self.m * (accel_des - gravity)

    def compute(self, pos_info, pos_i, dt):
        m = self.m
        g = np.array([[0], [0], [-9.801]], dtype=np.float64)
        C_d = self.C_d
        thrust_hover = self.thrust_hover

        mav_pos = pos_info["mav_pos"].reshape(-1, 1)
        mav_vel = pos_info["mav_vel"].reshape(-1, 1)
        mav_acc = (mav_vel - self.last_mav_vel) / dt \
            if self.last_mav_vel is not None else np.zeros((3, 1), dtype=np.float64)
        self.last_mav_vel = mav_vel
        sphere_pos = pos_info["sphere_pos"].reshape(-1, 1)
        sphere_vel = pos_info["sphere_vel"].reshape(-1, 1)
        sphere_acc = pos_info["sphere_acc"].reshape(-1, 1)

        n_c_in_c = self.n_cc.reshape(-1, 1)
        n_c_in_b = pos_info["R_cb"].dot(n_c_in_c)
        n_c_in_e = pos_info["mav_R"].dot(n_c_in_b)

        n_td_in_e = n_c_in_e

        # L_1
        k_b = self.k_b
        p_r = mav_pos - sphere_pos
        distance = max(float(np.linalg.norm(p_r)), 1e-6)
        n_t_in_e_precise = -p_r / distance
        if pos_i is not None:
            n_t_in_c = np.array(
                [[pos_i[0] - self.u0], [pos_i[1] - self.v0], [self.f]],
                dtype=np.float64)
            n_t_in_c /= max(float(np.linalg.norm(n_t_in_c)), 1e-6)
            n_t_in_b = pos_info["R_cb"].dot(n_t_in_c)
            n_t_in_e_visual = pos_info["mav_R"].dot(n_t_in_b)
        else:
            n_t_in_e_visual = n_t_in_e_precise
        z_1 = 1 - n_td_in_e.T.dot(n_t_in_e_visual)
        if z_1 >= k_b:
            z_1 = k_b - 1e-3
        L_1 = 0.5 * np.log(k_b*k_b / (k_b*k_b - z_1*z_1))
        K = z_1 / (k_b*k_b - z_1*z_1)
        if self.barrier_gain_max > 0.0:
            K = np.clip(K, -self.barrier_gain_max, self.barrier_gain_max)

        # L_2
        c_1 = self.c_1
        z_2 = p_r
        v_r = mav_vel - sphere_vel
        L_2 = L_1 + 0.5 * z_2.T.dot(z_2)
        alpha_1 = self.approach_speed * n_t_in_e_precise

        # L_3
        c_2 = self.c_2
        z_3 = v_r - alpha_1
        f_drag = -C_d * mav_vel.T.dot(mav_vel)
        L_3 = L_2 + 0.5 * z_3.T.dot(z_3)
        los_distance = max(distance, float(self.los_distance_floor), 1e-6)
        alpha_2 = -c_2*m*z_3 - m*g - c_1*m*v_r + m*n_t_in_e_precise + K*m/los_distance*(-np.identity(3)+n_t_in_e_visual.dot(n_t_in_e_visual.T)).dot(n_td_in_e)
        alpha_2 = self._limit_speed_force(alpha_2, mav_vel, g)

        # A forward-looking camera cannot retain the target if the desired
        # thrust vector pitches the vehicle beyond the vertical FOV.  Limit
        # horizontal thrust while preserving vertical authority.
        if 0.0 < self.tilt_max_deg < 89.0:
            vertical_force = max(float(alpha_2[2, 0]), 1e-6)
            horizontal_force = alpha_2[:2, 0]
            horizontal_norm = float(np.linalg.norm(horizontal_force))
            horizontal_max = vertical_force * np.tan(np.deg2rad(self.tilt_max_deg))
            if horizontal_norm > horizontal_max:
                alpha_2[:2, 0] = horizontal_force * (horizontal_max / horizontal_norm)

        e_3 = np.array([[0], [0], [1]], dtype=np.float64)
        n_f = pos_info["mav_R"].dot(e_3)
        f_d = np.linalg.norm(alpha_2)
        thrust_d = f_d / m / np.linalg.norm(g) * thrust_hover

        # L_4
        c_3 = self.c_3
        z_4 = 1. / m * (f_d * n_f - alpha_2)
        L_4 = L_3 + z_4.T.dot(z_4)

        a_r = mav_acc - sphere_acc
        z_2_dot = v_r
        z_3_dot = f_d/m*n_f + g + c_1*v_r
        alpha_2_dot = -c_2*m*z_3_dot - c_1*m*a_r - m*z_2_dot
        n_f_x = skew(n_f)
        A = K*np.cross(n_td_in_e.T, n_t_in_e_visual.T) + f_d / m * z_4.T.dot(n_f_x)
        B = z_4.T.dot(z_3 - 1./m*alpha_2_dot + c_3*z_4)

        # A is a 1x3 row vector.  A damped least-squares inverse avoids the
        # large rate spikes produced by pinv(A) when the LOS starts nearly
        # aligned and A is close to zero.
        damping_sq = max(float(self.omega_damping), 0.0) ** 2
        denominator = float(A.dot(A.T).item()) + damping_sq
        desired_force_axis = alpha_2 / max(float(np.linalg.norm(alpha_2)), 1e-9)
        omega_attitude_in_e = self.attitude_gain * np.cross(
            n_f.reshape(3), desired_force_axis.reshape(3)).reshape(3, 1)
        residual = float(np.asarray(B - A.dot(omega_attitude_in_e)).item())
        omega_in_e = omega_attitude_in_e + A.T * (
            residual / max(denominator, 1e-9))
        omega_in_b = pos_info["mav_R"].T.dot(omega_in_e).reshape(3)

        omega_norm = float(np.linalg.norm(omega_in_b))
        if self.omega_max > 0.0 and omega_norm > self.omega_max:
            omega_in_b *= self.omega_max / omega_norm
        if self.omega_filter_tau > 0.0 and dt > 0.0:
            alpha = dt / (self.omega_filter_tau + dt)
            omega_in_b = self.filtered_omega + alpha * (
                omega_in_b - self.filtered_omega)
        if self.omega_slew_max > 0.0 and dt > 0.0:
            delta = omega_in_b - self.last_omega
            delta_norm = float(np.linalg.norm(delta))
            max_delta = self.omega_slew_max * dt
            if delta_norm > max_delta:
                delta *= max_delta / delta_norm
            omega_in_b = self.last_omega + delta
        self.filtered_omega = omega_in_b.copy()
        self.last_omega = omega_in_b.copy()

        return [omega_in_b[0], omega_in_b[1], omega_in_b[2], thrust_d]

    def reset(self):
        self.last_mav_vel = None
        self.last_omega = np.zeros(3, dtype=np.float64)
        self.filtered_omega = np.zeros(3, dtype=np.float64)
