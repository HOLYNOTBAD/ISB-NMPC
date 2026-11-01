import numpy as np

from ..utils.math_utils import sat, sat_z


class BasicAttackController:
    def compute(self, pos_info, pos_i, image_center):
        yaw = pos_info["mav_original_angle"][0]
        cmd = [5*np.cos(yaw), 5*np.sin(yaw),
               0.01*(image_center[1] - pos_i[1]),
               0.01*(image_center[0] - pos_i[0])]
        return cmd


class RotateAttackController:
    def __init__(
        self,
        u0,
        v0,
        f,
        R_cb,
        n_cc,
        forward_ramp=0.03,
        forward_speed_max=10.0,
        lateral_gain=12.0,
        vertical_gain=10.0,
        yaw_gain=0.002,
    ):
        self.u0 = u0
        self.v0 = v0
        self.f = f
        self.R_cb = R_cb
        self.n_cc = n_cc
        self.forward_ramp = forward_ramp
        self.forward_speed_max = forward_speed_max
        self.lateral_gain = lateral_gain
        self.vertical_gain = vertical_gain
        self.yaw_gain = yaw_gain
        self.cnt = 0

    def compute(self, pos_info, pos_i, image_center, controller_reset):
        if controller_reset:
            self.cnt = 0

        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)

        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        cos_beta = n_bo.dot(n_bc)
        v_b = n_bo*cos_beta - n_bc

        self.cnt += 1
        v_m = np.array([0., 0., 0.])
        v_m[1] = sat(self.cnt * self.forward_ramp, self.forward_speed_max)
        v_m[0] = self.lateral_gain * v_b[0]
        v_m[2] = self.vertical_gain * v_b[2]
        v = pos_info["mav_R"].dot(v_m)
        yaw_rate = self.yaw_gain * (image_center[0] - pos_i[0])

        return [v[0], v[1], v[2], yaw_rate]

    def reset(self):
        self.cnt = 0


class RotateHighspeedAttackController:
    def __init__(self, u0, v0, f, R_cb, n_cc, v_norm_d=10,
                 max_speed=12.0):
        self.u0 = u0
        self.v0 = v0
        self.f = f
        self.R_cb = R_cb
        self.n_cc = n_cc
        self.v_norm_d = v_norm_d
        self.max_speed = max_speed
        self.cnt = 0

    def compute(self, pos_info, pos_i, image_center, controller_reset=False):
        if controller_reset:
            self.cnt = 0

        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)

        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        cos_beta = n_bo.dot(n_bc)
        v_b = n_bo*cos_beta - n_bc

        self.cnt += 1
        v_forward_base = pos_info["mav_R"].T.dot(pos_info["mav_vel"])[1]
        v_m = np.array([0., 0., 0.])
        v_m[1] = sat(self.cnt * 0.02, self.v_norm_d)
        v_m[0] = 15*v_b[0]
        v_m[2] = 12*v_b[2]
        v = pos_info["mav_R"].dot(v_m)
        speed = float(np.linalg.norm(v))
        if self.max_speed > 0.0 and speed > self.max_speed:
            v *= self.max_speed / speed
        yaw_rate = 0.002*(image_center[0] - pos_i[0])

        return [v[0], v[1], v[2], yaw_rate]

    def reset(self):
        self.cnt = 0


class RotateAttackAccelerationController:
    def __init__(self, u0, v0, f, R_cb, n_cc):
        self.u0 = u0
        self.v0 = v0
        self.f = f
        self.R_cb = R_cb
        self.n_cc = n_cc
        self.cnt = 0

    def compute(self, pos_info, pos_i, controller_reset):
        if controller_reset:
            self.cnt = 0

        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)

        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        n_td = np.array([np.cos(pos_info["mav_yaw"]), np.sin(pos_info["mav_yaw"]), 0.], dtype=np.float64)
        v_1 = 2.0 * (n_eo - n_td)
        v_2 = 1.0 * n_td

        v_d = v_1 + v_2
        v_d /= np.linalg.norm(v_d)
        V = np.linalg.norm(pos_info["mav_vel"])
        v_d *= V + 2

        a_d = sat(1.0 * (v_d - pos_info["mav_vel"]), 6.)

        yaw_rate = 0.002*(self.u0 - pos_i[0])

        return [a_d[0], a_d[1], a_d[2], yaw_rate]

    def reset(self):
        self.cnt = 0


class RotateAttackAccelerationController2:
    def __init__(self, u0, v0, f, R_cb, n_cc):
        self.u0 = u0
        self.v0 = v0
        self.f = f
        self.R_cb = R_cb
        self.n_cc = n_cc
        self.cnt = 0

    def compute(self, pos_info, pos_i, controller_reset, yaw_d=np.pi/2):
        if controller_reset:
            self.cnt = 0

        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)

        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        n_td = np.array([np.cos(pos_info["mav_yaw"]), np.sin(pos_info["mav_yaw"]), 0.], dtype=np.float64)
        v_1 = max(2.5 - pos_i[2]/120., 0.5) * (n_eo - n_td)
        v_2 = 1.0 * n_td

        v_d = v_1 + v_2
        v_d /= np.linalg.norm(v_d)
        V = np.linalg.norm(pos_info["mav_vel"])
        v_d *= V + 1.5

        a_d = sat(1.0 * (v_d - pos_info["mav_vel"]), 6.)
        a_d[2] = sat(a_d[2], 2.)

        yaw_rate = 0.0025*(self.u0 - pos_i[0])

        return [a_d[0], a_d[1], a_d[2], yaw_rate]

    def reset(self):
        self.cnt = 0


class RotateAttackAcceleration2VelController:
    def __init__(self, u0, v0, f, R_cb, n_cc):
        self.u0 = u0
        self.v0 = v0
        self.f = f
        self.R_cb = R_cb
        self.n_cc = n_cc
        self.cnt = 0

    def compute(self, pos_info, pos_i, controller_reset, yaw_d=np.pi/2):
        if controller_reset:
            self.cnt = 0

        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)

        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        n_td = np.array([np.cos(pos_info["mav_yaw"]), np.sin(pos_info["mav_yaw"]), 0.], dtype=np.float64)
        v_1 = max(2.0 - pos_i[2]/100., 0.5) * (n_eo - n_td)
        v_2 = 1.0 * n_td

        v_d = v_1 + v_2
        v_d /= np.linalg.norm(v_d)
        V = np.linalg.norm(pos_info["mav_vel"])
        v_d *= V + 0.6
        v_d[2] = sat_z(v_d[2], 1.)

        yaw_rate = 0.0025*(self.u0 - pos_i[0])

        return [v_d[0], v_d[1], v_d[2], yaw_rate]

    def reset(self):
        self.cnt = 0
