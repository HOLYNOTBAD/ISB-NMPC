import numpy as np

from ..utils.math_utils import sat, par, matrix


class DockingController:
    def __init__(self, u0, v0, f, R_cb, n_cc, kvod=2.5, v_approach=1.0):
        self.u0 = u0
        self.v0 = v0
        self.f = f
        self.R_cb = R_cb
        self.n_cc = n_cc
        self.kvod = kvod
        self.v_approach = v_approach
        self.change = 0.0

    def compute(self, pos_info, pos_i):
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)

        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        dot = n_eo.dot(n_ec)
        self.change = 1.0 - par(dot, 0.867, 0.966)
        if pos_i[1] == 0:
            self.change = 0.0

        vd1 = matrix(n_eo, n_eo)
        vI = np.eye(3) - vd1
        vod = (1.0 / (1.001 - dot)) * (1.0 - self.change) * self.kvod * vI.T.dot(n_ec)

        n_bc_norm = n_bc / np.linalg.norm(n_bc)
        v_rd_b = self.v_approach * n_bc_norm
        v_rd_e = pos_info["mav_R"].dot(v_rd_b)

        v = vod + self.change * v_rd_e
        v = sat(v, 1.0)

        return [v[0], v[1], v[2], 0.0]
