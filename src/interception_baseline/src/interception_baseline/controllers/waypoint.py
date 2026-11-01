import math
import numpy as np

from ..utils.math_utils import sat, min_angle_diff, saturation


class WaypointController:
    def __init__(self, v_norm_d=10):
        self.v_norm_d = v_norm_d
        self.cnt_WP = 1

    def compute(self, pos_info, target_position_local):
        self.cnt_WP += 1

        direction = np.array([target_position_local[0] - pos_info["mav_pos"][0],
                              target_position_local[1] - pos_info["mav_pos"][1]])
        direction /= np.linalg.norm(direction)
        v_horizontal = sat(self.cnt_WP * 0.05, self.v_norm_d) * direction
        yaw_d = math.atan2(direction[1], direction[0])
        cmd_yaw = saturation(0.2 * min_angle_diff(yaw_d, pos_info["mav_yaw"]), 1.0, -1.0)

        return [v_horizontal[0], v_horizontal[1], 0, cmd_yaw]

    def reset(self):
        self.cnt_WP = 1
