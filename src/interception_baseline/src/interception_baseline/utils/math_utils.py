import math
import numpy as np


class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def getx(self):
        return self.x

    def gety(self):
        return self.y


class Getlen:
    def __init__(self, p1, p2):
        self.x = p1.getx() - p2.getx()
        self.y = p1.gety() - p2.gety()
        self.len = math.sqrt((self.x ** 2) + (self.y ** 2))

    def getlen(self):
        return self.len


def vex(matrix_A):
    return np.array([matrix_A[2, 1], matrix_A[0, 2], matrix_A[1, 0]])


def matrix(B, C):
    return np.array([[B[0] * C[0], B[0] * C[1], B[0] * C[2]],
                     [B[1] * C[0], B[1] * C[1], B[1] * C[2]],
                     [B[2] * C[0], B[2] * C[1], B[2] * C[2]]])


def sat(a, maxv):
    n = np.linalg.norm(a)
    if n > maxv:
        return a / n * maxv
    else:
        return a


def par(d, d1, d2):
    if d < d1 or d == d1:
        return 1.0
    elif (d1 < d and d < d2) or d == d2:
        A = -2 / math.pow((d1 - d2), 3)
        B = 3 * (d1 + d2) / math.pow((d1 - d2), 3)
        C = -6 * d1 * d2 / math.pow((d1 - d2), 3)
        D = (math.pow(d2, 2) * (3 * d1 - d2)) / math.pow((d1 - d2), 3)
        return A * math.pow(d, 3) + B * math.pow(d, 2) + C * math.pow(d, 1) + D
    else:
        return 0.0


def sat_z(vz, satf):
    if vz > satf:
        return satf
    elif vz < -satf:
        return -satf
    else:
        return vz


def sat_integral(a, up, down):
    for i in range(len(a)):
        if a[i] > up:
            a[i] = up
        elif a[i] < down:
            a[i] = down
    return a


def saturation(a, up, down):
    if a > up:
        a = up
    elif a < down:
        a = down
    return a


def deadband(a, deadv):
    for i in range(len(a)):
        if abs(a[i]) < deadv:
            a[i] = 0
        elif a[i] > deadv or a[i] == deadv:
            a[i] = a[i] - deadv
        elif a[i] < - deadv or a[i] == - deadv:
            a[i] = a[i] + deadv
    return a


def skew(n):
    return np.array([[0, -n[2], n[1]],
                     [n[2], 0, -n[0]],
                     [-n[1], n[0], 0]], dtype=np.float64)


def min_angle_diff(a, b):
    diff = a - b
    if diff < 0:
        diff += 2 * np.pi
    if diff < np.pi:
        return diff
    else:
        return diff - 2 * np.pi


def angle_limiting(a):
    if a > np.pi:
        return a - 2 * np.pi
    if a < -np.pi:
        return a + 2 * np.pi
    return a


def pos_control(target_pos, feb_pos, kp, sat_vel, _sat_integral=sat_integral):
    err_pos = target_pos - feb_pos
    cmd_pos_vel = _sat_integral(kp * err_pos, sat_vel, -sat_vel)
    return [cmd_pos_vel[0], cmd_pos_vel[1], cmd_pos_vel[2]]


def yaw_control(target_yaw, feb_yaw, kp_yaw, sat_yaw, _min_angle_diff=min_angle_diff, _saturation=saturation):
    dlt_yaw = _min_angle_diff(target_yaw, feb_yaw)
    cmd_yaw = _saturation(kp_yaw * dlt_yaw, sat_yaw, -sat_yaw)
    return cmd_yaw
