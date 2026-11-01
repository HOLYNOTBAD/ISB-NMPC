"""为 ``loop_comparison`` 提供运动配置加载和相机几何计算。

本模块刻意不依赖 ROS，因此无需启动 Gazebo 就能单独测试坐标系转换、相机投影、
运动配置校验和统计口径。ROS 话题订阅与每轮实验的调度位于 ``loop_runner.py``。
"""

import csv
import copy
import math
import os

# 允许参与对比的控制器。
CONTROLLERS = ("backstepping", "rotate_highspeed_attack", "pslos", "isb_nmpc")
def norm(v):
    """计算任意维向量的欧氏范数。"""
    return math.sqrt(sum(x * x for x in v))


def rotate(q, v):
    """使用 ``[w,x,y,z]`` 四元数旋转向量，并先归一化实测四元数。"""
    if not all(math.isfinite(float(value)) for value in (*q, *v)):
        raise ValueError("non-finite quaternion or vector")
    qn = norm(q)
    if qn < 1e-9:
        raise ValueError("zero orientation quaternion")
    w, x, y, z = (a / qn for a in q)
    qv = (x, y, z)
    cross = (qv[1] * v[2] - qv[2] * v[1],
             qv[2] * v[0] - qv[0] * v[2],
             qv[0] * v[1] - qv[1] * v[0])
    cross2 = (qv[1] * cross[2] - qv[2] * cross[1],
              qv[2] * cross[0] - qv[0] * cross[2],
              qv[0] * cross[1] - qv[1] * cross[0])
    return tuple(v[i] + 2 * (w * cross[i] + cross2[i]) for i in range(3))


def quat_multiply(a, b):
    """返回两个 ``[w,x,y,z]`` 四元数的 Hamilton 积。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def geometry(uav_p, uav_v, q_w_b, target_p, target_v, camera):
    """计算目标的相机投影和相对运动几何量。

    坐标变换顺序为 world -> body FLU -> camera RDF；``closing_speed_mps``
    为正表示无人机和目标正在接近。归一化视场误差不大于 1 时目标可见。
    """
    values = (*uav_p, *uav_v, *q_w_b, *target_p, *target_v,
              *camera["p_B_C"], *camera["q_B_C"], camera["img_width"],
              camera["img_height"], camera["img_f"], camera["image_bound_margin"])
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("non-finite state or camera parameter")
    if camera["img_f"] <= 0:
        raise ValueError("camera focal length must be positive")
    # 将机体系下的相机外参平移量转到世界系，得到相机光心位置。
    p_wc_offset = rotate(q_w_b, camera["p_B_C"])
    p_wc = tuple(uav_p[i] + p_wc_offset[i] for i in range(3))
    relative = tuple(target_p[i] - uav_p[i] for i in range(3))
    camera_relative = tuple(target_p[i] - p_wc[i] for i in range(3))
    # 单位四元数的共轭即逆旋转：先 world -> body，再 body -> camera。
    q_b_w = (q_w_b[0], -q_w_b[1], -q_w_b[2], -q_w_b[3])
    q_c_b = (camera["q_B_C"][0],) + tuple(-a for a in camera["q_B_C"][1:])
    p_c = rotate(q_c_b, rotate(q_b_w, camera_relative))
    distance = norm(relative)
    if distance <= 1e-9:
        raise ValueError("coincident UAV and target positions")
    bearing = tuple(x / norm(p_c) for x in p_c)
    closing = sum(relative[i] * (uav_v[i] - target_v[i]) for i in range(3)) / distance
    rho_u = 0.5 * camera["img_width"] / camera["img_f"] - camera["image_bound_margin"]
    rho_v = 0.5 * camera["img_height"] / camera["img_f"] - camera["image_bound_margin"]
    if min(rho_u, rho_v) <= 0:
        raise ValueError("NMPC image radius is nonpositive")
    if bearing[2] <= 0:
        return dict(distance_m=distance, bearing=bearing, image_u=math.nan,
                    image_v=math.nan, error_u=math.inf, error_v=math.inf,
                    normalized_fov_error=math.inf, visible=False,
                    closing_speed_mps=closing)
    u, v = bearing[0] / bearing[2], bearing[1] / bearing[2]
    eu, ev = u / rho_u, v / rho_v  # NMPC pi_c = (0, 0)
    error = max(abs(eu), abs(ev))
    return dict(distance_m=distance, bearing=bearing, image_u=u,
                image_v=v, error_u=eu, error_v=ev,
                normalized_fov_error=error, visible=error <= 1,
                closing_speed_mps=closing)


def _resolve_params_reference(path, motion_config):
    """解析运动配置中引用的轨迹文件，兼容其相对 params 的旧路径写法。"""
    if os.path.isabs(path):
        return os.path.abspath(path)
    motion_dir = os.path.dirname(motion_config)
    direct = os.path.abspath(os.path.join(motion_dir, path))
    if os.path.isfile(direct):
        return direct
    # Current target_register layouts resolve trajectory/... relative to config/.
    config_relative = os.path.abspath(os.path.join(motion_dir, "..", path))
    if os.path.isfile(config_relative):
        return config_relative
    raise ValueError("balloon motion reference does not exist: " + path)


def _load_motion_config(path):
    """加载并执行最基本的气球运动配置校验。"""
    import yaml
    with open(path) as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict) or not config.get("balloons"):
        raise ValueError("invalid balloon motion config: " + path)
    return config


def motion_parameters(motion_config, uav_start_map, uav_collision_radius,
                      balloon_radius=None):
    """直接从 balloon motion YAML 构造 comparison 所需参数。"""
    motion_config = os.path.abspath(motion_config)
    source = _load_motion_config(motion_config)
    if len(source["balloons"]) != 1:
        raise ValueError("comparison motion config must contain exactly one balloon: " + motion_config)
    configured_start = [float(value) for value in source["balloons"][0].get("pos", [])]
    if len(configured_start) != 3:
        raise ValueError("balloons[0].pos must have three values")
    uav_start = [float(value) for value in uav_start_map]
    if len(uav_start) != 3:
        raise ValueError("uav_start_map must have three values")
    motion_mode = str(source.get("motion_mode", "velocity")).lower()
    source_trajectory = ""
    duration_sec = 0.0
    if motion_mode == "preset_traj":
        source_trajectory = _resolve_params_reference(source.get("preset_traj_file", ""),
                                                      motion_config)
        with open(source_trajectory, newline="") as handle:
            rows = list(csv.DictReader(handle))
        if len(rows) < 2:
            raise ValueError("preset trajectory requires at least two rows: " + source_trajectory)
        duration_sec = float(rows[-1]["time"])
    elif motion_mode not in ("random_walk", "velocity", "linear", "static"):
        raise ValueError("unsupported comparison motion_mode: " + motion_mode)
    balloon_radius = float(
        source.get("impact_radius", 1.50)
        if balloon_radius is None else balloon_radius
    )
    if not math.isfinite(balloon_radius) or balloon_radius <= 0.0:
        raise ValueError("balloon radius must be positive")
    return {
        "duration_sec": duration_sec,
        "uav_start_map": uav_start,
        "target_start_config": configured_start,
        "motion_mode": motion_mode,
        "source_motion_config": motion_config,
        "source_trajectory": source_trajectory,
        "trajectory_time_scale": 1.0,
        "collision_radius_m": float(uav_collision_radius) + balloon_radius,
        "balloon_radius_m": balloon_radius,
    }


def write_motion_files(motion_file, params):
    """根据本轮参数生成气球节点使用的运动 YAML 和可选轨迹 CSV。

    原始配置不会被修改。预设轨迹会同时缩放时间以及速度、加速度和 jerk，
    从而保持各阶导数与新的时间轴一致。
    """
    import yaml
    motion_file = os.path.abspath(motion_file)
    os.makedirs(os.path.dirname(motion_file), exist_ok=True)
    trajectory_file = os.path.splitext(motion_file)[0] + "_trajectory.csv"
    config = copy.deepcopy(_load_motion_config(params["source_motion_config"]))
    config["target_num"] = 1
    config["balloons"] = [copy.deepcopy(config["balloons"][0])]
    config["balloons"][0]["id"] = params.get("target_id", 1)
    config["balloons"][0]["pos"] = list(params["target_start_config"])
    config["interception_check_enabled"] = True
    # comparison assigns a distinct deterministic seed to every random-walk
    # trial.  Persist it in the generated YAML so independent controller runs
    # replay the same trial when they use the same base seed.
    if params["motion_mode"] == "random_walk" and "random_walk_seed" in params:
        config["random_walk_seed"] = int(params["random_walk_seed"])

    if params["motion_mode"] == "preset_traj":
        scale = float(params["trajectory_time_scale"])
        temp_trajectory = trajectory_file + ".tmp"
        with open(params["source_trajectory"], newline="") as source, \
                open(temp_trajectory, "w", newline="") as target:
            reader = csv.DictReader(source)
            if not reader.fieldnames:
                raise ValueError("trajectory CSV has no header: " + params["source_trajectory"])
            writer = csv.DictWriter(target, fieldnames=reader.fieldnames)
            writer.writeheader()
            # 若时间轴乘以 s，各阶导数分别乘以 1/s、1/s²、1/s³。
            derivative_scale = {
                "time": scale,
                "vx": 1.0 / scale, "vy": 1.0 / scale, "vz": 1.0 / scale,
                "ax": 1.0 / scale ** 2, "ay": 1.0 / scale ** 2,
                "az": 1.0 / scale ** 2,
                "jx": 1.0 / scale ** 3, "jy": 1.0 / scale ** 3,
                "jz": 1.0 / scale ** 3,
            }
            for row in reader:
                for field, factor in derivative_scale.items():
                    if field in row:
                        row[field] = "{:.9f}".format(float(row[field]) * factor)
                writer.writerow(row)
        os.replace(temp_trajectory, trajectory_file)
        config["preset_traj_file"] = trajectory_file
    elif params["motion_mode"] in ("velocity", "linear"):
        if os.path.isfile(trajectory_file):
            os.remove(trajectory_file)
        velocity = config["balloons"][0].get("vel", [0, 0, 0])
        factor = 1.0 / float(params["trajectory_time_scale"])
        config["balloons"][0]["vel"] = [float(value) * factor for value in velocity]
    elif params["motion_mode"] in ("static", "random_walk") and os.path.isfile(trajectory_file):
        os.remove(trajectory_file)

    # 原子替换，避免正在运行的气球节点读到只写了一部分的 YAML。
    temp = motion_file + ".tmp"
    with open(temp, "w") as handle:
        yaml.safe_dump(config, handle, sort_keys=True)
    os.replace(temp, motion_file)
    return trajectory_file if params["motion_mode"] == "preset_traj" else ""


if __name__ == "__main__":
    # 该入口仅预生成/检查运动文件；完整实验由 loop_comparison.py 启动。
    import argparse
    parser = argparse.ArgumentParser(description="Prepare a Gazebo balloon comparison motion file")
    parser.add_argument("--source-motion-file", required=True)
    parser.add_argument("--motion-file", required=True)
    parser.add_argument("--target-id", type=int, default=1)
    parser.add_argument("--uav-start", type=float, nargs=3, default=(0.0, 0.0, 10.0))
    parser.add_argument("--uav-collision-radius", type=float, default=0.34)
    parser.add_argument("--balloon-radius", type=float)
    args = parser.parse_args()
    params = motion_parameters(
        args.source_motion_file, args.uav_start, args.uav_collision_radius,
        args.balloon_radius)
    params["target_id"] = args.target_id
    print(write_motion_files(args.motion_file, params))
