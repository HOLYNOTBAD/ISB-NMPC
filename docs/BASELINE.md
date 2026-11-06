# 代码基线与验证

## 来源

- 本地来源仓库：`bvr`，远端 `HOLYNOTBAD/SBnmpc` 的 `SBnmpc_opensource` 分支。
- 实验提交：[`905ee8566494c5b0f6ddecd91ce2175d8c95f408`](https://github.com/HOLYNOTBAD/SBnmpc/commit/905ee8566494c5b0f6ddecd91ce2175d8c95f408)。
- 提交标题：`10302300 success experiment`；实际提交日期为 **2026-10-03**，不是 10 月 30 日。
- 2026-10-19 核对时，该提交仍是此分支最新提交。`HOLYNOTBAD/bvr` 是另一个远端，不能用其同名旧分支替代此实验版本。

## 公开版差异

`so3nmpc` → `isb_nmpc`，`perception_filters` → `dc_s2ekf`；相应命名空间、参数变量和
话题前缀同步调整，通用 `/nmpc/*` 状态接口保持实验版兼容。

控制器方程、ACADO 生成代码、滤波核心和拦截状态机沿用上述提交。
控制器日志默认路径改为可移植的临时目录，启动脚本可指定工作区日志路径。
HSV 检测器保留公开版的原始颜色检测，用于辅助评估；没有迁回实验版中依赖 YOLO 的目标关联与低通平滑。

仅提供 `simulation`、`loop_simulation`、`real_flight`。不迁入轨迹 NMPC、intrusion、
HIL、控制器对比、自动调参及其数据。仿真目标直接使用已有轨迹 CSV 或气球运动配置。
`0degree` 的 IMX577-2 相机配置为基线中的近似值，不代表独立标定或完成实飞验证。

本轮整理移除了循环执行器和配置加载器中不可用的对比分支、无引用的路径索引与本机图像记录脚本；
修复了依赖声明、消息生成顺序、参数安装目录、YOLO 插件安装及文档命令。
没有合入来源仓库工作区中的未提交修改。

## 验证（2026-10-19）

- ROS Noetic 环境，从空构建目录编译全部 15 个 Catkin 包并安装成功。
- 安装后的 `rosrun loop_simulation loop_simulation_node.py --help` 与结果汇总入口可执行。
- 安装后检测节点的 YOLO 库及 TensorRT 插件可解析。
- `rosdep check --from-paths src --ignore-src` 无缺失依赖。
- 34 项现有启动模式、局部坐标变换、接管条件与控制器回退测试通过。
- 9 项新增配置选择、环境覆盖、试次事件隔离与复位顺序回归测试通过。

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

这些检查不等同于重新完成 Gazebo 拦截实验或实飞。实验视频应注明真实来源、日期和场景，
历史录像不能作为当前提交重新实验的证明。已有研究录像与来源记录见[实验素材清单](media/manifest.json)。
