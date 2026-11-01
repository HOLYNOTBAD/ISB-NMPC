# 第三方代码与资源授权

顶层 `LICENSE` 适用于本项目原创代码，不替换第三方版权及许可证声明。
本清单依据仓库现有文件整理，不代表已完成全部来源及许可证兼容性审计。

| 路径 | 仓库已有声明 | 处理方式 |
| --- | --- | --- |
| `src/isb_nmpc/externals/qpoases/` | `LICENSE.txt` 为 LGPL-2.1 文本 | 保留随附许可证及源码中的版权、版本声明 |
| `src/yolo/src/src/logging.h` | NVIDIA，Apache-2.0 | 保留文件头声明 |
| `src/camera_driver/` | `package.xml` 声明 BSD-3-Clause | 保留原有包级声明 |
| `src/msgs/quadrotor_common/` | `package.xml` 声明 MIT | 保留原有包级声明 |
| `src/msgs/quadrotor_msgs/` | `package.xml` 声明 BSD，未明确版本 | 保留声明，具体版本和来源待核实 |
| `src/holydrone_ros/` | 独立 Git 子模块 | 参见其 `THIRD_PARTY.md` 及各文件声明，不受主仓库统一重新授权 |

ACADO 生成代码、YOLO 后端及其他外部来源代码仍需按各自来源核实适用条款；
缺少声明或 `package.xml` 中的 `TODO` 不代表这些代码已获得 GPL 重新授权。

Baylands 地形、纹理、模型、检测权重、TensorRT engine、演示视频及论文的授权需单独确认；
顶层软件许可证不自动授予这些资源的使用或再分发权利。
