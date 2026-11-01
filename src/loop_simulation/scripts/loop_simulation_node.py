#!/usr/bin/env python3
"""运行多轮包含真实感知链路的拦截仿真实验。

本文件是 ``loop_simulation`` 模式的轻量入口。实际的循环调度、FSM 复位、
原始状态/事件采集和结果写入由 :mod:`loop_simulation.loop_runner` 统一实现，并保留正常
的相机、检测与估计链路。
"""

import os
import sys


try:
    from loop_simulation.loop_runner import main
except ModuleNotFoundError as exc:
    if exc.name != "loop_simulation":
        raise
    # catkin 的开发空间包装器会执行源码脚本；尚未安装时从本包 src 目录导入。
    PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
    if PACKAGE_SRC not in sys.path:
        sys.path.insert(0, PACKAGE_SRC)
    from loop_simulation.loop_runner import main


if __name__ == "__main__":
    main(sys.argv[1:])
