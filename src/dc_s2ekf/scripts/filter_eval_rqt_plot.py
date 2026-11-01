#!/usr/bin/env python3
"""在一个 Qt 窗口中显示实时感知评估的三张 ROS 原生 rqt_plot 子图。"""

import sys

import rospy
import yaml
from python_qt_binding.QtCore import QTimer, Qt
from python_qt_binding.QtWidgets import (
    QApplication, QMainWindow, QSplitter, QVBoxLayout, QWidget)
from rqt_plot.data_plot import DataPlot
from rqt_plot.plot_widget import PlotWidget


class FixedHistoryDataPlot(DataPlot):
    """强制 x 轴始终显示最近指定秒数，避免后端重置窗口宽度。"""

    def __init__(self, history_s, parent=None):
        self.history_s = float(history_s)
        super().__init__(parent)

    def _merged_autoscale(self):
        # rqt_plot 的不同绘图后端可能在添加曲线时重置 x 范围。
        # 每次重绘前直接从所有曲线中找最新时间并恢复固定窗口。
        newest_time = None
        for curve in self._curves.values():
            if len(curve["x"]) > 0:
                curve_newest = float(curve["x"][-1])
                newest_time = (curve_newest if newest_time is None
                               else max(newest_time, curve_newest))
        if newest_time is not None:
            self.set_xlim([
                newest_time - self.history_s,
                newest_time,
            ])

        # 禁止父类再次移动 x 轴；仍由父类按上述可见范围缩放 y 轴。
        autoscroll = self._autoscroll
        self._autoscroll = False
        try:
            super()._merged_autoscale()
        finally:
            self._autoscroll = autoscroll


class FilterEvalPlotWindow(QMainWindow):
    """根据 YAML 配置创建纵向排列的多个 rqt_plot 控件。"""

    def __init__(self, config):
        super().__init__()
        window_config = config.get("window", {})
        self.setWindowTitle(
            window_config.get("title", "Live Filter Evaluation"))
        self.resize(int(window_config.get("width", 1400)),
                    int(window_config.get("height", 1000)))
        history_s = float(window_config.get("history_s", 10.0))

        self.plot_widgets = []
        splitter = QSplitter(Qt.Vertical)
        for plot_config in config.get("plots", []):
            panel = QWidget()
            layout = QVBoxLayout(panel)
            layout.setContentsMargins(4, 4, 4, 4)

            # PlotWidget 和 DataPlot 均来自 ROS Noetic 的 rqt_plot 包。
            # initial_topics 让每张子图启动时立即订阅配置中的话题。
            plot_widget = PlotWidget(
                initial_topics=list(plot_config.get("topics", [])),
                start_paused=False)

            # 话题由 YAML 固定，不需要 rqt_plot 默认的 Topic 输入框、
            # 添加/删除按钮、autoscroll、暂停和清空按钮。移除整行布局，
            # 避免三个子图各自占用一行垂直空间。
            controls_item = plot_widget.verticalLayout.takeAt(0)
            if controls_item is not None and controls_item.layout() is not None:
                controls_layout = controls_item.layout()
                for index in range(controls_layout.count()):
                    control_widget = controls_layout.itemAt(index).widget()
                    if control_widget is not None:
                        control_widget.hide()

            data_plot = FixedHistoryDataPlot(history_s, plot_widget)
            # x 轴保持固定长度并随时间滚动；y 轴每次根据当前可见窗口
            # 重新计算范围，既能放大也能缩小，让数据尽可能充满子图。
            data_plot.set_autoscale(x=False)
            data_plot.set_autoscale(y=DataPlot.SCALE_VISIBLE)
            data_plot.set_xlim([0.0, history_s])
            plot_widget.switch_data_plot_widget(data_plot)

            layout.addWidget(plot_widget)
            splitter.addWidget(panel)
            self.plot_widgets.append(plot_widget)

        self.setCentralWidget(splitter)

    def closeEvent(self, event):
        """关闭窗口时释放每张图创建的 ROS subscriber。"""
        for plot_widget in self.plot_widgets:
            plot_widget.clean_up_subscribers()
        rospy.signal_shutdown("filter evaluation plot window closed")
        super().closeEvent(event)


def main():
    rospy.init_node("filter_eval_rqt_plot", disable_signals=True)
    config_file = rospy.get_param("~config_file")
    with open(config_file, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    application = QApplication.instance() or QApplication(sys.argv)
    window = FilterEvalPlotWindow(config)
    window.show()

    # Qt 接管主循环；定时检查 ROS shutdown，使 rosnode kill 也能关闭 UI。
    shutdown_timer = QTimer()
    shutdown_timer.timeout.connect(
        lambda: application.quit() if rospy.is_shutdown() else None)
    shutdown_timer.start(200)
    return application.exec_()


if __name__ == "__main__":
    sys.exit(main())
