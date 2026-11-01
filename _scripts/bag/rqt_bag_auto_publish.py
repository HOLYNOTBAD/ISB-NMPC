#!/usr/bin/env python3
import os
import runpy
import sys

from python_qt_binding.QtCore import QObject, Signal, Slot
from rqt_bag.bag_widget import BagWidget

_load_bag = BagWidget.load_bag


class AutoPublisher(QObject):
    request = Signal(object)

    def __init__(self):
        super(AutoPublisher, self).__init__()
        self.request.connect(self.start)

    @Slot(object)
    def start(self, widget):
        timeline = widget._timeline
        timeline.set_publishing_state(True)
        # ``play_all`` in rqt_bag means step through every message, not
        # publish all topics.  That path ignores play_speed.  Keep it false
        # so the toolbar speed controls affect the time-based playback.
        timeline.play_all = False
        timeline.navigate_play()
        # navigate_play() resets the speed to 1.0; apply the requested initial
        # speed afterwards so the toolbar's fast-forward/rewind buttons can
        # continue changing it normally.
        timeline.play_speed = float(os.environ.get("RQT_BAG_PLAY_RATE", "0.5"))


_auto_publisher = AutoPublisher()


def load_bag_and_publish(self, filename):
    _load_bag(self, filename)
    _auto_publisher.request.emit(self)


BagWidget.load_bag = load_bag_and_publish
sys.argv[0] = "rqt_bag"
runpy.run_path("/opt/ros/noetic/bin/rqt_bag", run_name="__main__")
