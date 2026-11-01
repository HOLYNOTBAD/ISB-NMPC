"""Regression checks for package-owned launches and repeated trial resets.

Run with ROS and this workspace sourced: python3 -m unittest discover -s tests -v
No ROS master or flight hardware is used.
"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

import rospy
from swarm_msgs.msg import InterceptionEvent, MassPoint, MassPoints

ROOT = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(ROOT / 'src/loop_simulation/src'))
from loop_simulation import loop_runner as loop


class ProfileTests(unittest.TestCase):
    def test_interception_script_uses_package_launches_and_builtin_rosbag(self):
        for mode in ('sim', 'real', 'loopsim'):
            script = (ROOT / f'_scripts/run/interception_{mode}.sh').read_text()
            for forbidden in ('screen -', 'gzserver',
                              'gz model', 'rosrun ', '/bin/px4'):
                with self.subTest(forbidden=forbidden):
                    self.assertNotIn(forbidden, script)
            for package in ('holydrone',
                            'target_detection_yolo', 'dc_s2ekf', 'target_manager',
                            'target_estimate', 'plan_manage', 'isb_nmpc'):
                with self.subTest(package=package):
                    self.assertIn('start_launch ' + package + ' ', script)
            self.assertNotIn('interception_bringup', script)
            self.assertIn('rosbag record', script)
            self.assertNotIn('interception_rosbag.launch', script)
            self.assertNotIn('_CONFIG_FILE', script)
            self.assertIn('roslaunch camera_driver load_camera_param.launch', script)
            self.assertIn('camera_param_file:="${CAMERA_PARAM_FILE}"', script)

    def test_launch_files_parse(self):
        paths = list((ROOT / 'src').glob('*/launch/*.launch'))
        self.assertTrue(paths)
        for path in paths:
            with self.subTest(path=path.relative_to(ROOT)):
                ET.parse(path)

    def test_packages_own_their_mode_configuration(self):
        expected = {
            'src/holydrone_ros/launch/sitl.launch': 'px4_sitl',
            'src/target_register/launch/gazebo_balloon_sim.launch': 'balloon_motion_file',
            'src/target_detection_yolo/launch/target_detection_yolo_common.launch': 'detection_fsm.yaml',
            'src/target_detection_yolo/launch/target_detection_yolo_sim.launch': 'balloon_640x480_rtx5080.engine',
            'src/target_detection_yolo/launch/target_detection_yolo_real.launch': 'balloon_640x480_orinnx.engine',
            'src/dc_s2ekf/launch/bbox_s2_delay_ekf.launch': 'bbox_filter.yaml',
            'src/plan_manage/config/interception_real_fsm.yaml': 'airborne_takeover',
            'src/isb_nmpc/launch/so3_mpc_real.launch': '10ms_30degree_from_sim.yaml',
        }
        for relative, marker in expected.items():
            with self.subTest(path=relative):
                self.assertIn(marker, (ROOT / relative).read_text())

    def test_loop_runner_is_owned_by_loop_simulation_package(self):
        package = ROOT / 'src/loop_simulation'
        self.assertFalse((ROOT / 'src/params/package.xml').exists())
        self.assertTrue((package / 'launch/loop_simulation.launch').is_file())
        self.assertTrue((package / 'src/loop_simulation/loop_runner.py').is_file())
        self.assertTrue((package / 'scripts/summarize_trials.py').is_file())
        self.assertIn(
            'start_launch loop_simulation loop_simulation.launch',
            (ROOT / '_scripts/run/interception_loopsim.sh').read_text(),
        )


class TrialTests(unittest.TestCase):
    def make_runner(self):
        with patch.object(rospy, 'Subscriber'), patch.object(rospy, 'ServiceProxy'):
            return loop.LoopRunner(loop.parse_args([]))

    def test_previous_trial_event_is_ignored(self):
        runner = self.make_runner()
        runner.trial_start_time = rospy.Time(10)
        old = InterceptionEvent(success=True, reason='swept_proximity')
        old.header.stamp = rospy.Time(9)
        runner._event_cb(old)
        self.assertIsNone(runner.latest_event)
        current = InterceptionEvent(success=True, reason='swept_proximity')
        current.header.stamp = rospy.Time(11)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(11)):
            runner._event_cb(current)
        self.assertIs(runner._successful_interception_event(rospy.Time(10)), current)

    def test_empty_truth_clears_previous_target(self):
        runner = self.make_runner()
        point = MassPoint(volume=7)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(10)):
            runner._truth_cb(MassPoints(points=[point]))
            self.assertIs(runner.truth_target, point)
            runner._truth_cb(MassPoints())
        self.assertIsNone(runner.truth_target)

    def test_reset_order_prevents_previous_target_reacquisition(self):
        runner = self.make_runner()
        runner.fsm_state = 'WAIT_TARGET'
        runner.has_target_detection_yolo_reset = True
        runner.has_bbox_filter_reset = True
        names = []
        def trigger(service, name):
            names.append(name)
            return True, 'ok'
        with patch.object(runner, 'call_trigger', side_effect=trigger), \
                patch.object(runner, 'wait_until', return_value=True), \
                patch.object(rospy, 'loginfo'):
            self.assertEqual(runner.prepare_trial(2), (True, 'ready'))
        self.assertEqual(names, ['/balloons/clear_trial',
                                 '/target_detection_yolo_node/reset_trial',
                                 '/bbox_s2_delay_ekf/reset_trial',
                                 '/target_manager/reset_trial', '/balloons/reset_trial'])

    def test_reset_service_failure_stops_trial(self):
        runner = self.make_runner()
        with patch.object(runner, 'call_trigger', return_value=(False, 'service unavailable')) as call:
            self.assertEqual(runner.prepare_trial(1), (False, 'service unavailable'))
        self.assertEqual(call.call_count, 1)

    def test_optional_filter_service_can_be_absent(self):
        runner = self.make_runner()
        def wait(name, timeout):
            if name == '/bbox_s2_delay_ekf/reset_trial':
                raise rospy.ROSException('disabled')
        with patch.object(rospy, 'wait_for_service', side_effect=wait), \
                patch.object(rospy, 'loginfo'):
            runner.wait_for_services()
        self.assertTrue(runner.has_target_detection_yolo_reset)
        self.assertFalse(runner.has_bbox_filter_reset)


if __name__ == '__main__':
    unittest.main()
