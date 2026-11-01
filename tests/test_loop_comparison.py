"""Offline regression tests for the migrated controller comparison pipeline.

Run after sourcing devel/setup.bash: python3 -m unittest discover -s tests -p test_loop_comparison.py -v
"""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, MagicMock

import numpy as np
import rospy
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from quadrotor_msgs.msg import FsmState
from loop_comparison import interception_comparison as geometry
from loop_comparison import loop_runner as loop

ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ComparisonTests(unittest.TestCase):
    def test_motion_seed_replay_and_source_preserved(self):
        source = ROOT / 'src/target_register/config/balloon_motion/balloon_motion_random_walk_radius_1m.yaml'
        original = source.read_bytes()
        params = geometry.motion_parameters(str(source), [0, 0, 30], .34, 1.0)
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / 'active.yaml')
            params.update(random_walk_seed=1000, target_id=1)
            geometry.write_motion_files(path, params)
            first = Path(path).read_bytes()
            geometry.write_motion_files(path, params)
            self.assertEqual(first, Path(path).read_bytes())
            params.update(random_walk_seed=1001, target_id=2)
            geometry.write_motion_files(path, params)
            actual = yaml.safe_load(Path(path).read_text())
            self.assertEqual(actual['random_walk_seed'], 1001)
            self.assertEqual(actual['balloons'][0]['id'], 2)
        self.assertEqual(source.read_bytes(), original)

    def test_camera_geometry_and_gazebo_translation(self):
        path = ROOT / 'src/camera_driver/config/camera/camera_params_usb_yahbom_0degree.yaml'
        camera = yaml.safe_load(path.read_text())['camera']
        camera['image_bound_margin'] = .3
        result = geometry.geometry([0, 0, 30], [10, 0, 0], [1, 0, 0, 0],
                                   [100, 0, 30], [5, 0, 0], camera)
        self.assertTrue(result['visible'])
        self.assertAlmostEqual(result['image_u'], 0)
        self.assertAlmostEqual(result['closing_speed_mps'], 5)
        sync = load_module('camera_sync_test', ROOT / 'src/holydrone_ros/scripts/sync_gazebo_camera_model.py')
        self.assertEqual(sync.parse_camera_config(str(path)).position_body_m, tuple(camera['p_B_C']))

    def test_managed_commands_use_current_packages_without_camera_reload(self):
        args = loop.parse_args(['--source-motion-file', 'unused'], loop_comparison=True)
        args.isb_nmpc_param_file = 'mpc.yaml'
        args.baseline_param_file = 'baseline.yaml'
        runner = loop.LoopRunner.__new__(loop.LoopRunner)
        runner.args = args
        runner.controller_process = None
        for name in geometry.CONTROLLERS:
            process = MagicMock()
            process.poll.return_value = None
            expected_node = '/so3_mpc_bridge_node' if name == 'isb_nmpc' else '/interception_baseline_node'
            with patch.object(runner, 'stop_comparison_controller'), \
                 patch.object(loop.subprocess, 'Popen', return_value=process) as popen, \
                 patch.object(loop.subprocess, 'check_output', return_value=expected_node), \
                 patch.object(rospy.Time, 'now', return_value=rospy.Time(1)), \
                 patch.object(rospy, 'sleep'), patch.object(rospy, 'is_shutdown', return_value=False):
                runner.start_comparison_controller(name)
                command = popen.call_args[0][0]
                self.assertEqual(command[1], 'isb_nmpc' if name == 'isb_nmpc' else 'interception_baseline')
                self.assertFalse(any('camera_param' in item for item in command))

    def test_all_baselines_use_shared_extrinsics_and_compute_finite_commands(self):
        bridge = load_module('baseline_bridge_test', ROOT / 'src/interception_baseline/scripts/interception_baseline_node.py')
        config = yaml.safe_load((ROOT / 'src/interception_baseline/config/comparison.yaml').read_text())
        camera = yaml.safe_load((ROOT / 'src/camera_driver/config/camera/camera_params_usb_yahbom_0degree.yaml').read_text())['camera']
        for method in ('backstepping', 'pslos', 'rotate_highspeed_attack'):
            params = {'~'+k: v for k, v in config.items()}
            params.update({'/camera/'+k: v for k, v in camera.items()})
            params['~controller_type'] = method
            with patch.object(rospy, 'get_param', side_effect=lambda key, default=None: params.get(key, default)), \
                 patch.object(rospy, 'Subscriber'), patch.object(rospy, 'Publisher'), patch.object(rospy, 'Timer'):
                node = bridge.InterceptionBaselineNode()
            np.testing.assert_allclose(node.r_cb_flu, np.array(camera['R_cam_body']).reshape(3, 3))
            state = Odometry()
            state.pose.pose.position.z = 30
            state.pose.pose.orientation.w = 1
            target = Odometry()
            target.pose.pose.position.x = 100
            target.pose.pose.position.z = 30
            target.twist.twist.linear.x = 5
            node.latest_state, node.latest_target = state, target
            node.latest_p_image = PointStamped()
            node.latest_p_image.point.z = 1
            event = FsmState()
            event.state_name = 'EXEC_INTERCEPTION'
            node.fsm_state_callback(event)
            command = node.run_controller(node.build_pos_info(), node.build_pos_i(), .02, True)
            self.assertTrue(np.isfinite(np.asarray(command)).all(), method)


if __name__ == '__main__':
    unittest.main()
