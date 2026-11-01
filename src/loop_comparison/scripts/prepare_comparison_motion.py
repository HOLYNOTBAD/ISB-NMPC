#!/usr/bin/env python3
"""Create the active motion file before starting the Gazebo balloon node."""
import runpy

if __name__ == '__main__':
    runpy.run_module('loop_comparison.interception_comparison', run_name='__main__')
