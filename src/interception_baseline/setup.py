#!/usr/bin/env python3

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup


setup_args = generate_distutils_setup(
    packages=["interception_baseline", "interception_baseline.controllers",
              "interception_baseline.utils"],
    package_dir={"": "src"},
)

setup(**setup_args)
