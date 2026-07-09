#!/usr/bin/env python

"""Reduced LeRobot build for ARX5 + Quest3 teleoperation."""

from lerobot.__version__ import __version__  # noqa: F401

available_robots = ["arx5_follower"]
available_teleoperators = ["quest3_webxr"]
available_cameras = ["intelrealsense"]
available_motors: list[str] = []
available_policies: list[str] = []
available_envs: list[str] = []
available_datasets: list[str] = []
