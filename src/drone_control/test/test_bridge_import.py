"""Node bridge phai import duoc (10-27: thieu small_to_center trong pad_map -> node chet luc khoi dong,
mo phong khong thay tag nao ma moi test van dat)."""

import importlib

import pytest


def test_import_landing_target_bridge_node():
    pytest.importorskip('rclpy')
    pytest.importorskip('apriltag_msgs')
    importlib.import_module('drone_control.landing_target_bridge_node')
