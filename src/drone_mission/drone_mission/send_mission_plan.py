"""send_mission_plan - gui MissionPlan tu file YAML len /mission/plan (thu tren ban, chua co GCS).

Dung:  ros2 run drone_mission send_mission_plan <duong_dan.yaml>
Ket qua nhan/tu choi xem o /mission/state (detail) hoac log mission_manager_node.

Dinh dang (seq tu danh theo thu tu; action: none | pickup | dropoff):
  mission_id: 1
  plan_name: ban_home
  max_retries: 0          # 0 = dung mission.yaml
  search_timeout_s: 0.0   # 0 = dung mission.yaml
  waypoints:
    - {marker_id: 0, action: none, alt_m: 0.5, acceptance_radius_m: 0.3, max_vel_mps: 0.5,
       loiter_s: 2.0}
"""

import sys
import time

import rclpy
import yaml
from rclpy.utilities import remove_ros_args

from drone_interfaces.msg import MissionPlan, MissionWaypoint
from drone_mission.qos import EVENT_QOS

ACTIONS = {'none': MissionWaypoint.ACTION_NONE, 'pickup': MissionWaypoint.ACTION_PICKUP,
           'dropoff': MissionWaypoint.ACTION_DROPOFF}


def build_plan(doc):
    plan = MissionPlan()
    plan.mission_id = int(doc['mission_id'])
    plan.plan_name = str(doc.get('plan_name', ''))
    plan.max_retries = int(doc.get('max_retries', 0))
    plan.search_timeout_s = float(doc.get('search_timeout_s', 0.0))
    for i, w in enumerate(doc['waypoints']):
        wp = MissionWaypoint()
        wp.seq = i
        wp.expected_marker_id = int(w['marker_id'])
        wp.action = ACTIONS[str(w.get('action', 'none')).lower()]
        wp.alt_m = float(w['alt_m'])
        wp.acceptance_radius_m = float(w['acceptance_radius_m'])
        wp.max_vel_mps = float(w['max_vel_mps'])
        wp.loiter_s = float(w.get('loiter_s', 0.0))
        plan.waypoints.append(wp)
    return plan


def main(args=None):
    argv = remove_ros_args(sys.argv)
    if len(argv) != 2:
        print(__doc__)
        return 2
    with open(argv[1]) as f:
        plan = build_plan(yaml.safe_load(f))
    rclpy.init(args=args)
    node = rclpy.create_node('send_mission_plan')
    pub = node.create_publisher(MissionPlan, '/mission/plan', EVENT_QOS)
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('khong thay mission_manager_node subscribe /mission/plan sau 5 s')
        rclpy.try_shutdown()
        return 1
    pub.publish(plan)
    print(f'da gui ke hoach {plan.mission_id} ({len(plan.waypoints)} diem)')
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0
