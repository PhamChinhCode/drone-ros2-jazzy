#!/bin/bash
# Don ~/drone_logs: chi GIU bag doan drone hoat dong (FC arm / Pi co quyen / co nhiem vu) +-2 phut,
# doan khac giu 30 phut roi xoa; van giu tran DRONE_LOG_CAP_GB (mac dinh 10 GB). Logic va ly do:
# src/drone_safety/drone_safety/log_retention.py (user 10-23). Truoc 10-23 bag ghi lien tuc moi khi
# stack chay, 10 GB chi chua ~27 gio (phan lon luc nam tren ban) -> log bay cu bi xoa het.
#
# drone_startup.sh goi luc khoi dong va moi 10 phut. Chay tay (can moi truong ROS cua workspace):
#   source scripts/drone_startup.sh && scripts/prune_logs.sh
exec python3 -m drone_safety.log_retention
