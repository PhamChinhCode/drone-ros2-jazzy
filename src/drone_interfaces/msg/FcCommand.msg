# Lenh cap cao gui xuong flight controller. Mirror fc_command_t.

uint8 FC_CMD_ARM=1
uint8 FC_CMD_DISARM=2
uint8 FC_CMD_TAKEOFF=3
uint8 FC_CMD_GOTO_WAYPOINT=4
uint8 FC_CMD_HOLD=5
uint8 FC_CMD_PRECISION_LAND=6
uint8 FC_CMD_LAND_NOW=7
uint8 FC_CMD_RTH=8

uint32 seq                    # tang dan, dung de doi chieu voi last_command_seq_acked cua FC
uint8 command                 # FC_CMD_*
float32[3] target_ned         # chi dung voi FC_CMD_GOTO_WAYPOINT
float32 yaw_deg
float32 max_vel_mps
float32 acceptance_radius_m
