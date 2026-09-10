# Goi telemetry gop tu nhieu nguon, gui GCS o tan so thap (1-5 Hz) de tiet kiem bang thong.

uint32 mission_id
uint8 mission_state           # theo hang so cua MissionState
uint8 current_wp_index
float64 lat
float64 lon
float32 alt_m
float32[3] vel_ned
float32 battery_pct
float32 battery_v
bool fc_connected
int32 gcs_rssi_dbm
int32 marker_id_tracking      # -1 neu khong bam marker nao
uint8 gripper_state           # theo hang so cua GripperStatus
uint8 failsafe_type           # theo hang so cua FailsafeEvent; FS_NONE neu binh thuong
builtin_interfaces/Time stamp
