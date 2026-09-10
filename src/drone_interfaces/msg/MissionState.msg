# Trang thai hien tai cua may trang thai nhiem vu. Mirror mission_state_e.

uint8 IDLE=0
uint8 TAKEOFF=1
uint8 ENROUTE=2
uint8 MARKER_SEARCH=3
uint8 PRECISION_LAND=4
uint8 ACTUATE_GRIPPER=5
uint8 RETRY_LOITER=6
uint8 RTH=7
uint8 EMERGENCY_LAND=8
uint8 MISSION_COMPLETE=9
uint8 FAILSAFE=10

uint8 state
uint32 mission_id
uint8 current_wp_index
int32 expected_marker_id              # -1 khi khong o pha can marker
uint8 retry_count
builtin_interfaces/Time state_entered_stamp
string detail                         # ly do/ghi chu ngan, dung cho log va hien thi GCS
