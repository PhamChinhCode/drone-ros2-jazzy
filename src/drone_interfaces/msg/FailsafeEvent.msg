# Su kien failsafe. Mirror failsafe_type_e. QoS Reliable - khong duoc rot goi.

uint8 FS_NONE=0
uint8 FS_MARKER_TIMEOUT=1
uint8 FS_GRIP_CONFIRM_FAIL=2
uint8 FS_LINK_LOST=3
uint8 FS_LOW_BATTERY=4
uint8 FS_EKF_UNHEALTHY=5
uint8 FS_FC_COMM_LOST=6

# Muc leo thang yeu cau mission_manager_node thuc hien
uint8 ESCALATE_NONE=0
uint8 ESCALATE_LOITER=1
uint8 ESCALATE_RETRY_LOITER=2
uint8 ESCALATE_RTH=3
uint8 ESCALATE_EMERGENCY_LAND=4

uint8 type
uint8 escalate_to
bool active                   # false = su co da het, go canh bao
string detail
builtin_interfaces/Time stamp
