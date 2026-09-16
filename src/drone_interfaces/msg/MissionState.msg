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

# Ket qua cua chuyen bay GAN NHAT da ket thuc. CHOT lai: giu nguyen sau khi ve IDLE cho toi
# khi cat canh chuyen moi, nen GCS khoi dong lai luc nao cung hoi lai duoc (giao uoc GCS R5).
# Can thiet vi MISSION_COMPLETE la dich chung cua CA huy lenh, RTH, NAV_LAND va het ke hoach -
# rieng trang thai 9 khong phan biet duoc "xong" voi "bi huy".
uint8 RESULT_UNKNOWN=0          # chua chuyen nao ket thuc ke tu khi node chay
uint8 RESULT_COMPLETED=1        # lam het moi diem cua ke hoach
uint8 RESULT_ABORTED=2          # GCS huy nhiem vu (MAV_CMD 42100)
uint8 RESULT_RTH=3              # ve nha roi ha canh
uint8 RESULT_LANDED_CMD=4       # lenh ha canh tai cho (NAV_LAND)
uint8 RESULT_FAILSAFE=5         # su co hoac mat quyen dieu khien
uint8 RESULT_RETRIES_EXHAUSTED=6  # het luot thu tim/ha theo tag - ha canh tai cho, CHUA xong viec

uint8 state
uint8 mission_result            # hang so RESULT_* o tren
uint32 mission_id
uint8 current_wp_index
int32 expected_marker_id              # -1 khi khong o pha can marker
uint8 retry_count
builtin_interfaces/Time state_entered_stamp
string detail                         # ly do/ghi chu ngan, dung cho log va hien thi GCS
uint8 wp_total                        # tong so diem cua ke hoach; 0 = chua nap
# "Nha" cua RTH trong khung odom (ENU, m). CHI hop le khi home_valid: truoc khi odom neo theo
# bang tag, toa do thuoc mot khung khac va se NHAY khi neo, nen chot nha luc do la sai cho.
# Giao uoc GCS <-> Pi muc 5.2b y 4.
bool home_valid
float32[3] home_odom
