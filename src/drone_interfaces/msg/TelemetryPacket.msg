# Goi telemetry gop tu nhieu nguon, gui GCS o tan so thap (2 Hz) de tiet kiem bang thong.
# Anh xa 1-1 sang ban tin DRONE_TELEMETRY (42010) cua giao uoc GCS <-> Pi muc 8.4.
#
# NGUYEN TAC R3: bit = 0 trong valid_flags nghia la KHONG BIET, KHONG PHAI LA 0. Nguon nao
# chua co du lieu thi de gia tri mac dinh va HA CO tuong ung - dung thay bang mot so "an toan".
# Da bo truong fc_connected: trung nghia voi bit FC_LINK_VALID (giao uoc P10).

uint32 mission_id
uint8 mission_state           # theo hang so cua MissionState
uint8 current_wp_index
uint8 wp_total                # tong so diem cua ke hoach; 0 = chua nap
uint8 retry_count             # so lan da thu lai o diem hien tai
float64 lat
float64 lon
float32 alt_m                 # so voi goc ban do tag (= -z NED)
float32[3] vel_ned
float32 battery_pct
float32 battery_v
int16 gcs_rssi_dbm            # hieu luc theo bit RSSI_VALID
int32 marker_id_tracking      # tag dang BAM; hieu luc theo bit MARKER_VALID
int32 expected_marker_id      # tag dang TIM (khac tag dang bam); -1 khi khong tim
uint8 gripper_state           # theo hang so cua GripperStatus
uint8 failsafe_type           # theo hang so cua FailsafeEvent; FS_NONE neu binh thuong
uint32 tagmap_crc             # CRC-32 ban do tag, phat hien GCS va Pi lech bang tag
int32 home_n_mm               # "nha" cua RTH trong khung da neo; hieu luc theo HOME_VALID
int32 home_e_mm
uint16 valid_flags            # VALID_*
uint16 status_flags           # STATUS_*
uint32 contract_ver           # MAJOR*10000 + MINOR*100
# Ket qua chuyen bay gan nhat (MissionState.RESULT_*), chot lai qua ca luc ve IDLE.
# 0 = chua co chuyen nao ket thuc. Xem giao uoc GCS 11.5 P30.
uint8 flight_result
builtin_interfaces/Time stamp

# --- valid_flags: hieu luc DU LIEU (giao uoc muc 5.2) ---
uint16 VALID_POS=1            # odom DA NEO theo tags.yaml, EKF khoe, odom con tuoi
uint16 VALID_GLOBAL_POS=2     # hien LUON 0: FC chua co GPS
uint16 VALID_BATTERY=4        # hien LUON 0: FC chua gui BATTERY_STATUS
uint16 VALID_FC_LINK=8        # /mavros/state con tuoi VA connected
uint16 VALID_MARKER=16
uint16 VALID_GRIPPER=32
uint16 VALID_EKF_HEALTHY=64
uint16 VALID_RSSI=128
uint16 VALID_HOME=256         # home da chot trong khung da neo

# --- status_flags: TRANG THAI, khong phai hieu luc (giao uoc muc 5.2) ---
uint16 STATUS_ARMED=1
uint16 STATUS_PI_HAS_AUTHORITY=2   # OB_AUTH: nguoi lai da trao quyen qua ch5/ch8
uint16 STATUS_CARRYING=4           # gripper dang giu hang
