# Mot diem trong ke hoach nhiem vu. Mirror mission_waypoint_t phia FC/GCS.

uint8 ACTION_NONE=0
uint8 ACTION_PICKUP=1
uint8 ACTION_DROPOFF=2

uint8 seq                     # thu tu diem trong ke hoach, bat dau tu 0
float64 lat                   # do, WGS84 - dai dien chinh tu GCS
float64 lon                   # do, WGS84
float32 alt_m                 # do cao so voi TAG DICH cua diem nay: z_odom = tag_z + alt_m
                              # (KHONG phai so voi diem cat canh - chu thich cu ghi sai)
float32[3] pos_ned            # toa do local NED, mission_manager quy doi tu lat/lon khi bay
int32 expected_marker_id      # ID AprilTag phai xac thuc tai diem nay; -1 = khong can xac thuc
uint8 action                  # ACTION_*
float32 acceptance_radius_m   # ban kinh coi la "da toi noi"
float32 max_vel_mps           # gioi han van toc tren chang bay toi diem nay
float32 loiter_s              # thoi gian giu on dinh tai diem truoc khi lam action
