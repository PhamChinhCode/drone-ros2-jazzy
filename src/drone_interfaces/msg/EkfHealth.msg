# Co healthy nhi phan ma robot_localization khong tu co. Mirror ekf_state_t.healthy.

bool healthy
float32[3] pos_variance       # duong cheo hiep phuong sai vi tri x, y, z
string reason                 # ly do khi healthy=false
builtin_interfaces/Time stamp
# true khi odom da NEO theo tags.yaml it nhat mot lan (da fuse mot pose marker ma EKF dong y,
# hoac da ep /set_pose ve marker). Truoc do goc odom la cho EKF khoi dong, KHONG phai goc ban do
# tag - toa do khi ay thuoc mot khung khac va se NHAY khi neo. Giao uoc GCS<->Pi muc 5.2b.
# Latching: da neo roi thi khong ha xuong; do tin cay tuc thoi xet bang healthy.
bool anchored
