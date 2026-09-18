"""Phan tinh toan cua lop uoc luong, thuan Python de pytest duoc khong can ROS."""

import math

# FC dat covariance 1e6 khi van toc/do cao khong hop le (GIAO_UOC_FC_ROS2.md 11.2). Nguong 1e5
# tach bach voi moi phuong sai that (vai chuc m^2/s^2 tro xuong).
COVARIANCE_INVALID = 1e5


def fc_velocity_validity(twist_covariance):
    """(xy_hop_le, z_hop_le) tu duong cheo covariance twist cua /mavros/odometry/in.

    Quy tac 4.2a: van toc khong hop le thi BO HAN mau, khong dua vao EKF voi trong so thap.
    vx va vy hop le cung nhau (cung mot bit flow phia FC).
    """
    vx, vy, vz = twist_covariance[0], twist_covariance[7], twist_covariance[14]
    ok = lambda c: math.isfinite(c) and 0.0 <= c < COVARIANCE_INVALID  # noqa: E731
    return ok(vx) and ok(vy), ok(vz)


def rotate(q_xyzw, v):
    """Xoay vector v bang quaternion (x, y, z, w)."""
    x, y, z, w = q_xyzw
    # v' = v + 2w (q x v) + 2 q x (q x v)
    cx = y * v[2] - z * v[1]
    cy = z * v[0] - x * v[2]
    cz = x * v[1] - y * v[0]
    ccx = y * cz - z * cy
    ccy = z * cx - x * cz
    ccz = x * cy - y * cx
    return (v[0] + 2.0 * (w * cx + ccx),
            v[1] + 2.0 * (w * cy + ccy),
            v[2] + 2.0 * (w * cz + ccz))


def drone_position_from_tag(tag_in_odom, tag_in_base, q_odom_base):
    """Vi tri base_link trong odom tu toa do da biet cua tag va tag nhin tu than may.

    p_drone = p_tag_odom - R(odom<-base) * p_tag_base. Chi dung VI TRI tag, khong dung huong
    tag - huong lay tu IMU (on dinh hon huong PnP cua mot tag 13 cm).
    """
    r = rotate(q_odom_base, tag_in_base)
    return tuple(tag_in_odom[i] - r[i] for i in range(3))


def tilt_cos_from_quaternion(q_xyzw):
    """cos goc giua truc z than va truc z the gioi = cos(roll)*cos(pitch), khong phu thuoc yaw."""
    x, y = q_xyzw[0], q_xyzw[1]
    return 1.0 - 2.0 * (x * x + y * y)


def vertical_range(range_m, min_range, max_range, tilt_cos, blocked, max_tilt_deg, hyst_deg):
    """(do_cao_thang_dung | None, blocked) tu khoang cach laser doc truc than.

    Giong ekf_altitude cua FC: qua max_tilt_deg thi bo, va da bo roi thi phai ve duoi
    (max_tilt_deg - hyst_deg) moi nhan lai - khong co tre thi quanh nguong laser bat/tat tung mau.
    """
    limit = max_tilt_deg - hyst_deg if blocked else max_tilt_deg
    blocked = tilt_cos < math.cos(math.radians(limit))
    if blocked or not (min_range <= range_m <= max_range):
        return None, blocked
    return range_m * tilt_cos, blocked


def parse_known_tags(flat):
    """[id, x, y, z, id, x, y, z, ...] -> {id: (x, y, z)}. Sai do dai thi ValueError."""
    if len(flat) % 4:
        raise ValueError(f'known_tags phai co boi so cua 4 phan tu, dang co {len(flat)}')
    return {int(flat[i]): tuple(flat[i + 1:i + 4]) for i in range(0, len(flat), 4)}


class HealthMonitor:
    """Co healthy cua EKF: im lang, phuong sai vi tri vuot nguong lien tuc, hoac nan/inf."""

    def __init__(self, max_pos_variance, unhealthy_after_s, odom_timeout_s):
        self.max_pos_variance = max_pos_variance
        self.unhealthy_after_s = unhealthy_after_s
        self.odom_timeout_s = odom_timeout_s
        self.over_since_s = None

    def evaluate(self, now_s, odom_stamp_s, pos_variance, values_finite):
        """Tra (healthy, reason). odom_stamp_s = thoi diem nhan odom gan nhat, None neu chua co."""
        if odom_stamp_s is None:
            return False, 'chua nhan /odometry/filtered'
        if now_s - odom_stamp_s > self.odom_timeout_s:
            return False, f'EKF im lang {now_s - odom_stamp_s:.1f} s'
        if not values_finite:
            return False, 'odom co nan/inf'
        worst = max(pos_variance)
        if worst <= self.max_pos_variance:
            self.over_since_s = None
            return True, ''
        if self.over_since_s is None:
            self.over_since_s = now_s
        if now_s - self.over_since_s >= self.unhealthy_after_s:
            return False, (f'phuong sai vi tri {worst:.2f} m^2 > {self.max_pos_variance} '
                           f'qua {self.unhealthy_after_s:.1f} s')
        return True, ''


class MarkerResetPolicy:
    """Khi nao ep EKF ve vi tri marker (set_pose).

    EKF da lech xa thi pose0_rejection_threshold loai ca do marker DUNG, va chi nhan lai khi
    phuong sai tu phinh du lon - do 09-15: lech 20 m mat 23 s, qua dem 14->15/09 khong bao gio.
    Ep khi co pose marker moi va EKF con chay (khong im lang) ma: EKF khong healthy, hoac lech
    marker qua max_error_m lien tuc disagree_after_s. Moi lan ep cach nhau it nhat cooldown_s.
    """

    def __init__(self, max_error_m, disagree_after_s, marker_max_age_s, cooldown_s):
        self.max_error_m = max_error_m
        self.disagree_after_s = disagree_after_s
        self.marker_max_age_s = marker_max_age_s
        self.cooldown_s = cooldown_s
        self.disagree_since_s = None
        self.last_reset_s = None

    def evaluate(self, now_s, ekf_alive, ekf_healthy, ekf_pos, marker_pos, marker_stamp_s):
        """Tra ly do ep (chuoi khac rong) hoac '' neu khong ep. Goi deu dan moi chu ky."""
        if not ekf_alive or marker_stamp_s is None or now_s - marker_stamp_s > self.marker_max_age_s:
            self.disagree_since_s = None
            return ''
        error = math.dist(ekf_pos, marker_pos)
        if not math.isfinite(error) or error > self.max_error_m:
            if self.disagree_since_s is None:
                self.disagree_since_s = now_s
        else:
            self.disagree_since_s = None
        if self.last_reset_s is not None and now_s - self.last_reset_s < self.cooldown_s:
            return ''
        if not ekf_healthy:
            reason = 'EKF khong healthy'
        elif (self.disagree_since_s is not None
              and now_s - self.disagree_since_s >= self.disagree_after_s):
            reason = f'lech marker {error:.2f} m > {self.max_error_m} m qua {self.disagree_after_s:.1f} s'
        else:
            return ''
        self.last_reset_s = now_s
        self.disagree_since_s = None
        return reason
