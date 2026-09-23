"""geo - doi qua lai WGS84 (lat, lon, alt) <-> khung ban do tag (odom), thuan Python.

Khung ban do tag (= khung odom sau khi neo) la ENU met: x = truc E cua ban do, y = truc N cua ban
do, z = len (tags.yaml; giao uoc GCS 5.2b: n = y, e = x). Goc ban do dat tai mot diem WGS84 da biet
(GCS gui kem ban do tag, giao uoc GCS 8.7 - hoac khai tay trong tags.yaml).

Truc N cua ban do KHONG nhat thiet trung Bac that: north_yaw_deg la goc phuong vi cua truc N ban do
do tu Bac THAT, chieu kim dong ho (0 = ban do dat dung Bac/Dong).

Dung phep ECEF -> ENU chinh xac, khong dung xap xi phang: sai so cua xap xi phang tang theo binh
phuong khoang cach (~8 cm o 1 km, ~2 m o 5 km) - vua du de lech mot bai dap.
"""

import math
from dataclasses import dataclass

# WGS84
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


@dataclass(frozen=True)
class GeoOrigin:
    lat_deg: float
    lon_deg: float
    alt_m: float
    north_yaw_deg: float = 0.0


def _lla_to_ecef(lat_deg, lon_deg, alt_m):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    s, c = math.sin(lat), math.cos(lat)
    n = _A / math.sqrt(1.0 - _E2 * s * s)
    return ((n + alt_m) * c * math.cos(lon),
            (n + alt_m) * c * math.sin(lon),
            (n * (1.0 - _E2) + alt_m) * s)


def _ecef_to_lla(x, y, z):
    """Lap Bowring - hoi tu duoi 1 mm sau 3 vong o moi do cao cua drone."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - _E2))
    alt = 0.0
    for _ in range(5):
        s = math.sin(lat)
        n = _A / math.sqrt(1.0 - _E2 * s * s)
        alt = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - _E2 * n / (n + alt)))
    return math.degrees(lat), math.degrees(lon), alt


def _enu_basis(o):
    lat, lon = math.radians(o.lat_deg), math.radians(o.lon_deg)
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    e = (-so, co, 0.0)
    n = (-sl * co, -sl * so, cl)
    u = (cl * co, cl * so, sl)
    return e, n, u


def lla_to_map(lat_deg, lon_deg, alt_m, o):
    """WGS84 -> (x, y, z) khung ban do (ENU met cua ban do, goc tai o)."""
    p = _lla_to_ecef(lat_deg, lon_deg, alt_m)
    p0 = _lla_to_ecef(o.lat_deg, o.lon_deg, o.alt_m)
    d = (p[0] - p0[0], p[1] - p0[1], p[2] - p0[2])
    e, n, u = _enu_basis(o)
    east = sum(d[i] * e[i] for i in range(3))
    north = sum(d[i] * n[i] for i in range(3))
    up = sum(d[i] * u[i] for i in range(3))
    # Truc N ban do co phuong vi th (tu Bac that, chieu kim): N_ban_do = (sin th, cos th) trong
    # (E, N) that, E_ban_do = (cos th, -sin th).
    th = math.radians(o.north_yaw_deg)
    x = east * math.cos(th) - north * math.sin(th)
    y = east * math.sin(th) + north * math.cos(th)
    return x, y, up


def map_to_lla(x, y, z, o):
    """(x, y, z) khung ban do -> WGS84 (lat_deg, lon_deg, alt_m). Nghich dao cua lla_to_map."""
    th = math.radians(o.north_yaw_deg)
    east = x * math.cos(th) + y * math.sin(th)
    north = -x * math.sin(th) + y * math.cos(th)
    e, n, u = _enu_basis(o)
    p0 = _lla_to_ecef(o.lat_deg, o.lon_deg, o.alt_m)
    p = tuple(p0[i] + east * e[i] + north * n[i] + z * u[i] for i in range(3))
    return _ecef_to_lla(*p)


def origin_from_params(valid, lat_deg, lon_deg, alt_m, north_yaw_deg):
    """Tham so tags.yaml -> GeoOrigin, hoac None khi chua khai goc (valid = false)."""
    if not valid:
        return None
    if not (-90.0 <= lat_deg <= 90.0 and -180.0 <= lon_deg <= 180.0):
        raise ValueError(f'goc ban do ngoai mien: lat {lat_deg}, lon {lon_deg}')
    return GeoOrigin(float(lat_deg), float(lon_deg), float(alt_m), float(north_yaw_deg))


def gps_measurement(fix_type, h_acc_m, sats, min_fix_type, max_h_acc_m, min_sats):
    """Co dua mau GPS vao EKF khong. Tra (True, '') hoac (False, ly_do).

    Cong theo SAI SO MODULE TU BAO (h_acc), khong theo DOP: NAV-PVT cua u-blox co h_acc that, con
    FC gui eph = UINT16_MAX (giao uoc FC 1.8, muc 4.3). So ve tinh la chot chan thu hai: h_acc cua
    nghiem it ve tinh hay lac quan.
    """
    if fix_type < min_fix_type:
        return False, f'fix_type {fix_type} < {min_fix_type}'
    if not (math.isfinite(h_acc_m) and 0.0 < h_acc_m <= max_h_acc_m):
        return False, f'h_acc {h_acc_m:.1f} m > {max_h_acc_m} m'
    if sats < min_sats:
        return False, f'{sats} ve tinh < {min_sats}'
    return True, ''
