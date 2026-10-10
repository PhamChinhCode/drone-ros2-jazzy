"""Kiem luat dan tiep can thang hang truc bai - hinh hoc + mo phong vong kin."""

import math

import pytest

from drone_mission.approach_guidance import ApproachParams, blend_yaw, guide, wrap

P = ApproachParams()
PAD = (5.0, 3.0, 0.0)
YAW_PAD = math.radians(90)          # phia TREN cua bai huong Bac (+y ENU)


def test_tren_truc_sau_bai_thi_bam_truc_mui_dung_huong():
    g = guide((5.0, 3.0 - 1.0, 1.8), YAW_PAD, PAD, YAW_PAD)
    assert g.phase == 'final'
    assert g.target[0] == pytest.approx(5.0)
    assert 3.0 - 1.0 < g.target[1] <= 3.0                    # carrot tien ve tam bai
    assert g.yaw == pytest.approx(YAW_PAD)
    assert P.final_alt < g.target[2] < P.gate_alt            # dang ha doc


def test_lech_ngang_trong_hanh_lang_thi_dich_nam_tren_truc():
    g = guide((5.3, 3.0 - 1.5, 1.8), YAW_PAD, PAD, YAW_PAD)
    assert g.phase == 'final' and g.target[0] == pytest.approx(5.0)


def test_xa_sau_bai_thi_toi_cong_G():
    g = guide((5.0, 3.0 - 8.0, 5.0), YAW_PAD, PAD, YAW_PAD)
    assert g.phase == 'to_gate'
    assert g.target[:2] == pytest.approx((5.0, 3.0 - P.gate_dist))
    assert g.yaw == pytest.approx(YAW_PAD)                    # G nam thang phia truoc


def test_phia_truoc_bai_thi_vong_cung_ve_sau():
    g = guide((5.0, 3.0 + 3.0, 2.0), math.radians(-90), PAD, YAW_PAD)
    assert g.phase == 'orbit'
    r = math.hypot(g.target[0] - PAD[0], g.target[1] - PAD[1])
    assert r >= P.gate_dist - 1e-9                            # khong cat ngang qua bai


def test_vong_cung_chon_chieu_ngan_hon():
    """Dang o phia truoc lech sang Dong -> vong qua phia Dong."""
    g = guide((5.0 + 1.0, 3.0 + 2.5, 2.0), 0.0, PAD, YAW_PAD)
    assert g.phase == 'orbit' and g.target[0] > PAD[0]


def test_toi_noi_lui_sau_tam_bai():
    y = 3.0 - P.final_standoff                          # diem toi noi lui sau tam doc truc
    g = guide((5.05, y, 1.1), YAW_PAD, PAD, YAW_PAD)
    assert g.phase == 'arrived' and g.target[:2] == pytest.approx((5.0, y))
    assert guide((5.05, y, 1.1), YAW_PAD + math.radians(20), PAD, YAW_PAD).phase != 'arrived'
    assert guide((5.05, y, 1.4), YAW_PAD, PAD, YAW_PAD).phase == 'final'        # chua ha xong
    # Dung tren tam (= vot qua diem dung) khong con la "toi noi" - lui ve.
    assert guide((5.0, 3.0 + 0.10, 1.1), YAW_PAD, PAD, YAW_PAD).phase == 'final'


def test_carrot_khong_vuot_diem_dung():
    for d in (1.4, 0.8, 0.5, 0.3, 0.16):
        g = guide((5.0, 3.0 - d, 1.5), YAW_PAD, PAD, YAW_PAD)
        assert g.target[1] <= 3.0 - P.final_standoff + 1e-9, d


def test_vuot_qua_tam_mot_chut_khong_vong_cung():
    """Lo vuot tam bai 0,3 m ve phia truoc: van giu tam, khong bat bay vong quanh bai."""
    g = guide((5.0, 3.3, 1.1), YAW_PAD, PAD, YAW_PAD)
    assert g.phase == 'final' and g.target[:2] == pytest.approx((5.0, 3.0 - P.final_standoff))
    assert g.yaw == pytest.approx(YAW_PAD)


def test_tron_goc_qua_180():
    assert wrap(blend_yaw(math.radians(170), math.radians(-170), 0.5)) == pytest.approx(math.pi)


def mo_phong(start, yaw0, yaw_pad, v=0.5, rate=math.radians(30), dt=0.05, t_max=120.0):
    """Drone dong hoc don gian: bay thang toi dich tuc thoi voi toc do <= v, quay <= rate."""
    x, y, z = start
    yaw = yaw0
    vet = []
    t = 0.0
    while t < t_max:
        g = guide((x, y, z), yaw, PAD, yaw_pad)
        vet.append((x, y, z, yaw, g.phase))
        if g.phase == 'arrived':
            return vet
        ex, ey, ez = g.target[0] - x, g.target[1] - y, g.target[2] - z
        d = math.hypot(ex, ey)
        if d > 1e-9:
            k = min(v * dt, d) / d
            x, y = x + ex * k, y + ey * k
        z += max(-v * dt, min(v * dt, ez))
        yaw = wrap(yaw + max(-rate * dt, min(rate * dt, wrap(g.yaw - yaw))))
        t += dt
    raise AssertionError(f'khong toi noi sau {t_max} s, dung o {vet[-1]}')


@pytest.mark.parametrize('start,yaw0', [
    ((5.0, -6.0, 5.0), math.radians(90)),       # sau bai, dung truc
    ((-2.0, -2.0, 5.0), math.radians(0)),       # sau-trai, lech nhieu
    ((9.0, 4.0, 3.0), math.radians(180)),       # ben canh
    ((5.0, 9.0, 4.0), math.radians(-90)),       # phia TRUOC bai: phai vong
    ((3.0, 7.0, 3.0), math.radians(-60)),       # truoc-trai
])
@pytest.mark.parametrize('yaw_pad_deg', [90, -30, 180])
def test_mo_phong_toi_noi_dung_huong_khong_quay_tren_bai(start, yaw0, yaw_pad_deg):
    yaw_pad = math.radians(yaw_pad_deg)
    # Xoay ca kich ban quanh tam bai de bai co huong yaw_pad, giu nguyen hinh hoc tuong doi.
    a = yaw_pad - math.radians(90)
    sx, sy = start[0] - PAD[0], start[1] - PAD[1]
    s = (PAD[0] + sx * math.cos(a) - sy * math.sin(a),
         PAD[1] + sx * math.sin(a) + sy * math.cos(a), start[2])
    vet = mo_phong(s, wrap(yaw0 + a), yaw_pad)
    x, y, z, yaw, _ = vet[-1]
    assert abs(wrap(yaw - yaw_pad)) <= math.radians(P.arrive_yaw_deg)
    assert z == pytest.approx(PAD[2] + P.final_alt, abs=0.05)
    for x, y, z, yaw, phase in vet:
        horiz = math.hypot(x - PAD[0], y - PAD[1])
        if horiz < 0.6:
            # Yeu cau user: khong quay mui khi da o tren bai (camera quay theo se mat tag).
            assert abs(wrap(yaw - yaw_pad)) <= math.radians(P.arrive_yaw_deg), (x, y, yaw)
        if phase == 'final' and horiz > 0.3:
            # Tag (tam bai) phai nam trong FOV ngang (+-47 do, giu bien 35 do) suot doan cuoi.
            bearing = math.atan2(PAD[1] - y, PAD[0] - x)
            assert abs(wrap(bearing - yaw)) <= math.radians(35), (x, y, yaw)


def test_da_o_final_bi_qua_tinh_day_lech_nhe_thi_van_bam_truc():
    # 1 m sau bai, lech ngang 0,6 m: ngoai hanh lang 25 do (0,47 m) nhung trong final_hold_lateral.
    pos = (5.6, 3.0 - 1.0, 1.6)
    assert guide(pos, YAW_PAD, PAD, YAW_PAD).phase != 'final'          # chua vao final: ve cong G
    g = guide(pos, YAW_PAD, PAD, YAW_PAD, was_final=True)
    assert g.phase == 'final' and g.target[0] == pytest.approx(5.0)    # da vao: keo ve truc


def test_da_o_final_lech_qua_xa_thi_van_bo_final():
    g = guide((5.0 + P.final_hold_lateral + 0.3, 3.0 - 1.0, 1.6), YAW_PAD, PAD, YAW_PAD,
              was_final=True)
    assert g.phase != 'final'


def test_da_o_final_troi_ra_truoc_bai_thi_bo_final():
    g = guide((5.0, 3.0 + 1.0, 1.6), YAW_PAD, PAD, YAW_PAD, was_final=True)
    assert g.phase == 'orbit'
