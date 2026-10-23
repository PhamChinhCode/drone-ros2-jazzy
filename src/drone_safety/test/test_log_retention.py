from drone_safety.log_retention import chon_xoa, la_hoat_dong, trung_hoat_dong

NOW = 100000.0
MB = 1024 ** 2


def bag(i, tuoi_s, size=10 * MB):
    """File bag thu i ngung ghi cach day tuoi_s giay."""
    return (f'bag_{i}.mcap', NOW - tuoi_s, size)


def test_hoat_dong_khi_arm_hoac_co_quyen_hoac_co_nhiem_vu():
    assert not la_hoat_dong(False, False, False)
    assert not la_hoat_dong(False, None, False)        # khong biet quyen = khong tinh
    assert la_hoat_dong(True, None, False)              # bay tay: FC arm
    assert la_hoat_dong(False, True, False)
    assert la_hoat_dong(False, False, True)


def test_moc_trong_bien_2_phut_truoc_va_sau_file():
    assert trung_hoat_dong(1000, 1060, [1180])
    assert trung_hoat_dong(1000, 1060, [880])
    assert not trung_hoat_dong(1000, 1060, [1181])


def test_file_khong_hoat_dong_giu_30_phut_roi_xoa():
    files = [bag(0, 1900), bag(1, 1700), bag(2, 0)]
    assert chon_xoa(files, [], NOW, cap_bytes=10 ** 12) == ['bag_0.mcap']


def test_giu_doan_hoat_dong_cong_2_phut():
    moc = [NOW - 5000 + k for k in range(300)]          # bay 5 phut, cach day ~1,4 gio
    files = [bag(i, 5400 - 60 * i) for i in range(12)] + [bag(99, 0)]
    xoa = chon_xoa(files, moc, NOW, cap_bytes=10 ** 12)
    giu = [f[0] for f in files if f[0] not in xoa]
    # file i ket thuc NOW-5400+60i, bat dau 60 s truoc; hoat dong NOW-5000..NOW-4701, bien 120 s
    assert giu == [f'bag_{i}.mcap' for i in range(5, 12)] + ['bag_99.mcap']


def test_vuot_tran_xoa_file_cu_nhat_ke_ca_hoat_dong_tru_file_dang_ghi():
    moc = [NOW - 100]
    files = [bag(0, 160, 6 * MB), bag(1, 100, 6 * MB), bag(2, 0, 6 * MB)]
    assert chon_xoa(files, moc, NOW, cap_bytes=7 * MB) == ['bag_0.mcap', 'bag_1.mcap']


def test_khong_co_file():
    assert chon_xoa([], [], NOW, cap_bytes=0) == []
