"""Chi so dap ung buoc cho viec tune cruise.*, thuan Python de pytest duoc."""


def analyze(samples, start, target, band_m):
    """samples: [(t, gia_tri_tren_truc)] voi t = 0 luc doi setpoint, sap theo t.

    Tra dict:
      rise_s        - tu 10 % toi 90 % quang buoc (None neu chua toi 90 %);
      overshoot_pct - vuot qua dich, % quang buoc (0 neu khong vuot);
      settle_s      - thoi diem tu do tro di luon nam trong +-band_m quanh dich
                      (None neu mau cuoi van ngoai dai);
      final_error_m - |mau cuoi - dich|.
    """
    span = target - start
    if not samples or span == 0:
        raise ValueError('can it nhat mot mau va quang buoc khac 0')
    frac = [(t, (v - start) / span) for t, v in samples]
    t10 = next((t for t, f in frac if f >= 0.1), None)
    t90 = next((t for t, f in frac if f >= 0.9), None)
    rise = None if t10 is None or t90 is None else t90 - t10
    overshoot = max(0.0, (max(f for _, f in frac) - 1.0) * 100.0)
    settle = None                       # lan VAO dai cuoi cung; ra dai la xoa
    for t, v in samples:
        if abs(v - target) > band_m:
            settle = None
        elif settle is None:
            settle = t
    return {'rise_s': rise, 'overshoot_pct': overshoot, 'settle_s': settle,
            'final_error_m': abs(samples[-1][1] - target)}
