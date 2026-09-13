"""Nhan ra cham dat - tieu chi Pi chot o GIAO_UOC_FC_ROS2.md 11.1 #12e, thuan Python.

Cham dat khi 1-3 cung dung lien tuc hold_s:
  1. laser <= moc mat dat + margin_m;
  2. |vz| < vz_still_mps (van toc tu ODOMETRY, covariance vz hop le);
  3. Pi van dang ra lenh xuong (vz lenh <= -cmd_descend_mps).
Dieu kien 3 loai truong hop bay ngang qua hop cao. KHONG dung ga (cham dat ga van ~27 %) va
KHONG dung landed_state. Gui DISARM hay khong (OB_DIS_RDY) la viec cua noi goi.

Moc mat dat = trung vi laser trong ref_window_s luc dang disarm va nam yen; ngoai khoang
[ref_min_m, ref_max_m] hoac khong co mau (vd arm luc cam tay) thi dung 0,17 m nhu FC.
Nguong la SO TAM - chua do khi canh quay (12.B).
"""

from collections import deque
from statistics import median

GROUND_REF_DEFAULT_M = 0.17


class LandingDetector:

    def __init__(self, margin_m=0.05, vz_still_mps=0.05, cmd_descend_mps=0.1, hold_s=1.0,
                 ref_window_s=2.0, ref_min_m=0.15, ref_max_m=0.25):
        self.margin_m = margin_m
        self.vz_still_mps = vz_still_mps
        self.cmd_descend_mps = cmd_descend_mps
        self.hold_s = hold_s
        self.ref_window_s = ref_window_s
        self.ref_min_m = ref_min_m
        self.ref_max_m = ref_max_m
        self._ref_samples = deque()
        self._ground_ref_m = None
        self._since_s = None

    @property
    def ground_ref_m(self):
        return GROUND_REF_DEFAULT_M if self._ground_ref_m is None else self._ground_ref_m

    def update_ground_ref(self, armed, range_m, vz_mps, now_s):
        """Goi moi mau laser. Chi hoc moc khi dang disarm va nam yen; da arm thi giu moc."""
        if armed:
            return
        while self._ref_samples and now_s - self._ref_samples[0][0] > self.ref_window_s:
            self._ref_samples.popleft()
        if range_m is None or vz_mps is None or abs(vz_mps) >= self.vz_still_mps:
            self._ref_samples.clear()
            self._ground_ref_m = None
            return
        self._ref_samples.append((now_s, range_m))
        span = now_s - self._ref_samples[0][0]
        ref = median(r for _, r in self._ref_samples)
        if span >= self.ref_window_s * 0.9 and self.ref_min_m <= ref <= self.ref_max_m:
            self._ground_ref_m = ref
        else:
            self._ground_ref_m = None

    def step(self, range_m, vz_mps, cmd_vz_mps, now_s):
        """Tra True khi da cham dat. Mot mau thieu hoac sai la dem lai tu dau."""
        ok = (range_m is not None and vz_mps is not None and cmd_vz_mps is not None
              and range_m <= self.ground_ref_m + self.margin_m
              and abs(vz_mps) < self.vz_still_mps
              and cmd_vz_mps <= -self.cmd_descend_mps)
        if not ok:
            self._since_s = None
            return False
        if self._since_s is None:
            self._since_s = now_s
        return now_s - self._since_s >= self.hold_s
