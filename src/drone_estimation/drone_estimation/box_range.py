"""Bu laser khi vet sang quet qua nap hop hang - thuan Python, pytest duoc.

MTF-01 lap 0,10 m sau tam, hop 12 cm nam ngay duoi duoi drone luc lay hang: lui ve hop thi vet sang
chac chan di qua nap, luc treo gap chi con khe ~3 cm (mo phong ToF theo tia 2026-10-22: drone leo
13 cm, laser < 0,10 m thanh "mat laser" dung luc gap). Chi BAT khi nhiem vu bao dang o pha hop
(/mission/box_height > 0) - ngoai pha do bac, ban, thung khong bi hieu nham la hop.

Cach lam: theo doi do cao TOI DAT (ground). So doc ngan hon ground qua nua chieu cao hop -> vet sang
tren nap: tra so doc + chieu cao hop. ground chi doi toi da max_rate_mps (so doc vat mep khong keo
lech duoc), bat dau tu so doc dau tien khi bat (luc do drone tren tam bai, chua toi hop). Laser mat
(duoi min_range) khi nap hop da gan hon min_range -> dang tren nap: giu ground, KHONG bao mat laser.
"""


class BoxRange:

    def __init__(self, max_rate_mps=0.5, min_range_m=0.10):
        self.max_rate_mps = max_rate_mps
        self.min_range_m = min_range_m
        self.height = 0.0
        self.ground = None
        self.t = None
        self.on_box = False

    def set_box(self, height_m):
        """Chieu cao hop dang lay (m); 0 = khong o pha hop, tat bu va quen trang thai."""
        if height_m != self.height:
            self.height = height_m
            self.ground = self.t = None
            self.on_box = False

    def update(self, h, t):
        """h: do cao laser thang dung (m) hoac None (mat); t: thoi diem (s).

        Tra (do cao toi dat hoac None, do_that): do_that False = so giu lai khi laser mat tren nap,
        khong nen dua vao EKF.
        """
        if self.height <= 0.0:
            return h, h is not None
        if h is None:
            if self.ground is not None and self.ground - self.height < self.min_range_m + 0.05:
                self.on_box = True
                return self.ground, False
            return None, False
        if self.ground is None:
            self.ground, self.t = h, t
            return h, True
        self.on_box = self.ground - h > self.height / 2
        g = h + self.height if self.on_box else h
        step = self.max_rate_mps * max(t - self.t, 0.0)
        self.ground = min(max(g, self.ground - step), self.ground + step)
        self.t = t
        return g, True
