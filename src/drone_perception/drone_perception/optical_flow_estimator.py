"""Logic optical flow thuan, tach khoi ROS de pytest duoc khong can khoi dong node."""

import cv2


class OpticalFlowEstimator:
    """Lucas-Kanade thua: nhe CPU hon Farneback day, dung lam diem xuat phat tren Pi 4."""

    def __init__(self, max_corners=100, quality_level=0.3,
                 min_distance=7, min_tracked_features=8, max_lk_error=20.0):
        self.max_corners = max_corners
        self.quality_level = quality_level
        self.min_distance = min_distance
        self.min_tracked_features = min_tracked_features
        self.max_lk_error = max_lk_error
        self.prev_gray = None
        self.prev_pts = None

    def reset(self):
        """Bo dac trung dang bam, buoc lan process ke tiep phat hien lai tu dau."""
        self.prev_gray = None
        self.prev_pts = None

    def _detect_features(self, gray):
        return cv2.goodFeaturesToTrack(
            gray, maxCorners=self.max_corners, qualityLevel=self.quality_level,
            minDistance=self.min_distance)

    def track(self, gray):
        """Tra ve ((diem_cu Nx2, diem_moi Nx2), n_tracked); cap None = KHONG duoc publish gi ca.

        Chi bam diem tren anh; doi sang van toc that (nghieng camera, tu the, do cao) la viec
        cua flow_geometry.base_velocity.
        """
        # Khung dau tien chua co gi de so sanh -> chi phat hien dac trung.
        if self.prev_gray is None or self.prev_pts is None:
            self.prev_gray = gray
            self.prev_pts = self._detect_features(gray)
            return None, 0

        new_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None)
        if new_pts is None:
            self.reset()
            return None, 0

        # CHI loc theo status la KHONG DU: do thuc tren anh trang, LK van bao status=1 cho
        # ca 100/100 diem va tra ve van toc rac (-6.5 m/s). Phai loc them theo err -
        # bam tot cho err ~0, con anh trang/nhieu cho err 70-150, tach bach hoan toan.
        giu = (status.ravel() == 1) & (err.ravel() < self.max_lk_error)
        good_new = new_pts[giu].reshape(-1, 2)
        good_old = self.prev_pts[giu].reshape(-1, 2)
        n_tracked = len(good_new)

        # Khong bao gio publish gia tri nhieu: robot_localization xu ly THIEU phep do tot hon
        # nhieu so voi phep do SAI ma covariance lai khai thap.
        if n_tracked < self.min_tracked_features:
            self.reset()
            return None, n_tracked

        self.prev_gray = gray
        self.prev_pts = good_new.reshape(-1, 1, 2)
        return (good_old, good_new), n_tracked
