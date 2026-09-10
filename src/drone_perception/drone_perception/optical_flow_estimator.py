"""Logic optical flow thuan, tach khoi ROS de pytest duoc khong can khoi dong node."""

import cv2
import numpy as np


class OpticalFlowEstimator:
    """Lucas-Kanade thua: nhe CPU hon Farneback day, dung lam diem xuat phat tren Pi 4."""

    def __init__(self, focal_px, max_corners=100, quality_level=0.3,
                 min_distance=7, min_tracked_features=8):
        self.focal_px = focal_px
        self.max_corners = max_corners
        self.quality_level = quality_level
        self.min_distance = min_distance
        self.min_tracked_features = min_tracked_features
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

    def process(self, gray, dt, altitude_m):
        """Tra ve (vel_xy_mps, n_tracked). vel None nghia la KHONG duoc publish gi ca.

        TODO: hien thuc theo khung o muc 1.4 tai lieu huong dan:
          1. neu chua co dac trung -> _detect_features, tra (None, 0)
          2. calcOpticalFlowPyrLK, loc theo status
          3. n_tracked < min_tracked_features -> reset va tra (None, n_tracked)
             (khong bao gio publish gia tri nhieu - robot_localization xu ly thieu phep do
             tot hon nhieu so voi phep do sai ma covariance khai thap)
          4. flow_px = np.median(good_new - good_old, axis=0)   # median chong nhieu hon mean
          5. vel = flow_px * altitude_m / focal_px / dt
        """
        raise NotImplementedError('optical flow chua duoc hien thuc')
