"""Nen anh camera de ghi vao bag (camera_log_node) - thuan Python, pytest duoc."""

import cv2
import numpy as np


def encode_jpeg(width, height, step, data, quality):
    """Anh mono8 (bytes theo hang, step byte/hang) -> bytes JPEG, hoac None neu loi."""
    img = np.frombuffer(data, np.uint8).reshape(height, step)[:, :width]
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return buf.tobytes() if ok else None
