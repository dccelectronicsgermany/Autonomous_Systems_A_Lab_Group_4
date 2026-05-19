#!/usr/bin/env python3
"""
HSHL Line Following Student Lab — Your Implementation
=====================================================

Implement your line following algorithm by filling in the function below:

    detect_line(image)  — called for every camera frame (~30 fps)

─────────────────────────────────────────────────────────────────────────────
INPUTS  (what you receive from the camera)
─────────────────────────────────────────────────────────────────────────────
Camera frame  →  detect_line(image)
  image         np.ndarray, shape (720, 1280, 3), BGR colour order
                Same convention as OpenCV.

─────────────────────────────────────────────────────────────────────────────
OUTPUTS  (what your function must return)
─────────────────────────────────────────────────────────────────────────────
detect_line(image)  →  float | None
  Return a steering value in range [-1.0, 1.0]:
    -1.0  = steer full left
     0.0  = go straight (line is centered)
    +1.0  = steer full right
        None  = cannot detect line (framework uses neutral steering fallback)

─────────────────────────────────────────────────────────────────────────────
ALGORITHM TIPS
─────────────────────────────────────────────────────────────────────────────
1. The line is painted GREEN on the road (BGR: 0, 255, 0)
2. Use color range thresholding to detect green pixels
3. Find the line center using contour moments
4. Compare line center to image center to get steering offset
5. Use morphological operations to reduce noise
6. Return None if no line is detected

See docs/line_detection_example.py for a complete example implementation.

─────────────────────────────────────────────────────────────────────────────
HELPERS
─────────────────────────────────────────────────────────────────────────────
    self.show_notification(text)  white  — general info
    self.show_warning(text)       yellow — caution
    self.show_alert(text)         red    — critical
    self.current_image            latest camera frame (or None)
"""
import cv2          # type: ignore
import joblib       # type: ignore
import numpy as np  # type: ignore
import rclpy        # type: ignore
from pathlib import Path

from .interface import LineFollowingInterface

# Load model and steering map once at startup
_BASE      = Path(__file__).parent.parent / "models"
_MODEL     = joblib.load(str(_BASE / "svm_line_follower.joblib"))
_STEER_MAP = np.load(str(_BASE / "class_to_steer.npy"))  # [-0.30, 0.00, +0.30]

_PATCH_SIZE   = 64
_TARGET_W     = 320
_TARGET_H     = 180
_ROI_FRACTION = 0.45


def _compute_edges(roi_gray: np.ndarray) -> np.ndarray:
    blurred = cv2.GaussianBlur(roi_gray, (5, 5), 0)
    return cv2.Canny(blurred, 50, 150)


def _dominant_segment(edges: np.ndarray):
    lines = cv2.HoughLinesP(
        edges, rho=1, theta=np.pi / 180,
        threshold=25, minLineLength=25, maxLineGap=15,
    )
    if lines is None:
        return None
    best, best_len, best_angle = None, 0, 0.0
    for x1, y1, x2, y2 in lines[:, 0]:
        length = np.hypot(x2 - x1, y2 - y1)
        if length > best_len:
            best_len = length
            best = ((x1 + x2) / 2, (y1 + y2) / 2)
            best_angle = float(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
    return best[0], best[1], best_angle


def _crop_patch(edge_img: np.ndarray, cx: float, cy: float) -> np.ndarray:
    h, w = edge_img.shape
    half = _PATCH_SIZE // 2
    x1 = max(0, int(cx) - half); x2 = min(w, int(cx) + half)
    y1 = max(0, int(cy) - half); y2 = min(h, int(cy) + half)
    return cv2.resize(edge_img[y1:y2, x1:x2], (_PATCH_SIZE, _PATCH_SIZE))


def _extract_features(img_bgr: np.ndarray):
    """Returns (feature_vector, x_norm) or (None, None) if no segment found."""
    img   = cv2.resize(img_bgr, (_TARGET_W, _TARGET_H))
    gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    y0    = int(_TARGET_H * _ROI_FRACTION)
    roi   = gray[y0:, :]
    edges = _compute_edges(roi)
    seg   = _dominant_segment(edges)
    if seg is None:
        return None, None
    cx, cy, angle = seg
    patch      = _crop_patch(edges, cx, cy)
    patch_feat = (patch.astype(np.float32) / 255.0).flatten()
    x_norm     = np.float32((cx / roi.shape[1]) * 2.0 - 1.0)
    a_norm     = np.float32(angle / 90.0)
    return np.append(patch_feat, [x_norm, a_norm]), float(x_norm)


class MyLineFollower(LineFollowingInterface):
    """SVM-based line follower using Lab07b edge-patch pipeline."""

    def __init__(self):
        super().__init__("my_line_follower")
        self._frame_count = 0
        self.on_camera_image(self.detect_line)
        self.get_logger().info("MyLineFollower (SVM + Lab07b pipeline) initialized")

    def detect_line(self, image: np.ndarray) -> float | None:
        self._frame_count += 1

        feat, x_norm = _extract_features(image)
        if feat is None:
            self.show_warning("No line segment detected")
            return None

        cls   = int(_MODEL.predict(feat.reshape(1, -1))[0])
        steer = float(np.clip(_STEER_MAP[cls], -1.0, 1.0))

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"steer={steer:+.2f}  class={['LEFT','STRAIGHT','RIGHT'][cls]}  x_norm={x_norm:+.2f}"
            )
        self.show_notification(f"steer={steer:+.2f}")

        return steer


def main(args=None):
    """Main entry point for the line follower node."""
    rclpy.init(args=args)
    follower = MyLineFollower()
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
