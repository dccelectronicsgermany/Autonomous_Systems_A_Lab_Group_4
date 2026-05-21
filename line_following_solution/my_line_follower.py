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
import pickle
import numpy as np  # type: ignore
import rclpy        # type: ignore
from pathlib import Path

from .interface import LineFollowingInterface

_MODEL = pickle.load(open(str(Path(__file__).parent / "SVM_Line_Following_SVR.pkl"), "rb"))

_LOWER_GREEN = np.array([40,  40,  40])
_UPPER_GREEN = np.array([90, 255, 255])
_MIN_AREA    = 100
_CAM_OFFSET  = 0.05  # camera mounted left of center — subtract to correct steering bias


class MyLineFollower(LineFollowingInterface):
    """
    Student implementation of line following.

    Detect a green line and steer to stay centered on it.
    """

    _Kp = 1.0
    _Kd = 0.08

    def __init__(self):
        super().__init__("my_line_follower")
        self._frame_count = 0
        self._lost_frames  = 0
        self._prev_error   = 0.0

        self.on_camera_image(self.detect_line)
        self.get_logger().info("MyLineFollower initialized — 6-feature SVR (near zone + edge) → PD steering")

    def detect_line(self, image: np.ndarray) -> float | None:
        """
        Detect the green line and return steering command.

        Args:
            image: BGR image from camera, shape (720, 1280, 3)

        Returns:
            Steering value in [-1.0, 1.0], or None if line not detected.
        """
        self._frame_count += 1

        def _green_mask(roi: np.ndarray) -> np.ndarray:
            hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask   = cv2.inRange(hsv, _LOWER_GREEN, _UPPER_GREEN)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            return mask

        def _svm_features(mask: np.ndarray) -> np.ndarray | None:
            """Compute 6 features from near-zone green mask: 5 contour + 1 edge-based center."""
            rows, cols = mask.shape[:2]
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None
            contour = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(contour)
            if area < _MIN_AREA:
                return None
            [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy, x, y   = float(vx[0]), float(vy[0]), float(x[0]), float(y[0])
            if abs(vy) > 0.01:
                line_x = x + (rows // 2 - y) * (vx / vy)
            else:
                M      = cv2.moments(contour)
                line_x = M["m10"] / M["m00"] if M["m00"] > 0 else cols / 2
            offset       = float(np.clip((line_x - cols / 2.0) / (cols / 2.0), -1.0, 1.0))
            angle_norm   = float(np.clip(np.degrees(np.arctan2(vy, vx)) / 90.0, -1.0, 1.0))
            area_norm    = float(np.clip(area / (rows * cols), 0.0, 1.0))
            _, _, bw, bh = cv2.boundingRect(contour)
            aspect_ratio = float(np.clip(bw / (bh + 1e-6), 0.0, 1.0))
            hull         = cv2.convexHull(contour)
            hull_area    = cv2.contourArea(hull)
            solidity     = float(np.clip(area / (hull_area + 1e-6), 0.0, 1.0))
            # edge-based center: midpoint between leftmost and rightmost edge column
            edges       = cv2.Canny(mask, 50, 150)
            edge_cols   = np.where(edges.any(axis=0))[0]
            if len(edge_cols) >= 2:
                edge_center = (float(edge_cols[0]) + float(edge_cols[-1])) / 2.0
                edge_offset = float(np.clip((edge_center - cols / 2.0) / (cols / 2.0), -1.0, 1.0))
            else:
                edge_offset = offset
            return np.array([offset, angle_norm, area_norm, aspect_ratio, solidity, edge_offset], dtype=np.float32)

        h         = image.shape[0]
        near_mask = _green_mask(image[h // 2:, :])
        feat      = _svm_features(near_mask)

        if feat is None:
            self._lost_frames += 1
            if self._lost_frames > 10:
                self.show_alert("Line lost!")
            else:
                self.show_warning("No green line detected")
            return None

        self._lost_frames = 0
        svr_offset = float(np.clip(_MODEL.predict(feat.reshape(1, -1))[0] - _CAM_OFFSET, -1.0, 1.0))

        # PD on SVR-predicted offset (no integral — SVR output is already smooth)
        error            = svr_offset
        derivative       = error - self._prev_error
        self._prev_error = error
        pid   = self._Kp * error + self._Kd * derivative
        steer = float(np.clip(pid, -1.0, 1.0))

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
