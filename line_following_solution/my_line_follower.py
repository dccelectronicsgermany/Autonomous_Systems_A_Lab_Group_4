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

_MODEL = pickle.load(open(str(Path(__file__).parent / "svm_line_follower_5feat.pkl"), "rb"))

_LOWER_GREEN = np.array([40,  40,  40])
_UPPER_GREEN = np.array([90, 255, 255])
_MIN_AREA    = 100
_CAM_OFFSET  = 0.15   # camera is mounted left of center — shifts line right in image


class MyLineFollower(LineFollowingInterface):
    """
    Student implementation of line following.

    Detect a green line and steer to stay centered on it.
    """

    _Kp = 0.8
    _Ki = 0.01
    _Kd = 0.15

    def __init__(self):
        super().__init__("my_line_follower")
        self._frame_count = 0
        self._lost_frames  = 0
        self._prev_error   = 0.0
        self._integral     = 0.0

        self.on_camera_image(self.detect_line)
        self.get_logger().info("MyLineFollower initialized — 5-feature SVM + PID steering")

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

        def _offset_from_mask(mask: np.ndarray) -> float | None:
            """Get normalized horizontal offset from a green mask. Returns None if no contour."""
            cols = mask.shape[1]
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None
            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) < _MIN_AREA:
                return None
            rows = mask.shape[0]
            [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy, x, y   = float(vx[0]), float(vy[0]), float(x[0]), float(y[0])
            if abs(vy) > 0.01:
                line_x = x + (rows // 2 - y) * (vx / vy)
            else:
                M      = cv2.moments(contour)
                line_x = M["m10"] / M["m00"] if M["m00"] > 0 else cols / 2
            return float(np.clip((line_x - cols / 2.0) / (cols / 2.0), -1.0, 1.0))

        def _svm_features(img_bgr: np.ndarray, mask: np.ndarray) -> np.ndarray | None:
            """Compute 5 SVM features from full image + its green mask (matches training pipeline)."""
            rows, cols = img_bgr.shape[:2]
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
            return np.array([offset, angle_norm, area_norm, aspect_ratio, solidity], dtype=np.float32)

        h = image.shape[0]
        near_start = int(h * 0.50)

        # Near zone — current position + SVM features (trained on full image so use full image mask)
        near_mask  = _green_mask(image[near_start:, :])
        offset     = _offset_from_mask(near_mask)
        if offset is not None:
            offset = float(np.clip(offset - _CAM_OFFSET, -1.0, 1.0))
        full_mask  = _green_mask(image)
        feat       = _svm_features(image, full_mask)

        if offset is None or feat is None:
            self._lost_frames += 1
            if self._lost_frames > 10:
                self.show_alert("Line lost!")
            else:
                self.show_warning("No green line detected")
            return None

        self._lost_frames = 0
        cls = int(_MODEL.predict(feat.reshape(1, -1))[0])

        # PID steering
        error             = offset
        self._integral    = float(np.clip(self._integral + error, -1.0, 1.0))
        derivative        = error - self._prev_error
        self._prev_error  = error
        steer = float(np.clip(
            self._Kp * error + self._Ki * self._integral + self._Kd * derivative,
            -1.0, 1.0
        ))

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"steer={steer:+.2f}  class={['LEFT','STRAIGHT','RIGHT'][cls]}  "
                f"near={offset:+.2f}  "
                f"P={self._Kp*error:+.2f}  I={self._Ki*self._integral:+.2f}  D={self._Kd*derivative:+.2f}  frame={self._frame_count}"
            )
        self.show_notification(f"steer={steer:+.2f}  [{['LEFT','STRAIGHT','RIGHT'][cls]}]")
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
