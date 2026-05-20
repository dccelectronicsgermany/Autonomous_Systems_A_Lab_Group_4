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

_MODEL = pickle.load(open(str(Path(__file__).parent / "Team_4.pkl"), "rb"))


class MyLineFollower(LineFollowingInterface):
    """
    Student implementation of line following.

    Detect a green line and steer to stay centered on it.
    """

    def __init__(self):
        super().__init__("my_line_follower")
        self._frame_count = 0

        # Register camera callback
        self.on_camera_image(self.detect_line)
        self.get_logger().info("MyLineFollower initialized — ready to detect green line")

    def detect_line(self, image: np.ndarray) -> float | None:
        """
        Detect the green line and return steering command.

        Args:
            image: BGR image from camera, shape (720, 1280, 3)

        Returns:
            Steering value in [-1.0, 1.0], or None if line not detected.
        """
        self._frame_count += 1

        # Resize and extract ROI
        img = cv2.resize(image, (320, 180))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        y0 = int(180 * 0.45)
        roi = gray[y0:, :]

        # Edge detection
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Find dominant line segment
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=25, minLineLength=25, maxLineGap=15)
        if lines is None:
            self.show_warning("No line segment detected")
            return None

        best, best_len, best_angle = None, 0, 0.0
        for x1, y1, x2, y2 in lines[:, 0]:
            length = np.hypot(x2 - x1, y2 - y1)
            if length > best_len:
                best_len = length
                best = ((x1 + x2) / 2, (y1 + y2) / 2)
                best_angle = float(np.degrees(np.arctan2(y2 - y1, x2 - x1)))

        cx, cy = best
        x_norm = float((cx / roi.shape[1]) * 2.0 - 1.0)

        # Crop patch around segment center
        half = 32
        h, w = edges.shape
        patch = cv2.resize(
            edges[max(0, int(cy) - half):min(h, int(cy) + half),
                  max(0, int(cx) - half):min(w, int(cx) + half)],
            (64, 64)
        )
        patch_feat = (patch.astype(np.float32) / 255.0).flatten()
        feat = np.append(patch_feat, [np.float32(x_norm), np.float32(best_angle / 90.0)])

        cls = int(_MODEL.predict(feat.reshape(1, -1))[0])
        steer = float(np.clip(x_norm, -1.0, 1.0))

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"steer={steer:+.2f}  class={['LEFT','STRAIGHT','RIGHT'][cls]}  x_norm={x_norm:+.2f}  frame={self._frame_count}"
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
