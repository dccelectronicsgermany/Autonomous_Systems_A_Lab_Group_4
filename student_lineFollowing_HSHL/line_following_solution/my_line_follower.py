#!/usr/bin/env python3
"""
HSHL Line Following — Green Line Detection (Instructor Approach)

Detects the green line using color thresholding + contour moments,
then returns a continuous steering value in [-1.0, 1.0].
"""
import cv2          # type: ignore
import numpy as np  # type: ignore
import rclpy        # type: ignore

from .interface import LineFollowingInterface


class MyLineFollower(LineFollowingInterface):

    def __init__(self):
        super().__init__("my_line_follower")
        self._frame_count = 0
        self._lost_frames = 0
        self.on_camera_image(self.detect_line)
        self.get_logger().info("MyLineFollower (green line detection) initialized")

    def detect_line(self, image: np.ndarray) -> float | None:
        self._frame_count += 1

        # Bottom half of image only (road area)
        h, w = image.shape[:2]
        roi = image[h // 2:, :]

        # Green color mask (BGR)
        lower_green = np.array([0, 100, 0])
        upper_green = np.array([100, 255, 100])
        mask = cv2.inRange(roi, lower_green, upper_green)

        # Noise reduction
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self._lost_frames += 1
            if self._lost_frames > 10:
                self.show_alert("Line lost!")
            else:
                self.show_warning("No line detected")
            return None

        largest_contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest_contour) < 100:
            self._lost_frames += 1
            self.show_warning("Contour too small")
            return None

        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None

        line_center_x = M["m10"] / M["m00"]

        # Continuous steering: offset from image center, scaled to [-1, 1]
        image_center_x = w / 2.0
        offset = (line_center_x - image_center_x) / image_center_x
        steering = float(np.clip(offset, -1.0, 1.0))

        self._lost_frames = 0

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"steer={steering:+.2f}  line_x={line_center_x:.0f}  frame={self._frame_count}"
            )
        self.show_notification(f"steer={steering:+.2f}")

        return steering


def main(args=None):
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
