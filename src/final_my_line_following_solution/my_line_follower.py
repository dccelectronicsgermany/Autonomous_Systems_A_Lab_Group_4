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

_MODEL = pickle.load(open(str(Path(__file__).parent / "Trapezoid_V2_sharp_turns.pkl"), "rb"))

_LOWER_GREEN = np.array([45,  60,  60])
_UPPER_GREEN = np.array([85, 255, 255])
_CAR_CENTER_X = 693   # px — empirically measured from frame 4802

_TRAP_BOT_Y, _TRAP_TOP_Y     = 0.85, 0.45
_TRAP_BOT_X_L, _TRAP_BOT_X_R = 0.328, 0.789
_TRAP_TOP_X_L, _TRAP_TOP_X_R = 0.403, 0.697


class MyLineFollower(LineFollowingInterface):
    """
    Student implementation of line following.

    Detect a green line and steer to stay centered on it.
    """

    # PD gains per throttle band: (throttle_max, Kp, Kd)
    # Tuned so higher throttle gets more aggressive Kp and more damping Kd.
    # Throttle is set via DEFAULT_THROTTLE env var (default 0.35).
    # Gains selected based on classical OpenCV controller performance at throttle=0.35.
    # Higher speeds reduce Kp to prevent overshoot, increase Kd to suppress oscillation.
    _GAIN_TABLE = [
        (0.40, 1.0,  0.10),   # throttle ≤ 0.40 — default, tested at 0.35
        (0.55, 0.90, 0.15),   # throttle ≤ 0.55 — untested, conservative
        (0.70, 0.80, 0.20),   # throttle ≤ 0.70 — untested, conservative
        (1.00, 0.70, 0.25),   # throttle > 0.70 — untested, conservative
    ]

    def __init__(self):
        super().__init__("my_line_follower")
        self._frame_count = 0
        self._lost_frames  = 0
        self._prev_error   = 0.0
        self._last_steer   = 0.0

        # Select gains based on configured throttle
        self._Kp, self._Kd = self._select_gains(self.default_throttle)

        self.on_camera_image(self.detect_line)
        self.get_logger().info(
            f"MyLineFollower initialized — throttle={self.default_throttle:.2f} "
            f"Kp={self._Kp} Kd={self._Kd} — "
            "6-feature SVR [off_bot,off_mid,off_top,has_bot,has_mid,has_top]"
        )

    @classmethod
    def _select_gains(cls, throttle: float):
        for t_max, kp, kd in cls._GAIN_TABLE:
            if throttle <= t_max:
                return kp, kd
        return cls._GAIN_TABLE[-1][1], cls._GAIN_TABLE[-1][2]

    def detect_line(self, image: np.ndarray) -> float | None:
        """
        Detect the green line and return steering command.

        Args:
            image: BGR image from camera, shape (720, 1280, 3)

        Returns:
            Steering value in [-1.0, 1.0], or None if line not detected.
        """
        self._frame_count += 1
        H, W = image.shape[:2]
        half = W / 2.0

        # ── Trapezoid mask ──────────────────────────────────────────────
        trap_pts = np.array([
            [int(_TRAP_TOP_X_L * W), int(_TRAP_TOP_Y * H)],
            [int(_TRAP_TOP_X_R * W), int(_TRAP_TOP_Y * H)],
            [int(_TRAP_BOT_X_R * W), int(_TRAP_BOT_Y * H)],
            [int(_TRAP_BOT_X_L * W), int(_TRAP_BOT_Y * H)],
        ], dtype=np.int32)
        tm = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(tm, [trap_pts], 255)

        # ── Green mask ──────────────────────────────────────────────────
        hsv    = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        green  = cv2.inRange(hsv, _LOWER_GREEN, _UPPER_GREEN)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        green  = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)
        green  = cv2.morphologyEx(green, cv2.MORPH_OPEN,  kernel)
        green  = cv2.bitwise_and(green, tm)

        y_top = trap_pts[:, 1].min()
        y_bot = trap_pts[:, 1].max()
        gmask = green[y_top:y_bot, :]
        rr    = gmask.shape[0]

        ys, xs = np.where(gmask > 0)
        if len(ys) < 20:
            self._lost_frames += 1
            if self._lost_frames > 10:
                self.show_alert("Line lost!")
                return None
            else:
                self.show_warning(f"No line — holding last steer {self._last_steer:+.3f}")
                return self._last_steer

        # Reject compact blobs (foliage, noise) — real lines span many rows even on 90° turns
        row_span = int(ys.max() - ys.min())
        if row_span < int(rr * 0.15):
            self._lost_frames += 1
            self.show_warning(f"Blob rejected (row_span={row_span}/{rr}) — holding last steer")
            return self._last_steer

        self._lost_frames = 0

        # ── Band sampling ────────────────────────────────────────────────
        def _band_adaptive(s_start, s_end, slice_half=0.07):
            step = -1 if s_start > s_end else 1
            for row in range(int(rr * s_start), int(rr * s_end), step):
                lo, hi = max(0, row - int(rr * slice_half)), min(rr, row + int(rr * slice_half))
                idx = (ys >= lo) & (ys < hi)
                if idx.sum() >= 5:
                    return float(xs[idx].mean())
            return None

        def _band_fixed(frac, slice_half=0.07):
            lo = int(rr * max(0.0, frac - slice_half))
            hi = int(rr * min(1.0, frac + slice_half))
            if hi <= lo: hi = lo + 1
            idx = (ys >= lo) & (ys < hi)
            return float(xs[idx].mean()) if idx.sum() >= 3 else None

        cx_bot = _band_adaptive(0.99, 0.60)
        cx_mid = _band_fixed(0.35)
        cx_top = _band_adaptive(0.01, 0.35)

        off_bot = float(np.clip((cx_bot - _CAR_CENTER_X) / half, -1.0, 1.0)) if cx_bot is not None else None
        off_mid = float(np.clip((cx_mid - _CAR_CENTER_X) / half, -1.0, 1.0)) if cx_mid is not None else None
        off_top = float(np.clip((cx_top - _CAR_CENTER_X) / half, -1.0, 1.0)) if cx_top is not None else None

        has_bot = off_bot is not None
        has_mid = off_mid is not None
        has_top = off_top is not None

        if not (has_bot or has_mid or has_top):
            self._lost_frames += 1
            self.show_warning(f"Bands empty — holding last steer {self._last_steer:+.3f}")
            return self._last_steer

        # ── 6-feature vector ─────────────────────────────────────────────
        feat = np.array([
            off_bot if has_bot else 0.0,
            off_mid if has_mid else 0.0,
            off_top if has_top else 0.0,
            1.0 if has_bot else 0.0,
            1.0 if has_mid else 0.0,
            1.0 if has_top else 0.0,
        ], dtype=np.float32)

        svr_offset = float(np.clip(_MODEL.predict(feat.reshape(1, -1))[0], -1.0, 1.0))

        # ── PD controller ────────────────────────────────────────────────
        # Kp and Kd are selected at startup from _GAIN_TABLE based on throttle.
        error            = svr_offset
        derivative       = error - self._prev_error
        self._prev_error = error
        steer = float(np.clip(self._Kp * error + self._Kd * derivative, -1.0, 1.0))

        # ── Log every 30 frames (~1 second at 30fps) ─────────────────────
        if self._frame_count % 30 == 0:
            direction = "LEFT " if steer < -0.05 else "RIGHT" if steer > 0.05 else "STRAIGHT"
            self.get_logger().info(
                f"frame={self._frame_count:6d} | "
                f"svr={svr_offset:+.3f} | "
                f"steer={steer:+.3f} | "
                f"{direction} | "
                f"bot={feat[0]:+.3f} mid={feat[1]:+.3f} top={feat[2]:+.3f}"
            )

        self._last_steer = steer
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
