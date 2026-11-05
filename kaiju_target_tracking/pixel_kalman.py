"""Basic constant-velocity Kalman tracker for a camera horizontal pixel coordinate.

IMPORTANT: State is (horizontal pixel x, horizontal velocity in pixels/s).
It is *not* an arena position (m) or full camera+ToF sensor fusion.
This module has no ROS dependency and can be unit-tested offline.
"""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class TargetEstimate:
    x_px: float
    vx_px_s: float
    seconds_since_detection: float


class PixelKalman:
    def __init__(self, width_px=640.0, max_prediction_s=0.35,
                 reset_after_s=1.0, measurement_variance=400.0,
                 acceleration_variance=22500.0):
        self.width_px = float(width_px)
        self.max_prediction_s = max_prediction_s
        self.reset_after_s = reset_after_s
        self.r = measurement_variance
        self.q = acceleration_variance
        self.reset()

    def reset(self):
        self.initialized = False
        self.x = 0.0
        self.v = 0.0
        self.p00 = 400.0
        self.p01 = 0.0
        self.p11 = 1600.0
        self.state_time = None
        self.last_detection_time = None

    def _initialize(self, measurement_px, timestamp_s):
        self.reset()
        self.initialized = True
        self.x = float(measurement_px)
        self.state_time = timestamp_s
        self.last_detection_time = timestamp_s

    def _predict_to(self, timestamp_s):
        dt = timestamp_s - self.state_time
        if dt < 0:
            raise ValueError('timestamps must be nondecreasing')
        if dt == 0:
            return
        self.x += dt * self.v
        p00, p01, p11 = self.p00, self.p01, self.p11
        # F = [[1, dt], [0, 1]]
        # Q assumes random acceleration with variance q.
        self.p00 = p00 + 2 * dt * p01 + dt * dt * p11 + self.q * dt**4 / 4
        self.p01 = p01 + dt * p11 + self.q * dt**3 / 2
        self.p11 = p11 + self.q * dt * dt
        self.state_time = timestamp_s

    def observe(self, camera_x_px, timestamp_s):
        """Return True if a valid new detection was accepted."""
        if not isfinite(camera_x_px) or not (0 <= camera_x_px <= self.width_px):
            return False
        if not isfinite(timestamp_s):
            return False
        if (not self.initialized or
                timestamp_s - self.last_detection_time > self.reset_after_s):
            self._initialize(camera_x_px, timestamp_s)
            return True
        if timestamp_s < self.state_time:
            return False
        self._predict_to(timestamp_s)
        innovation = float(camera_x_px) - self.x
        k0 = self.p00 / (self.p00 + self.r)
        k1 = self.p01 / (self.p00 + self.r)
        old_p00, old_p01, old_p11 = self.p00, self.p01, self.p11
        self.x += k0 * innovation
        self.v += k1 * innovation
        self.p00 = (1.0 - k0) * old_p00
        self.p01 = (1.0 - k0) * old_p01
        self.p11 = old_p11 - k1 * old_p01
        self.last_detection_time = timestamp_s
        return True

    def estimate(self, timestamp_s):
        """Predict briefly across missed frames, then return None (target lost).

        This intentionally does not drive or stop the Zumo motors; the fight/search
        strategy remains the responsibility of the control node.
        """
        if not self.initialized or not isfinite(timestamp_s):
            return None
        age = timestamp_s - self.last_detection_time
        if age < 0 or age > self.max_prediction_s or timestamp_s < self.state_time:
            return None
        self._predict_to(timestamp_s)
        return TargetEstimate(
            x_px=max(0.0, min(self.width_px, self.x)),
            vx_px_s=self.v,
            seconds_since_detection=age,
        )
