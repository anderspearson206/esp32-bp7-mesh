"""
Dead reckoning from two odometry sources: distance from the wheels, heading from the T265.

Shared by radio_daemon.py (python3, --ros-wheel-topic) and rover_nav.py (the ROS node on the
car, python2 under ROS Melodic), so keep it python2-compatible: no f-strings, no annotations.
Copy it next to whichever of those runs on the car.
"""

import math
import threading

GAP_S = 10.0          # a source silent this long restarted: its next message only sets a baseline
HEAD_STALE_S = 0.5    # T265 (~200 Hz) silent this long -> turn by the wheel odometry's yaw
MAX_STEP_M = 2.0      # wheel step longer than this in one message = wheel odometry restarted


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class OdomFusion(object):
    """The T265 keeps a good heading (gyro) but under-reports distance badly outdoors when its
    cameras lose tracking (sun/shadow, plain pavement, vibration). Wheel odometry has the
    opposite problem: its distance is good but its heading comes from the steering command.
    Each wheel odometry message moves the fused pose by its signed step along the fused
    heading. Only heading *changes* are taken from each source, so a restart of either
    (pose back to 0) does not move the fused pose: after a silence (wheels: gap_s, T265:
    head_stale_s) the next message only sets a new baseline. A T265 relocalisation (pose
    jump without a gap) does turn the heading. While the T265 is silent for head_stale_s
    the heading follows the wheel odometry's yaw instead, so no turn is counted twice.
    """

    def __init__(self, gap_s=GAP_S, head_stale_s=HEAD_STALE_S, max_step_m=MAX_STEP_M):
        self.gap_s, self.head_stale_s, self.max_step_m = gap_s, head_stale_s, max_step_m
        self.x = self.y = self.z = 0.0
        self.heading = None           # degrees, counter-clockwise from +x
        self.distance = 0.0           # total wheel distance integrated [m]
        self.resets = 0               # wheel steps dropped as odometry restarts
        self._head_last = None        # (yaw, t) of the last heading message
        self._wheel_last = None       # (x, y, yaw, t) of the last wheel message
        self._lock = threading.Lock()

    def on_heading(self, yaw, z, now):
        if yaw is None:
            return
        with self._lock:
            last = self._head_last
            if self.heading is None:
                self.heading = yaw
            elif last is not None and now - last[1] <= self.head_stale_s:
                self.heading = wrap_deg(self.heading + wrap_deg(yaw - last[0]))
            self._head_last = (yaw, now)
            self.z = z

    def on_wheel(self, x, y, yaw, now):
        with self._lock:
            last, self._wheel_last = self._wheel_last, (x, y, yaw, now)
            if last is None or now - last[3] > self.gap_s:
                return
            dx, dy = x - last[0], y - last[1]
            if math.hypot(dx, dy) > self.max_step_m:
                self.resets += 1
                return
            head_fresh = self._head_last is not None and now - self._head_last[1] <= self.head_stale_s
            if yaw is not None and last[2] is not None:
                if not head_fresh:
                    self.heading = (yaw if self.heading is None
                                    else wrap_deg(self.heading + wrap_deg(yaw - last[2])))
                # signed step along the wheel odometry's own heading (negative when reversing)
                wy = math.radians(last[2] + wrap_deg(yaw - last[2]) / 2.0)
                ds = dx * math.cos(wy) + dy * math.sin(wy)
            else:
                ds = math.hypot(dx, dy)
            if self.heading is None:
                return
            h = math.radians(self.heading)
            self.x += ds * math.cos(h)
            self.y += ds * math.sin(h)
            self.distance += abs(ds)

    def source(self, now):
        """'fused' (T265 heading), 'wheel' (T265 silent, wheel heading) or 'wait' (no data yet)."""
        with self._lock:
            if self._wheel_last is None or self.heading is None:
                return 'wait'
            if self._head_last is not None and now - self._head_last[1] <= self.head_stale_s:
                return 'fused'
            return 'wheel'

    def wheel_age(self, now):
        with self._lock:
            return None if self._wheel_last is None else now - self._wheel_last[3]

    def location(self):
        with self._lock:
            return {'lat': self.x, 'lon': self.y, 'alt': self.z, 'yaw': self.heading}
