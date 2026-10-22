#!/usr/bin/env python
"""
rover_nav.py - ROS node on the rover: fused odometry + following routes sent from the BS.

1. Pose, published as nav_msgs/Odometry on ~fused_topic (/car/fused_odom) at ~pose_rate;
   radio_daemon.py sends that to the BS (--ros-location-topic /car/fused_odom), so the route
   the BS planned and the track it draws are in the same frame the follower steers in.
   Two sources, switched from the BS dashboard (radio_daemon.py writes <nav_dir>/pose.json
   {"mode": "fused"|"camera"}; /tmp, so a reboot goes back to ~pose_mode):
     fused   (default) odom_fusion.py: distance from the wheel odometry, heading from the T265
     camera  the T265's own position and heading (its distance under-reads outdoors)
   The fusion keeps running in both modes. On a switch the output is re-anchored so the
   pose continues where it was (no jump: the map alignment and an active route stay valid).
2. Route following: radio_daemon.py writes routes from the BS dashboard to
   <nav_dir>/route.json ({"cmd", "speed", "pts": [[x, y], ...] in the fused odometry frame}
   or {"cmd", "stop": true}). The node follows the newest one with pure pursuit and
   publishes on the navigation input of the ackermann mux.

Safety: it only drives while RB (~rb_button) is held on the controller and its joystick
messages are fresh, the T265 heading is live (not wheel-only) and the wheel odometry is
fresh. Releasing RB or losing the controller stops it; LB (manual teleop) always overrides
in the mux.

RB not required (dashboard setting; radio_daemon.py writes <nav_dir>/rb.json {"required":
false}, /tmp, so a reboot requires RB again): joy_teleop's idle zero on input/teleop
(priority 5) would block input/navigation, so commands go to ~auto_topic instead, an extra
mux input above teleop (ackermann_cmd_mux/param/mux.yaml: "Autonomy", input/autonomy,
priority 6, timeout 0.2). The controller must still be connected (fresh joystick messages,
else it stops) and holding LB (~lb_button) silences this node, so the mux falls back to
manual teleop within its 0.2 s timeout. It also stops for good at the end of the route, if it ends up more than ~max_xte
from the route, or when the BS sends a stop. A route that was already in route.json when
the node started is never driven (no replay after a stack restart).

Progress goes to <nav_dir>/status.json for radio_daemon.py:
    {"t", "nav", "cmd", "ni", "np", "xte", "loc", "mode"}
    nav: idle | hold_rb | no_joy | manual | no_t265 | no_wheels | following | done |
         stopped_off_route | stopped_bs      (no_joy / manual: RB not required, see above)
    loc: fused | wheel | wait (fused mode), camera | no_t265 (camera mode)
    rb:  1 = RB required, 0 = not

Python 2 (ROS Melodic rospy) and 3. Needs odom_fusion.py next to it.
"""
from __future__ import division, print_function

import json
import math
import os
import sys
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom_fusion import OdomFusion, wrap_deg  # noqa: E402

DENSIFY_M = 0.25        # route resampled to this spacing for progress tracking
SEARCH_AHEAD_M = 6.0    # nearest-point search window ahead of the current progress
STATUS_PERIOD_S = 0.5
CAM_STALE_S = 0.5       # T265 silent this long = no camera pose
MODES = ('fused', 'camera')


def yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def densify(pts, step):
    out = [pts[0]]
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        L = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(L / step)))
        for k in range(1, n + 1):
            out.append((x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n))
    s = [0.0]
    for (x0, y0), (x1, y1) in zip(out[:-1], out[1:]):
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    return out, s


class Route(object):
    def __init__(self, cmd, speed, pts):
        self.cmd = cmd
        self.speed = speed
        self.pts, self.s = densify([tuple(p) for p in pts], DENSIFY_M)
        self.idx = 0
        self.xte = 0.0

    @property
    def length(self):
        return self.s[-1]

    def track(self, x, y):
        """Advance progress to the nearest route point within the search window ahead."""
        best, best_d = self.idx, float('inf')
        i = self.idx
        while i < len(self.pts) and self.s[i] <= self.s[self.idx] + SEARCH_AHEAD_M:
            d = math.hypot(self.pts[i][0] - x, self.pts[i][1] - y)
            if d < best_d:
                best, best_d = i, d
            i += 1
        self.idx, self.xte = best, best_d

    def point_at(self, s):
        if s >= self.s[-1]:
            return self.pts[-1]
        i = self.idx
        while i + 1 < len(self.s) and self.s[i + 1] < s:
            i += 1
        return self.pts[i + 1] if i + 1 < len(self.pts) else self.pts[-1]


class RoverNav(object):
    def __init__(self):
        p = rospy.get_param
        self.loc_topic = p('~loc_topic', '/camera/odom/sample')
        self.wheel_topic = p('~wheel_topic', '/car/vesc/odom')
        self.joy_topic = p('~joy_topic', '/car/teleop/joy')
        self.cmd_topic = p('~cmd_topic', '/car/mux/ackermann_cmd_mux/input/navigation')
        self.auto_topic = p('~auto_topic', '/car/mux/ackermann_cmd_mux/input/autonomy')
        self.fused_topic = p('~fused_topic', '/car/fused_odom')
        self.nav_dir = p('~nav_dir', '/tmp/rover_nav')
        self.rb_button = int(p('~rb_button', 5))
        self.lb_button = int(p('~lb_button', 4))
        self.joy_timeout = float(p('~joy_timeout', 0.3))
        self.wheel_timeout = float(p('~wheel_timeout', 0.5))
        self.wheelbase = float(p('~wheelbase', 0.225))   # racecar-uw-nano vesc.yaml
        self.max_steer = float(p('~max_steer', 0.34))
        self.max_speed = float(p('~max_speed', 1.0))
        self.min_speed = float(p('~min_speed', 0.25))
        self.lookahead = float(p('~lookahead', 1.2))
        self.max_xte = float(p('~max_xte', 2.5))
        self.goal_tol = float(p('~goal_tol', 0.5))
        self.rate = float(p('~rate', 20.0))
        self.pose_rate = float(p('~pose_rate', 50.0))
        mode = p('~pose_mode', 'fused')

        self.fusion = OdomFusion()
        self._cam = None                           # (x, y, z, yaw_deg, t) latest T265 pose
        self.mode = mode if mode in MODES else 'fused'
        self._anchor = (0.0, 0.0, 0.0)             # output = rotate(src, a[0] deg) + (a[1], a[2])
        self._pose_mtime = None
        self.joy_t, self.rb, self.lb = 0.0, False, False
        self.rb_required = True
        self._rb_mtime = None
        self.route = None
        self.state = 'idle'
        self.seen_cmd = self._route_file_cmd()     # never drive a route left over from before
        self._route_mtime = None
        self._last_status = 0.0
        self._zero_until = 0.0
        if not os.path.isdir(self.nav_dir):
            os.makedirs(self.nav_dir)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, AckermannDriveStamped, queue_size=1)
        self.auto_pub = rospy.Publisher(self.auto_topic, AckermannDriveStamped, queue_size=1)
        self.odom_pub = rospy.Publisher(self.fused_topic, Odometry, queue_size=5)
        rospy.Subscriber(self.loc_topic, Odometry, self._on_t265, queue_size=10)
        rospy.Subscriber(self.wheel_topic, Odometry, self._on_wheel, queue_size=10)
        rospy.Subscriber(self.joy_topic, Joy, self._on_joy, queue_size=5)
        self._poll_pose_mode()
        self._poll_rb()
        rospy.loginfo("rover_nav: pose %s (fused = %s distance + %s heading) -> %s; routes from %s; "
                      "drives only while button %d is held", self.mode,
                      self.wheel_topic, self.loc_topic, self.fused_topic, self.nav_dir, self.rb_button)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self._tick)
        rospy.Timer(rospy.Duration(1.0 / self.pose_rate), self._publish_pose)

    # inputs
    def _on_t265(self, msg):
        pp, now = msg.pose.pose, time.time()
        yaw = yaw_deg(pp.orientation)
        self._cam = (pp.position.x, pp.position.y, pp.position.z, yaw, now)
        self.fusion.on_heading(yaw, pp.position.z, now)

    def _on_wheel(self, msg):
        pp = msg.pose.pose
        self.fusion.on_wheel(pp.position.x, pp.position.y, yaw_deg(pp.orientation), time.time())

    # pose source
    def _src_pose(self):
        """(x, y, z, yaw_deg) of the current mode's source, or None."""
        if self.mode == 'camera':
            return self._cam[:4] if self._cam is not None else None
        loc = self.fusion.location()
        return None if loc['yaw'] is None else (loc['lat'], loc['lon'], loc['alt'], loc['yaw'])

    def pose(self):
        """Output pose (x, y, z, yaw_deg) in the frame the BS aligned and routes are in."""
        src = self._src_pose()
        if src is None:
            return None
        a, tx, ty = self._anchor
        c, s_ = math.cos(math.radians(a)), math.sin(math.radians(a))
        return (tx + c * src[0] - s_ * src[1], ty + s_ * src[0] + c * src[1], src[2], wrap_deg(src[3] + a))

    def source(self, now):
        if self.mode == 'camera':
            return 'camera' if self._cam is not None and now - self._cam[4] <= CAM_STALE_S else 'no_t265'
        return self.fusion.source(now)

    def set_mode(self, mode):
        if mode not in MODES or mode == self.mode:
            return
        out = self.pose()
        self.mode = mode
        src = self._src_pose()
        if out is None or src is None:
            self._anchor = (0.0, 0.0, 0.0)
        else:
            a = wrap_deg(out[3] - src[3])
            c, s_ = math.cos(math.radians(a)), math.sin(math.radians(a))
            self._anchor = (a, out[0] - (c * src[0] - s_ * src[1]), out[1] - (s_ * src[0] + c * src[1]))
        self._last_status = 0.0
        rospy.loginfo("rover_nav: pose source -> %s (re-anchored, pose continues)", mode)

    def _poll_pose_mode(self):
        path = os.path.join(self.nav_dir, 'pose.json')
        try:
            mtime = os.path.getmtime(path)
            if mtime == self._pose_mtime:
                return
            self._pose_mtime = mtime
            with open(path) as f:
                self.set_mode(str(json.load(f)['mode']))
        except (IOError, OSError, ValueError, KeyError, TypeError):
            return

    def _poll_rb(self):
        path = os.path.join(self.nav_dir, 'rb.json')
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            mtime = None
        if mtime == self._rb_mtime:
            return
        self._rb_mtime = mtime
        required = True                            # no file (e.g. after a reboot) = RB required
        if mtime is not None:
            try:
                with open(path) as f:
                    required = bool(json.load(f).get('required', True))
            except (IOError, OSError, ValueError, AttributeError):
                required = True
        if required != self.rb_required:
            self.rb_required = required
            self._last_status = 0.0
            rospy.logwarn("rover_nav: RB %s", "required again" if required else
                          "NOT required: driving on %s, hold LB to take over" % self.auto_topic)

    def _publish_pose(self, _event):
        p = self.pose()
        if p is None:
            return
        out = Odometry()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = 'fused_odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = p[0], p[1], p[2]
        h = math.radians(p[3])
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(h / 2), math.cos(h / 2)
        self.odom_pub.publish(out)

    def _on_joy(self, msg):
        self.joy_t = time.time()
        self.rb = len(msg.buttons) > self.rb_button and msg.buttons[self.rb_button] == 1
        self.lb = len(msg.buttons) > self.lb_button and msg.buttons[self.lb_button] == 1

    # route file
    def _route_file_cmd(self):
        try:
            with open(os.path.join(self.nav_dir, 'route.json')) as f:
                return int(json.load(f)['cmd'])
        except (IOError, OSError, ValueError, KeyError, TypeError):
            return 0

    def _poll_route(self):
        path = os.path.join(self.nav_dir, 'route.json')
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return
        if mtime == self._route_mtime:
            return
        self._route_mtime = mtime
        try:
            with open(path) as f:
                r = json.load(f)
            cmd = int(r['cmd'])
        except (IOError, OSError, ValueError, KeyError, TypeError):
            return
        if cmd <= self.seen_cmd:
            return
        self.seen_cmd = cmd
        if r.get('stop'):
            if self.route is not None and self.state not in ('done',) and not self.state.startswith('stopped'):
                self._finish('stopped_bs')
            rospy.loginfo("rover_nav: stop from the BS (cmd %d)", cmd)
            return
        try:
            speed = min(max(float(r['speed']), 0.0), self.max_speed)
            self.route = Route(cmd, speed, r['pts'])
        except (KeyError, TypeError, ValueError, IndexError) as e:
            rospy.logwarn("rover_nav: bad route in %s: %s", path, e)
            return
        self.state = 'hold_rb'
        rospy.loginfo("rover_nav: new route cmd %d: %d points, %.1f m at %.2f m/s - %s",
                      cmd, len(r['pts']), self.route.length, speed,
                      "hold RB to drive" if self.rb_required else "driving (RB not required, LB overrides)")

    def _finish(self, state):
        self.state = state
        self._last_status = 0.0                    # report the final state right away
        self._zero_until = time.time() + 1.0       # keep commanding a stop for a moment
        self._publish(0.0, 0.0)
        rospy.loginfo("rover_nav: route %s -> %s", self.route.cmd if self.route else '-', state)

    # control loop
    def _publish(self, speed, steer):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.drive.speed = speed
        cmd.drive.steering_angle = steer
        if self.rb_required:
            self.cmd_pub.publish(cmd)
        elif not (self.lb and time.time() - self.joy_t <= self.joy_timeout):
            # above teleop in the mux: silent while LB is held, so manual takes over
            self.auto_pub.publish(cmd)

    def _tick(self, _event):
        now = time.time()
        self._poll_pose_mode()
        self._poll_route()
        self._poll_rb()
        joy_ok = now - self.joy_t <= self.joy_timeout
        rb = self.rb and joy_ok
        go = rb if self.rb_required else (joy_ok and not self.lb)
        src = self.source(now)
        wheel_age = self.fusion.wheel_age(now)
        r = self.route
        active = r is not None and self.state not in ('done',) and not self.state.startswith('stopped')
        if not active:
            # hold still while RB is held or just after a route ended; otherwise stay silent so
            # another navigation source (the OPTIONS planner) isn't fought on the same input
            if (self.rb_required and rb) or now < self._zero_until:
                self._publish(0.0, 0.0)
            self._write_status(now, src)
            return
        pose = self.pose()
        x, y, hdg = (pose[0], pose[1], pose[3]) if pose is not None else (0.0, 0.0, None)
        if hdg is not None:
            r.track(x, y)
        # gates: anything wrong -> explicit zero on the navigation input. Doesn't rely on the
        # mux timing out (with RB released teleop's own zero wins anyway; with the controller
        # gone nothing else would be commanding).
        if not go:
            self.state = ('hold_rb' if self.rb_required else 'manual' if joy_ok else 'no_joy')
        elif wheel_age is None or wheel_age > self.wheel_timeout:
            self.state = 'no_wheels'
        elif src not in ('fused', 'camera') or hdg is None:
            self.state = 'no_t265'
        else:
            self.state = 'following'
        if self.state != 'following':
            self._publish(0.0, 0.0)
            self._write_status(now, src)
            return
        # by distance along the route only: a loop ends where it starts
        if r.length - r.s[r.idx] < self.goal_tol:
            self._finish('done')
            self._write_status(now, src)
            return
        if r.xte > self.max_xte:
            rospy.logwarn("rover_nav: %.1f m off the route - stopping", r.xte)
            self._finish('stopped_off_route')
            self._write_status(now, src)
            return
        # pure pursuit to the point one lookahead along the route
        tx, ty = r.point_at(r.s[r.idx] + self.lookahead)
        ld = max(math.hypot(tx - x, ty - y), 0.3)
        alpha = math.radians(wrap_deg(math.degrees(math.atan2(ty - y, tx - x)) - hdg))
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
        steer = max(-self.max_steer, min(self.max_steer, steer))
        speed = r.speed * (0.6 if abs(steer) > 0.2 else 1.0)
        speed = min(speed, max(self.min_speed, 0.5 * (r.length - r.s[r.idx])))
        self._publish(speed, steer)
        self._write_status(now, src)

    def _write_status(self, now, src):
        if now - self._last_status < STATUS_PERIOD_S:
            return
        self._last_status = now
        r = self.route
        st = {'t': now, 'nav': self.state, 'loc': src, 'mode': self.mode,
              'cmd': r.cmd if r else self.seen_cmd, 'rb': 1 if self.rb_required else 0}
        if r is not None:
            st.update(ni=r.s[r.idx], np=r.length, xte=r.xte)
        tmp = os.path.join(self.nav_dir, 'status.json.tmp')
        try:
            with open(tmp, 'w') as f:
                json.dump(st, f)
            getattr(os, 'replace', os.rename)(tmp, os.path.join(self.nav_dir, 'status.json'))
        except (IOError, OSError) as e:
            rospy.logwarn_throttle(30, "rover_nav: could not write status: %s" % e)


if __name__ == '__main__':
    rospy.init_node('rover_nav')
    RoverNav()
    rospy.spin()
