#!/usr/bin/env python
"""
calib_distance.py - baseline odometry calibration on the rover, run by hand over SSH.

Drive the car along a measured straight line; the script compares the wheel odometry
(/car/vesc/odom) with the true distance and suggests a new speed_to_erpm_gain for vesc.yaml.
It also uses the T265 (/camera/odom/sample) heading to tell whether the car curves with the
steering at zero, and suggests a steering_angle_to_servo_offset.

Usage (stack started from the dashboard with Start ROS, controller connected):
    python calib_distance.py --true 6.6
Per run: put a reference point of the car (e.g. the front axle) on the start mark, press
Enter, drive slowly with LB + left stick, stop at the end mark, press Enter, and type the distance actually driven (Enter keeps --true). 'q' ends and prints
the averaged suggestion.

Controller mode, for when SSH is over Ethernet and the cable has to come off:
    nohup python -u calib_distance.py --true 6.6 --joy > ~/calib.log 2>&1 &
then unplug. Cross (X) starts a run and ends it; every run counts as --true, so stop on
the end mark. Square cancels the run in progress, or between runs drops the last one.
Triangle finishes and writes the summary; plug back in and read ~/calib.log. (It also tries
to rumble the controller on each press, but the cars' controller has no rumble.)

Steering with the right stick to stay on the line is fine for the distance (it uses the path
length). Only runs where the right stick stayed centred count for the steering estimate; the
script watches the stick and leaves the others out of it.

Nothing is written to vesc.yaml; apply the values yourself and restart the stack.

Python 2 (ROS Melodic rospy) and 3.
"""
from __future__ import division, print_function

import argparse
import math
import os
import re
import subprocess
import sys
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

try:
    input = raw_input   # Python 2
except NameError:
    pass

VESC_YAML = os.path.expanduser(
    '~/catkin_ws/src/mushr/mushr_base/vesc/vesc_main/config/racecar-uw-nano/vesc.yaml')
WHEELBASE = 0.225   # m, racecar-uw-nano
BEEP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'beep.py')
# /car/teleop/joy indices on the cars' controller: 0 square, 1 cross, 2 circle, 3 triangle
# (none used by joy_teleop).
BTN_RUN, BTN_FINISH, BTN_DROP = 1, 3, 0     # cross, triangle, square
STEER_AXIS = 2          # right stick, joy_teleop.yaml
STEER_TOUCHED = 0.05    # |axis| above this during a run = steered (x0.34 rad -> ~1 deg)


def log(msg):
    print(msg)
    sys.stdout.flush()


def beep(n, ms=None):
    """Rumble the controller n times (mushr_base/scripts/beep.py), without waiting."""
    try:
        with open(os.devnull, 'w') as null:   # the cars' controller has no rumble; keep the log clean
            subprocess.Popen(['python3', BEEP, str(n)] + ([str(ms)] if ms else []),
                             stdout=null, stderr=null)
    except OSError:
        pass


class Buttons(object):
    """Rising edges of controller buttons from the joy_teleop topic, as a queue, and the
    largest steering-stick deflection since reset_steer()."""

    def __init__(self, topic):
        self.cv = threading.Condition()
        self.prev = []
        self.events = []
        self.steer_max = 0.0
        self.msgs = 0
        rospy.Subscriber(topic, Joy, self._cb, queue_size=20)

    def reset_steer(self):
        with self.cv:
            self.steer_max = 0.0

    def _cb(self, msg):
        with self.cv:
            self.msgs += 1
            if len(msg.axes) > STEER_AXIS:
                self.steer_max = max(self.steer_max, abs(msg.axes[STEER_AXIS]))
            for b in (BTN_RUN, BTN_FINISH, BTN_DROP):
                now = len(msg.buttons) > b and msg.buttons[b]
                was = len(self.prev) > b and self.prev[b]
                if now and not was:
                    self.events.append(b)
                    self.cv.notify()
            self.prev = list(msg.buttons)

    def next(self):
        with self.cv:
            while not self.events and not rospy.is_shutdown():
                self.cv.wait(0.5)
            return self.events.pop(0) if self.events else BTN_FINISH


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaml_value(path, key):
    try:
        with open(path) as f:
            m = re.search(r'^%s:\s*([-0-9.eE+]+)' % key, f.read(), re.M)
        return float(m.group(1)) if m else None
    except IOError:
        return None


class Track(object):
    """Path length, end-to-end displacement and heading change of one odometry topic."""

    def __init__(self, topic):
        self.topic = topic
        self.lock = threading.Lock()
        self.msgs = 0
        self.last = None        # (x, y, yaw)
        self.recording = False
        rospy.Subscriber(topic, Odometry, self._cb, queue_size=50)

    def _cb(self, msg):
        p = msg.pose.pose
        cur = (p.position.x, p.position.y, yaw(p.orientation))
        t = rospy.get_time()
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        with self.lock:
            self.msgs += 1
            if self.recording and self.last is not None:
                self.path += math.hypot(cur[0] - self.last[0], cur[1] - self.last[1])
                self.dyaw += wrap(cur[2] - self.last[2])
                if v > 0.05 and t - self.last_t < 0.5:
                    self.moving += t - self.last_t
                self.vmax = max(self.vmax, v)
            self.last = cur
            self.last_t = t

    def start(self):
        with self.lock:
            self.start_pose = self.last
            self.path = 0.0
            self.dyaw = 0.0
            self.moving = 0.0       # s with speed > 0.05 m/s
            self.vmax = 0.0
            self.recording = True

    def stop(self):
        with self.lock:
            self.recording = False
            if self.start_pose is None or self.last is None:
                return None
            disp = math.hypot(self.last[0] - self.start_pose[0], self.last[1] - self.start_pose[1])
            avg = self.path / self.moving if self.moving > 0 else 0.0
            return {'path': self.path, 'disp': disp, 'dyaw': self.dyaw, 'avg': avg, 'vmax': self.vmax}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--true', type=float, required=True, help='measured length of the line (m)')
    ap.add_argument('--wheel-topic', default='/car/vesc/odom')
    ap.add_argument('--t265-topic', default='/camera/odom/sample')
    ap.add_argument('--yaml', default=VESC_YAML)
    ap.add_argument('--joy', action='store_true', help='controller buttons instead of the keyboard')
    ap.add_argument('--joy-topic', default='/car/teleop/joy')
    args = ap.parse_args(rospy.myargv()[1:])

    gain = yaml_value(args.yaml, 'speed_to_erpm_gain')
    s_gain = yaml_value(args.yaml, 'steering_angle_to_servo_gain')
    s_off = yaml_value(args.yaml, 'steering_angle_to_servo_offset')
    print('vesc.yaml: speed_to_erpm_gain=%s steering_angle_to_servo_gain=%s '
          'steering_angle_to_servo_offset=%s' % (gain, s_gain, s_off))

    rospy.init_node('calib_distance', anonymous=True, disable_signals=True)
    wheel = Track(args.wheel_topic)
    t265 = Track(args.t265_topic)
    buttons = Buttons(args.joy_topic)
    rospy.sleep(1.5)
    if wheel.msgs == 0:
        print('No messages on %s. Is the stack running (Start ROS) and the motor battery in?'
              % args.wheel_topic)
        if args.joy:
            beep(3, 500)
        return
    if t265.msgs == 0:
        print('No messages on %s: distance only, no steering check.' % args.t265_topic)
    if buttons.msgs == 0:
        print('No messages on %s: is the controller connected?' % args.joy_topic)

    ratios, deltas = [], []

    def start():
        wheel.start()
        t265.start()
        buttons.reset_steer()

    def record(w, c, true):
        if not w or w['path'] < 0.5:
            log('  wheel odometry moved %.2f m; run ignored.' % (w['path'] if w else 0.0))
            return False
        ratio = w['path'] / true
        ratios.append(ratio)
        log('  wheel odom: path %.3f m, straight %.3f m -> %.1f%% of true %.3f m (%+.3f m)'
            % (w['path'], w['disp'], 100.0 * ratio, true, w['path'] - true))
        log('  speed (wheel odom): avg %.2f m/s while moving, max %.2f m/s' % (w['avg'], w['vmax']))
        steered = buttons.steer_max > STEER_TOUCHED
        if c and c['path'] > 0.1:
            log('  T265: path %.3f m, heading change %+.1f deg (%s)'
                % (c['path'], math.degrees(c['dyaw']), 'left' if c['dyaw'] > 0 else 'right'))
        if steered:
            log('  steered (right stick up to %.0f%%): not used for the steering estimate'
                % (100 * buttons.steer_max))
            deltas.append(None)
        elif c and c['path'] > 0.1:
            # Curvature with zero steering -> the steering angle the car actually had.
            delta = math.atan(WHEELBASE * c['dyaw'] / true)
            deltas.append(delta)
            log('  no steering input: implied steering at zero command %+.2f deg' % math.degrees(delta))
        else:
            deltas.append(None)
        return True

    if args.joy:
        log('Controller mode: X = start/stop run, square = cancel run / drop last run, triangle = finish.')
        beep(1, 500)
        run = 0
        while True:
            b = buttons.next()
            if b == BTN_FINISH:
                break
            if b == BTN_DROP:
                if ratios:
                    ratios.pop()
                    deltas.pop()
                    log('Dropped the last run (%d left).' % len(ratios))
                beep(3)
                continue
            run += 1
            start()
            log('\nRun %d: recording' % run)
            beep(1)
            b = buttons.next()
            w, c = wheel.stop(), t265.stop()
            if b == BTN_DROP:
                log('  run cancelled')
                beep(3)
                continue
            beep(2)
            record(w, c, args.true)
            if b == BTN_FINISH:
                break
        beep(1, 500)
    else:
        run = 0
        while not rospy.is_shutdown():
            run += 1
            if input('\nRun %d: car on the start mark, Enter to start (q to finish): ' % run).strip() == 'q':
                break
            start()
            input('Recording. Drive to the end mark, stop, then Enter: ')
            w, c = wheel.stop(), t265.stop()
            ans = input('Distance actually driven [%.3f m]: ' % args.true).strip()
            record(w, c, float(ans) if ans else args.true)
    deltas = [d for d in deltas if d is not None]

    if not ratios:
        log('No runs.')
        return
    r = sum(ratios) / len(ratios)
    print('\n== %d run(s): wheel odometry reads %.1f%% of true (runs: %s)'
          % (len(ratios), 100.0 * r, ', '.join('%.1f' % (100 * x) for x in ratios)))
    if gain is not None:
        print('speed_to_erpm_gain: %.0f -> %.0f' % (gain, gain * r))
    if not deltas:
        print('No runs without steering input: no steering-centre estimate.')
    elif s_gain is not None and s_off is not None:
        d = sum(deltas) / len(deltas)
        print('Steering estimate from %d run(s) without steering input.' % len(deltas))
        # servo = gain * angle + offset; at zero command the wheels sat at angle d,
        # so the true centre is offset - gain * d (positive angle = left, ROS convention).
        print('Mean implied steering at zero command: %+.2f deg' % math.degrees(d))
        print('steering_angle_to_servo_offset: %.3f -> %.3f (check: the car should then roll straight)'
              % (s_off, s_off - s_gain * d))
    print('Apply in %s, then Stop ROS / Start ROS from the dashboard.' % args.yaml)


if __name__ == '__main__':
    main()
