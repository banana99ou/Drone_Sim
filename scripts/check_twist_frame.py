#!/usr/bin/env python3
"""Measure whether Gazebo reports odometry twist in the body or world frame.

Why this exists: the controller has a parameter `odom.twist_in_body_frame`.
Getting it wrong is INVISIBLE at hover and while flying level, because the two
frames coincide when attitude is identity. It only corrupts control during
banked flight -- which is exactly when a planner's trajectory is being tested.
An assumption that only breaks under the conditions you care about is worth
measuring rather than believing.

Method: differentiate the ground-truth POSITION to get world velocity (which is
unambiguous), then compare it against the reported twist interpreted both ways.
Whichever interpretation matches is the convention in use. Only samples with
meaningful tilt and speed are counted, since level samples cannot discriminate.

Run it with the sim flying a banked trajectory:
    ros2 launch dsim_bringup sim.launch.py reference:=circle
    python3 scripts/check_twist_frame.py
"""
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

MIN_TILT_RAD = 0.10      # ~6 deg; below this the two frames are indistinguishable
MIN_SPEED = 0.5          # m/s
NEEDED_SAMPLES = 200


def quat_to_matrix(x, y, z, w):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ]


class TwistFrameChecker(Node):
    def __init__(self):
        super().__init__('twist_frame_checker')
        self.prev = None
        self.err_body = 0.0     # error assuming twist is in the BODY frame
        self.err_world = 0.0    # error assuming twist is in the WORLD frame
        self.n = 0
        self.skipped = 0
        self.create_subscription(
            Odometry, '/drone/truth', self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'measuring... need {NEEDED_SAMPLES} samples with tilt > '
            f'{math.degrees(MIN_TILT_RAD):.0f} deg and speed > {MIN_SPEED} m/s')

    def cb(self, msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        v = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)

        if self.prev is not None:
            t0, p0 = self.prev
            dt = t - t0
            if 1e-4 < dt < 0.1:
                v_world_fd = [(p[i] - p0[i]) / dt for i in range(3)]
                speed = math.sqrt(sum(c * c for c in v_world_fd))

                R = quat_to_matrix(q.x, q.y, q.z, q.w)
                # body z axis vs world z axis -> tilt
                tilt = math.acos(max(-1.0, min(1.0, R[2][2])))

                if speed > MIN_SPEED and tilt > MIN_TILT_RAD:
                    v_if_body = [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]
                    self.err_body += math.dist(v_if_body, v_world_fd)
                    self.err_world += math.dist(list(v), v_world_fd)
                    self.n += 1
                else:
                    self.skipped += 1
        self.prev = (t, p)

        if self.n >= NEEDED_SAMPLES:
            self.report()
            raise SystemExit(0)

    def report(self):
        mb = self.err_body / self.n
        mw = self.err_world / self.n
        print()
        print(f'samples used: {self.n}   (skipped {self.skipped} level/slow samples)')
        print(f'  mean error if twist is BODY  frame: {mb:.4f} m/s')
        print(f'  mean error if twist is WORLD frame: {mw:.4f} m/s')
        print()
        if min(mb, mw) > 0.25:
            print('INCONCLUSIVE: both interpretations disagree with the finite '
                  'differences. Something else is wrong (wrong topic, or the '
                  'stamps are not sim time).')
            return
        ratio = max(mb, mw) / max(min(mb, mw), 1e-9)
        if ratio < 2.0:
            print('INCONCLUSIVE: the two are too close to separate. Fly a more '
                  'aggressively banked trajectory (try reference:=lemniscate).')
            return
        if mb < mw:
            print('RESULT: twist is in the BODY frame.')
            print('  -> set odom.twist_in_body_frame: true   (this is the default)')
        else:
            print('RESULT: twist is in the WORLD frame.')
            print('  -> set odom.twist_in_body_frame: false  in '
                  'src/dsim_bringup/config/gains.yaml')


def main():
    rclpy.init()
    node = TwistFrameChecker()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if node.n and node.n < NEEDED_SAMPLES:
            node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
