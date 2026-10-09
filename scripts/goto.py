#!/usr/bin/env python3
"""Fly to one or more waypoints with the flight controller's position mode.

    ros2 run my_drone goto.py 0,0,2 3,3,2 -2,1,1,90 0,0,0.3

Each waypoint is x,y,z[,yaw_deg] in the odom frame (metres, z up). Without a
yaw the drone keeps its heading. The script arms the drone, sends each goal
on /goal_pose, waits until it is within ARRIVE_POS and slower than
ARRIVE_VEL (and within ARRIVE_YAW of the yaw, if given), and prints how long that took and how far it overshot. It leaves
the drone hovering at the last waypoint (arrow_teleop can take over and land).
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool

ARRIVE_POS = 0.15  # m
ARRIVE_VEL = 0.2   # m/s
ARRIVE_YAW = 5.0   # deg, when the waypoint gives a yaw
TIMEOUT = 30.0     # s of sim time per waypoint


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def parse(arg):
    v = [float(s) for s in arg.split(",")]
    if len(v) not in (3, 4):
        raise ValueError(f"waypoint '{arg}' must be x,y,z or x,y,z,yaw_deg")
    return v


class Goto(Node):
    def __init__(self):
        super().__init__(
            "goto", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.enable_pub = self.create_publisher(Bool, "/enable", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.odom = None

    def on_odom(self, msg):
        self.odom = msg

    def wait_for_odom(self):
        while rclpy.ok() and self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def fly(self, wp):
        p0 = self.odom.pose.pose.position
        start = (p0.x, p0.y, p0.z)
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = wp[:3]
        if len(wp) == 4:
            yaw = math.radians(wp[3])
            goal.pose.orientation.z = math.sin(yaw / 2)
            goal.pose.orientation.w = math.cos(yaw / 2)
        else:
            goal.pose.orientation = self.odom.pose.pose.orientation

        d = [g - s for g, s in zip(wp[:3], start)]
        dist = math.sqrt(sum(x * x for x in d))
        u = [x / dist for x in d] if dist > 1e-6 else [0.0, 0.0, 0.0]
        overshoot = 0.0
        t0 = last_send = self.now()
        self.goal_pub.publish(goal)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            t = self.now()
            if t - last_send > 0.5:  # resend: the first may beat discovery
                self.enable_pub.publish(Bool(data=True))
                self.goal_pub.publish(goal)
                last_send = t
            p = self.odom.pose.pose.position
            v = self.odom.twist.twist.linear
            err = [p.x - wp[0], p.y - wp[1], p.z - wp[2]]
            overshoot = max(overshoot, sum(e * k for e, k in zip(err, u)))
            yaw_err = 0.0
            if len(wp) == 4:
                yaw_err = abs((yaw_deg(self.odom.pose.pose.orientation) - wp[3] + 180) % 360 - 180)
            if (math.sqrt(sum(e * e for e in err)) < ARRIVE_POS
                    and math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2) < ARRIVE_VEL
                    and yaw_err < ARRIVE_YAW):
                print(f"  reached {wp} in {t - t0:4.1f} s, overshoot {overshoot:.2f} m")
                return True
            if t - t0 > TIMEOUT:
                print(f"  TIMEOUT on {wp}: at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), "
                      f"yaw {yaw_deg(self.odom.pose.pose.orientation):.0f}")
                return False
        return False


def main():
    try:
        waypoints = [parse(a) for a in remove_ros_args(sys.argv)[1:]]
    except ValueError as e:
        sys.exit(str(e))
    if not waypoints:
        sys.exit(__doc__)

    rclpy.init()
    node = Goto()
    try:
        node.wait_for_odom()
        node.enable_pub.publish(Bool(data=True))
        ok = all(node.fly(wp) for wp in waypoints)
    except KeyboardInterrupt:
        ok = False
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
