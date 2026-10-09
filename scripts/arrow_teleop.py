#!/usr/bin/env python3
"""Arrow-key teleop for the F450.

Publishes geometry_msgs/Twist on /cmd_vel in the vehicle body frame, which
is what Gazebo's MulticopterVelocityControl consumes:

    linear.x   forward / back velocity   (m/s)
    linear.y   left / right velocity     (m/s)
    linear.z   climb rate                (m/s)
    angular.z  yaw rate                  (rad/s)

Setpoints are held until changed and decay to zero if the terminal goes
quiet, so a dropped keypress leaves the drone hovering rather than
drifting away.
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

BANNER = """
+----------------------------------------------------------+
|                   F450 arrow-key teleop                  |
+----------------------------------------------------------+
|                                                          |
|   Up / Down arrow ...... fly forward / backward          |
|   Left / Right arrow ... strafe left / right             |
|                                                          |
|   w / s ................ climb / descend                 |
|   a / d ................ yaw left / right                |
|                                                          |
|   space ................ stop and hover                  |
|   + / - ................ speed scale up / down           |
|   q ..................... quit (lands the setpoint)      |
|                                                          |
+----------------------------------------------------------+
"""

# key -> (dx, dy, dz, dyaw) as multiples of the current speed scale
BINDINGS = {
    "\x1b[A": (1.0, 0.0, 0.0, 0.0),    # up arrow
    "\x1b[B": (-1.0, 0.0, 0.0, 0.0),   # down arrow
    "\x1b[D": (0.0, 1.0, 0.0, 0.0),    # left arrow
    "\x1b[C": (0.0, -1.0, 0.0, 0.0),   # right arrow
    "w": (0.0, 0.0, 1.0, 0.0),
    "s": (0.0, 0.0, -1.0, 0.0),
    "a": (0.0, 0.0, 0.0, 1.0),
    "d": (0.0, 0.0, 0.0, -1.0),
    " ": (0.0, 0.0, 0.0, 0.0),
}

LINEAR_SPEED = 1.5   # m/s at scale 1.0
VERTICAL_SPEED = 1.0  # m/s at scale 1.0
YAW_SPEED = 1.0      # rad/s at scale 1.0

PUBLISH_HZ = 20.0
# Hold a setpoint this long after the last keypress, then ease back to hover.
HOLD_SECONDS = 0.6


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__("arrow_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd = [0.0, 0.0, 0.0, 0.0]
        self.scale = 1.0
        self.ticks_since_key = 0
        self.hold_ticks = int(HOLD_SECONDS * PUBLISH_HZ)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.tick)

    def apply(self, binding):
        dx, dy, dz, dyaw = binding
        self.cmd = [
            dx * LINEAR_SPEED * self.scale,
            dy * LINEAR_SPEED * self.scale,
            dz * VERTICAL_SPEED * self.scale,
            dyaw * YAW_SPEED * self.scale,
        ]
        self.ticks_since_key = 0

    def tick(self):
        self.ticks_since_key += 1
        if self.ticks_since_key > self.hold_ticks:
            # Ease to hover rather than cutting the setpoint dead.
            self.cmd = [v * 0.8 if abs(v) > 1e-3 else 0.0 for v in self.cmd]

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = self.cmd[0], self.cmd[1], self.cmd[2]
        msg.angular.z = self.cmd[3]
        self.pub.publish(msg)

    def status(self):
        return (
            f"\r vx {self.cmd[0]:+5.2f}  vy {self.cmd[1]:+5.2f}  "
            f"vz {self.cmd[2]:+5.2f}  yaw {self.cmd[3]:+5.2f}  "
            f"[scale {self.scale:.1f}]   "
        )


def read_key(timeout):
    """Return one keypress, decoding the 3-byte escape sequences arrows send."""
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch
    # Arrow keys arrive as ESC [ A..D; grab the rest without blocking.
    if select.select([sys.stdin], [], [], 0.05)[0]:
        return ch + sys.stdin.read(2)
    return ch


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ArrowTeleop()

    print(BANNER)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.02)
            if key is not None:
                if key in BINDINGS:
                    node.apply(BINDINGS[key])
                elif key in ("+", "="):
                    node.scale = min(2.0, node.scale + 0.1)
                elif key in ("-", "_"):
                    node.scale = max(0.1, node.scale - 0.1)
                elif key in ("q", "\x03"):
                    break
                sys.stdout.write(node.status())
                sys.stdout.flush()
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        # Always leave the drone hovering, never mid-manoeuvre.
        node.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nteleop stopped, drone holding hover")


if __name__ == "__main__":
    main()
