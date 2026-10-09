#!/usr/bin/env python3
"""Arrow-key teleop for the drones in this package.

Publishes geometry_msgs/Twist on /cmd_vel in the vehicle's heading frame,
which flight_controller (or Gazebo's MulticopterVelocityControl, with
controller:=gz) flies as a velocity setpoint:

    linear.x   forward / back velocity   (m/s)
    linear.y   left / right velocity     (m/s)
    linear.z   climb rate                (m/s)
    angular.z  yaw rate                  (rad/s)

Setpoints are held until changed and decay to zero if the terminal goes
quiet, so a dropped keypress leaves the drone hovering rather than
drifting away.

Teleop owns arming: it arms the controller (/enable true) when it starts.
On quit (q or Ctrl+C) it lands the drone if airborne, then disarms, which
stops the props. A second Ctrl+C while landing disarms immediately.
"""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

BANNER = """
+----------------------------------------------------------+
|                  Drone arrow-key teleop                  |
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
|   q / Ctrl+C ........... land, stop motors, quit         |
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

# Landing on quit: descend fast high up, gently near the ground, and call it
# landed once the vertical speed has stayed ~0 for LANDED_SECONDS.
LAND_FAST = 1.0       # m/s above SLOW_BELOW
LAND_SLOW = 0.3       # m/s below SLOW_BELOW
SLOW_BELOW = 0.6      # m (odom z, ground at 0)
LANDED_VZ = 0.05      # m/s
LANDED_SECONDS = 0.5
LAND_TIMEOUT = 30.0   # s, then disarm regardless


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__("arrow_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.enable_pub = self.create_publisher(Bool, "/enable", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.cmd = [0.0, 0.0, 0.0, 0.0]
        self.scale = 1.0
        self.ticks_since_key = 0
        self.hold_ticks = int(HOLD_SECONDS * PUBLISH_HZ)
        self.z = None
        self.vz = None
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.tick)
        # Re-send the arm request now and then: the bridge may come up after
        # us, and a single message would be lost.
        self.arm_timer = self.create_timer(0.5, lambda: self.set_armed(True))

    def on_odom(self, msg):
        self.z = msg.pose.pose.position.z
        self.vz = msg.twist.twist.linear.z

    def set_armed(self, armed):
        self.enable_pub.publish(Bool(data=armed))

    def send(self, vx=0.0, vy=0.0, vz=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = vx, vy, vz
        msg.angular.z = yaw
        self.pub.publish(msg)

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
        self.send(*self.cmd)

    def status(self):
        return (
            f"\r vx {self.cmd[0]:+5.2f}  vy {self.cmd[1]:+5.2f}  "
            f"vz {self.cmd[2]:+5.2f}  yaw {self.cmd[3]:+5.2f}  "
            f"[scale {self.scale:.1f}]   "
        )

    def land_and_disarm(self):
        """Descend until the drone stops moving vertically, then disarm."""
        self.destroy_timer(self.timer)       # stop the normal setpoint stream
        self.destroy_timer(self.arm_timer)
        if self.z is None:
            print("\nno /odom (is the sim running?), disarming")
        else:
            print(f"\nlanding from z={self.z:.2f} m ... (Ctrl+C again to cut motors now)")
            try:
                start = time.monotonic()
                still_since = None
                while time.monotonic() - start < LAND_TIMEOUT:
                    speed = LAND_FAST if self.z > SLOW_BELOW else LAND_SLOW
                    self.send(vz=-speed)
                    rclpy.spin_once(self, timeout_sec=1.0 / PUBLISH_HZ)
                    moving = abs(self.vz) > LANDED_VZ
                    if moving or time.monotonic() - start < 1.0:
                        still_since = None
                    elif still_since is None:
                        still_since = time.monotonic()
                    elif time.monotonic() - still_since > LANDED_SECONDS:
                        print(f"landed (z={self.z:.2f} m)")
                        break
                else:
                    print("landing timed out")
            except KeyboardInterrupt:
                print("landing skipped")
        self.send()
        # Several sends: this is the message that stops the props, and it
        # must not be lost to a slow subscriber.
        for _ in range(5):
            self.set_armed(False)
            rclpy.spin_once(self, timeout_sec=0.02)
        print("motors stopped")


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


def drive(node):
    """Keyboard loop. Returns on q; Ctrl+C raises KeyboardInterrupt."""
    while True:
        key = read_key(0.02)
        if key is not None:
            if key in BINDINGS:
                node.apply(BINDINGS[key])
            elif key in ("+", "="):
                node.scale = min(2.0, node.scale + 0.1)
            elif key in ("-", "_"):
                node.scale = max(0.1, node.scale - 0.1)
            elif key == "q":
                return
            sys.stdout.write(node.status())
            sys.stdout.flush()
        rclpy.spin_once(node, timeout_sec=0.0)


def main():
    if not sys.stdin.isatty():
        sys.exit("arrow_teleop needs an interactive terminal (run it with ros2 run, not from a launch file)")

    # Without this rclpy installs its own SIGINT handler, which tears the
    # context down on Ctrl+C before we can land, disarm and restore the
    # terminal. Let Ctrl+C arrive as a plain KeyboardInterrupt instead.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ArrowTeleop()
    print(BANNER)
    print("armed: motors running, w to take off")

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        try:
            drive(node)
        except KeyboardInterrupt:
            pass
    finally:
        # First and unconditionally: give the terminal back (echo, line
        # editing), whatever happens below.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    try:
        node.land_and_disarm()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
