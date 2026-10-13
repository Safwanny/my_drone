#!/usr/bin/env python3
"""Fly PX4 through simple ROS topics (offboard mode over uXRCE-DDS).

goto, arrow_teleop and anything else written against these topics fly PX4:

    /enable    std_msgs/Bool              true: offboard + arm, false: kill
    /land      std_msgs/Empty             PX4 Land mode (disarms on touchdown)
    /goal_pose geometry_msgs/PoseStamped  position + yaw, world frame
    /cmd_vel   geometry_msgs/Twist        heading-frame velocity + yaw rate
    /odom      nav_msgs/Odometry          out: PX4's estimate (EKF2)
    /armed     std_msgs/Bool (latched)    out: armed state, on change

Frames. ROS side: ENU world frame "odom", body FLU. PX4 side: its NED local
frame, body FRD. PX4's local frame starts where EKF2 initialised, i.e. where
the drone sat at boot (the spawn point, origin_* parameters), and PX4 flies
in it: offboard setpoints are local. So a goal is a point relative to the
takeoff spot, and /odom minus /ground_truth/odom is
the error PX4 actually flies with (drift since boot). Its absolute GNSS
error at boot (ref_lat/lon/alt) is not part of that, as for any offboard
mission flown in the local frame.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Empty

NAN = float("nan")


# ---------------------------------------------------------------- frames
def qmul(a, b):
    """Hamilton product of (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


S = math.sqrt(0.5)
Q_ENU_NED = (0.0, S, S, 0.0)   # rotates NED vectors into ENU
Q_FRD_FLU = (0.0, 1.0, 0.0, 0.0)  # rotates FLU vectors into FRD


def att_ned_to_enu(q_ned_frd):
    """Body attitude: FRD-in-NED -> FLU-in-ENU (both (w, x, y, z))."""
    return qmul(qmul(Q_ENU_NED, q_ned_frd), Q_FRD_FLU)


def rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    w, x, y, z = qmul(qmul(q, (0.0, *v)), (q[0], -q[1], -q[2], -q[3]))
    return (x, y, z)


def yaw_enu(q_flu_enu):
    w, x, y, z = q_flu_enu
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Origin:
    """World ENU position of PX4's local NED origin."""

    def __init__(self, x, y, z):
        self.offset = (x, y, z)

    def ned_to_world(self, p):
        x, y, z = self.offset
        return (p[1] + x, p[0] + y, -p[2] + z)

    def world_to_ned(self, p):
        x, y, z = self.offset
        return (p[1] - y, p[0] - x, -(p[2] - z))


# ---------------------------------------------------------------- node
class OffboardAdapter(Node):
    def __init__(self):
        super().__init__("offboard_adapter")
        # Where the drone sat when PX4 booted: the spawn point; z is base_link's
        # height resting on its landing plate.
        self.ref = Origin(self.declare_parameter("origin_x", 0.0).value,
                          self.declare_parameter("origin_y", 0.0).value,
                          self.declare_parameter("origin_z", 0.0493).value)
        self.alt_max = self.declare_parameter("alt_max", 30.0).value
        self.takeoff_alt = self.declare_parameter("default_goal_alt", 1.5).value
        self.cmd_timeout = self.declare_parameter("cmd_vel_timeout", 0.5).value

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", px4_qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.on_status, px4_qos)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odometry, px4_qos)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.on_ack, px4_qos)

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.armed_pub = self.create_publisher(
            Bool, "armed", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, "enable", self.on_enable, 10)
        self.create_subscription(Empty, "land", self.on_land, 10)
        self.create_subscription(PoseStamped, "goal_pose", self.on_goal, 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        self.armed = None
        self.nav_state = None
        self.landed = True
        self.pos = None     # world ENU, from PX4's estimate
        self.vel = None     # world ENU
        self.yaw = 0.0      # ENU
        self.hold = None    # (x, y, z, yaw) world ENU position setpoint
        self.cmd_vel = None
        self.last_cmd = None
        self.last_arm_request = None
        # Setpoints are streamed only while PX4 is (being put) in offboard:
        # PX4's other modes publish the same trajectory_setpoint topic, and
        # ours would override them (Land would never detect touchdown).
        self.want_offboard = False
        self.create_timer(0.05, self.tick)  # PX4 wants the offboard stream at >= 2 Hz

    # ------------------------------------------------------------ PX4 in
    def on_status(self, m):
        armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if m.nav_state != self.nav_state:
            self.get_logger().info(f"PX4 mode {m.nav_state}")
            if self.nav_state == offboard:
                self.want_offboard = False  # PX4 left offboard (Land, a failsafe): its call
        self.nav_state = m.nav_state
        if armed != self.armed:
            self.armed = armed
            self.armed_pub.publish(Bool(data=armed))
            self.get_logger().info("armed" if armed else "disarmed")
            if not armed:
                self.cmd_vel = None
                self.hold = None
                self.want_offboard = False

    def on_ack(self, m):
        if m.result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().warn(f"PX4 rejected command {m.command} (result {m.result})")

    def on_odometry(self, m):
        if m.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        q = att_ned_to_enu(tuple(float(c) for c in m.q))
        self.pos = self.ref.ned_to_world(m.position)
        v = m.velocity
        self.vel = (float(v[1]), float(v[0]), -float(v[2]))  # NED -> ENU
        self.yaw = yaw_enu(q)

        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        p = o.pose.pose.position
        p.x, p.y, p.z = self.pos
        r = o.pose.pose.orientation
        r.w, r.x, r.y, r.z = q
        # Twist in the body frame, as gz's odometry (and REP 105) has it.
        vb = rotate((q[0], -q[1], -q[2], -q[3]), self.vel)
        t = o.twist.twist
        t.linear.x, t.linear.y, t.linear.z = vb
        w = m.angular_velocity  # body FRD -> FLU
        t.angular.x, t.angular.y, t.angular.z = float(w[0]), -float(w[1]), -float(w[2])
        self.odom_pub.publish(o)

    # ------------------------------------------------------------ ROS in
    def on_enable(self, m):
        if not m.data:
            if self.armed:
                self.get_logger().warn("motor kill")
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)  # forced
            return
        if self.armed or self.pos is None:
            return
        now = self.get_clock().now()
        # Callers repeat the request; don't flood PX4 with arm commands.
        if self.last_arm_request and (now - self.last_arm_request).nanoseconds < 1e9:
            return
        self.last_arm_request = now
        self.get_logger().info("arming (offboard)")
        if self.hold is None:
            self.hold = (*self.pos, self.yaw)
        self.want_offboard = True
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # custom mode: offboard
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def on_land(self, _):
        if self.armed:
            self.cmd_vel = None
            self.want_offboard = False
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def on_goal(self, m):
        p = m.pose.position
        if not (math.isfinite(p.x) and math.isfinite(p.y)) or p.z < 0.0 or p.z > self.alt_max:
            self.get_logger().warn(
                f"goal {p.x:.2f} {p.y:.2f} {p.z:.2f} rejected: z must be 0..{self.alt_max:.0f} m")
            return
        if self.pos is None:
            return
        # RViz's 2D Goal Pose arrives with z = 0: keep the altitude we fly at.
        z = p.z if p.z > 0.01 else (self.pos[2] if not self.landed else self.takeoff_alt)
        o = m.pose.orientation
        yaw = (yaw_enu((o.w, o.x, o.y, o.z)) if (o.x or o.y or o.z or o.w)
               else (self.hold[3] if self.hold else self.yaw))
        self.hold = (p.x, p.y, z, yaw)
        self.cmd_vel = None
        self.offboard()

    def on_cmd_vel(self, m):
        self.cmd_vel = (m.linear.x, m.linear.y, m.linear.z, m.angular.z)
        self.last_cmd = self.get_clock().now()
        self.offboard()

    # ------------------------------------------------------------ PX4 out
    def offboard(self):
        """Back to offboard if something else (Land, a failsafe) took over."""
        if self.armed and self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.want_offboard = True
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def command(self, cmd, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.timestamp = self.px4_time()
        c.command = cmd
        c.param1, c.param2 = p1, p2
        c.target_system = c.target_component = 1
        c.source_system = c.source_component = 1
        c.from_external = True
        self.cmd_pub.publish(c)

    def px4_time(self):
        return self.get_clock().now().nanoseconds // 1000

    def tick(self):
        if self.pos is None:
            return
        self.landed = not self.armed or self.pos[2] < 0.15
        stale = (self.last_cmd is None or
                 (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout)
        if self.cmd_vel and not stale:
            vx, vy, vz, yaw_rate = self.cmd_vel
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            v_enu = (vx * c - vy * s, vx * s + vy * c, vz)
            self.setpoint(velocity=(v_enu[1], v_enu[0], -v_enu[2]), yawspeed=-yaw_rate)
            self.hold = None
            return
        if self.cmd_vel is not None and self.vel is not None and math.hypot(*self.vel) > 0.3:
            # Stick released: brake first, so the hold point is not behind us.
            self.setpoint(velocity=(0.0, 0.0, 0.0), yawspeed=0.0)
            return
        self.cmd_vel = None
        if self.hold is None:
            self.hold = (*self.pos, self.yaw)
        x, y, z, yaw = self.hold
        self.setpoint(position=self.ref.world_to_ned((x, y, z)), yaw=wrap(math.pi / 2 - yaw))

    def setpoint(self, position=None, velocity=None, yaw=NAN, yawspeed=NAN):
        t = self.px4_time()
        # The heartbeat always goes out: PX4 only offers offboard mode while
        # it keeps arriving.
        mode = OffboardControlMode(timestamp=t, position=position is not None,
                                   velocity=velocity is not None)
        self.mode_pub.publish(mode)
        if self.armed and not self.want_offboard:
            return
        sp = TrajectorySetpoint(timestamp=t)
        sp.position = [float(c) for c in position] if position else [NAN] * 3
        sp.velocity = [float(c) for c in velocity] if velocity else [NAN] * 3
        sp.acceleration = [NAN] * 3
        sp.yaw, sp.yawspeed = float(yaw), float(yawspeed)
        self.sp_pub.publish(sp)


def main():
    rclpy.init()
    node = OffboardAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
