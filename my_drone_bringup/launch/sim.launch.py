"""Launch Build 03 in Gazebo Harmonic, bridged to ROS 2.

Brings up gz sim with the flight world, robot_state_publisher fed from the
xacro, the model spawn, the ROS<->GZ bridge, the flight stack and RViz. The
flight stack is our flight_controller (controller:=custom) or PX4 SITL with
the uXRCE-DDS agent and our offboard adapter (controller:=px4); either way
it is flown through the same topics (/enable, /land, /goal_pose, /cmd_vel,
/odom, /armed). The drone spawns disarmed (props still); arrow_teleop arms
it on start and lands + disarms it on quit, goto arms it and flies
waypoints.

    ros2 launch my_drone_bringup sim.launch.py
    ros2 launch my_drone_bringup sim.launch.py rviz:=false
    ros2 launch my_drone_bringup sim.launch.py controller:=px4  # PX4 SITL (~/PX4-Autopilot)

Teleop is NOT launched here: reading the keyboard needs a real TTY, which a
launch-spawned process does not get. Run it yourself in a second terminal:

    ros2 run my_drone_teleop arrow_teleop
"""

import os
import signal
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Every process this launch starts inherits this variable, including the
# ones gz forks behind launch's back, so stragglers can always be found. Its
# value is the launch's PID, so a session shutting down only sweeps its own
# processes, not those of a new session started meanwhile.
MARKER = "MY_DRONE_SIM"


def marked_pids(session=None):
    """PIDs (ours, other than this launch) tagged with MARKER, by any session
    or only by `session`."""
    me = os.getpid()
    tag = (MARKER + "=" + (f"{session}\0" if session else "")).encode()
    pids = []
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            with open(f"/proc/{d}/environ", "rb") as f:
                if tag in f.read():
                    pids.append(int(d))
        except OSError:  # gone, or not ours
            pass
    return pids


def kill_stragglers(context, when):
    """Kill sim processes that would otherwise outlive the launch.

    At startup: anything left from an earlier session (crashed launch,
    closed terminal). A leftover gz server keeps the same world name, so it
    answers the next spawn request (models fail to appear) and publishes a
    second /clock (RViz "jump back in time", robot_state_publisher "Moved
    backwards in time").

    On exit: launch stops its own children, but not what they forked (the
    Gazebo window's Ruby wrapper forks the real GUI in its own process
    group), so terminate the rest here.
    """
    pids = marked_pids(os.getpid() if when == "on exit" else None)
    if when == "on exit":
        # Direct children are launch's job; it SIGINTs them and escalates.
        children = set(map(int, subprocess.run(
            ["pgrep", "-P", str(os.getpid())], capture_output=True, text=True
        ).stdout.split()))
        pids = [p for p in pids if p not in children]
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL if when == "at startup" else signal.SIGTERM)
        except ProcessLookupError:
            pass
    if pids:
        return [LogInfo(msg=f"Stopped {len(pids)} leftover sim process(es) {when}: {pids}")]
    return []


PX4_AIRFRAME = "4950_gz_build03"


def start_px4(context):
    """Run PX4 SITL from $PX4_DIR (default ~/PX4-Autopilot) on our airframe.

    PX4 only reads airframes from its own build tree, so link ours in there
    rather than patching PX4's sources.
    """
    build = os.path.join(
        os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot")),
        "build", "px4_sitl_default")
    px4_bin = os.path.join(build, "bin", "px4")
    if not os.path.exists(px4_bin):
        raise RuntimeError(f"no PX4 SITL build at {build}: see README, PX4 SITL")
    src = os.path.join(get_package_share_directory("my_drone_px4"), "airframes", PX4_AIRFRAME)
    dst = os.path.join(build, "etc", "init.d-posix", "airframes", PX4_AIRFRAME)
    if os.path.lexists(dst):
        os.remove(dst)
    os.symlink(src, dst)
    return [ExecuteProcess(
        cmd=[px4_bin, "-d"],
        cwd=build,
        additional_env={
            "PX4_SYS_AUTOSTART": PX4_AIRFRAME.split("_")[0],
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_WORLD": "empty_flight",
            "PX4_GZ_MODEL_NAME": "build03",
        },
        output="screen",
        on_exit=Shutdown(reason="PX4 exited"),
    )]


def generate_launch_description():
    pkg = get_package_share_directory("my_drone_bringup")
    description = get_package_share_directory("my_drone_description")

    drone = "build03"  # names the description xacro, the bridge config and the gz model
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")
    spawn_z = LaunchConfiguration("z")
    controller = LaunchConfiguration("controller")

    args = [
        DeclareLaunchArgument(
            "controller",
            default_value="custom",
            choices=["custom", "px4"],
            description="custom: our flight_controller node (position, "
            "velocity, attitude, rate, mixer) on the exact state. "
            "px4: PX4 SITL (EKF2 on the simulated sensors, then its own "
            "cascade), attached to the spawned model.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg, "worlds", "empty_flight.sdf"),
            description="Path to the SDF world to load.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Open RViz with Gazebo."
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="No Gazebo window (server only). Useful for tests.",
        ),
        DeclareLaunchArgument(
            "seed",
            default_value="",
            description="Seed for the GNSS/barometer errors (sensor_realism), "
            "for repeatable runs. Empty: random.",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.2",
            description="Spawn height, so the landing gear clears the ground.",
        ),
    ]

    # Expand the xacro once and hand the same URDF to both RSP and the spawner.
    # value_type=str is required: without it launch tries to YAML-parse the
    # URDF and chokes on the XML.
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [FindPackageShare("my_drone_description"), "urdf", [drone, ".urdf.xacro"]]
                ),
            ]
        ),
        value_type=str,
    )

    # Lets gz resolve package://my_drone_description/meshes/... (URDF->SDF
    # turns it into model://my_drone_description/..., looked up under
    # <install>/share).
    mesh_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.dirname(description)
    )

    # The server runs inside ros_gz_sim's gzserver node rather than the
    # `gz sim` Ruby wrapper, which forks the real server into its own
    # process group where neither Ctrl+C nor launch can reach it. If the
    # server dies, take everything else down with it.
    gz_server = Node(
        package="ros_gz_sim",
        executable="gzserver",
        output="screen",
        parameters=[{"world_sdf_file": world, "verbosity_level": 3}],
        on_exit=Shutdown(reason="Gazebo server exited"),
    )

    # The Gazebo window is a client of that server. Closing it ends the
    # whole session, same as Ctrl+C.
    gz_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=["gz", "sim", "-g", "-v", "3"],
        output="screen",
        on_exit=Shutdown(reason="Gazebo window closed"),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", drone,
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", spawn_z,
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [pkg, "config", ["bridge_", drone, ".yaml"]]
                ),
                "use_sim_time": True,
            }
        ],
    )

    is_custom = IfCondition(PythonExpression(["'", controller, "' == 'custom'"]))
    is_px4 = IfCondition(PythonExpression(["'", controller, "' == 'px4'"]))

    # Our controller flies on the exact state: bridge gz's odometry to /odom
    # too (with PX4, /odom is EKF2's estimate, from the offboard adapter).
    truth_odom = Node(
        condition=is_custom,
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="truth_odom_bridge",
        arguments=[f"/model/{drone}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
        remappings=[(f"/model/{drone}/odometry", "/odom")],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    flight_controller = Node(
        condition=is_custom,
        package="my_drone_control",
        executable="flight_controller",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("my_drone_control"), "config", "controller.yaml"]
            ),
            {"robot_description": robot_description, "use_sim_time": True},
        ],
    )

    # PX4 SITL, attached to the model spawned above (standalone: it neither
    # starts gz nor spawns its own model). Started once the spawner has
    # exited, so the model exists when PX4's gz_bridge looks for it.
    px4 = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[OpaqueFunction(function=start_px4)]),
        condition=is_px4,
    )

    # PX4's uORB topics <-> ROS 2 (/fmu/...), and our topics -> PX4 offboard.
    dds_agent = ExecuteProcess(
        condition=is_px4,
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="log",
    )
    offboard_adapter = Node(
        condition=is_px4,
        package="my_drone_px4",
        executable="offboard_adapter",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # GNSS and barometer errors (gz's own noise can't drift or delay). Its
    # output is what PX4 and the ROS bridge read as navsat / air_pressure.
    sensor_realism = OpaqueFunction(function=lambda context: [ExecuteProcess(
        cmd=["ros2", "run", "my_drone_bringup", "sensor_realism.py", "--model", drone]
        + (["--seed", s] if (s := LaunchConfiguration("seed").perform(context)) else []),
        output="screen",
    )])

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg, "rviz", "drone.rviz")],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Clear leftovers from an earlier session before starting anything, tag
    # everything we start, and sweep up whatever outlives the launch.
    clean_start = OpaqueFunction(function=kill_stragglers, args=["at startup"])
    tag = SetEnvironmentVariable(MARKER, str(os.getpid()))
    clean_exit = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[OpaqueFunction(function=kill_stragglers, args=["on exit"])]
        )
    )

    return LaunchDescription(
        args
        + [clean_start, tag, clean_exit, mesh_path]
        + [gz_server, gz_gui, robot_state_publisher, bridge, spawn,
           sensor_realism, truth_odom, flight_controller, px4, dds_agent,
           offboard_adapter, rviz]
    )
