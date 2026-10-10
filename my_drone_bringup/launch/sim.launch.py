"""Launch Build 03 in Gazebo Harmonic, bridged to ROS 2.

Brings up gz sim with the flight world, robot_state_publisher fed from the
xacro, the model spawn, the ROS<->GZ bridge, our flight_controller and RViz.
The drone spawns disarmed (props still); arrow_teleop arms it on start and
lands + disarms it on quit, goto.py arms it and flies waypoints.

    ros2 launch my_drone_bringup sim.launch.py
    ros2 launch my_drone_bringup sim.launch.py rviz:=false
    ros2 launch my_drone_bringup sim.launch.py controller:=gz   # Gazebo's built-in one

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
from launch.event_handlers import OnShutdown
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
            choices=["custom", "gz"],
            description="custom: our flight_controller node (position, "
            "velocity, attitude, rate, mixer). gz: Gazebo's built-in "
            "MulticopterVelocityControl (velocity only, no /goal_pose).",
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
                " controller:=",
                controller,
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

    flight_controller = Node(
        condition=IfCondition(PythonExpression(["'", controller, "' == 'custom'"])),
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
           flight_controller, rviz]
    )
