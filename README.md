# my_drone

Quadcopter simulation for ROS 2 Jazzy + Gazebo Harmonic. Two 7-inch
airframes built from CAD, both with 2807SL motors and 7040 tri-blade props
on 6S:

| `drone:=` | Airframe | Layout | All-up mass |
|---|---|---|---|
| `build03` (default, **main drone**) | Custom Drone Build 03 | true X, 404 mm | 0.931 kg |
| `jeno7` | WEAREFPV JeNo 7 | stretched X, 302 mm | 0.854 kg |

Props spin in the standard props-in pattern, seen from above:
front-left CW, front-right CCW, rear-left CCW, rear-right CW.

## Build

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select my_drone
```

## Run

Every new terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

Launch (opens Gazebo and RViz):

```bash
ros2 launch my_drone sim.launch.py
```

```bash
ros2 launch my_drone sim.launch.py drone:=jeno7
```

Options: `rviz:=false`, `headless:=true` (no Gazebo window), `z:=0.5`
(spawn height).

Fly, in a second terminal (needs a real keyboard/TTY):

```bash
ros2 run my_drone arrow_teleop.py
```

| Key | Action |
|---|---|
| `w` / `s` | climb / descend (`w` to take off) |
| arrows | forward / back / strafe |
| `a` / `d` | yaw left / right |
| `space` | stop and hover |
| `+` / `-` | speed scale |
| `q` / Ctrl+C | land, stop the motors, quit (Ctrl+C twice: stop motors now) |

The drone spawns disarmed with its props still. Teleop arms it when it
starts (props spin up to idle-hover speed); quitting teleop lands the drone
if it is airborne and then disarms it, so the props stop. Your terminal is
restored either way.

Stop the sim with Ctrl+C in the launch terminal or by closing the Gazebo
window: either stops everything. Any process a crashed session left behind
is cleared at the next launch.

### RViz

Side panels start hidden; click the arrow on the left/right window edge to
open Displays / Views. Left-drag orbits, middle-drag pans, wheel zooms.
Saved views: *Follow drone* (default), *World (fixed)*, *Top down*.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (body-frame velocity) | in |
| `/enable` | `std_msgs/Bool` | in |
| `/odom` | `nav_msgs/Odometry` | out |
| `/imu` | `sensor_msgs/Imu` | out |
| `/joint_states` | `sensor_msgs/JointState` (prop angles) | out |
| `/tf`, `/clock` | | out |

## Layout

```
urdf/quad_mesh_common.xacro  motor/prop physics + spinning prop macros (shared)
urdf/quad_gazebo.xacro       gz plugins: motor models, velocity controller, IMU, odom
urdf/<drone>_params.xacro    mass, inertia, rotor positions, controller gains
urdf/<drone>.urdf.xacro      frame meshes + collision boxes
config/bridge_<drone>.yaml   ROS <-> gz topic bridge
meshes/                      STLs (mm) copied from the CAD folders
tools/mass_budget.py         mass / CoM / inertia from meshes + part weights
```

## When real weights are known

Masses are estimates: printed frames by mesh volume (PAHT-CF, 1.06 g/cm³),
motors, props and battery from datasheets, and electronics guessed. After
weighing:

1. Edit the tables at the top of `tools/mass_budget.py` (it reads the CAD
   folders under `/media/safwan/Shared/Drone_Frames`, or `$DRONE_FRAMES`).
2. Run `python3 tools/mass_budget.py` and paste the printed values into
   `urdf/<drone>_params.xacro`.
3. Rescale `attitude_gain` / `angular_rate_gain` by the inertia change per
   axis and `velocity_gain` by the mass change. Gazebo's controller
   normalises gains by both, so this keeps the same response.

## History

The original F450 model was removed in 0.2.0. `git checkout f450-final`
restores it.
