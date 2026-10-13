# my_drone

![Build 03 hovering in Gazebo](docs/images/hero_front.png)

Quadcopter simulation for ROS 2 Jazzy + Gazebo Harmonic: **Custom Drone
Build 03**, a 7-inch true-X quad (404 mm, 0.931 kg all-up) built from CAD,
with 2807SL motors and 7040 tri-blade props on 6S.

It flies two ways, on two branches: on its own cascaded flight controller
and state estimator, and on PX4 v1.17 (SITL). Both take the same commands
(keyboard teleop, go-to-goal waypoints) and publish the same state, so
they can be compared flight for flight. How our controller works, with
plots from the sim: **[docs/controller.pdf](docs/controller.pdf)**.

| RViz: body and rotor frames | RViz: odometry trail of a waypoint route |
|---|---|
| ![RViz: body and rotor frames](docs/images/rviz.png) | ![RViz: odometry trail of a waypoint route](docs/images/rviz_route.png) |

Props spin in the standard props-in pattern, seen from above:
front-left CW, front-right CCW, rear-left CCW, rear-right CW.

## Branches

| Branch | Flight stack | Has |
|---|---|---|
| `controller` | ours: cascaded controller + EKF (C++) | `my_drone_control` |
| `px4` | PX4 v1.17 SITL: EKF2 + PX4's cascade | `my_drone_px4` |
| `main` | none on its own: the point both branched from | |

Both branches share the model (`my_drone_description`), the operator tools
(`my_drone_teleop`), the simulation package (`my_drone_bringup`, set up
for that branch's stack), this README and `docs/`. New work goes on a
feature branch off `controller` or `px4` and merges back there. Changes to
this README or `docs/` go on `main` and are merged into both.

A workspace builds whichever branch is checked out, so after switching
branches, rebuild (below) before launching.

## Build

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to my_drone_bringup my_drone_teleop
```

On `px4`, do the one-time PX4 setup ([PX4 SITL](#px4-sitl-px4-branch))
first.

## Run

Every new terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

Launch (opens Gazebo and RViz, and starts the branch's flight stack):

```bash
ros2 launch my_drone_bringup sim.launch.py
```

Options: `rviz:=false`, `headless:=true` (no Gazebo window), `z:=0.5`
(spawn height). On `controller` also `controller:=gz` (Gazebo's built-in
velocity controller instead of ours; no `/goal_pose` then).

Sensors (IMU, barometer, magnetometer, GNSS) have the noise of an
x500-class flight controller's parts: IIM42653 IMU, BMP390 barometer
(3 Pa), IIS2MDC magnetometer, F9P-class GNSS (~2 cm). On `px4` the GNSS is
exact in gz and PX4 adds that noise itself, as on PX4's own x500.

Fly, in a second terminal (needs a real keyboard/TTY):

```bash
ros2 run my_drone_teleop arrow_teleop
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
starts (props spin up to idle-hover speed); quitting teleop sends `/land`,
and the flight stack descends and disarms on touchdown, so the props stop.
Your terminal is restored either way.

Land or kill from anywhere:

```bash
ros2 topic pub --once /land std_msgs/msg/Empty
```
```bash
ros2 topic pub --once /enable std_msgs/msg/Bool "{data: false}"
```

The first lands and disarms; the second stops the motors at once, in the
air too.

Stop the sim with Ctrl+C in the launch terminal or by closing the Gazebo
window: either stops everything. Any process a crashed session left behind
is cleared at the next launch.

Go to goal, in a second terminal: each waypoint is `x,y,z[,yaw_deg]` in
the odom frame. The script arms the drone, flies the waypoints in order and
prints the time and overshoot for each, then leaves it hovering. Ctrl+C
(or a waypoint that times out) lands it instead; Ctrl+C again kills the
motors. Waypoints need 0 < z <= 30 m.

```bash
ros2 run my_drone_teleop goto 0,0,2 3,3,2 -2,1,1,90 0,0,0.3
```

RViz's *2D Goal Pose* tool works too (it keeps the current altitude, or
takes off to 1.5 m).

### RViz

Side panels start hidden; click the arrow on the left/right window edge to
open Displays / Views. Left-drag orbits, middle-drag pans, wheel zooms.
Saved views: *Follow drone* (default), *World (fixed)*, *Top down*.

The drone model sits at its exact (gz) pose; the orange trail is `/odom`,
the flight stack's estimate. The gap between them is the estimation error.

## Topics

The same on both branches, except `/motor_speed` (`controller` only; PX4
drives the motors itself).

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (heading-frame velocity + yaw rate) | in |
| `/goal_pose` | `geometry_msgs/PoseStamped` (position + yaw, odom frame) | in |
| `/enable` | `std_msgs/Bool` (arm / disarm; false = motor kill) | in |
| `/land` | `std_msgs/Empty` (descend, disarm on touchdown) | in |
| `/armed` | `std_msgs/Bool` (latched, on change) | out |
| `/motor_speed` | `actuator_msgs/Actuators` (rotor speeds, rad/s) | controller → gz |
| `/odom` | `nav_msgs/Odometry` (the flight stack's estimate: our EKF, or PX4's EKF2) | out |
| `/ground_truth/odom` | `nav_msgs/Odometry` (gz's exact state, for scoring only) | out |
| `/imu` | `sensor_msgs/Imu` (250 Hz; no orientation) | out |
| `/air_pressure` | `sensor_msgs/FluidPressure` (50 Hz) | out |
| `/magnetic_field` | `sensor_msgs/MagneticField` (100 Hz, gauss: a gz Harmonic quirk) | out |
| `/navsat` | `sensor_msgs/NavSatFix` (10 Hz) | out |
| `/joint_states` | `sensor_msgs/JointState` (prop angles) | out |
| `/tf`, `/clock` | | out |

`/odom`'s origin is where the drone sat at start (the spawn point), so
goals are relative to the takeoff spot on both branches and `/odom` minus
`/ground_truth/odom` is the error the flight stack flies with.

## Our controller (`controller` branch)

### State estimator

The controller flies on what the sensors say, not on Gazebo's exact state.
`my_drone_control/src/state_estimator.cpp` (ROS I/O, start-up) around
`my_drone_control/include/my_drone_control/estimator.hpp` (the filter, no
ROS) is a 15-state error-state EKF, the usual multicopter design: the IMU
predicts position, velocity and attitude (and its own biases), GNSS
position, barometer height and the magnetometer's direction correct them.

The filter starts once the drone has sat still for 1 s; that spot
(`origin` in `controller.yaml`) is the odom origin. On a hover / move /
90 deg yaw route its error is ~2 cm horizontal and ~4 cm vertical RMS,
~0.25 deg in attitude.

### Flight controller

`my_drone_control/src/flight_controller.cpp` (ROS I/O, modes, failsafes)
around `my_drone_control/include/my_drone_control/control.hpp` (the math,
no ROS). It is the cascade PX4 and ArduPilot use, in physical units:

```
/goal_pose ─▶ position P ─▶ velocity PID ─▶ thrust vector ─▶ attitude P ─▶ rate PID ─▶ mixer ─▶ /motor_speed
/cmd_vel   ───────────────────▲            (tilt limit)    (quaternion)              (from rotor geometry)
```

- Each `/imu` sample (250 Hz) runs the loop, like a real FC's gyro. `/odom`
  (50 Hz) is the state estimate; attitude is propagated with the gyro in
  between.
- Modes: the last input wins. `/goal_pose` gives position mode, `/cmd_vel`
  gives velocity mode. Centre the stick (or stop sending) and it brakes, then
  holds position.
- On the ground it idles until something asks to climb. If `/odom` stops it
  levels out and sinks slowly. Disarm stops the motors at once.
- Touchdown (asked down, not moving, thrust under hover, below
  `land_slow_alt`) disarms in every mode, so landing does not depend on the
  client staying alive. Goals below 0 or above `alt_max` are rejected.
- Mass, inertia, motor constants and rotor layout come from the
  `<quad_physics>` element `quad_gazebo.xacro` writes into the URDF, so they
  always match the Gazebo model. Gains are in `my_drone_control/config/controller.yaml`
  (physical units, so they do not need rescaling when the mass changes).
  Tune inner to outer: rate, attitude, velocity, position.

`colcon test --packages-select my_drone_control` runs the controller's
sign and scale checks and the EKF's convergence checks. Scalar limits
(`tilt_max_deg`, timeouts, ...) are range-checked by the parameter server,
so a bad value fails at start-up.

## PX4 SITL (`px4` branch)

PX4 v1.17.0: EKF2 estimates the state from the simulated IMU, barometer,
magnetometer and GNSS, and PX4's own cascade flies on that estimate.

It needs the one-time setup below, and a terminal sourced after that build
(`source ~/ros2_ws/install/setup.bash`): a terminal sourced earlier does not
see `my_drone_px4` or `MicroXRCEAgent`, and the launch stops with a message
saying which is missing.

The launch starts PX4 attached to the model it spawns, on our airframe
`my_drone_px4/airframes/4950_gz_build03` (rotor geometry, motor range,
hover thrust, rate gains scaled for this airframe; PX4 only reads airframes
from its build tree, so the launch links the file in there), plus the
uXRCE-DDS agent and `offboard_adapter`. PX4 is looked for in
`~/PX4-Autopilot`, or `$PX4_DIR`.

`offboard_adapter` flies PX4 in offboard mode from the topics above, so
`goto` and `arrow_teleop` work on it: `/enable` arms (or kills), `/land`
switches PX4 to Land (it disarms on touchdown), `/goal_pose` and `/cmd_vel`
become offboard setpoints, and `/odom` and `/armed` come from PX4. PX4 flies
in its local frame, which starts where the drone sat at boot (the spawn
point).

PX4 ramps its takeoff in over a few seconds, and if it leaves offboard on
its own (Land, a failsafe) the adapter stops sending setpoints until a new
goal or `/cmd_vel` arrives.

`EKF2_GND_EFF_DZ` is 0 in the airframe because the sim has no ground
effect (the default 4 m dead zone made the height sink ~1 m at every
touchdown). Everything else in EKF2 is PX4's default, as on the x500.

PX4's console commands, from a second terminal with ROS sourced:

```bash
cd ~/PX4-Autopilot/build/px4_sitl_default && ./bin/px4-commander takeoff
```
```bash
cd ~/PX4-Autopilot/build/px4_sitl_default && ./bin/px4-commander land
```

Flight logs land in `~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`
(open them in PlotJuggler or https://review.px4.io).

One-time setup (no sudo needed on a machine that already builds this repo;
PX4 finds Gazebo's libraries through ROS):

```bash
git clone --branch v1.17.0 --depth 1 --recurse-submodules --shallow-submodules https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
```
```bash
python3 -m venv --system-site-packages ~/.venvs/px4 && ~/.venvs/px4/bin/pip install ninja -r ~/PX4-Autopilot/Tools/setup/requirements.txt
```
```bash
source /opt/ros/jazzy/setup.bash && cd ~/PX4-Autopilot && PATH=~/.venvs/px4/bin:$PATH make px4_sitl
```
```bash
cd ~/ros2_ws/src && vcs import < my_drone/px4.repos && cd .. && colcon build --packages-select px4_msgs microxrcedds_agent
```

PX4's own x500, to check the install (Gazebo window opens; PX4's console
is in this terminal, try `commander takeoff` and `commander land`):

```bash
source /opt/ros/jazzy/setup.bash && cd ~/PX4-Autopilot && PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
```

Without QGroundControl, the x500 refuses to arm ("No connection to the
GCS") until `param set NAV_DLL_ACT 0` and `param set COM_RC_IN_MODE 4`
(sim only; our airframe sets both). PX4 topics in ROS 2, in a second
terminal:

```bash
source ~/ros2_ws/install/setup.bash && MicroXRCEAgent udp4 -p 8888
```

## Layout

Packages are split by what they depend on: the controller builds and
tests without Gazebo, the description can be reused on hardware, and
each flight stack sits in its own package on its own branch.

```
my_drone_description/   the model, no runtime deps beyond xacro
  urdf/quad_mesh_common.xacro  motor/prop physics + spinning prop macros
  urdf/quad_gazebo.xacro       gz plugins: motor models, sensors, exact odometry
                               (controller: + <quad_physics>, gz velocity controller)
  urdf/build03_params.xacro    mass, inertia, rotor positions
  urdf/build03.urdf.xacro      frame meshes + collision boxes
  meshes/                      STLs (mm) copied from the CAD folders
my_drone_control/       controller branch: flight controller and state estimator (C++)
  include/my_drone_control/control.hpp   the cascade maths, ROS-free, header-only
  include/my_drone_control/estimator.hpp the EKF, ROS-free, header-only
  src/flight_controller.cpp              node: topics, modes, failsafes
  src/state_estimator.cpp                node: sensors -> /odom
  config/controller.yaml                 gains, limits, estimator noise
  test/test_control.cpp                  gtest sign/scale checks
  test/test_estimator.cpp                gtest EKF convergence checks
my_drone_px4/           px4 branch: PX4 side
  airframes/4950_gz_build03              PX4 airframe for the Build 03 Prototype
  scripts/offboard_adapter.py            our topics <-> PX4 offboard over uXRCE-DDS
  test/test_frames.py                    NED/FRD <-> ENU/FLU conversion checks
px4.repos               px4 branch: px4_msgs + Micro XRCE-DDS agent sources
my_drone_teleop/        operator tools (Python): arrow_teleop, goto
my_drone_bringup/       simulation: launch/sim.launch.py, worlds/,
                        config/bridge_build03.yaml (ROS <-> gz), rviz/
tools/mass_budget.py    mass / CoM / inertia from meshes + part weights (not installed)
docs/                   controller write-up (controller.html|pdf; build_pdf.py prints
                        it with Firefox, make_figures.py plots data/flight.npz), images
```

## When real weights are known

Masses are estimates: printed frames by mesh volume (PAHT-CF, 1.06 g/cm³),
motors, props and battery from datasheets, and electronics guessed. After
weighing:

1. Edit the tables at the top of `tools/mass_budget.py` (it reads the CAD
   folders under `/media/safwan/Shared/Drone_Frames`, or `$DRONE_FRAMES`).
2. Run `python3 tools/mass_budget.py` and paste the printed values into
   `my_drone_description/urdf/build03_params.xacro` (on both branches).
3. Our controller picks the new values up by itself. For `controller:=gz`,
   rescale `attitude_gain` / `angular_rate_gain` by the inertia change per
   axis and `velocity_gain` by the mass change. PX4 does not use mass or
   inertia directly; its hover thrust estimator adapts, and a large inertia
   change may need the rate gains (`MC_*RATE_K` in the airframe) retuned.

## History

The original F450 model was removed in 0.2.0 (`git checkout f450-final`
restores it). The WEAREFPV JeNo 7 was removed in 0.3.0 (`git checkout
v0.2.1` has it). 0.4.0 split the single `my_drone` package into packages;
`git checkout v0.3.0` has the old layout. After 0.4.0 the project split
into the `controller` and `px4` branches.
