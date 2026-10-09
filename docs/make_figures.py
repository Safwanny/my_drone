#!/usr/bin/env python3
"""Plots for docs/controller.html from a recorded sim flight (docs/data/flight.npz).

The flight: hover at (0, 0, 1.5), the goto.py route
3,3,2 -2,1,1,90 2,-2,2,90 0,0,2,-90 0,0,2,180 0,0,1.5,0, the steps
4,0,1.5 / 4,0,3 / 0,0,1.5, then velocity mode: 2 s of 1.5 m/s forward +
0.5 rad/s yaw, released. Arrays (sim time in column 0):
  odom: t x y z qw qx qy qz vx vy vz wx wy wz   (velocities in body frame)
  mot:  t w0 w1 w2 w3                            (rotor speeds, rad/s)
  goal: t x y z qw qx qy qz                      (/goal_pose as sent)
  cmd:  t vx vy vz yaw_rate                      (/cmd_vel as sent)

    python3 docs/make_figures.py      # writes docs/figures/*.svg
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures")
C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"  # validated, fixed order
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"

plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9, "legend.fontsize": 8,
    "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.6, "legend.frameon": False, "savefig.bbox": "tight",
    "svg.fonttype": "none", "font.family": "DejaVu Sans",
})

d = np.load(os.path.join(HERE, "data", "flight.npz"))
odom, mot, goal, cmd = d["odom"], d["mot"], d["goal"], d["cmd"]
t0 = odom[0, 0]
t = odom[:, 0] - t0
p = odom[:, 1:4]
qw, qx, qy, qz = odom[:, 4:8].T
roll = np.degrees(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2)))
pitch = np.degrees(np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1)))
yaw = np.degrees(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2)))


def body_to_world(v):
    """Rotate odom's body-frame velocity into the world frame."""
    out = np.empty_like(v)
    for i, (w, x, y, z) in enumerate(zip(qw, qx, qy, qz)):
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        out[i] = r @ v[i]
    return out


v = body_to_world(odom[:, 8:11])
gt = goal[:, 0] - t0
gyaw = np.degrees(np.arctan2(2 * (goal[:, 4] * goal[:, 7] + goal[:, 5] * goal[:, 6]),
                             1 - 2 * (goal[:, 6] ** 2 + goal[:, 7] ** 2)))


def held(ts, vals, tq):
    """Zero-order hold of a setpoint stream onto query times (nan before the first)."""
    i = np.searchsorted(ts, tq, side="right") - 1
    out = vals[np.clip(i, 0, None)].astype(float)
    out[i < 0] = np.nan
    return out


cmd_t = cmd[:, 0] - t0 if len(cmd) else np.array([np.inf])
pos_mode = t < cmd_t[0]
sp = held(gt, goal[:, 1:4], t)
sp[~pos_mode] = np.nan
sp_yaw = held(gt, gyaw, t)
sp_yaw[~pos_mode] = np.nan


def break_wraps(a):
    """nan where an angle jumps across +-180, so the line doesn't draw the jump."""
    a = a.copy()
    a[1:][np.abs(np.diff(a)) > 180] = np.nan
    return a


def window(a, b):
    return (t >= a) & (t <= b)


def save(fig, name):
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)


os.makedirs(OUT, exist_ok=True)
route_end = gt[np.argmax(goal[:, 1] == 4.0)]  # first step goal = end of the route

# 1. Route, top view
m = window(0, route_end)
fig, ax = plt.subplots(figsize=(4.2, 4.0))
ax.plot(p[m, 0], p[m, 1], color=C1, label="flown path")
wps = [(3, 3), (-2, 1), (2, -2), (0, 0)]
ax.scatter(*zip(*wps), s=60, marker="o", facecolor="white", edgecolor=C2, linewidth=2,
           zorder=3, label="waypoints")
for k, (x, y) in enumerate(wps):
    ax.annotate(f"{k + 1}" if k < 3 else "home", (x, y), textcoords="offset points",
                xytext=(7, 6), color=INK, fontsize=8)
ax.set(xlabel="x (m)", ylabel="y (m)", aspect="equal", title="Waypoint route, top view")
ax.legend(loc="lower left")
save(fig, "route_xy.svg")

# 2. Route: position vs setpoint per axis
fig, axs = plt.subplots(3, 1, figsize=(6.4, 4.6), sharex=True)
for ax, k, c, lab in zip(axs, range(3), (C1, C2, C3), ("x", "y", "z")):
    ax.plot(t[m], sp[m, k], color=INK2, linestyle="--", linewidth=1.2, label="setpoint")
    ax.plot(t[m], p[m, k], color=c, label=f"{lab} measured")
    ax.set_ylabel(f"{lab} (m)")
    ax.legend(loc="upper right", ncol=2)
axs[-1].set_xlabel("sim time (s)")
axs[0].set_title("Position tracking over the route")
save(fig, "route_xyz.svg")

# 3. Step 0 -> 4 m in x: position, velocity, pitch (the cascade at work)
ts = gt[np.argmax(goal[:, 1] == 4.0)]
m = window(ts - 0.5, ts + 5.5)
tt = t[m] - ts
fig, axs = plt.subplots(3, 1, figsize=(6.4, 5.0), sharex=True)
axs[0].plot(tt, sp[m, 0], color=INK2, linestyle="--", linewidth=1.2, label="setpoint")
axs[0].plot(tt, p[m, 0], color=C1, label="x")
axs[0].set_ylabel("x (m)")
axs[1].axhline(3.0, color=INK2, linestyle=":", linewidth=1, label="vel_max_xy = 3 m/s")
axs[1].plot(tt, v[m, 0], color=C2, label="vx (world)")
axs[1].set_ylabel("vx (m/s)")
axs[2].axhline(35, color=INK2, linestyle=":", linewidth=1, label="tilt limit 35°")
axs[2].plot(tt, pitch[m], color=C3, label="pitch")
axs[2].set_ylabel("pitch (°)")
axs[2].set_xlabel("time since goal (s)")
for ax in axs:
    ax.legend(loc="upper right")
axs[0].set_title("Step response: goal 4 m ahead")
save(fig, "step_x.svg")

# 4. Yaw tracking over the route
m = window(0, route_end)
fig, ax = plt.subplots(figsize=(6.4, 2.4))
ax.plot(t[m], break_wraps(sp_yaw[m]), color=INK2, linestyle="--", linewidth=1.2, label="setpoint")
ax.plot(t[m], break_wraps(yaw[m]), color=C1, label="yaw")
ax.set(xlabel="sim time (s)", ylabel="yaw (°)", yticks=[-180, -90, 0, 90, 180],
       title="Yaw tracking (wraps at ±180°)")
ax.legend(loc="lower left", ncol=2)
save(fig, "yaw.svg")

# 5. Rotor speeds around the -90 -> 180 deg yaw turn
ty = gt[np.argmax(np.isclose(gyaw, 180) | np.isclose(gyaw, -180))]
mt = (mot[:, 0] - t0 >= ty - 0.5) & (mot[:, 0] - t0 <= ty + 2.5)
hover = np.sqrt(0.931 * 9.80665 / 4 / 3.7e-6)
fig, ax = plt.subplots(figsize=(6.4, 2.8))
# Diagonal rotors get identical commands in a pure yaw, so plot the pairs.
tm = mot[mt, 0] - t0 - ty
ax.plot(tm, mot[mt, 1:3].mean(1), color=C1, label="ccw pair (0 front-right, 1 rear-left)")
ax.plot(tm, mot[mt, 3:5].mean(1), color=C2, label="cw pair (2 front-left, 3 rear-right)")
ax.axhline(230, color=INK2, linestyle=":", linewidth=1)
ax.annotate("idle floor 230 rad/s", (2.45, 230), textcoords="offset points",
            xytext=(0, 4), ha="right", color=INK2, fontsize=8)
ax.axhline(hover, color=INK2, linestyle=":", linewidth=1)
ax.annotate(f"hover {hover:.0f} rad/s", (2.45, hover), textcoords="offset points",
            xytext=(0, 4), ha="right", color=INK2, fontsize=8)
ax.set(xlabel="time since yaw goal (s)", ylabel="rotor speed (rad/s)",
       title="Mixer during a 90° yaw turn: start, then brake")
ax.legend(loc="upper right")
save(fig, "motors_yaw.svg")

# 6. Velocity mode: stick, release, brake, hold
m = window(cmd_t[0] - 0.5, t[-1])
tc = cmd_t[0]
cvx = held(cmd_t, cmd[:, 1], t[m])
cvx[t[m] > cmd_t[-1]] = 0.0
heading_v = v[m, 0] * np.cos(np.radians(yaw[m])) + v[m, 1] * np.sin(np.radians(yaw[m]))
fig, axs = plt.subplots(2, 1, figsize=(6.4, 3.8), sharex=True)
axs[0].plot(t[m] - tc, cvx, color=INK2, linestyle="--", linewidth=1.2, label="stick: forward speed")
axs[0].plot(t[m] - tc, heading_v, color=C1, label="forward speed")
axs[0].set_ylabel("m/s")
dist = np.linalg.norm(p[m, :2] - p[m, :2][0], axis=1)
axs[1].plot(t[m] - tc, dist, color=C2, label="distance from start")
axs[1].set(ylabel="m", xlabel="time since stick input (s)")
for ax in axs:
    ax.legend(loc="center right")
axs[0].set_title("Velocity mode: fly, release, brake, hold")
save(fig, "velocity_mode.svg")

print("wrote", sorted(os.listdir(OUT)))
