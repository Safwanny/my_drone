#!/usr/bin/env python3
"""Mass budget -> xacro numbers for build03.

Printed frame parts get mass from mesh volume x density x infill; bought
parts (motors, props, battery, electronics) get researched/assumed masses.
Inertia is integrated over the real meshes. Edit the tables below once the
real parts are weighed, re-run, and paste the output into
urdf/build03_params.xacro.

    python3 tools/mass_budget.py

Reads the CAD folders directly (not installed with the package).
"""
import glob
import os

import numpy as np

FRAMES = os.environ.get("DRONE_FRAMES", "/media/safwan/Shared/Drone_Frames")

# ---- materials (kg/m^3) ----
PAHT_CF = 1060   # Bambu PAHT-CF TDS

# ---- bought parts (kg) ----
MOTOR = 0.055        # 2807 1300KV incl. 200 mm 18AWG leads (DarwinFPV 55.7 g)
PROP_NUT = 0.0114    # HQ 7x4x3 ~9.4 g + M5 flange nut ~2 g
BATTERY = 0.290      # 6S 1800 mAh LiPo, 88 x 35 x 46 mm
STACK = 0.035        # FC + 4-in-1 ESC, 30.5 mm stack
AVIONICS = 0.040     # RX + GPS + VTX + camera
WIRING = 0.030       # power leads, XT60, strap


def load(path):
    b = open(path, "rb").read()
    n = int.from_bytes(b[80:84], "little")
    dt = np.dtype([("n", "<3f4"), ("v", "<3f4", (3,)), ("a", "<u2")])
    return np.frombuffer(b, dtype=dt, count=n, offset=84)["v"].astype(float) / 1000.0


def mesh_props(tris):
    """Volume, centroid and per-unit-mass second moment (about origin) of a closed mesh."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    d = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = d.sum() / 6
    com = ((a + b + c) * d[:, None]).sum(0) / (24 * vol)
    S = np.empty((3, 3))
    for i in range(3):
        for j in range(3):
            S[i, j] = (d * (2 * (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j])
                            + a[:, i] * b[:, j] + a[:, j] * b[:, i] + a[:, i] * c[:, j]
                            + a[:, j] * c[:, i] + b[:, i] * c[:, j] + b[:, j] * c[:, i])).sum() / 120
    return vol, com, S / vol


def mesh_item(path, rho=0.0, fill=1.0, mass=None):
    vol, com, S = mesh_props(load(path))
    return (os.path.basename(path)[:-4], mass if mass else vol * rho * fill, com, S)


def box_item(name, mass, centre_mm, size_mm):
    c = np.array(centre_mm) / 1000.0
    return (name, mass, c, np.outer(c, c) + np.diag((np.array(size_mm) / 1000.0) ** 2 / 12))


def total(items):
    m = sum(i[1] for i in items)
    com = sum(i[1] * i[2] for i in items) / m
    S = sum(i[1] * i[3] for i in items) - m * np.outer(com, com)
    return m, com, np.trace(S) * np.eye(3) - S


def build03():
    p = f"{FRAMES}/Custom_Drone_Build_03/parts/"
    items = [mesh_item(p + "carried_over_from_02/Base.stl", PAHT_CF, 0.75),           # 6 mm, 40% gyroid
             mesh_item(p + "new_in_03/Chassis_with_pillar_pads.stl", PAHT_CF, 0.75)]  # 5 mm, 40% gyroid
    items += [mesh_item(f, PAHT_CF) for f in sorted(glob.glob(p + "new_in_03/Arm_plate_*.stl"))]
    items += [mesh_item(f, PAHT_CF) for f in sorted(glob.glob(p + "new_in_03/Pillar_*.stl"))]
    items += [mesh_item(f, mass=MOTOR) for f in sorted(glob.glob(p + "new_in_03/Motor_*.stl"))]
    motors = [(142.8, -143.2), (-143.2, -142.8), (-142.8, 143.2), (143.2, 142.8)]
    items += [box_item("prop+nut", PROP_NUT, (x, y, 72), (0, 0, 0)) for x, y in motors]
    items += [box_item("battery", BATTERY, (0, 0, 70), (88, 35, 46)),          # strapped on chassis
              box_item("stack", STACK, (0, 0, 20), (36, 36, 15)),              # between base and arms
              box_item("avionics", AVIONICS, (0, 0, 40), (60, 60, 20)),
              box_item("wiring", WIRING, (0, 0, 30), (100, 100, 20)),
              box_item("screws", 0.025, (0, 0, 35), (200, 200, 20))]
    # CAD is already REP-103 (front +X, Z up): identity
    return items, np.eye(3), motors, 66.7, 40.0


def report(name, items, R, motors, hub_z_mm, motor_base_mm):
    m, com, I = total(items)
    Ir = R @ I @ R.T
    assert abs(m - sum(i[1] for i in items)) < 1e-9
    assert np.allclose(Ir, Ir.T) and np.all(np.linalg.eigvalsh(Ir) > 0)
    assert Ir[2, 2] <= Ir[0, 0] + Ir[1, 1] + 1e-9          # triangle inequality
    print(f"==== {name}: {m * 1000:.0f} g, CoM (CAD mm) {np.round(com * 1000, 1)}")
    print(f'  <xacro:property name="mass_total" value="{m:.3f}"/>')
    print(f'  <xacro:property name="body_ixx" value="{Ir[0, 0]:.5f}"/>')
    print(f'  <xacro:property name="body_iyy" value="{Ir[1, 1]:.5f}"/>')
    print(f'  <xacro:property name="body_izz" value="{Ir[2, 2]:.5f}"/>')
    off = R @ -com
    print(f"  CAD-mesh visual origin (m): xyz=\"{off[0]:.4f} {off[1]:.4f} {off[2]:.4f}\"")
    for x, y in motors:
        r = R @ (np.array([x, y, hub_z_mm]) / 1000 - com)
        print(f"  rotor (ROS, from CoM): {r[0]:+.4f} {r[1]:+.4f} {r[2]:+.4f}"
              f"   motor base z {motor_base_mm / 1000 - com[2]:+.4f}")


if __name__ == "__main__":
    report("build03", *build03())
