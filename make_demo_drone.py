#!/usr/bin/env python3
r"""
make_demo_drone.py

Generate a GENERIC placeholder quadrotor USD (demo_drone.usd) from primitives.
It contains NO proprietary CAD -- 100% original geometry -- but follows the same
conventions the Starling toolchain produces, so the whole pipeline (especially
test_starling_functions.py) runs out-of-the-box with no CAD required.

Conventions reproduced:
  - Stage: meters, Z-up, defaultPrim /World
  - /World/Drone : single rigid body (RigidBodyAPI + MassAPI, 0.5 kg) with a box
    collider (visible=off) and primitive visuals (body + 4 prop disks + module)
  - rotor_0..3 : Xform thrust-point frames, each with attr  drone:spinDir (+-1)
                 (X-quad diagonal pairs same sign)
  - tof_link / cam_tracking_front / cam_hires_front / cam_tracking_down /
    cam_hires_down : Xform sensor frames with drone:sensorType + drone:viewDir.
    FORWARD = -Y (front module), DOWN = -Z, matching the real layout.

Run (full Isaac Lab):
    python make_demo_drone.py --out demo_drone.usd
Then:
    python test_starling_functions.py --usd demo_drone.usd --out demo_out
"""

import argparse
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.join(here, "demo_drone.usd"))
ap.add_argument("--mass", type=float, default=0.5)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf  # noqa: E402


def box(stage, path, size_xyz, pos=(0, 0, 0), color=(0.6, 0.6, 0.6), visible=True):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in pos]))
    c.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size_xyz]))
    c.CreateDisplayColorAttr([tuple(float(v) for v in color)])
    if not visible:
        UsdGeom.Imageable(c).CreateVisibilityAttr("invisible")
    return c


def disk(stage, path, radius, height, pos, color=(0.15, 0.15, 0.18)):
    cy = UsdGeom.Cylinder.Define(stage, path)
    cy.CreateRadiusAttr(float(radius))
    cy.CreateHeightAttr(float(height))
    cy.CreateAxisAttr("Z")
    cy.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in pos]))
    cy.CreateDisplayColorAttr([tuple(float(v) for v in color)])
    return cy


def frame(stage, path, pos, attrs, rot_axis=None, rot_deg=0.0):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in pos]))
    if rot_axis == "Z" and rot_deg:
        xf.AddRotateZOp().Set(float(rot_deg))
    elif rot_axis == "Y" and rot_deg:
        xf.AddRotateYOp().Set(float(rot_deg))
    p = xf.GetPrim()
    for name, (tc, val) in attrs.items():
        p.CreateAttribute(name, tc).Set(val)
    return xf


def main():
    if os.path.exists(args.out):
        os.remove(args.out)
    stage = Usd.Stage.CreateNew(args.out)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    D = "/World/Drone"
    S = Sdf.ValueTypeNames
    drone = UsdGeom.Xform.Define(stage, D)
    UsdPhysics.RigidBodyAPI.Apply(drone.GetPrim()).CreateRigidBodyEnabledAttr(True)
    mass = UsdPhysics.MassAPI.Apply(drone.GetPrim())
    mass.CreateMassAttr(args.mass)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))

    # ---- visuals ----
    box(stage, f"{D}/visual/body", (0.18, 0.14, 0.05), (0, 0, 0), (0.20, 0.22, 0.28))
    # front sensor module (at -Y = forward)
    box(stage, f"{D}/visual/sensor_module", (0.05, 0.035, 0.045),
        (0.0, -0.085, 0.02), (0.05, 0.05, 0.06))

    rotors = [("rotor_0", (0.13, 0.09)), ("rotor_1", (0.13, -0.09)),
              ("rotor_2", (-0.13, 0.09)), ("rotor_3", (-0.13, -0.09))]
    for name, (x, y) in rotors:
        # arm (thin box from center to motor)
        box(stage, f"{D}/visual/arm_{name}", (abs(x), 0.015, 0.012),
            (x / 2, y / 2, 0.0), (0.12, 0.12, 0.14))
        # propeller disk (visual only)
        disk(stage, f"{D}/visual/prop_{name}", 0.085, 0.006, (x, y, 0.028))
        spin = 1 if (x * y >= 0) else -1
        frame(stage, f"{D}/{name}", (x, y, 0.028),
              {"drone:spinDir": (S.Int, spin)})

    # ---- sensor frames (forward = -Y, down = -Z) ----
    cams = [
        ("cam_tracking_front", (0.015, -0.095, 0.02), "Z", -90.0, "tracking", "forward"),
        ("cam_hires_front",    (-0.015, -0.095, 0.02), "Z", -90.0, "hires",    "forward"),
        ("tof_link",           (0.0, -0.095, 0.005),  "Z", -90.0, "tof_depth", "forward"),
        ("cam_tracking_down",  (0.015, 0.0, -0.032),  "Y", 90.0,  "tracking",  "down"),
        ("cam_hires_down",     (-0.015, 0.0, -0.032), "Y", 90.0,  "hires",     "down"),
    ]
    for name, pos, axis, deg, stype, vdir in cams:
        frame(stage, f"{D}/{name}", pos,
              {"drone:sensorType": (S.String, stype),
               "drone:viewDir": (S.String, vdir)},
              rot_axis=axis, rot_deg=deg)

    # ---- collision: one invisible box ----
    col = box(stage, f"{D}/collision", (0.30, 0.24, 0.10), (0, 0, 0),
              (1, 1, 1), visible=False)
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())

    stage.GetRootLayer().Save()
    print(f"[write] {args.out}")
    print("Try: python test_starling_functions.py --usd demo_drone.usd --out demo_out")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
