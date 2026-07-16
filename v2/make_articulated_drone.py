#!/usr/bin/env python3
r"""
make_articulated_drone.py (v2) — articulated placeholder quadrotor USD.

base_link + 4 rotor bodies, each on a Z-axis revolute joint with a velocity
drive so the props actually spin (diagonal pairs counter-rotate). 100% original
primitive geometry, no CAD.

  /World/Drone     ArticulationRootAPI
    base_link      RigidBody + Mass + box collider (body, arms, sensor frames)
    rotor_0..3     RigidBody + Mass (hub + blade), revolute joint + spin drive

    python v2/make_articulated_drone.py --out articulated_drone.usd
    python v2/test_articulated.py --usd articulated_drone.usd
"""

import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--out", default="articulated_drone.usd")
ap.add_argument("--base-mass", type=float, default=0.42)
ap.add_argument("--rotor-mass", type=float, default=0.02)
ap.add_argument("--spin-dps", type=float, default=900.0, help="prop spin speed (deg/s)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf  # noqa: E402


def box(stage, path, size, pos=(0, 0, 0), color=(0.6, 0.6, 0.6), visible=True):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in pos]))
    c.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
    c.CreateDisplayColorAttr([tuple(float(v) for v in color)])
    if not visible:
        UsdGeom.Imageable(c).CreateVisibilityAttr("invisible")
    return c


def cyl(stage, path, radius, height, pos=(0, 0, 0), color=(0.15, 0.15, 0.18)):
    cy = UsdGeom.Cylinder.Define(stage, path)
    cy.CreateRadiusAttr(float(radius))
    cy.CreateHeightAttr(float(height))
    cy.CreateAxisAttr("Z")
    cy.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in pos]))
    cy.CreateDisplayColorAttr([tuple(float(v) for v in color)])
    return cy


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
    UsdPhysics.ArticulationRootAPI.Apply(drone.GetPrim())

    # ---------- base_link ----------
    base = UsdGeom.Xform.Define(stage, f"{D}/base_link")
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim()).CreateRigidBodyEnabledAttr(True)
    bm = UsdPhysics.MassAPI.Apply(base.GetPrim())
    bm.CreateMassAttr(args.base_mass)
    bm.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    box(stage, f"{D}/base_link/body", (0.18, 0.14, 0.05), (0, 0, 0), (0.20, 0.22, 0.28))
    box(stage, f"{D}/base_link/nose", (0.05, 0.035, 0.04), (0, -0.085, 0.02), (0.05, 0.05, 0.06))
    col = box(stage, f"{D}/base_link/collision", (0.30, 0.24, 0.10), (0, 0, 0),
              (1, 1, 1), visible=False)
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())

    # sensor frames on base_link (forward = -Y, down = -Z), same as make_demo_drone
    cams = [
        ("tof_link",           (0.0, -0.095, 0.005),  "Z", -90.0, "tof_depth", "forward"),
        ("cam_tracking_front", (0.015, -0.095, 0.02), "Z", -90.0, "tracking",  "forward"),
        ("cam_hires_front",    (-0.015, -0.095, 0.02), "Z", -90.0, "hires",    "forward"),
        ("cam_tracking_down",  (0.015, 0.0, -0.032),  "Y", 90.0,  "tracking",  "down"),
        ("cam_hires_down",     (-0.015, 0.0, -0.032), "Y", 90.0,  "hires",     "down"),
    ]
    for nm, pos, axis, deg, stype, vdir in cams:
        fr = UsdGeom.Xform.Define(stage, f"{D}/base_link/{nm}")
        fr.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in pos]))
        (fr.AddRotateYOp() if axis == "Y" else fr.AddRotateZOp()).Set(float(deg))
        fr.GetPrim().CreateAttribute("drone:sensorType", S.String).Set(stype)
        fr.GetPrim().CreateAttribute("drone:viewDir", S.String).Set(vdir)

    # ---------- 4 rotors + revolute joints ----------
    rotors = [("rotor_0", (0.13, 0.09), 1), ("rotor_1", (0.13, -0.09), -1),
              ("rotor_2", (-0.13, 0.09), -1), ("rotor_3", (-0.13, -0.09), 1)]
    for name, (x, y), spin in rotors:
        z = 0.03
        rot = UsdGeom.Xform.Define(stage, f"{D}/{name}")
        rot.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        UsdPhysics.RigidBodyAPI.Apply(rot.GetPrim()).CreateRigidBodyEnabledAttr(True)
        rm = UsdPhysics.MassAPI.Apply(rot.GetPrim())
        rm.CreateMassAttr(args.rotor_mass)
        rot.GetPrim().CreateAttribute("drone:spinDir", S.Int).Set(spin)
        # hub + a 2-blade prop (thin long box so the spin is visible)
        cyl(stage, f"{D}/{name}/hub", 0.02, 0.02, (0, 0, 0))
        blade = box(stage, f"{D}/{name}/blade", (0.16, 0.012, 0.005), (0, 0, 0.012),
                    (0.1, 0.6, 1.0) if spin > 0 else (1.0, 0.5, 0.1))

        joint = UsdPhysics.RevoluteJoint.Define(stage, f"{D}/{name}_joint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(f"{D}/base_link")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(f"{D}/{name}")])
        joint.CreateAxisAttr("Z")
        joint.CreateLocalPos0Attr(Gf.Vec3f(x, y, z))     # joint in base_link frame
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))     # joint in rotor frame
        # velocity drive -> the prop spins at spin * spin-dps
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)                    # 0 stiffness = velocity mode
        drive.CreateDampingAttr(500.0)
        drive.CreateTargetVelocityAttr(float(spin) * args.spin_dps)   # deg/s
        drive.CreateMaxForceAttr(1.0e4)

    stage.GetRootLayer().Save()
    print(f"[write] {args.out}")
    print("Articulation: base_link + 4 revolute-jointed rotors (velocity-driven spin).")
    print("Next: python v2/test_articulated.py --usd", args.out)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
