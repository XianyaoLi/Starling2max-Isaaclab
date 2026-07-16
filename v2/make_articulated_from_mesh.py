#!/usr/bin/env python3
r"""
make_articulated_from_mesh.py (v2) — articulate the single-body starling2max.usd.

Flatten the asset, move each prop's spinning meshes (blade + top cap) into an
independent rigid body, and join it to the base with a Z-axis revolute joint
driven to spin. Body/arms/motors/cameras stay in the base link. Each prop = the
blade + cap nearest a rotor frame; motors are left behind.

    python v2/make_articulated_from_mesh.py --in starling2max.usd --out starling2max_v2.usd
    python v2/test_articulated.py --usd starling2max_v2.usd
"""

import argparse
import math
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--in", dest="src", default="starling2max.usd")
ap.add_argument("--out", default="starling2max_v2.usd")
ap.add_argument("--blades", nargs="+",
                default=["Body1_147", "Body1_148", "Body1_144", "Body1_145"])
ap.add_argument("--caps", nargs="+",
                default=["Body1_3", "Body1_5", "Body1_7", "Body1_9"])
ap.add_argument("--spin-dps", type=float, default=900.0)
ap.add_argument("--base-mass", type=float, default=0.47)
ap.add_argument("--prop-mass", type=float, default=0.008)
args = ap.parse_args()

if not os.path.isfile(args.src):
    print(f"[FATAL] not found: {args.src}")
    sys.exit(1)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf  # noqa: E402


def variants(names):
    s = set()
    for n in names:
        s.add(n); s.add(n.replace("-", "_")); s.add(n.replace("_", "-"))
    return s


def main():
    src = Usd.Stage.Open(os.path.abspath(args.src))
    if src is None:
        print("[FATAL] could not open source"); return
    flat = src.Flatten()                       # self-contained layer (bakes references)
    stage = Usd.Stage.Open(flat)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())

    def center(prim):
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        c = (r.GetMin() + r.GetMax()) * 0.5
        return (float(c[0]), float(c[1]), float(c[2]))

    # locate the named blade/cap prims + their world centers
    def locate(names):
        want = variants(names)
        found = {}
        for prim in stage.Traverse():
            if prim.GetName() in want and prim.GetName() not in found:
                # world transform, so we can preserve placement after reparenting
                M = xcache.GetLocalToWorldTransform(prim)
                found[prim.GetName()] = (prim.GetPath().pathString, center(prim), M)
        return found
    blades = locate(args.blades)
    caps = locate(args.caps)
    print(f"[info] blades found: {list(blades)}")
    print(f"[info] caps found:   {list(caps)}")

    # rotor frame positions + spin sign (existing rotor_0..3 with drone:spinDir)
    rotors = []
    for prim in stage.Traverse():
        sd = prim.GetAttribute("drone:spinDir")
        if sd and sd.IsValid() and sd.Get() is not None:
            t = xcache.GetLocalToWorldTransform(prim).ExtractTranslation()
            rotors.append(((float(t[0]), float(t[1]), float(t[2])), int(sd.Get())))
    print(f"[info] {len(rotors)} rotor frames")

    def nearest(items, pos):
        best, bd = None, 1e9
        for nm, (path, c, M) in items.items():
            d = (c[0]-pos[0])**2 + (c[1]-pos[1])**2
            if d < bd:
                bd, best = d, (nm, path, M)
        return best

    # group: each rotor gets the nearest blade + nearest cap
    groups = []                                # (rotor_pos, spin, [(name,path,M),...])
    for pos, spin in rotors:
        b = nearest(blades, pos)
        c = nearest(caps, pos)
        members = [x for x in (b, c) if x]     # each = (name, path, world_matrix)
        groups.append((pos, spin, members))
        print(f"[group] rotor({pos[0]:+.3f},{pos[1]:+.3f}) spin={spin:+d} <- "
              f"{[m[0] for m in members]}")

    # create the prop bodies, then reparent the spinning meshes + collision
    for i in range(len(groups)):
        stage.DefinePrim(f"/World/Drone/prop_{i}", "Xform")

    edit = Sdf.BatchNamespaceEdit()
    for i, (_, _, members) in enumerate(groups):
        for name, path, _M in members:
            edit.Add(path, f"/World/Drone/prop_{i}/{name}")
    if stage.GetPrimAtPath("/World/Drone/collision"):
        edit.Add("/World/Drone/collision", "/World/Drone/visual/collision")
    if not flat.Apply(edit):
        print("[FATAL] namespace edit (reparent) failed"); return

    # preserve each moved mesh's placement: set its local transform to the world
    # transform it had (prop_i is identity, so local == old world -> stays put)
    for i, (_, _, members) in enumerate(groups):
        for name, _path, M in members:
            moved = stage.GetPrimAtPath(f"/World/Drone/prop_{i}/{name}")
            if moved and moved.IsValid():
                UsdGeom.Xformable(moved).MakeMatrixXform().Set(M)

    # physics: articulation root on /World/Drone, base = visual, props spin
    drone = stage.GetPrimAtPath("/World/Drone")
    for schema in ("PhysicsRigidBodyAPI", "PhysicsMassAPI"):
        try:
            drone.RemoveAppliedSchema(schema)
        except Exception:
            pass
    UsdPhysics.ArticulationRootAPI.Apply(drone)

    base = stage.GetPrimAtPath("/World/Drone/visual")
    UsdPhysics.RigidBodyAPI.Apply(base).CreateRigidBodyEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(base).CreateMassAttr(args.base_mass)

    for i, (pos, spin, _) in enumerate(groups):
        prop = stage.GetPrimAtPath(f"/World/Drone/prop_{i}")
        UsdPhysics.RigidBodyAPI.Apply(prop).CreateRigidBodyEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(prop).CreateMassAttr(args.prop_mass)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"/World/Drone/prop_{i}_joint")
        j.CreateBody0Rel().SetTargets([Sdf.Path("/World/Drone/visual")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"/World/Drone/prop_{i}")])
        j.CreateAxisAttr("Z")
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos))       # base frame == world (identity)
        j.CreateLocalPos1Attr(Gf.Vec3f(*pos))       # prop frame == world (identity)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(500.0)
        d.CreateTargetVelocityAttr(float(spin) * args.spin_dps)
        d.CreateMaxForceAttr(1.0e4)

    if os.path.exists(args.out):
        os.remove(args.out)
    flat.Export(args.out)
    print(f"[write] {args.out}")
    print("Base = /World/Drone/visual ; spinning props = /World/Drone/prop_0..N")
    print("Test:  python v2/test_articulated.py --usd", args.out)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
