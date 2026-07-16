#!/usr/bin/env python3
r"""
add_drone_physics.py — wrap drone_fixed.usd as a single physics rigid body.

Writes drone_physics.usd:
  /World/Drone            RigidBodyAPI + MassAPI (--mass, default 0.5 kg)
  /World/Drone/visual     reference to drone_fixed.usd (visual only)
  /World/Drone/collision  collider by --collision:
      box    (default) one Cube at the asset bbox — fast, robust
      hull   convexHull per mesh — tighter, slow to cook (~1877 meshes)
      decomp convexDecomposition per mesh — most accurate, slowest

Single body (stage A); v2 splits into base_link + 4 rotors (--list-corners finds
the prop clusters). With no explicit inertia, PhysX derives it from the collider.

    python add_drone_physics.py --in drone_fixed.usd --out drone_physics.usd
    python add_drone_physics.py --list-corners        # report prop clusters only
"""

import argparse
import math
import os
import sys

# ---- CLI BEFORE SimulationApp ----------------------------------------------
here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--in", dest="src", default="drone_fixed.usd")
ap.add_argument("--out", dest="dst", default="drone_physics.usd")
ap.add_argument("--mass", type=float, default=0.5, help="Total mass in kg (default 0.5)")
ap.add_argument("--collision", choices=["box", "hull", "decomp"], default="box")
ap.add_argument("--list-corners", action="store_true",
                help="Only analyze/print the 4 propeller clusters, do not write output")
args = ap.parse_args()

if not os.path.isfile(args.src):
    print(f"[FATAL] input not found: {args.src}")
    sys.exit(1)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf, PhysxSchema  # noqa: E402


def bbox_of(stage, prim):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              includedPurposes=[UsdGeom.Tokens.default_,
                                                UsdGeom.Tokens.render],
                              useExtentsHint=True)
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


def list_corners():
    """Cluster mesh centers into the 4 quadrants (X/Y) to locate propellers."""
    stage = Usd.Stage.Open(args.src)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             includedPurposes=[UsdGeom.Tokens.default_,
                                               UsdGeom.Tokens.render])
    quads = {"+X+Y": [], "+X-Y": [], "-X+Y": [], "-X-Y": []}
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        mn, mx = cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMin(), \
                 cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMax()
        c = (mn + mx) * 0.5
        key = ("+X" if c[0] >= 0 else "-X") + ("+Y" if c[1] >= 0 else "-Y")
        # Only count meshes clearly OUT at a corner (not central electronics).
        if math.hypot(c[0], c[1]) > 0.08:  # >8 cm from center = likely arm/prop
            quads[key].append((prim.GetPath().pathString, c[0], c[1], c[2]))
    print("\n=== Propeller-cluster candidates (meshes >8cm from center) ===")
    for k, items in quads.items():
        if not items:
            print(f"{k}: (none)")
            continue
        cx = sum(i[1] for i in items) / len(items)
        cy = sum(i[2] for i in items) / len(items)
        cz = sum(i[3] for i in items) / len(items)
        print(f"{k}: {len(items):4d} meshes  cluster-center=({cx:.3f},{cy:.3f},{cz:.3f})")
    print("Use these centers as the rotor_0..3 frame origins when you split the body.")


def main():
    if args.list_corners:
        list_corners()
        return

    if os.path.exists(args.dst):
        os.remove(args.dst)
    stage = Usd.Stage.CreateNew(args.dst)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # Rigid body root.
    drone = UsdGeom.Xform.Define(stage, "/World/Drone")
    stage.SetDefaultPrim(world.GetPrim())
    rb = UsdPhysics.RigidBodyAPI.Apply(drone.GetPrim())
    rb.CreateRigidBodyEnabledAttr(True)
    massapi = UsdPhysics.MassAPI.Apply(drone.GetPrim())
    massapi.CreateMassAttr(args.mass)
    # Center of mass at body origin (asset already centered ~origin).
    massapi.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))

    # Visual: reference the fixed geometry (no collision on these).
    visual = stage.DefinePrim("/World/Drone/visual")
    src_default = Usd.Stage.Open(args.src).GetDefaultPrim().GetPath()
    visual.GetReferences().AddReference(args.src, src_default)

    # Collision.
    if args.collision == "box":
        # Size a box collider to the asset bounding box.
        mn, mx = bbox_of(stage, visual)
        size = mx - mn
        center = (mn + mx) * 0.5
        cube = UsdGeom.Cube.Define(stage, "/World/Drone/collision")
        cube.CreateSizeAttr(1.0)
        # Scale unit cube to bbox, translate to bbox center.
        cube.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], center[2]))
        cube.AddScaleOp().Set(Gf.Vec3f(size[0], size[1], size[2]))
        UsdGeom.Imageable(cube).CreateVisibilityAttr("invisible")  # collider not drawn
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        print(f"[coll ] box collider size=({size[0]:.3f},{size[1]:.3f},{size[2]:.3f}) m")
    else:
        approx = "convexHull" if args.collision == "hull" else "convexDecomposition"
        n = 0
        for prim in stage.Traverse():
            if prim.GetTypeName() != "Mesh":
                continue
            UsdPhysics.CollisionAPI.Apply(prim)
            mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mc.CreateApproximationAttr().Set(approx)
            n += 1
        print(f"[coll ] {approx} applied to {n} meshes "
              f"(heavy: expect long cook / slow sim)")

    stage.GetRootLayer().Save()
    print(f"[write] {args.dst}")
    print(f"[done ] mass={args.mass} kg  collision={args.collision}")
    print("Verify: python test_drone_usd.py --usd drone_physics.usd --physics")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
