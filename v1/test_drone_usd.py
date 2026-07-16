#!/usr/bin/env python3
r"""
test_drone_usd.py — inspect a STEP->OBJ->USD drone asset before Isaac Lab use.

Reports: hierarchy + mesh count, world bbox and per-part extents/centers, a scale
sanity check against the real Starling diagonal (0.322 m) — the raw asset scans
~10x too big, so this FAILS until fixed (that's the point) — and whether any
UsdPhysics is authored (a pure geometry conversion has none).

--physics wraps it in a single rigid body and drops it on a ground plane to
confirm it behaves as one body. --headless skips the viewport.

    python test_drone_usd.py --usd drone.usd
    python test_drone_usd.py --usd drone.usd --physics
"""

import argparse
import math
import os
import sys

# ---- CLI (parse BEFORE SimulationApp so we can pass headless) ---------------
parser = argparse.ArgumentParser(description="Inspect/test drone.usd in Isaac Sim")
parser.add_argument(
    "--usd",
    default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone.usd"),
    help="Path to the drone USD file (default: drone.usd next to this script)",
)
parser.add_argument("--headless", action="store_true", help="Run without a viewport")
parser.add_argument(
    "--physics",
    action="store_true",
    help="Add a rigid body + collision and drop the asset onto a ground plane",
)
parser.add_argument(
    "--steps", type=int, default=200, help="Physics steps to run in --physics mode"
)
# The real Starling 2 Max airframe diagonal, for the scale sanity check.
parser.add_argument("--expected-diagonal", type=float, default=0.322)
args = parser.parse_args()

if not os.path.isfile(args.usd):
    print(f"[FATAL] USD not found: {args.usd}")
    sys.exit(1)

# ---- Isaac / Omniverse imports MUST come after SimulationApp ----------------
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics, Gf, UsdLux  # noqa: E402


def banner(title: str) -> None:
    print("\n" + "=" * 68)
    print(f" {title}")
    print("=" * 68)


def world_bbox(stage, prim):
    """World-aligned bounding box (min, max) as Gf.Vec3d for a prim subtree."""
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return rng.GetMin(), rng.GetMax()


def main():
    # ---- 1. Open the asset onto a fresh stage --------------------------
    banner(f"OPENING  {args.usd}")
    ctx = omni.usd.get_context()
    # Open directly so we inspect the authored contents as-is.
    ctx.open_stage(args.usd)
    stage = ctx.get_stage()
    if stage is None:
        print("[FATAL] Failed to get stage after open_stage()")
        simulation_app.close()
        sys.exit(1)

    up = UsdGeom.GetStageUpAxis(stage)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    print(f"Default prim : {stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else '<none>'}")
    print(f"Up axis      : {up}")
    print(f"metersPerUnit: {mpu}   (1 unit = {mpu} m)")

    # ---- 2. Hierarchy + mesh / physics inventory -----------------------
    banner("PRIM INVENTORY")
    mesh_prims, xform_prims = [], []
    has_rigid = has_collision = has_mass = False
    for prim in stage.Traverse():
        t = prim.GetTypeName()
        if t == "Mesh":
            mesh_prims.append(prim)
        elif t in ("Xform", "Scope"):
            xform_prims.append(prim)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            has_rigid = True
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            has_collision = True
        if prim.HasAPI(UsdPhysics.MassAPI):
            has_mass = True

    print(f"Mesh prims   : {len(mesh_prims)}")
    print(f"Xform/Scope  : {len(xform_prims)}")
    print(f"RigidBodyAPI : {'YES' if has_rigid else 'no'}")
    print(f"CollisionAPI : {'YES' if has_collision else 'no'}")
    print(f"MassAPI      : {'YES' if has_mass else 'no'}")
    if not (has_rigid or has_collision or has_mass):
        print(">> Pure geometry conversion: NO physics authored. Expected for STEP->OBJ->USD.")

    # ---- 3. Whole-asset bounding box + scale sanity check ---------------
    banner("BOUNDING BOX  (world, meters)")
    default_prim = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    bmin, bmax = world_bbox(stage, default_prim)
    size = bmax - bmin
    # Convert to meters using metersPerUnit.
    size_m = Gf.Vec3d(size[0] * mpu, size[1] * mpu, size[2] * mpu)
    print(f"min   : ({bmin[0]:.4f}, {bmin[1]:.4f}, {bmin[2]:.4f})")
    print(f"max   : ({bmax[0]:.4f}, {bmax[1]:.4f}, {bmax[2]:.4f})")
    print(f"size  : ({size_m[0]:.4f}, {size_m[1]:.4f}, {size_m[2]:.4f}) m")

    # Diagonal in the horizontal plane (depends on up axis).
    if up == "Y":
        footprint = math.hypot(size_m[0], size_m[2])
    else:  # Z-up
        footprint = math.hypot(size_m[0], size_m[1])
    print(f"footprint diagonal : {footprint:.4f} m")
    print(f"expected (Starling): {args.expected_diagonal:.4f} m")
    ratio = footprint / args.expected_diagonal if args.expected_diagonal else float("nan")
    print(f"ratio measured/expected : {ratio:.2f}x")
    if not (0.5 <= ratio <= 2.0):
        print(f">> [SCALE WARNING] Asset is {ratio:.1f}x the real drone. "
              f"Fix the STEP export scale / propeller-pod placement before Isaac Lab.")
    else:
        print(">> Scale within 2x of expected. OK.")

    # ---- 3b. Per-top-level-part extents (spot misplaced parts) ---------
    banner("TOP-LEVEL PARTS  (center in meters)")
    root = default_prim
    children = [c for c in root.GetChildren() if c.IsA(UsdGeom.Xformable)]
    if not children:
        children = mesh_prims  # flat asset: report meshes directly
    rows = []
    for c in children:
        try:
            cmin, cmax = world_bbox(stage, c)
        except Exception:
            continue
        center = (cmin + cmax) * 0.5
        d = cmax - cmin
        rows.append((c.GetName(),
                     center[0] * mpu, center[1] * mpu, center[2] * mpu,
                     max(d) * mpu))
    # Sort by distance from origin so outliers surface at the top.
    rows.sort(key=lambda r: math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2), reverse=True)
    print(f"{'part':<20}{'cx':>9}{'cy':>9}{'cz':>9}{'maxdim':>9}")
    for name, cx, cy, cz, dim in rows[:30]:
        flag = "  <-- far from origin" if math.sqrt(cx*cx+cy*cy+cz*cz) > args.expected_diagonal else ""
        print(f"{name:<20}{cx:>9.3f}{cy:>9.3f}{cz:>9.3f}{dim:>9.3f}{flag}")

    # ---- 4. Optional physics drop test ---------------------------------
    if args.physics:
        banner("PHYSICS DROP TEST")
        from omni.isaac.core import World
        from omni.isaac.core.objects.ground_plane import GroundPlane

        world = World(stage_units_in_meters=1.0)
        GroundPlane(prim_path="/World/groundPlane", z_position=0.0)

        # Light so the viewport isn't black.
        dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        dome.CreateIntensityAttr(1000.0)

        # IMPORTANT: only add physics if the asset does NOT already have it.
        # Re-applying physics on an already-physics-enabled asset (e.g.
        # drone_physics.usd) nests a rigid body inside a rigid body and turns
        # raw meshes into triangle simulation shapes -> PhysX rejects them and
        # the asset free-falls through the floor. For those assets use
        # verify_physics.py instead.
        if has_rigid or has_collision or has_mass:
            print(">> Asset already has physics; NOT re-applying. "
                  "Use verify_physics.py for a proper drop test.")
        else:
            # Make the whole asset ONE rigid body with convex-hull collision.
            UsdPhysics.RigidBodyAPI.Apply(default_prim)
            UsdPhysics.CollisionAPI.Apply(default_prim)
            mass_api = UsdPhysics.MassAPI.Apply(default_prim)
            mass_api.CreateMassAttr(0.5)  # real takeoff weight ~500 g
            for m in mesh_prims:
                col = UsdPhysics.CollisionAPI.Apply(m)
                mcol = UsdPhysics.MeshCollisionAPI.Apply(m)
                mcol.CreateApproximationAttr().Set("convexHull")

        world.reset()
        xf = UsdGeom.Xformable(default_prim)
        for i in range(args.steps):
            world.step(render=not args.headless)
            if i % 40 == 0:
                m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                pos = m.ExtractTranslation()
                print(f"step {i:4d}  pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})")
        print(">> If it settled on the ground as a single body, collision/mass work.")

    banner("DONE")
    print("Inspection complete. Review the SCALE WARNING and TOP-LEVEL PARTS above.")
    if not args.headless:
        print("Viewport left open for visual inspection. Ctrl-C to quit.")
        try:
            while simulation_app.is_running():
                simulation_app.update()
        except KeyboardInterrupt:
            pass
    simulation_app.close()


if __name__ == "__main__":
    main()
