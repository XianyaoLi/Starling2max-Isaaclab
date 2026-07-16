#!/usr/bin/env python3
r"""
add_drone_frames.py — add sensor/rotor frames, write starling2max.usd.

Frame positions come from named meshes' world bounding box (ComputeWorldBound),
not their Translate: STEP->OBJ->USD baked positions into vertices, so every
pivot reads as origin.

Starling 2 Max (C29) layout, verified in viewport:
  Front module @ -Y  => FORWARD = -Y : Body1_75/77 (tracking+hires, bigger=hires);
                        ToF has no mesh, offset from Body1_77 (--tof-ref/--tof-offset)
  Down module  @ +Y facing -Z        : Body1_95/93 (tracking+hires)
  Motors                             : Body1_4/2/6/8 (rotor thrust points)

Frame local +X aims along the view dir (forward: RotateZ; down: RotateY=90).
Sensors carry drone:sensorType + drone:viewDir; rotors carry drone:spinDir (+-1).

    python add_drone_frames.py --in drone_physics.usd --out starling2max.usd
    # override names: --front-meshes / --down-meshes / --rotor-meshes / --tof-ref
"""

import argparse
import math
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--in", dest="src", default="drone_physics.usd")
ap.add_argument("--out", dest="dst", default="starling2max.usd")
ap.add_argument("--front-meshes", nargs="+", default=["Body1_75", "Body1_77"])
ap.add_argument("--down-meshes", nargs="+", default=["Body1_95", "Body1_93"])
ap.add_argument("--rotor-meshes", nargs="+",
                default=["Body1_4", "Body1_2", "Body1_6", "Body1_8"])
ap.add_argument("--tof-ref", default="Body1_77",
                help="ToF placed at this mesh's center + --tof-offset")
ap.add_argument("--tof-offset", type=float, nargs=3, default=[-0.018, 0.0, 0.0],
                metavar=("DX", "DY", "DZ"),
                help="ToF lateral offset from --tof-ref (default -X, per viewport check)")
ap.add_argument("--surface-margin", type=float, default=0.004,
                help="Extra gap (m) beyond the sensor surface so the camera is not "
                     "occluded by its own housing (default 4mm)")
args = ap.parse_args()

if not os.path.isfile(args.src):
    print(f"[FATAL] input not found: {args.src}")
    sys.exit(1)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Sdf, Gf  # noqa: E402


def build_center_lookup(stage):
    """Map mesh prim NAME -> (world_center Vec3d, world_size Vec3d)."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              includedPurposes=[UsdGeom.Tokens.default_,
                                                UsdGeom.Tokens.render])
    lut = {}
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        lut[prim.GetName()] = ((mn + mx) * 0.5, mx - mn)
    return lut


def lookup(lut, name):
    """Fetch by name, tolerant of - vs _ ."""
    for cand in (name, name.replace("-", "_"), name.replace("_", "-")):
        if cand in lut:
            return lut[cand]
    print(f"[WARN] mesh '{name}' not found; using origin")
    return (Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 0, 0))


def vol(size):
    return size[0] * size[1] * size[2]


def add_frame(stage, path, pos, rot_axis=None, rot_deg=0.0, attrs=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    if rot_axis == "Y" and rot_deg:
        xf.AddRotateYOp().Set(float(rot_deg))
    elif rot_axis == "Z" and rot_deg:
        xf.AddRotateZOp().Set(float(rot_deg))
    prim = xf.GetPrim()
    for name, (tc, val) in (attrs or {}).items():
        prim.CreateAttribute(name, tc).Set(val)
    return xf


def main():
    src_stage = Usd.Stage.Open(args.src)
    lut = build_center_lookup(src_stage)
    print(f"[info ] {len(lut)} named meshes indexed")

    if os.path.exists(args.dst):
        os.remove(args.dst)
    stage = Usd.Stage.CreateNew(args.dst)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    world.GetPrim().GetReferences().AddReference(
        args.src, src_stage.GetDefaultPrim().GetPath())

    D = "/World/Drone"
    S = Sdf.ValueTypeNames

    # ---- rotors from motor meshes ----
    print("\n=== rotor frames (from motor meshes) ===")
    for i, mname in enumerate(args.rotor_meshes):
        c, _ = lookup(lut, mname)
        spin = 1 if (c[0] * c[1] >= 0) else -1
        add_frame(stage, f"{D}/rotor_{i}", c,
                  attrs={"drone:spinDir": (S.Int, spin),
                         "drone:srcMesh": (S.String, mname)})
        print(f"rotor_{i} <- {mname}: ({c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f})  "
              f"spinDir={spin:+d}  r={math.hypot(c[0],c[1]):.4f}")

    # ---- forward heading from the front module ----
    front_centers = [lookup(lut, n)[0] for n in args.front_meshes]
    fanchor = Gf.Vec3d(sum(c[0] for c in front_centers)/len(front_centers),
                       sum(c[1] for c in front_centers)/len(front_centers),
                       sum(c[2] for c in front_centers)/len(front_centers))
    fwd_yaw = math.degrees(math.atan2(fanchor[1], fanchor[0]))  # +X rotated by this -> heading
    print(f"\n[info ] forward heading = {fwd_yaw:.1f} deg (module at "
          f"({fanchor[0]:+.3f},{fanchor[1]:+.3f}))  -> local +X points forward")

    # ---- assign tracking/hires by size (bigger = hires) ----
    def assign(meshes):
        items = [(n, lookup(lut, n)) for n in meshes]
        items.sort(key=lambda it: vol(it[1][1]), reverse=True)  # biggest first
        hires = items[0]
        tracking = items[-1]
        return tracking, hires

    # ---- push each camera OUT to the sensor surface (avoid self-occlusion) ----
    margin = args.surface_margin
    hx = math.cos(math.radians(fwd_yaw))   # forward heading unit vector (xy)
    hy = math.sin(math.radians(fwd_yaw))

    def to_surface(center, size, vdir):
        c = Gf.Vec3d(center[0], center[1], center[2])
        if vdir == "forward":
            ext = abs(hx) * size[0] + abs(hy) * size[1]   # extent along heading
            c[0] += hx * (ext * 0.5 + margin)
            c[1] += hy * (ext * 0.5 + margin)
        elif vdir == "down":
            c[2] -= (size[2] * 0.5 + margin)              # push down to belly surface
        return c

    print("\n=== camera frames (pushed to sensor surface; VERIFY IN VIEWPORT) ===")
    ft, fh = assign(args.front_meshes)   # each = (name, (center, size))
    dt, dh = assign(args.down_meshes)
    cam_specs = [
        ("cam_tracking_front", ft, "Z", fwd_yaw, "tracking", "forward"),
        ("cam_hires_front",    fh, "Z", fwd_yaw, "hires",    "forward"),
        ("cam_tracking_down",  dt, "Y", 90.0,    "tracking", "down"),
        ("cam_hires_down",     dh, "Y", 90.0,    "hires",    "down"),
    ]
    for name, item, axis, deg, stype, vdir in cam_specs:
        src, (center, size) = item
        pos = to_surface(center, size, vdir)
        add_frame(stage, f"{D}/{name}", pos, rot_axis=axis, rot_deg=deg,
                  attrs={"drone:sensorType": (S.String, stype),
                         "drone:viewDir": (S.String, vdir),
                         "drone:srcMesh": (S.String, src)})
        print(f"{name:<20} <- {src:<9} ({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f})  {vdir}")

    # ---- ToF: lateral offset from a front cam, then surface push ----
    tref, tsize = lookup(lut, args.tof_ref)
    base = Gf.Vec3d(tref[0] + args.tof_offset[0],
                    tref[1] + args.tof_offset[1],
                    tref[2] + args.tof_offset[2])
    tof = to_surface(base, tsize, "forward")
    add_frame(stage, f"{D}/tof_link", tof, rot_axis="Z", rot_deg=fwd_yaw,
              attrs={"drone:sensorType": (S.String, "tof_depth"),
                     "drone:viewDir": (S.String, "forward")})
    print(f"{'tof_link':<20} <- {args.tof_ref}+off ({tof[0]:+.4f},{tof[1]:+.4f},{tof[2]:+.4f})  forward")

    stage.GetRootLayer().Save()
    print(f"\n[write] {args.dst}")
    print("NOTE: cameras pushed +{:.0f}mm beyond each sensor surface to avoid self-".format(margin*1000))
    print("      occlusion; set a small near-clip on the CameraCfg too. ToF on -X side.")
    print("Verify: python verify_physics.py --usd starling2max.usd --headless")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
