#!/usr/bin/env python3
r"""
fix_drone_usd.py — fix units / up-axis for Isaac Lab (meters, Z-up).

The STEP->OBJ->USD asset is authored in mm but tagged metersPerUnit=0.01, so it
renders 10x too big, and it is Y-up. This writes a NEW file that wraps the asset
under a root Xform (scale 0.001 mm->m, +90 deg about X for Y-up->Z-up) and sets
metersPerUnit=1.0, upAxis=Z. drone.usd is never modified.

(pxr comes from the Isaac runtime, so we boot a headless SimulationApp before
importing it.)

    python fix_drone_usd.py --in drone.usd --out drone_fixed.usd
    python fix_drone_usd.py --in drone.usd --keep-up     # leave Y-up
"""

import argparse
import os
import sys

# ---- CLI (parse BEFORE SimulationApp) --------------------------------------
here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--in", dest="src", default="drone.usd")
ap.add_argument("--out", dest="dst", default="drone_fixed.usd")
ap.add_argument("--scale", type=float, default=0.001,
                help="Uniform scale applied to geometry (0.001 = mm->m). Default 0.001")
ap.add_argument("--keep-up", action="store_true",
                help="Keep the original Y-up (skip the Y->Z rotation)")
args = ap.parse_args()

if not os.path.isfile(args.src):
    print(f"[FATAL] input not found: {args.src}")
    sys.exit(1)

# ---- Bootstrap the Isaac runtime so `pxr` is importable ---------------------
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf  # noqa: E402


def main():
    # Open the original (read-only reference target); we author on a NEW layer
    # so drone.usd is never modified.
    print(f"[open ] {args.src}")
    src_stage = Usd.Stage.Open(args.src)
    src_default = src_stage.GetDefaultPrim()
    if not src_default:
        print("[FATAL] source has no defaultPrim; cannot reference cleanly.")
        simulation_app.close()
        sys.exit(1)
    src_default_path = src_default.GetPath()
    print(f"[info ] source defaultPrim   = {src_default_path}")
    print(f"[info ] source metersPerUnit = {UsdGeom.GetStageMetersPerUnit(src_stage)}")
    print(f"[info ] source upAxis        = {UsdGeom.GetStageUpAxis(src_stage)}")

    # Fresh stage that references the original under a fix-up Xform.
    if os.path.exists(args.dst):
        os.remove(args.dst)
    stage = Usd.Stage.CreateNew(args.dst)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)          # target: meters
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)    # target: Z-up

    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())

    fix = UsdGeom.Xform.Define(stage, "/World/drone")
    # Order matters: rotate first (Y-up -> Z-up), then scale.
    if not args.keep_up:
        fix.AddRotateXOp().Set(90.0)   # +90 about X maps +Y -> +Z
    fix.AddScaleOp().Set(Gf.Vec3f(args.scale, args.scale, args.scale))

    # Reference the original asset's default prim under the fix-up xform.
    ref_prim = stage.DefinePrim("/World/drone/asset")
    ref_prim.GetReferences().AddReference(args.src, src_default_path)

    stage.GetRootLayer().Save()

    print(f"[write] {args.dst}")
    print(f"[done ] scale={args.scale}  "
          f"upAxis={'Y (kept)' if args.keep_up else 'Z'}  metersPerUnit=1.0")
    print("Next: python test_drone_usd.py --usd drone_fixed.usd --headless")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
