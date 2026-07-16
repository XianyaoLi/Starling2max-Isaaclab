#!/usr/bin/env python3
r"""
fix_drone_usd.py

Fix the unit / up-axis metadata on the STEP->OBJ->USD drone asset so it is
usable in Isaac Lab (which works in METERS, Z-up).

Diagnosis (from test_drone_usd.py):
  - Coordinates are authored in MILLIMETERS (bbox +-178 units, real drone
    tip-to-tip ~0.5 m -> 178 units == 178 mm).
  - But metersPerUnit was set to 0.01 (cm), so everything renders 10x too big.
  - Stage is Y-up; Isaac Lab/Sim convention is Z-up.

What this does (writes a NEW file, never touches drone.usd):
  1. Wraps the asset under a root Xform with:
        - scale  = --scale   (default 0.001: millimeters -> meters)
        - rotate = +90 deg about X to convert Y-up -> Z-up (unless --keep-up)
  2. Sets stage metersPerUnit = 1.0 (meters) and upAxis = Z.
  3. Saves to --out (default drone_fixed.usd).

NOTE: pxr (USD) in a full Isaac Lab install is provided by the Isaac runtime,
which is bootstrapped by SimulationApp. So we start a HEADLESS SimulationApp
first (exactly like test_drone_usd.py), THEN import pxr. That is why a plain
`from pxr import ...` at module top failed with ModuleNotFoundError.

Run (full Isaac Lab):
    python fix_drone_usd.py --in drone.usd --out drone_fixed.usd
    python fix_drone_usd.py --in drone.usd --scale 0.001      # mm -> m (default)
    python fix_drone_usd.py --in drone.usd --keep-up          # leave Y-up

Then re-verify:
    python test_drone_usd.py --usd drone_fixed.usd --headless
Expect: metersPerUnit 1.0, footprint diagonal ~0.5 m, ratio ~1.5x (OK).
"""

import argparse
import os
import sys

# ---- CLI (parse BEFORE SimulationApp) --------------------------------------
here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--in", dest="src", default=os.path.join(here, "drone.usd"))
ap.add_argument("--out", dest="dst", default=os.path.join(here, "drone_fixed.usd"))
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
