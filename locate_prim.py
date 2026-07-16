#!/usr/bin/env python3
r"""
locate_prim.py

Find the TRUE world-space position of a mesh even though its transform pivot is
at the origin (the STEP->OBJ->USD conversion baked positions into the vertices,
so every mesh's Translate reads 0 / center -- that is why you can't read a
part's location off the Property panel).

This uses the world BOUNDING BOX (ComputeWorldBound), which reflects the real
geometry position regardless of the pivot.

Workflow to fix camera/rotor frame placement:
  1. In the Isaac Sim viewport, click the real lens / module / motor mesh.
  2. Read its "Prim Path" from the Property panel (e.g.
     /World/Drone/visual/Body19 or a deep .../MeshXYZ).
  3. Run this with that path to get the true world center:
        python locate_prim.py --usd starling2max.usd --path /World/Drone/visual/Body19
  4. Feed the printed center back into add_drone_frames.py:
        python add_drone_frames.py --front-pos X Y Z --down-pos X Y Z

Other modes:
  --list /World/Drone/visual --depth 1     browse children with world centers
  --near 0.09 0.0 0.0 --radius 0.03        list meshes near a world point
"""

import argparse
import math
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.join(here, "starling2max.usd"))
ap.add_argument("--path", action="append", default=[],
                help="Prim path(s) to locate (repeatable)")
ap.add_argument("--name", action="append", default=[],
                help="Find prims whose path/name CONTAINS this substring "
                     "(case-insensitive, repeatable). Handles - vs _ automatically.")
ap.add_argument("--list", dest="list_root", default=None,
                help="List descendants of this prim with world centers")
ap.add_argument("--depth", type=int, default=1, help="Depth for --list")
ap.add_argument("--near", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                help="List meshes whose world center is within --radius of this point")
ap.add_argument("--radius", type=float, default=0.03)
args = ap.parse_args()

if not os.path.isfile(args.usd):
    print(f"[FATAL] not found: {args.usd}")
    sys.exit(1)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402


def main():
    stage = Usd.Stage.Open(args.usd)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              includedPurposes=[UsdGeom.Tokens.default_,
                                                UsdGeom.Tokens.render])

    def wbox(prim):
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        c = (mn + mx) * 0.5
        d = mx - mn
        return c, d

    # --- locate explicit paths ---
    for p in args.path:
        prim = stage.GetPrimAtPath(p)
        if not prim or not prim.IsValid():
            print(f"[miss ] {p}  (not found)")
            continue
        c, d = wbox(prim)
        print(f"[loc  ] {p}")
        print(f"        world center = ({c[0]:+.4f}, {c[1]:+.4f}, {c[2]:+.4f}) m")
        print(f"        world size   = ({d[0]:.4f}, {d[1]:.4f}, {d[2]:.4f}) m")

    # --- name substring search ---
    for needle in args.name:
        # Match against both '-' and '_' variants, case-insensitive.
        variants = {needle.lower(),
                    needle.lower().replace("-", "_"),
                    needle.lower().replace("_", "-")}
        print(f"[name ] search '{needle}':")
        found = 0
        for prim in stage.Traverse():
            if prim.GetTypeName() not in ("Mesh", "Xform", "Scope"):
                continue
            p = prim.GetPath().pathString.lower()
            if any(v in p for v in variants):
                c, d = wbox(prim)
                print(f"  {prim.GetPath().pathString}")
                print(f"     center=({c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f})  "
                      f"size=({d[0]:.4f},{d[1]:.4f},{d[2]:.4f})")
                found += 1
        if not found:
            print("  (no match)")

    # --- list children ---
    if args.list_root:
        root = stage.GetPrimAtPath(args.list_root)
        if not root or not root.IsValid():
            print(f"[miss ] {args.list_root} (not found)")
        else:
            print(f"[list ] descendants of {args.list_root} (depth {args.depth}):")
            base_depth = args.list_root.count("/")
            for prim in Usd.PrimRange(root):
                d = prim.GetPath().pathString.count("/") - base_depth
                if d == 0 or d > args.depth:
                    continue
                if prim.GetTypeName() not in ("Mesh", "Xform", "Scope"):
                    continue
                c, sz = wbox(prim)
                print(f"  {'  '*d}{prim.GetName():<24} {prim.GetTypeName():<6} "
                      f"c=({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f})")

    # --- near a point ---
    if args.near is not None:
        px, py, pz = args.near
        print(f"[near ] meshes within {args.radius} m of "
              f"({px:+.3f},{py:+.3f},{pz:+.3f}):")
        hits = []
        for prim in stage.Traverse():
            if prim.GetTypeName() != "Mesh":
                continue
            c, _ = wbox(prim)
            dist = math.sqrt((c[0]-px)**2 + (c[1]-py)**2 + (c[2]-pz)**2)
            if dist <= args.radius:
                hits.append((dist, prim.GetPath().pathString, c))
        for dist, path, c in sorted(hits):
            print(f"  d={dist:.3f}  {path}  c=({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f})")
        if not hits:
            print("  (none -- widen --radius)")

    if not (args.path or args.name or args.list_root or args.near is not None):
        print("Nothing to do. Pass --path / --name / --list / --near. See --help.")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
