#!/usr/bin/env python3
r"""
flatten_usd.py — bake a referenced USD into one self-contained file.

Flattens a reference chain (e.g. starling2max.usd -> drone_physics.usd ->
drone_fixed.usd -> drone.usd) so the result loads from any working directory.

    python v1/flatten_usd.py --usd starling2max.usd --out starling2max_flat.usd
"""

import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default="starling2max.usd")
ap.add_argument("--out", default="starling2max_flat.usd")
args = ap.parse_args()

if not os.path.isfile(args.usd):
    print(f"[FATAL] not found: {args.usd}")
    sys.exit(1)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Usd  # noqa: E402


def main():
    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print(f"[FATAL] could not open {args.usd}")
        return
    if os.path.exists(args.out):
        os.remove(args.out)
    flat = stage.Flatten()          # composes all references into one Sdf.Layer
    flat.Export(args.out)
    print(f"[write] {args.out}  (self-contained, no external references)")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
