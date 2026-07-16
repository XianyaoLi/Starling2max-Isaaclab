#!/usr/bin/env python3
r"""
verify_physics.py

Cleanly verify drone_physics.usd by dropping it onto a ground plane using its
OWN authored physics (rigid body + box collider). Does NOT re-apply physics --
that was the bug that made the drone free-fall through the floor when using
test_drone_usd.py --physics on an already-physics-enabled asset.

Run:
    python verify_physics.py --usd drone_physics.usd            # windowed
    python verify_physics.py --usd drone_physics.usd --headless # no viewport
    python verify_physics.py --usd drone_physics.usd --spawn-z 0.5 --steps 200

PASS = the printed z settles to a small constant (~half the box height, ~0.06 m)
instead of decreasing without bound.
"""

import argparse
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.join(here, "drone_physics.usd"))
ap.add_argument("--headless", action="store_true")
ap.add_argument("--spawn-z", type=float, default=0.5, help="Drop height in meters")
ap.add_argument("--steps", type=int, default=240)
args = ap.parse_args()

if not os.path.isfile(args.usd):
    print(f"[FATAL] not found: {args.usd}")
    sys.exit(1)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

# Prefer the new isaacsim.* namespace (Isaac Sim 4.5+); fall back to the old
# omni.isaac.core.* one. SingleRigidPrim is the SINGLE-body wrapper: it takes
# prim_path (singular) and get_world_pose()/set_world_pose() -- NOT the view's
# prim_paths_expr/get_world_poses(), which was the cause of the TypeError.
try:
    from isaacsim.core.api import World  # noqa: E402
    from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
    from isaacsim.core.prims import SingleRigidPrim as RigidBody  # noqa: E402
except ImportError:
    from omni.isaac.core import World  # noqa: E402
    from omni.isaac.core.utils.stage import add_reference_to_stage  # noqa: E402
    from omni.isaac.core.prims import RigidPrim as RigidBody  # noqa: E402


def find_rigid_body_path(stage, under):
    for prim in stage.Traverse():
        if prim.GetPath().pathString.startswith(under) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim.GetPath().pathString
    return None


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    ref_root = "/World/drone_ref"
    add_reference_to_stage(usd_path=args.usd, prim_path=ref_root)

    stage = world.stage
    rb_path = find_rigid_body_path(stage, ref_root)
    if rb_path is None:
        print(f"[FATAL] no RigidBodyAPI prim found under {ref_root}. "
              f"Did you run add_drone_physics.py?")
        simulation_app.close()
        sys.exit(1)
    print(f"[info ] rigid body prim: {rb_path}")

    drone = RigidBody(prim_path=rb_path, name="drone")
    world.scene.add(drone)
    world.reset()

    # Spawn above the ground.
    drone.set_world_pose(position=np.array([0.0, 0.0, args.spawn_z]))

    print(f"[drop ] from z={args.spawn_z} m, {args.steps} steps")
    last_z = None
    for i in range(args.steps):
        world.step(render=not args.headless)
        if i % 30 == 0 or i == args.steps - 1:
            pos, _ = drone.get_world_pose()
            z = float(pos[2])
            print(f"step {i:4d}  z={z:.4f} m")
            last_z = z

    print("\n----- verdict -----")
    if last_z is not None and last_z > -0.05:
        print(f"PASS: settled at z={last_z:.4f} m (box half-height ~0.06). "
              f"Rigid body + collision work.")
    else:
        print(f"FAIL: z={last_z} -- fell through. Check collider/ground.")

    if not args.headless:
        print("Viewport open. Ctrl-C to quit.")
        try:
            while simulation_app.is_running():
                simulation_app.update()
        except KeyboardInterrupt:
            pass
    simulation_app.close()


if __name__ == "__main__":
    main()
