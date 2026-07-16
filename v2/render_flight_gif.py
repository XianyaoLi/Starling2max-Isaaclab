#!/usr/bin/env python3
r"""
render_flight_gif.py — looping GIF of any drone USD taking off and flying a
circle. Auto-detects the base rigid body; articulated assets spin their props
on their own (velocity-driven joints). Single-body assets just fly the circle.

    python v2/render_flight_gif.py --usd articulated_drone.usd --out docs/flight_v2.gif
    python v2/render_flight_gif.py --usd starling2max.usd       --out v1/starling_v1_flight.gif
    python v2/render_flight_gif.py --usd starling2max_v2.usd    --out v2/starling_v2_flight.gif
"""

import argparse
import math
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--res", type=int, default=480)
ap.add_argument("--hover-z", type=float, default=0.7)
ap.add_argument("--radius", type=float, default=0.5)
args = ap.parse_args()

if not os.path.isfile(args.usd):
    print(f"[FATAL] not found: {args.usd}")
    sys.exit(1)
os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import UsdGeom, UsdLux, UsdPhysics, Gf  # noqa: E402

try:
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.prims import RigidPrim
    from isaacsim.sensors.camera import Camera
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.prims import RigidPrimView as RigidPrim
    from omni.isaac.sensor import Camera

G, MASS = 9.81, 0.5


def quat_to_R(q):                       # (w,x,y,z) -> rotation matrix
    w, x, y, z = [float(v) for v in q]
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def lin_ang(v):
    try:
        a = np.asarray(v.get_velocities())[0]
        return np.asarray(a[:3], float), np.asarray(a[3:], float)
    except Exception:
        return (np.asarray(v.get_linear_velocities())[0].astype(float),
                np.asarray(v.get_angular_velocities())[0].astype(float))


def look_xform(eye, center):
    m = Gf.Matrix4d(1.0)
    m.SetLookAt(Gf.Vec3d(*[float(e) for e in eye]),
                Gf.Vec3d(*[float(c) for c in center]), Gf.Vec3d(0, 0, 1))
    return m.GetInverse()


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    UsdLux.DomeLight.Define(world.stage, "/World/Dome").CreateIntensityAttr(1400.0)

    root = "/World/Drone_ref"
    add_reference_to_stage(usd_path=os.path.abspath(args.usd), prim_path=root)

    base_path = None                    # base = "base_link"/"visual", else non-prop body
    for prim in world.stage.Traverse():
        p = prim.GetPath().pathString
        if not p.startswith(root) or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        nm = prim.GetName()
        if nm in ("base_link", "visual"):
            base_path = p
            break
        if base_path is None and not (nm.startswith("prop_") or nm.startswith("rotor_")):
            base_path = p
    print(f"[info] base: {base_path}")

    H = args.hover_z
    cam = UsdGeom.Camera.Define(world.stage, "/World/chase")
    cam.MakeMatrixXform().Set(look_xform((1.6, -1.6, H + 0.9), (0.0, 0.0, H)))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    cam.CreateFocalLengthAttr(20.0)
    chase = Camera(prim_path="/World/chase", resolution=(args.res, args.res))

    world.reset()
    chase.initialize()
    base = RigidPrim(prim_paths_expr=base_path, name="base")
    base.set_world_poses(positions=np.array([[args.radius, 0.0, 0.15]]))

    EZ = np.array([0.0, 0.0, 1.0])
    frames = []
    for i in range(420):
        # target: gentle takeoff, then a circle of --radius at H
        if i < 120:
            tgt = np.array([args.radius, 0.0, 0.15 + (H - 0.15) * i / 120.0])
        else:
            a = (i - 120) * 0.025
            tgt = np.array([args.radius * math.cos(a), args.radius * math.sin(a), H])
        pos = np.asarray(base.get_world_poses()[0][0], float)
        q = np.asarray(base.get_world_poses()[1][0], float)
        lin, ang = lin_ang(base)
        z_b = quat_to_R(q) @ EZ
        acc = np.array([4.0*(tgt[0]-pos[0]) - 3.5*lin[0],
                        4.0*(tgt[1]-pos[1]) - 3.5*lin[1],
                        8.0*(tgt[2]-pos[2]) - 4.5*lin[2]])
        F = MASS * (acc + G * EZ)
        F[2] = max(F[2], 0.0)
        tau = 0.6 * np.cross(z_b, EZ) - 0.12 * ang
        base.apply_forces_and_torques_at_pos(forces=F[None, :], torques=tau[None, :],
                                             positions=pos[None, :], is_global=True)
        world.step(render=True)
        if i >= 120 and i % 3 == 0:
            a = np.asarray(chase.get_rgba())
            if a.size:
                frames.append(a[:, :, :3].astype(np.uint8))

    if not frames:
        print("[FATAL] no frames"); return
    try:
        import imageio
        imageio.mimsave(args.out, frames, duration=0.05, loop=0)
    except Exception:
        from PIL import Image
        ims = [Image.fromarray(f) for f in frames]
        ims[0].save(args.out, save_all=True, append_images=ims[1:], duration=50, loop=0)
    print(f"[gif ] {args.out}  ({len(frames)} frames, looping)")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        try:
            simulation_app.close()
        except Exception:
            pass
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
