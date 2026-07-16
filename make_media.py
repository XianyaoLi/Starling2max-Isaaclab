#!/usr/bin/env python3
r"""
make_media.py

Render README media for a drone USD:
  1) a 6-view contact sheet (+X/-X/-Y/+Y/top/bottom) -> docs/contact_sheet.png
  2) a short flight GIF (takeoff + circle, body-frame geometric controller)
        -> docs/flight.gif

Default asset is demo_drone.usd (100% original placeholder, no CAD copyright),
which is the right thing to show in a public README. You can point --usd at any
drone USD, but only publish media of geometry you have the rights to.

Run (full Isaac Lab):
    python make_demo_drone.py --out demo_drone.usd     # if not done yet
    python make_media.py --usd demo_drone.usd
"""

import argparse
import math
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.join(here, "demo_drone.usd"))
ap.add_argument("--out", default=os.path.join(here, "docs"))
ap.add_argument("--res", type=int, default=480)
ap.add_argument("--gif-steps", type=int, default=360)
ap.add_argument("--gif-every", type=int, default=4)
args = ap.parse_args()

if not os.path.isfile(args.usd):
    print(f"[FATAL] not found: {args.usd} (run make_demo_drone.py first)")
    sys.exit(1)
os.makedirs(args.out, exist_ok=True)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf  # noqa: E402

try:
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.prims import RigidPrim
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.prims import RigidPrimView as RigidPrim
try:
    from isaacsim.sensors.camera import Camera
except ImportError:
    from omni.isaac.sensor import Camera

G, MASS = 9.81, 0.5


def look_xform(eye, center, up):
    v = Gf.Matrix4d(1.0)
    v.SetLookAt(Gf.Vec3d(*[float(e) for e in eye]),
                Gf.Vec3d(*[float(c) for c in center]),
                Gf.Vec3d(*[float(u) for u in up]))
    return v.GetInverse()


def make_cam(stage, path, eye, center, up, res):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.MakeMatrixXform().Set(look_xform(eye, center, up))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 200.0))
    cam.CreateFocalLengthAttr(24.0)
    w = Camera(prim_path=path, resolution=(res, res))
    w.initialize()
    return w


def quat_to_R(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


def lin_ang(view):
    try:
        v = np.asarray(view.get_velocities())[0]
        return v[:3], v[3:]
    except Exception:
        return (np.asarray(view.get_linear_velocities())[0],
                np.asarray(view.get_angular_velocities())[0])


def rgb(frame_cam):
    a = np.asarray(frame_cam.get_rgba())
    if a.size == 0:
        a = np.asarray(frame_cam.get_current_frame().get("rgba", np.zeros((8, 8, 4))))
    return a[:, :, :3].astype(np.uint8)


def main():
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    world.scene.add_default_ground_plane()
    UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1500.0)

    root = "/World/Drone_ref"
    add_reference_to_stage(usd_path=args.usd, prim_path=root)
    rb = next((p.GetPath().pathString for p in stage.Traverse()
               if p.GetPath().pathString.startswith(root)
               and p.HasAPI(UsdPhysics.RigidBodyAPI)), None)

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath(root)).ComputeAlignedRange()
    size = rng.GetMax() - rng.GetMin()
    R = 1.7 * float(max(size))                 # camera distance

    world.reset()
    drone = RigidPrim(prim_paths_expr=rb, name="drone")

    # ---------- 1) 6-view contact sheet ----------
    H = 0.5
    drone.set_world_poses(positions=np.array([[0.0, 0.0, H]]))
    for _ in range(10):
        world.render()
    C = (0.0, 0.0, H)
    views = [("+X right", (R, 0, H), (0, 0, 1)),
             ("-X left",  (-R, 0, H), (0, 0, 1)),
             ("-Y front", (0, -R, H), (0, 0, 1)),
             ("+Y back",  (0, R, H), (0, 0, 1)),
             ("+Z top",   (0, 0, H + R), (0, -1, 0)),
             ("-Z bottom", (0, 0, H - R), (0, 1, 0))]
    imgs = []
    for i, (label, eye, up) in enumerate(views):
        cam = make_cam(stage, f"/World/view_{i}", eye, C, up, args.res)
        for _ in range(18):
            world.render()
        imgs.append((label, rgb(cam)))
    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 3, figsize=(12, 8))
        for ax, (label, im) in zip(axes.ravel(), imgs):
            ax.imshow(im)
            ax.set_title(label, fontsize=11)
            ax.axis("off")
        fig.suptitle("Drone in Isaac Lab — 6 views (forward = -Y)", fontsize=13)
        fig.tight_layout()
        p = os.path.join(args.out, "contact_sheet.png")
        fig.savefig(p, dpi=110)
        print(f"[img ] {p}")
    except Exception as e:
        print(f"[warn] contact sheet failed ({e})")

    # ---------- 2) flight GIF ----------
    chase = make_cam(stage, "/World/chase", (1.4, -1.4, 1.2), (0, 0, 0.9), (0, 0, 1), args.res)
    drone.set_world_poses(positions=np.array([[0.0, 0.0, 0.15]]))
    world.reset()
    drone.set_world_poses(positions=np.array([[0.0, 0.0, 0.15]]))

    EZ = np.array([0.0, 0.0, 1.0])
    KP_XY, KD_XY, KP_Z, KD_Z, KP_ATT, KD_ATT = 4.0, 3.5, 8.0, 4.5, 0.6, 0.18
    MAX_TILT = math.radians(30.0)
    frames = []

    def target(i):
        if i < 120:
            return np.array([0.0, 0.0, 0.15 + (1.0 - 0.15) * (i / 120.0)])
        a = (i - 120) * 0.025
        return np.array([0.5 * math.cos(a), 0.5 * math.sin(a), 1.0])

    for i in range(args.gif_steps):
        tgt = target(i)
        pos = np.asarray(drone.get_world_poses()[0][0], dtype=float)
        q = np.asarray(drone.get_world_poses()[1][0], dtype=float)
        lin, ang = lin_ang(drone)
        lin = np.asarray(lin, float)
        ang = np.asarray(ang, float)
        Rm = quat_to_R(q)
        z_b = Rm @ EZ
        a_des = np.array([KP_XY * (tgt[0] - pos[0]) - KD_XY * lin[0],
                          KP_XY * (tgt[1] - pos[1]) - KD_XY * lin[1],
                          KP_Z * (tgt[2] - pos[2]) - KD_Z * lin[2]])
        F = MASS * (a_des + G * EZ)
        fz = max(F[2], 0.4 * MASS * G)
        fxy = F[:2].copy()
        mx = fz * math.tan(MAX_TILT)
        n = float(np.linalg.norm(fxy))
        if n > mx and n > 1e-9:
            fxy *= mx / n
        F = np.array([fxy[0], fxy[1], fz])
        thrust = max(float(np.dot(F, z_b)), 0.0)
        z_des = F / (float(np.linalg.norm(F)) + 1e-9)
        tau = KP_ATT * np.cross(z_b, z_des) - KD_ATT * ang
        drone.apply_forces_and_torques_at_pos(
            forces=(thrust * z_b)[None, :], torques=tau[None, :],
            positions=pos[None, :], is_global=True)
        world.step(render=True)
        if i % args.gif_every == 0:
            frames.append(rgb(chase))

    gif = os.path.join(args.out, "flight.gif")
    try:
        import imageio
        imageio.mimsave(gif, frames, duration=0.05)
        print(f"[gif ] {gif}  ({len(frames)} frames)")
    except Exception:
        try:
            from PIL import Image
            ims = [Image.fromarray(f) for f in frames]
            ims[0].save(gif, save_all=True, append_images=ims[1:], duration=50, loop=0)
            print(f"[gif ] {gif}  ({len(frames)} frames, PIL)")
        except Exception as e:
            print(f"[warn] gif failed ({e}); install imageio or Pillow")

    print("[done] media in", args.out)


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
