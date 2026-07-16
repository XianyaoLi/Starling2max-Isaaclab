#!/usr/bin/env python3
r"""
test_articulated.py (v2) — hover + spin test for an articulated quadrotor.

Hold a stable hover (position + attitude control on the base) while the 4 props
spin via their joint velocity drives. Captures the 5 cameras + ToF depth, prints
joint velocities to confirm the props turn, then hands over to keyboard tele-op.

    python v2/test_articulated.py --usd articulated_drone.usd
"""

import argparse
import math
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default="articulated_drone.usd")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--hover-z", type=float, default=0.5)
ap.add_argument("--out", default="art_cam_out", help="dir for the 5 camera images")
ap.add_argument("--no-cams", action="store_true", help="skip the camera capture")
ap.add_argument("--res", type=int, nargs=2, default=[640, 480])
args = ap.parse_args()

if not os.path.isfile(args.usd):
    print(f"[FATAL] not found: {args.usd} (run make_articulated_drone.py first)")
    sys.exit(1)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
from pxr import UsdLux, UsdGeom, Gf  # noqa: E402

try:
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.prims import RigidPrim, Articulation
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.prims import RigidPrimView as RigidPrim
    from omni.isaac.core.articulations import ArticulationView as Articulation
try:
    from isaacsim.sensors.camera import Camera
except ImportError:
    from omni.isaac.sensor import Camera


# ---- camera helpers (same convention as v1: local +X orientation via SetLookAt) ----
VIEW = {"forward": ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),
        "down":    ((0.0, 0.0, -1.0), (0.0, -1.0, 0.0))}


def cam_local_xform(eye, view, up):
    v = Gf.Matrix4d(1.0)
    v.SetLookAt(Gf.Vec3d(*[float(e) for e in eye]),
                Gf.Vec3d(*[float(e + d) for e, d in zip(eye, view)]),
                Gf.Vec3d(*[float(u) for u in up]))
    return v.GetInverse()


def save_rgb(path, rgba):
    arr = np.asarray(rgba)
    if arr.size == 0:
        return
    arr = arr[:, :, :3].astype(np.uint8)
    try:
        from PIL import Image
        Image.fromarray(arr).save(path)
    except Exception:
        import matplotlib.pyplot as plt
        plt.imsave(path, arr)
    print(f"[img ] {path}")


def depth_via_replicator(cam_path, res, world):
    import omni.replicator.core as rep
    rp = rep.create.render_product(cam_path, (res[0], res[1]))
    annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    annot.attach(rp)
    for _ in range(40):
        world.render()
    d = np.asarray(annot.get_data())
    try:
        annot.detach(); rp.destroy()
    except Exception:
        pass
    return d

G, MASS = 9.81, 0.5
DEG = 180.0 / math.pi


def quat_to_R(q):                       # q = (w, x, y, z)
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


def lin_ang(view):
    try:
        v = np.asarray(view.get_velocities())[0]
        return np.asarray(v[:3], float), np.asarray(v[3:], float)
    except Exception:
        return (np.asarray(view.get_linear_velocities())[0].astype(float),
                np.asarray(view.get_angular_velocities())[0].astype(float))


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    UsdLux.DomeLight.Define(world.stage, "/World/Dome").CreateIntensityAttr(1200.0)

    root = "/World/Drone_ref"
    add_reference_to_stage(usd_path=os.path.abspath(args.usd), prim_path=root)

    # find the base rigid body: "base_link"/"visual", else first non-prop body
    from pxr import UsdPhysics
    base_path = None
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
    if base_path is None:
        base_path = f"{root}/Drone/base_link"
    print(f"[info] base link: {base_path}")

    # solid red wall in front (-Y): backdrop for the ToF/front cameras AND a real
    # static collider, so flying into it stops the drone instead of passing through.
    # Created before reset() so PhysX cooks the collider.
    if not args.no_cams:
        wall = UsdGeom.Cube.Define(world.stage, "/World/wall")
        wall.CreateSizeAttr(1.0)
        wall.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.6, args.hover_z))
        wall.AddScaleOp().Set(Gf.Vec3f(2.0, 0.05, 1.2))
        wall.CreateDisplayColorAttr([(0.9, 0.3, 0.2)])
        UsdPhysics.CollisionAPI.Apply(wall.GetPrim())

    world.reset()
    base = RigidPrim(prim_paths_expr=base_path, name="base")
    base.set_world_poses(positions=np.array([[0.0, 0.0, args.hover_z]]))

    art = None
    try:
        art = Articulation(prim_paths_expr=f"{root}/Drone", name="art")
        art.initialize()
    except Exception as e:
        print(f"[warn] joint readout unavailable ({e}); props still spin via drives")

    # ---- camera capture: 5 cameras + ToF (same as v1, NO cosmetic markers) ----
    if not args.no_cams:
        os.makedirs(args.out, exist_ok=True)
        xc = UsdGeom.XformCache()
        frames = []
        for prim in world.stage.Traverse():
            if not prim.GetPath().pathString.startswith(root):
                continue
            sa = prim.GetAttribute("drone:sensorType")
            if sa and sa.IsValid() and sa.Get():
                vd = prim.GetAttribute("drone:viewDir")
                w = xc.GetLocalToWorldTransform(prim).ExtractTranslation()
                frames.append((prim.GetName(), sa.Get(),
                               vd.Get() if vd and vd.IsValid() else "forward",
                               (float(w[0]), float(w[1]), float(w[2]))))
        print(f"[cams] {len(frames)} sensor frames")
        cams = []
        for nm, stype, vdir, wpos in frames:
            view, up = VIEW.get(vdir, VIEW["forward"])
            cp = f"{base_path}/cam_{nm}"
            c = UsdGeom.Camera.Define(world.stage, cp)
            c.MakeMatrixXform().Set(cam_local_xform(wpos, view, up))
            c.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
            c.CreateFocalLengthAttr(18.0)
            cam = Camera(prim_path=cp, resolution=(args.res[0], args.res[1]))
            cam.initialize()
            cams.append((nm, stype, vdir, cp, cam))
        for _ in range(40):
            world.render()
        for nm, stype, vdir, cp, cam in cams:
            save_rgb(os.path.join(args.out, f"{nm}_{vdir}.png"), cam.get_rgba())
            if stype == "tof_depth":
                d = np.asarray(depth_via_replicator(cp, args.res, world), float)
                valid = d[np.isfinite(d) & (d > 1e-4)] if d.size else np.array([])
                if valid.size:
                    print(f"[tof ] min={valid.min():.2f} mean={valid.mean():.2f} m")
        print(f"[cams] images in {args.out}/")

    EZ = np.array([0.0, 0.0, 1.0])
    KP_XY, KD_XY, KP_Z, KD_Z = 3.0, 3.0, 8.0, 4.5     # position
    KATT, KRATE = 0.6, 0.12                            # attitude / body-rate
    dt, MOVE = 1.0 / 60.0, 0.4
    target = np.array([0.0, 0.0, args.hover_z], float)
    held = set()
    st = {"reset": False}

    # keyboard teleop (same keys as V1; HOME reset, SPACE avoided = Isaac play/pause)
    if not args.headless:
        try:
            import carb.input
            import omni.appwindow
            ki = carb.input.KeyboardInput
            KM = {ki.UP: "fwd", ki.DOWN: "back", ki.LEFT: "left", ki.RIGHT: "right",
                  ki.PAGE_UP: "up", ki.PAGE_DOWN: "down"}

            def on_key(e, *a):
                if e.type == carb.input.KeyboardEventType.KEY_PRESS:
                    if e.input == ki.HOME:
                        st["reset"] = True
                    elif e.input in KM:
                        held.add(KM[e.input])
                elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
                    if e.input in KM:
                        held.discard(KM[e.input])
                return True
            carb.input.acquire_input_interface().subscribe_to_keyboard_events(
                omni.appwindow.get_default_app_window().get_keyboard(), on_key)
            print("[keys] Up/Down=fwd/back  Left/Right=strafe (fwd=-Y)  "
                  "PageUp/PageDown=up/down  Home=reset")
        except Exception as e:
            print(f"[keys] unavailable ({e})")

    def control():
        if "fwd" in held:   target[1] -= MOVE * dt
        if "back" in held:  target[1] += MOVE * dt
        if "left" in held:  target[0] += MOVE * dt
        if "right" in held: target[0] -= MOVE * dt
        if "up" in held:    target[2] += MOVE * dt
        if "down" in held:  target[2] -= MOVE * dt
        target[2] = max(target[2], 0.1)
        pos = np.asarray(base.get_world_poses()[0][0], float)
        q = np.asarray(base.get_world_poses()[1][0], float)      # (w,x,y,z)
        if st["reset"]:
            base.set_world_poses(positions=np.array([[0.0, 0.0, args.hover_z]]),
                                 orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
            try:
                base.set_velocities(np.zeros((1, 6)))
            except Exception:
                pass
            target[:] = [0.0, 0.0, args.hover_z]
            st["reset"] = False
            world.step(render=not args.headless)
            return pos
        lin, ang = lin_ang(base)
        R = quat_to_R(q)
        z_b = R @ EZ
        a = np.array([KP_XY*(target[0]-pos[0]) - KD_XY*lin[0],
                      KP_XY*(target[1]-pos[1]) - KD_XY*lin[1],
                      KP_Z*(target[2]-pos[2]) - KD_Z*lin[2]])
        F = MASS * (a + G * EZ)
        F[2] = max(F[2], 0.0)
        tau = KATT * np.cross(z_b, EZ) - KRATE * ang    # keep level + damp spin
        base.apply_forces_and_torques_at_pos(
            forces=F[None, :], torques=tau[None, :],
            positions=pos[None, :], is_global=True)
        world.step(render=not args.headless)
        return pos

    print(f"[ready] stable hover, 4 props spinning. Fly it with the keyboard.")
    for i in range(200):                                # settle to a stable hover
        pos = control()
        if i % 40 == 0 or i == 199:
            jvs = "n/a"
            if art is not None:
                try:
                    jv = np.asarray(art.get_joint_velocities())[0] * DEG   # rad/s -> deg/s
                    jvs = "[" + ", ".join(f"{v:+.0f}" for v in jv) + "] deg/s"
                except Exception:
                    pass
            print(f"step {i:4d}  z={pos[2]:.3f}  prop spin={jvs}")

    if not args.headless:
        print("[fly] keyboard to fly; props keep spinning. Ctrl-C to quit.")
        try:
            while simulation_app.is_running():
                control()
        except KeyboardInterrupt:
            pass


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
