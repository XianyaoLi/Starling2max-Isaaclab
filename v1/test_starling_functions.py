#!/usr/bin/env python3
r"""
test_starling_functions.py — functional test of starling2max.usd.

  TEST 1 cameras: capture a frame at each of the 5 sensor frames — RGB for the 4
         cameras, depth for the ToF (Replicator distance_to_image_plane). A wall
         a known distance in front (-Y) lets you check the ToF reading. -> --out
  TEST 2 rotors: closed-loop altitude PD applies thrust at the 4 rotor points so
         the drone gently rises to --hover-z and holds while you inspect it.

Orientation: FORWARD = -Y (3-sensor front module), DOWN = -Z.

    python test_starling_functions.py --usd starling2max.usd --out cam_test_out
    python test_starling_functions.py --test rotors --hover-z 1.1
"""

import argparse
import math
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--usd", default="starling2max.usd")
ap.add_argument("--out", default="cam_test_out")
ap.add_argument("--test", choices=["cameras", "rotors", "both"], default="both")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--res", type=int, nargs=2, default=[640, 480], metavar=("W", "H"))
ap.add_argument("--spawn-z", type=float, default=0.6)
ap.add_argument("--hover-z", type=float, default=1.1, help="Target hover altitude (m)")
ap.add_argument("--obstacle-dist", type=float, default=0.6)
ap.add_argument("--steps", type=int, default=300)
args = ap.parse_args()

if not os.path.isfile(args.usd):
    print(f"[FATAL] not found: {args.usd}")
    sys.exit(1)
os.makedirs(args.out, exist_ok=True)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf  # noqa: E402

try:
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.api.objects import FixedCuboid
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.prims import RigidPrimView as RigidPrim
    from omni.isaac.core.objects import FixedCuboid
try:
    from isaacsim.sensors.camera import Camera
except ImportError:
    from omni.isaac.sensor import Camera

G, MASS = 9.81, 0.5


# ---------- image io ----------
def save_rgb(path, rgba):
    arr = np.asarray(rgba)
    if arr.size == 0:
        print(f"[warn] empty rgb -> {os.path.basename(path)}")
        return
    arr = arr[:, :, :3].astype(np.uint8)
    try:
        from PIL import Image
        Image.fromarray(arr).save(path)
    except Exception:
        import matplotlib.pyplot as plt
        plt.imsave(path, arr)
    print(f"[img ] {path}")


def save_depth(npy, png, depth, expected=None):
    d = np.asarray(depth, dtype=np.float32)
    if d.size == 0:
        print("[tof ] annotator returned empty array")
        return
    np.save(npy, d)
    valid = d[np.isfinite(d) & (d > 1e-4)]
    try:
        import matplotlib.pyplot as plt
        vis = np.where(np.isfinite(d), d, 0.0)
        hi = np.percentile(valid, 95) if valid.size else 1.0
        plt.imsave(png, np.clip(vis, 0, hi), cmap="turbo")
    except Exception:
        pass
    if not valid.size:
        print("[tof ] no valid (finite) depth -> ToF sees empty space; "
              "check tof RGB image / orientation")
        return
    print(f"[tof ] depth min={valid.min():.3f} mean={valid.mean():.3f} "
          f"max={valid.max():.3f} m  ({valid.size} pts)")
    if expected is not None:
        ok = "OK" if abs(valid.min() - expected) < 0.25 else "CHECK"
        print(f"[tof ] expected front face ~{expected:.2f} m; "
              f"measured {valid.min():.2f} m -> {ok}")


def depth_via_replicator(cam_path, res, world):
    import omni.replicator.core as rep
    rp = rep.create.render_product(cam_path, (res[0], res[1]))
    annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    annot.attach(rp)
    for _ in range(40):
        world.render()
    data = np.asarray(annot.get_data())
    # Detach/destroy so the synthetic-data graph isn't left dangling (that
    # dangling node is what segfaults during Python finalization on exit).
    try:
        annot.detach()
    except Exception:
        pass
    try:
        rp.destroy()
    except Exception:
        pass
    return data


# ---------- scene helpers ----------
# view direction + world up per drone:viewDir. Up = +Z for forward cams (so the
# horizon is level, not rolled); for a down cam use forward(-Y) as the image up.
VIEW = {"forward": ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),
        "down":    ((0.0, 0.0, -1.0), (0.0, -1.0, 0.0))}


def cam_local_xform(eye, view, up):
    """Local-to-body transform for a USD camera (looks down -Z) so it points
    along `view` with `up` as image-up. Body is level, so body frame == world."""
    eye = Gf.Vec3d(float(eye[0]), float(eye[1]), float(eye[2]))
    center = eye + Gf.Vec3d(float(view[0]), float(view[1]), float(view[2]))
    v = Gf.Matrix4d(1.0)
    v.SetLookAt(eye, center, Gf.Vec3d(float(up[0]), float(up[1]), float(up[2])))
    return v.GetInverse()


def make_camera(stage, body_path, name, eye_local, viewdir, near=0.012, far=1000.0):
    view, up = VIEW.get(viewdir, VIEW["forward"])
    path = body_path + f"/test_cam_{name}"
    cam = UsdGeom.Camera.Define(stage, path)
    cam.MakeMatrixXform().Set(cam_local_xform(eye_local, view, up))
    cam.CreateClippingRangeAttr(Gf.Vec2f(near, far))
    cam.CreateFocalLengthAttr(18.0)
    return path


def sensor_frames(stage, root):
    out = []
    for p in stage.Traverse():
        if not p.GetPath().pathString.startswith(root):
            continue
        st = p.GetAttribute("drone:sensorType")
        if st and st.IsValid() and st.Get():
            vd = p.GetAttribute("drone:viewDir")
            out.append((p, st.Get(), vd.Get() if vd and vd.IsValid() else "forward"))
    return out


def rotor_frames(stage, root):
    out = []
    for p in stage.Traverse():
        if not p.GetPath().pathString.startswith(root):
            continue
        sd = p.GetAttribute("drone:spinDir")
        if sd and sd.IsValid() and sd.Get() is not None:
            out.append((p, int(sd.Get())))
    return out


def lin_ang_vel(view):
    try:
        v = np.asarray(view.get_velocities())[0]
        return v[:3], v[3:]
    except Exception:
        return (np.asarray(view.get_linear_velocities())[0],
                np.asarray(view.get_angular_velocities())[0])


def main():
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    world.scene.add_default_ground_plane()
    UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1200.0)

    root = "/World/Drone_ref"
    add_reference_to_stage(usd_path=args.usd, prim_path=root)

    rb_path = next((p.GetPath().pathString for p in stage.Traverse()
                    if p.GetPath().pathString.startswith(root)
                    and p.HasAPI(UsdPhysics.RigidBodyAPI)), None)
    print(f"[info ] rigid body: {rb_path}")

    od = args.obstacle_dist
    # Tall, wide wall so the ToF center ray (horizontal, at any drone height)
    # hits it instead of flying over the top to the far ground.
    FixedCuboid(prim_path="/World/front_obstacle",
                position=np.array([0.0, -od, args.spawn_z]),
                scale=np.array([2.0, 0.05, 3.0]),
                color=np.array([0.9, 0.3, 0.2]))

    frames = sensor_frames(stage, root)
    cam_prims = []
    for p, st, vd in frames:
        fl = UsdGeom.Xformable(p).GetLocalTransformation().ExtractTranslation()
        cam_prims.append((p.GetName(), st, vd,
                          make_camera(stage, rb_path, p.GetName(),
                                      (fl[0], fl[1], fl[2]), vd)))
    print(f"[info ] cameras: {[c[0] for c in cam_prims]}")

    world.reset()
    drone = RigidPrim(prim_paths_expr=rb_path, name="drone")
    drone.set_world_poses(positions=np.array([[0.0, 0.0, args.spawn_z]]))

    # ---------------- TEST 1: cameras ----------------
    if args.test in ("cameras", "both"):
        print("\n========== TEST 1: CAMERAS ==========")
        cams = []
        for name, st, vd, cpath in cam_prims:
            c = Camera(prim_path=cpath, resolution=(args.res[0], args.res[1]))
            c.initialize()
            cams.append((name, st, vd, cpath, c))
        for _ in range(40):
            world.render()
        for name, st, vd, cpath, c in cams:
            rgba = c.get_rgba()
            if np.asarray(rgba).size == 0:
                rgba = c.get_current_frame().get("rgba", np.array([]))
            save_rgb(os.path.join(args.out, f"{name}_{vd}.png"), rgba)
            if st == "tof_depth":
                depth = depth_via_replicator(cpath, args.res, world)
                save_depth(os.path.join(args.out, "tof_depth.npy"),
                           os.path.join(args.out, "tof_depth.png"), depth)
                # Use the CENTER region (aimed at the obstacle), NOT global min --
                # the global min is the drone's own props at the frame edges.
                d = np.asarray(depth, np.float32)
                if d.ndim >= 2:
                    h, w = d.shape[:2]
                    patch = d[h//2-12:h//2+12, w//2-12:w//2+12]
                    pv = patch[np.isfinite(patch) & (patch > 1e-4)]
                    if pv.size:
                        cd = float(np.median(pv))
                        ok = "OK" if abs(cd - od) < 0.25 else "CHECK"
                        print(f"[tof ] CENTER depth {cd:.2f} m vs obstacle {od:.2f} m "
                              f"-> {ok}  (edge min is self/props, expected)")
                    else:
                        print("[tof ] center region empty -- obstacle not centered?")
        print(f"[done] images in {args.out}/")

    # ---------------- TEST 2: rotors + hover ----------------
    blade_ops, rotor_pos = [], []
    if args.test in ("rotors", "both"):
        print("\n========== TEST 2: ROTORS + HOVER ==========")
        rotors = rotor_frames(stage, root)
        print(f"[info ] rotors: {[(p.GetName(), s) for p, s in rotors]}")
        for prim, spin in rotors:
            blade = UsdGeom.Cube.Define(stage, prim.GetPath().AppendChild("blade"))
            rot = blade.AddRotateZOp()
            blade.AddScaleOp().Set(Gf.Vec3f(0.09, 0.008, 0.003))
            blade.CreateDisplayColorAttr([(0.1, 0.6, 1.0) if spin > 0 else (1.0, 0.5, 0.1)])
            blade_ops.append((rot, spin))
            t = UsdGeom.Xformable(prim).GetLocalTransformation().ExtractTranslation()
            rotor_pos.append((np.array([t[0], t[1], t[2]]), spin))

        world.reset()
        drone.set_world_poses(positions=np.array([[0.0, 0.0, 0.15]]))  # just above ground

    # ---- proper quadrotor controller (community-standard geometric control) ----
    # Thrust is applied along the BODY z-axis (NOT world-up). So the drone must
    # lean to translate, and if a collision tips it over the thrust tips with it
    # -> it loses lift and destabilizes exactly like a real quad; the attitude
    # loop then tries to recover. No fake "tilt>30 -> cut motors" rule; collisions
    # are handled by PhysX. This is the standard cascaded position+attitude scheme
    # used by Isaac Lab's quadcopter and PX4 (geometric control, Lee et al.).
    dt = 1.0 / 60.0
    MOVE = 0.40                       # m/s keyboard setpoint speed
    EZ = np.array([0.0, 0.0, 1.0])
    KP_XY, KD_XY, KP_Z, KD_Z = 4.0, 3.5, 8.0, 4.5    # position gains (accel)
    KP_ATT, KD_ATT = 0.6, 0.18        # attitude gains, tuned to the box inertia
    MAX_TILT = math.radians(35.0)     # cap how far the position loop may lean
    target = np.array([0.0, 0.0, 0.0], dtype=float)  # setpoint; 0 -> sits on ground
    held = set()
    state = {"angle": 0.0}

    def quat_to_R(q):                 # q=(w,x,y,z) -> body->world rotation matrix
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return np.array([
            [1 - 2*(y*y+z*z), 2*(x*y-w*z),     2*(x*z+w*y)],
            [2*(x*y+w*z),     1 - 2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y),     2*(y*z+w*x),     1 - 2*(x*x+y*y)],
        ])

    # keyboard (arrows + PageUp/Down + Space; avoids Isaac's WASDQE fly / F / G)
    if not args.headless and args.test in ("rotors", "both"):
        try:
            import carb.input
            import omni.appwindow
            ki = carb.input.KeyboardInput
            # NOTE: do NOT use SPACE -- it is Isaac Sim's play/pause hotkey, which
            # pauses the timeline so world.step() stops and teleop dies. HOME is safe.
            KEYMAP = {ki.UP: "fwd", ki.DOWN: "back", ki.LEFT: "left",
                      ki.RIGHT: "right", ki.PAGE_UP: "up", ki.PAGE_DOWN: "down",
                      ki.HOME: "reset"}

            def on_key(e, *a):
                if e.input in KEYMAP:
                    if e.type == carb.input.KeyboardEventType.KEY_PRESS:
                        held.add(KEYMAP[e.input])
                    elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
                        held.discard(KEYMAP[e.input])
                return True

            appwin = omni.appwindow.get_default_app_window()
            _sub = carb.input.acquire_input_interface().subscribe_to_keyboard_events(
                appwin.get_keyboard(), on_key)
        except Exception as e:
            print(f"[keys] keyboard unavailable ({e})")

    def do_reset():
        drone.set_world_poses(positions=np.array([[0.0, 0.0, 0.15]]),
                              orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
        try:
            drone.set_velocities(np.zeros((1, 6)))
        except Exception:
            try:
                drone.set_linear_velocities(np.zeros((1, 3)))
                drone.set_angular_velocities(np.zeros((1, 3)))
            except Exception:
                pass
        target[:] = [0.0, 0.0, 0.0]
        held.discard("reset")

    def hover_step():
        # keyboard nudges the position setpoint
        if "fwd" in held:   target[1] -= MOVE * dt
        if "back" in held:  target[1] += MOVE * dt
        if "left" in held:  target[0] += MOVE * dt   # forward=-Y => left=+X
        if "right" in held: target[0] -= MOVE * dt
        if "up" in held:    target[2] += MOVE * dt
        if "down" in held:  target[2] -= MOVE * dt
        target[2] = max(target[2], 0.0)

        pos_all, ori_all = drone.get_world_poses()
        pos = np.asarray(pos_all[0], dtype=float)
        q = np.asarray(ori_all[0], dtype=float)      # (w,x,y,z)

        if "reset" in held:                          # SPACE: teleport upright, resume
            do_reset()
            world.step(render=not args.headless)
            return pos[2]

        lin, ang = lin_ang_vel(drone)
        lin = np.asarray(lin, dtype=float)
        ang = np.asarray(ang, dtype=float)           # world-frame angular velocity
        R = quat_to_R(q)
        z_b = R @ EZ                                 # body-z (thrust axis) in world

        # position loop -> desired force (world), with gravity comp
        a_des = np.array([
            KP_XY * (target[0] - pos[0]) - KD_XY * lin[0],
            KP_XY * (target[1] - pos[1]) - KD_XY * lin[1],
            KP_Z  * (target[2] - pos[2]) - KD_Z  * lin[2],
        ])
        F_des = MASS * (a_des + G * EZ)
        # limit desired lean: cap horizontal force vs vertical
        fz = max(F_des[2], 0.4 * MASS * G)
        fxy = F_des[:2].copy()
        max_xy = fz * math.tan(MAX_TILT)
        n = float(np.linalg.norm(fxy))
        if n > max_xy and n > 1e-9:
            fxy *= max_xy / n
        F_des = np.array([fxy[0], fxy[1], fz])

        # thrust = desired force projected on the CURRENT body-z (so tilt reduces lift)
        thrust = max(float(np.dot(F_des, z_b)), 0.0)
        # attitude loop: torque body-z toward desired thrust direction + damp spin
        z_des = F_des / (float(np.linalg.norm(F_des)) + 1e-9)
        tau = KP_ATT * np.cross(z_b, z_des) - KD_ATT * ang     # world-frame torque
        F_world = thrust * z_b                                  # force along body-z

        drone.apply_forces_and_torques_at_pos(
            forces=F_world[None, :], torques=tau[None, :],
            positions=pos[None, :], is_global=True)

        state["angle"] += 12.0                        # spin blade markers
        for rot, spin in blade_ops:
            rot.Set(state["angle"] * spin)

        world.step(render=not args.headless)
        return pos[2]

    if args.test in ("rotors", "both"):
        print("[ctrl] Body-frame thrust + attitude control (leans to move).")
        print("       PageUp = take off / climb   PageDown = descend / land")
        print("       Up/Down = forward/back   Left/Right = strafe   (forward = -Y)")
        print("       Collisions are physical: a hard hit tips/destabilizes it and")
        print("       the controller tries to recover.  HOME = reset upright.")
        print("       (SPACE is avoided -- it is Isaac's play/pause and freezes the sim.)")
        for i in range(120):          # let it settle onto the ground
            z = hover_step()
            if i % 40 == 0 or i == 119:
                print(f"settle {i:4d}  z={z:.3f} m")
        print("[ready] resting on ground — press PageUp in the viewport to take off.")

    print("\nAll requested tests finished.")
    if not args.headless:
        print("Viewport open — inspect freely. Ctrl-C to quit.")
        try:
            while simulation_app.is_running():
                if args.test in ("rotors", "both"):
                    hover_step()          # keep hovering for inspection
                else:
                    world.render()
        except KeyboardInterrupt:
            pass
    # actual shutdown handled by the __main__ finally below


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        print("\n[ERROR] exception in main -- this (not the exit) is the real bug:")
        traceback.print_exc()
    finally:
        # Hard-exit on EVERY path (normal / exception / Ctrl-C) so we never fall
        # into Isaac Sim's segfaulting atexit/Py_FinalizeEx plugin teardown.
        try:
            simulation_app.close()
        except Exception:
            pass
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
