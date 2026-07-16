# starling2max-isaaclab

A small, reproducible **toolchain** for turning a multirotor CAD model into a
physics-ready **USD** for **NVIDIA Isaac Sim / Isaac Lab**, and functionally
testing it (cameras + ToF depth, hover, keyboard tele-op, collisions).

It was built while bringing a **ModalAI Starling 2 Max** (C29, with ToF) into
Isaac Lab, but the scripts are generic and work on any quad-shaped mesh.

> **⚠️ No proprietary geometry is included.** The Starling CAD is distributed by
> ModalAI under gated terms and is **not** redistributed here. This repo ships
> only the **scripts** plus a **placeholder drone**
> so the pipeline runs out-of-the-box. Bring your own CAD for the real model —
> see *[Using your own CAD](#using-your-own-cad)*.

---

## Preview

*(Rendered from the original `demo_drone.usd` placeholder — no proprietary CAD.)*

<p align="center">
  <img src="docs/flight.gif" width="45%" alt="v1 — single-body flight">
  &nbsp;&nbsp;
  <img src="docs/flight_v2.gif" width="45%" alt="v2 — articulated, real spinning rotors">
</p>

<p align="center"><b>left:</b> v1 single rigid body &nbsp;·&nbsp; <b>right:</b> v2 articulated (4 independent rotors that actually spin)</p>

> **On the real vehicle.** `v1/` and `v2/` each hold a short flight clip of the
> actual Starling. These are **renders shown only to illustrate the kind of
> result this toolchain can achieve on a real airframe** — the CAD/model itself
> is not distributed (see [License](#license--attribution)). Not affiliated with,
> or endorsed by, ModalAI.

---

## What it does

```
 CAD (STEP)                         ── you export, see "Using your own CAD"
   │  (Fusion 360 / FreeCAD / Blender)
   ▼
 OBJ ── (Isaac Sim importer / usdcat) ─▶ raw USD (mm, Y-up, no physics)
   │
   ├─ fix_drone_usd.py        → *_fixed.usd     (meters, Z-up, correct scale)
   ├─ add_drone_physics.py    → *_physics.usd   (rigid body + box collider + mass)
   └─ add_drone_frames.py     → *.usd           (rotor + camera/ToF frames)
                                     │
                                     ▼
                        test_starling_functions.py
              (5 cameras + ToF depth, hover, keyboard tele-op, collisions)
```

No CAD? Generate the placeholder and run the whole thing:

```bash
# v1 — single rigid body: cameras + ToF, hover, tele-op, collisions
python v1/make_demo_drone.py --out demo_drone.usd
python v1/test_starling_functions.py --usd demo_drone.usd --out demo_out

# v2 — articulated: the 4 propellers are independent bodies that actually spin
python v2/make_articulated_drone.py --out articulated_drone.usd
python v2/test_articulated.py --usd articulated_drone.usd
```

---

## Requirements

- **Isaac Sim / Isaac Lab 4.5+** (provides `isaacsim.*`, `pxr`, Replicator).
- Run every script with the Isaac Python (each script boots a headless
  `SimulationApp` before importing `pxr`, so a plain `python foo.py` inside the
  `env_isaaclab` env works).
- `matplotlib` / `Pillow` optional (nicer image saving; falls back gracefully).

---

## Scripts

**`v1/`** — single-rigid-body pipeline (run e.g. `python v1/fix_drone_usd.py`):

| Script | Purpose |
|---|---|
| `test_drone_usd.py` | Inspect a USD: hierarchy, world bbox, **scale sanity check**, physics presence |
| `fix_drone_usd.py` | Fix units / up-axis (mm→m, Y-up→Z-up, `metersPerUnit=1`) → `*_fixed.usd` |
| `add_drone_physics.py` | Add a single rigid body + **box/hull/decomp** collider + mass → `*_physics.usd`; `--list-corners` finds rotor clusters |
| `locate_prim.py` | Report a mesh's **true world center** by name/path (pivots are baked to origin after CAD→OBJ→USD) |
| `add_drone_frames.py` | Add `rotor_0..3` (with `drone:spinDir`) + camera/ToF frames (with `drone:sensorType`/`drone:viewDir`), read from named meshes |
| `verify_physics.py` | Drop-test the asset onto a ground plane using its own physics |
| `test_starling_functions.py` | Functional test: 5 cameras + ToF depth, quad hover, keyboard tele-op, physical collisions |
| `flatten_usd.py` | Bake a referenced USD into one self-contained file |
| `make_demo_drone.py` | Generate the original placeholder quadrotor USD (no CAD needed) |
| `make_media.py` | Render README media (6-view contact sheet + flight GIF) → `docs/` |

**`v2/`** — articulated rotors: the 4 propellers become independent bodies on
revolute joints that actually spin (velocity-driven).

| Script | Purpose |
|---|---|
| `make_articulated_drone.py` | Generate an articulated **placeholder** quad (base + 4 spinning rotors, no CAD) |
| `make_articulated_from_mesh.py` | Turn a single-body asset into an articulated one — move each prop's meshes into a jointed body |
| `test_articulated.py` | Functional test: stable hover, 5 cameras + ToF, **spinning props**, keyboard tele-op |
| `render_flight_gif.py` | Render a looping takeoff+circle flight GIF for any drone USD |

---

## Functional test controls

`test_starling_functions.py` starts the drone on the ground. In the viewport:

| Key | Action |
|---|---|
| **Page Up / Page Down** | take off / climb · descend / land |
| **↑ / ↓** | forward / back (forward = **−Y**) |
| **← / →** | strafe left / right |
| **Home** | reset upright on the ground |

> Do **not** use `Space` — it is Isaac Sim's play/pause and freezes the sim.

The controller is a standard cascaded **position + attitude geometric controller**
with **body-frame thrust**, so the drone must lean to translate and collisions
destabilize it physically (handled by PhysX) instead of being ignored.

---

## Using your own CAD

The scripts operate on geometry; they contain none. Before you start, know
**what is generic vs. what is Starling-specific** — not every drone is a
4-rotor X-quad with a "3 forward + 2 down" camera stack, so the pipeline is
**not** one-command-fits-all:

- **Generic (any airframe):** `fix_drone_usd.py` (units / up-axis / scale),
  `add_drone_physics.py` (rigid body / collider / mass), `test_drone_usd.py`,
  `verify_physics.py`, and `test_starling_functions.py` — the last simply drives
  **whatever** rotor/sensor frames it finds by attribute.
- **Starling-specific defaults:** `add_drone_frames.py` assumes a 4-rotor X-quad
  with the M0173 layout (forward = −Y, 3 forward + 2 down cameras) and hard-coded
  mesh names. A different rotor count, camera layout, or forward axis **will not**
  match those defaults — you must pass your own meshes or edit the frame
  definitions.

Steps:

1. **Obtain the CAD yourself** under the vendor's terms (Starling: ModalAI
   developer portal / forum).
2. **STEP → OBJ** in Fusion 360 / FreeCAD / Blender (single assembly; note units).
3. **OBJ → USD** via Isaac Sim's importer or `usdcat` → e.g. `drone.usd`.
4. Fix + physics (generic, works on any mesh):
   ```bash
   python v1/test_drone_usd.py    --usd drone.usd                  # inspect (expect wrong scale/axis)
   python v1/fix_drone_usd.py     --in  drone.usd --out drone_fixed.usd
   python v1/add_drone_physics.py --in  drone_fixed.usd --out drone_physics.usd
   ```
5. **Define YOUR own frames** (the airframe-specific part). Find mesh
   names/positions, then place rotor/sensor frames to match your drone:
   ```bash
   python v1/locate_prim.py      --usd drone_physics.usd --name <substr>
   python v1/add_drone_frames.py --in  drone_physics.usd --out my_drone.usd \
       --rotor-meshes <m0> <m1> <m2> <m3> \
       --front-meshes <a> <b> --down-meshes <c> <d>     # adjust to YOUR layout
   ```
   For anything unlike the Starling (no down cameras, 6 rotors, different
   forward, etc.), edit `add_drone_frames.py` — its defaults are a Starling
   *example*, not a universal spec.
6. Test: `python v1/test_starling_functions.py --usd my_drone.usd`

---

## Roadmap

- **v1** — single-rigid-body USD + rotor/camera/ToF frames, geometric flight
  controller, keyboard tele-op, camera/ToF/collision tests.
- **v2** — **articulated rotors**: the 4 propellers are split into
  revolute-jointed bodies that actually spin (velocity-driven), with a stable
  hover controller on the base and the same camera/ToF/tele-op tests.
- **next** — per-rotor thrust / aerodynamics so the spinning props generate the
  lift themselves (Articulation), matching Isaac Lab's quadcopter RL setup.

---

## License & attribution

- **Code** in this repository: [MIT](LICENSE).
- **Geometry / CAD** of any real vehicle (e.g. ModalAI Starling): **not
  included and not licensed here** — it belongs to its owner and is subject to
  the terms under which you obtained it. Do not commit vendor CAD to this repo
  (`.gitignore` blocks the common formats).
- Not affiliated with or endorsed by ModalAI.
