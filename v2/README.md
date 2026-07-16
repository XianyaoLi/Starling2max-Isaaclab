# v2 — articulated rotors

The 4 propellers are split into independent rigid bodies, each on a revolute
joint with a velocity drive, so they **actually spin** (diagonal pairs
counter-rotate). A stable hover controller runs on the base while the props
turn, alongside the same 5 cameras + ToF depth and keyboard tele-op as v1. See
the [main README](../README.md) for the full script table and usage.

```bash
# placeholder (no CAD): articulate + test
python v2/make_articulated_drone.py --out articulated_drone.usd
python v2/test_articulated.py --usd articulated_drone.usd
```

<p align="center">
  <img src="starling_v2_flight.gif" width="55%" alt="Starling 2 Max — v2 articulated flight, spinning props">
</p>

> **Render of the real vehicle**, shown only to illustrate the kind of result
> this toolchain can achieve on a real airframe. The CAD/model itself is **not**
> distributed (see the main README's License section). Not affiliated with, or
> endorsed by, ModalAI.
