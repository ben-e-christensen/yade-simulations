# Stewart platform motion — reference for YADE sim

Source: octo/src/main.cpp + kinematics.h (firmware), rig_controller_pkg/cycle_controller.py + gui.py (Pi orchestration). Read 2026-10-22.

## What the experiment cycle does
wiggle (W orbit) -> S (return level) -> fixed settle delay -> photo + CNC raster of charge probe (platform static) -> re-home CNC -> repeat.
GUI defaults: tilt 10 deg, period 4 s, 3 rotations (12 s wiggle), settle 3 s.

## W (orbit) motion, exactly as commanded
- Platform frame origin held fixed at (0, 0, NEUTRAL_Z) in base frame; no translation.
- Orientation = single axis-angle rotation by tilt theta about horizontal axis a = (sin phi, -cos phi, 0), phi = 2*pi*t/T (Rodrigues), recomputed at 50 Hz.
- Downhill direction = (-cos phi, -sin phi): low point starts at -x when W is sent and rotates CCW viewed from +z.
- Pose is periodic (pure nutation, no accumulated yaw).
- Pivot height only adds a small circular floor translation (<1% effect); sim assumes floor at pivot.

## Hardware limits (NEUTRAL_Z = 180, crank 30 mm, rod 140 mm)
- Max full-circle orbit tilt: ~11.5 deg (README's 16.5 is stale, computed at NEUTRAL_Z=178).
- 10 deg / 4 s orbit: peak needed 507 steps/s vs 500 cap; steady-state lag <~2 deg crank; ~1.3 s to lock on at W start; S return ~1.8 s.

## Bead + plate material facts (2026-10-23)
- Beads: McMaster solid PTFE, 1/8 in, density 0.077 lb/in^3 = 2131 kg/m^3, listed CoF 0.05-0.08.
- Plate: aluminum.
- Restitution PTFE bead on aluminum plate, measured by audio bounce timing (restitution_from_audio.py, 5 drops, 28 ratios): e = 0.663 (median 0.676, std 0.059, SE 0.011), flat vs impact speed 0.2-2 m/s.
- Drum sims used density 1190 and mu 0.4 — wrong for PTFE; likely contributor to drum sims piling higher than the rig.
- No-skid threshold at 10 deg tilt = 2.88 deg friction angle; real PTFE-Al (2.9-4.6 deg) sits right at it — real beads may partly skid.

## Slip test (stewart_slip_test.py) history
- v1: Cundall-Strack, zero damping -> bead bounced indefinitely (h up to ~5 mm, floor contact 1-7%), reached wall every step. Results unusable. Platform kinematics verified good (track err ~6e-7 deg).
- v2: Hertz-Mindlin with RESTITUTION=0.66, PTFE density, per-pair friction via MatchMaker (bead-bead 0.065), per-step friction verification, h_mean/h_max in summary, slide fraction normalized by floor-contact time. Figure-8, 10 deg, 1.4 s period, 30 s steps, ramp 5 cycles.

## Sim decisions
- Container: ring ID 201.2 mm, 1 mm wall, raised to 0.5 in (12.7 mm) in sim (real ~3.2 mm). Floor built as one large triangle facet (no STL) to avoid facet-edge double contacts.
- Bead count tunable, up to ~1000, centered hex patch.
- Rolling resistance not yet added — next calibration after slip test.