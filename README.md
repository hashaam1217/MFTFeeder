# MFT Feeder

Custom vacuum sheet feeder for singulating PE-coated liquid packaging board (292 x 394 mm, 42.0 g, 365 gsm) from a magazine elevator onto the flighted conveyor feeding a CCM Wirecut line.

## Why custom

Commercial friction-roller feeders were ruled out on substrate physics: the PE coating defeats friction-roller grip. That constraint anchors the project's technological uncertainty and drives the vacuum pick approach.

## Scope

- **Elevator**: 300-sheet magazine stack (~12.6 kg), ~140 mm travel. Two base-driven Tr8x2 self-locking Acme screws, single NEMA 17 with a 1:1 closed GT2 sync loop, KFL08 flange bearings.
- **X axis**: MGN15H 500 mm rail (300 mm travel) on a 2020 beam, NEMA 23 (3.0 Nm) with closed-loop driver, GT2 belt drive.
- **Z head**: belt drive on MGN12H 150 mm rails, carrying only the vacuum tube and cup; venturi and vacuum switch stay on the stationary mast (dead volume, not moving mass, dominates cycle time).
- **Frame**: 800 mm all-2020 cube, racking controlled by shear panels and diagonals.
- **Controls**: Arduino Uno R4 WiFi (5 V throughout), ADS1115 for amplitude sensing, 24 V rail (36 V conditional on traverse-time requirements), DIN-rail power with e-stop relay.
- **Sensing**: head-mounted load cell for double-pick detection (single vs double sheet by weight), vacuum switch for pick confirmation, SS-5GL mechanical endstops.

Staging: bench commissioning, then experiments EXP-1 through EXP-6. Stage A uses carry-aside discard ("pick static / carry-aside discard"); Stage B adds velocity-matched release onto the conveyor.

## Where it stands (August 2026)

**Locked**: overall architecture above, including motor/driver selections, elevator screw and sync-loop design, pneumatic component placement, and the retirement of the X-static rule (X now carries each picked sheet to the drop zone as the discard path).

**In progress (CAD)**: the Z head and elevator parts are being modeled in Fusion — ZArm, ZGantry, Z-Bracket, RodHolder, LongRodHolder, Clip, and the magazine Elevator.

Active CAD to-dos:
- ZArm: enlarge the tube bore to ~12.8-13 mm; add four rear holes for T-nut side-mounting to the extrusion; integrate the SS-5GL endstop (19.8 x 6.4 mm footprint).
- ZGantry: add mounting features for the SS-5GL endstop.

**Open items**: elevator detailing (screw-end treatment, guide-rod bearings, bed material and teeter), Wirecut flight pitch measurement (gates the 36 V decision), head weigh-in (gates final load cell spec), e-stop wiring before first X motion, BOM completion, T-nut order (M3 drop-in for rails, M5 slide-in elsewhere), QA/sanitation sign-off.

## Design rules learned the hard way

- Re-derive component specs from current load and travel; read the actual datasheet (the recurring failure was parts that matched checked attributes and failed on the unchecked one).
- Separate torque and thrust paths; keep preload in metal, not polymer.
- Gravity-preloaded axes need no anti-backlash nut.
- Never loop an open belt into a rotating transmission.
- Material escalation: PETG, then PET-CF, then 6061; printed parts excluded from sustained preload.
