# Constraints the implementation uses

Customer numbers come from `spec/customer_cell.json`. Controller numbers come from `src/advaitic_cell/`. Outcomes come from the committed files in `results/`. An independent re-run is quoted only in section 7, with the figures from that re-run.

## 1. Customer acceptance

Checked by the plant and by `systems-check`, not by the robot's claim.

| Quantity | Spec |
|---|---|
| Radial error | ≤ 4 mm |
| Ring-centre height above the bottle base | 130–136 mm |
| Tilt | ≤ 0.1745 rad (10°) |
| Settled hold | ≥ 0.5 s, released, bottle contact, no kit contact |
| Cycle | ≤ 12 s from the bottle crossing local x = −0.30 m to `seat_verified` |
| Nominal mission | ≤ 270 s |
| Fault mission | ≤ 360 s |
| Belt speed | 25–35 mm/s, pitch 450 mm, belts do not pause |
| Dock | 25 mm and 0.0349 rad (2°) |
| Transit | ≤ 0.35 m/s and ≤ 0.6 rad/s |
| Base speed while assembling | ≤ 10 mm/s |
| Kits | 5 accepted at A, then 5 at B |

Spec geometry the scene is built from: ring OD 80 mm, ID 48 mm, thickness 8 mm, mass 0.04 kg; bottle body diameter 60 mm; cell 6×6 m; belt top 0.75 m. The simulated ring is 20 boxes on a 32 mm radius, each 8 mm thick, with the hole left open. The bottle body is a cylinder of radius 30 mm plus a flange and a guide cone. The scene does not apply a separate ±0.5 mm manufacturing tolerance.

On the committed `seat_verified` events, radial error is 3.05–3.28 mm, height is 0.1338–0.1356 m, and tilt is 0.0002–0.054 rad. Accepted inspections in the nominal seed 0 log have tilt below 0.001 rad. Seed 1's inspection stream includes rejects (kits 203 and 205); those tilts are not in that band.

## 2. Controller clamps

| What | Value in code |
|---|---|
| Physics step | 0.004 s, `implicitfast` |
| Base command | ±0.34 m/s, ±0.55 rad/s, and 0 unless the arm is stowed |
| Follow speed | 0.28 m/s when heading error < 0.6 rad, else 0.06 m/s |
| MJCF base actuators | slides ±0.4 m/s, yaw ±1.2 rad/s |
| Dock accept | estimate < 22 mm and < 0.030 rad from the spec dock, measured base speed ≤ 8 mm/s |
| Station B creep goal | spec dock y + 8 mm |
| Arm joints | 7. Stow is the zero pose, within 0.22 rad |
| IK damping | `0.04 * I` added to `J Jᵀ`. `J` is 6×7 |
| Joint stream, used while stowing | 2.8 rad/s, 20 rad/s² |
| Cartesian servo object | Constructed at 0.35 m/s XY, 0.22 m/s Z, 1.2 m/s². The hand loop does not step it |
| Finger pinch command | 0.60. Open command is the pose whose gap is at least 0.135 m |
| Weld engage | closest ring < 16 mm and gap < 0.11 m |
| Weld release | gap > 0.090 m, plus the seat gates in `solution/LLD.md` |
| Place trim | gain 0.05 (A) or 0.08 (B), clamp ±12 mm |
| Human rays | 64 at z = 0.6 m. Range test 1.0 m from the base origin, or the aisle rectangle |
| Human clear | 5 consecutive misses and ≥ 2.0 s, then `resume` 0.2 s later |
| PLC poll | every 25 steps (100 ms) in `work` |
| PLC port | 1502 |

The arm is not commanded with a 6-element velocity limit of 1.4 rad/s, and the base is not commanded with a 0.55 m/s cruise or a 1.0 m/s² linear cap. Gripper pinch force is not a controlled setpoint.

## 3. Committed timing

From `results/*/summary.json` and `trace.json`.

| Run | Sim time | `depart_a_t` | `dock_b_t` | Slowest accepted cycle |
|---|---:|---:|---:|---:|
| nominal 0 | 223.50 s | 114.7 s | 141.0 s | 11.980 s (kit 201) |
| nominal 1 | 249.00 s | 112.7 s | 139.7 s | 10.208 s |
| nominal 2 | 223.80 s | 115.2 s | 141.8 s | 9.988 s |
| human stop | 319.44 s | 196.1 s | 223.5 s | 9.228 s |
| PLC retry | 223.50 s | 114.7 s | 141.0 s | 11.980 s (kit 201) |

Seed belt speeds and first-release phases, from `scenario_params` and confirmed in `trace.json`: seed 0 is 0.033163 m/s and 0.6006 s; seed 1 is 0.033279 m/s and 0.6122 s; seed 2 is 0.033395 m/s and 0.6238 s.

Measured docks on nominal seed 0 (the `dock` events): A `(2.992, 1.997, −1.557)` against `(3, 2, −π/2)`, residual about 8.5 mm and 0.77°. B `(3.004, 3.997, 1.583)` against `(3, 4, +π/2)`, residual about 5.2 mm and 0.70°. `dock_errors_m` in that trace is the estimate-versus-base error at the two docks: 2.5 mm and 1.9 mm. Seed 1 and seed 2 store station B's event yaw unwrapped, near −4.70 rad; the wrapped residual is in `dock_offsets_m`.

## 4. Safety behaviour that is implemented

In `human_stop`, `_work` casts the person rays after the intruder script has started. The stop starts only while a job is active, and the position hold then runs until resume. Navigation does not call this check. There is no speed scaling with range, no 1.8 m warning zone, and no 500 ms clear timer. The committed human-stop log records `human_enter` and `stop_command` at the same sample (t = 35.928 s), `human_exit` at 38.500 s, and `resume` at 38.700 s. `belt_contacts` on every committed summary is 0. The person geom has `contype = 0`.

The spec text calls this simulation acceptance behaviour, not a certified safety function.

## 5. Modbus behaviour that is implemented

FC03 reads and FC16 writes of the map in `solution/HLD.md`, on port 1502. The sequence increments by one after an acknowledgement. A duplicate sequence with the same payload returns status 2 and is not applied again.

The retry path runs for one command: the first `ASSEMBLY_DONE` of `plc_retry`. It does not scan CRC, and Modbus/TCP frames in this stack have no CRC field. There is no 500 ms watchdog and no 50/100/200 ms backoff. The wait loop sleeps 10 ms while polling register 1, and the socket timeout is 2 s.

## 6. Determinism

`scenario_params` uses a fixed LCG, so a seed always selects the same belt speed and release phase. Sensor noise uses `numpy.random.Generator`. The physics step is `implicitfast` at 4 ms.

The committed logs are one set of outputs. They are not a bit-identical reproduction on every machine. Section 7 records the mismatch that was measured.

## 7. Re-run that was measured

One Linux re-run with MuJoCo 3.8.1 from `uv.lock`, reported with the figures below and not re-measured in this change:

| Case | Belt speed | Result |
|---|---:|---|
| Nominal seed 0 | 33.16 mm/s | Station A matched the committed log. Kit 201 took 12.036 s. The committed log has 11.980 s. The 12 s cycle check fails |
| Seed 16 | 25.015 mm/s | 6/10 accepted at the 270 s cap. 7 kits seated. Slowest seated cycle 12.31 s. Kits 103, 104, and 202 rejected |
| Seed 102 | 34.967 mm/s | 8/10 accepted at the 270 s cap. 16 kits arrived, 9 seated |

Seeds 0–2 sit in a narrow part of the 25–35 mm/s range. Those three re-run rows are the evidence for that limit. No other speeds were measured for this document.
