# Low-Level Design

Behaviour below is what `src/advaitic_cell/` executes. Customer limits live in `spec/customer_cell.json`. Command clamps in this file are the controller's, and they sit inside those limits.

## 1. Step and model

`spec.DT` is 0.004 s. `scene.build_xml` sets `timestep="0.004"` and `integrator="implicitfast"` (12 solver iterations, 8 line-search iterations, pyramidal cones). `configure` assigns `model.opt.timestep = DT` again.

Each `Mission.run` iteration calls `plant.drive`, `_step_robot`, `_watch_contacts`, optionally snapshots `qpos`, `mj_step`, optionally restores that snapshot, zeroes base velocity during `work`, then `plant.measure`. Samples and the dock check run every 25 steps (10 Hz). The nominal progress print uses a display target of 223.5 s; the mission limit itself is 270 s nominal and 360 s for the fault scenarios.

The ring is 20 box geoms (`RING_SEGMENTS`) so the hole stays open. Kits and the person are mocap. Arm-link STL collision meshes are stored with contact bits cleared. Finger capsules with `contype` and `conaffinity` 6 meet the ring. Plate keep-out on the running path is the joint-space chord test (`_chord_clear` / `_hits_plate`), not mesh contact. `CartesianServo` has a separate Z floor; the hand loop does not step it.

## 2. Base

Planner: A* on a 0.05 m grid. Cells within 0.32 m of the walls or either conveyor are blocked. `plan_to_dock` ends at the approach point 0.60 m before the dock along the dock heading, then the tracker creeps to the dock. Station 2's dock y is `spec_y + 0.008`.

`DiffDriveTracker` modes:

| Mode | Command |
|---|---|
| `follow` | 0.28 m/s if heading error < 0.6 rad, else 0.06 m/s, further limited by distance to the path end. Yaw rate `clip(1.8 * err, ±0.50)` |
| `align` | Speed 0. Yaw rate `clip(2.0 * err, ±0.50)`. Creep when yaw error < 0.04 rad |
| `creep` | Along-track `clip(0.7 * along, −0.04, 0.07)` m/s when lateral error ≤ 8 mm, else up to 0.06 m/s toward the point. Done when distance < 10 mm and yaw error < 0.015 rad |
| `done` | Zero |

`_apply_base` then clips to ±0.34 m/s and ±0.55 rad/s. If the arm is not stowed, both commands are forced to zero and `stow_step` runs. Actuator ranges in the MJCF are ±0.4 m/s on the slides and ±1.2 rad/s on yaw. Customer caps are 0.35 m/s and 0.6 rad/s.

Dock declaration (`_maybe_dock`, 10 Hz) requires tracker `done`, a fresh wall update, estimate within 22 mm and 0.030 rad of `dock_pose(station)` (the spec pose, not the 8 mm offset), and `base_speed` ≤ 0.008 m/s. The logged pose is the simulated base.

Odometry (`Estimator.predict`) takes the base `qvel`, projects it onto the estimated heading, applies a fixed scale in ±1%, and adds Gaussian noise (σ = 0.0015 m/s, σ = 0.004 rad/s). There is no separate wheel encoder or IMU body. Scan match (`correct`) uses 8 rays at z = 0.30 m against group-0 geoms, searched on ±3 cm in x and y (1.5 cm steps) and ±0.04 rad. It runs every 50 steps while `follow` is active (5 Hz) and is applied when the mean range cost is under 1.5 m. When the tracker is not in `follow`, four cardinal wall ranges at z = 0.30 m update x or y only when they agree with the estimate within 0.12 m. `_maybe_dock` calls the same update again while the tracker is `done`.

## 3. Arm

Seven arm joints. Inverse kinematics is iterative damped least squares on a scratch `MjData`, using `mj_jacSite`.

The error is 6×1: site position error, and the tool z-axis error `(0, 0, −1) − R[:, 2]` scaled by an orientation weight. The Jacobian is 6×7. Each iteration solves

```
dq = Jᵀ (J Jᵀ + 0.04 I)⁻¹ e
```

then adds a null-space step toward `POSTURE` with `posture_weight = 0.02`. The 0.04 term is constant. Default `orient_weight` is 0.35 and is reduced when the tool is already pointing down. Seeds include the current command, `BELT_POSE`, `GRASP_POSE`, and `POSTURE`. A candidate whose joint-space chord intersects a kit box is rejected.

`move_down` plays a min-jerk blend in joint space between the current command and the chosen solution (`_begin_blend`). The scalar is `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵`. Duration is `clip(max(lo, Δq / rate), lo, hi)` with `(lo, hi, rate) = (0.06 s, 0.40 s, 2.40 rad/s)` while tracking and `(0.10 s, 2.2 s, 1.20 rad/s)` otherwise. The blend command is copied into `JointStream` and the stream velocity is zeroed, so the stream's acceleration limit does not shape these moves. Hover, descend, close, lift, carry, lower, settle, open, retreat, and pre-position all call `move_down`.

`JointStream` (vmax 2.8 rad/s, amax 20 rad/s²) is what `_command` uses for stowing. Stow targets the zero vector. Position gains on the seven arm actuators are four times the Menagerie values.

`CartesianServo` in `motion.py` can sample a trapezoid in XY and a quintic on a committed vertical segment, and it holds Z at a floor until XY error is under 10 cm. `Mission` constructs one with `vmax_xy = 0.35` m/s, `vmax_z = 0.22` m/s, and `amax = 1.2` m/s² (the class defaults are 0.28, 0.26, and 1.8). The running hand loop never calls `step`. It only resets the servo at phase changes. `_approach` and `_locked` are the methods that step it, and the mission does not call them. Unit tests in `tests/test_cell_logic.py` call `CartesianServo` directly. The pick-to-seat motion that actually runs is the joint-space blend between IK solutions, aimed at the world points from `_ring_target` and `_place_hand_target`.

The finger open command is the sampled command whose fingertip gap is the smallest value at or above 0.135 m. Pinch command is 0.60. The release decision uses the measured fingertip distance.

## 4. Hand states

Heights: ring centre on the kit is 0.779 m. `GRASP_Z` is 0.056 m above that. `HOVER_Z` is 0.085 m above `GRASP_Z`. Lift and carry height are 0.968 m. Retreat height is 1.005 m, climbed at 0.45 m/s. The hand waits at station-local x = −0.08 m (place threshold −0.084 m at station B) because the seat at the upstream end of the window is out of reach.

Tool-down is the cosine of the gripper site's z axis with world −Z (`_tool_down`). Ring-up is the ring body's z-axis cosine.

| State | Advance when | Also |
|---|---|---|
| `hover` | error < 35 mm and tool-down > 0.88 | Abort after 7 s. If the ring is still upstream of local x = −0.08 m, the target stays at that x |
| `descend` | error < 30 mm, tool-down > 0.92, hand z within 30 mm of `GRASP_Z` | Abort after 6.5 s |
| `close` | after ≥ 0.30 s, gap < 0.12 m, tool-down > 0.90, then `_try_weld` | Abort after 1.4 s if still here |
| `lift` | error < 55 mm, tool-down > 0.88, ring z ≥ ring-centre + 30 mm | XY frozen at the grasp column. Abort if tool-down < 0.70 after 0.40 s, or after 4.5 s |
| `carry` | bottle local x ≥ −0.08 m (A) or −0.084 m (B), radial < 28 mm, ring z ≥ 0.93 m, tool-down > 0.88 | Waits on a column. Lead passed to the bottle target is 80 ms |
| `lower` | radial < 10 mm, ring height above kit top in [0.128, 0.145] m, ring-up > 0.90 | Abort if tool-down < 0.80, or after 3 s |
| `settle` | after 0.10 s: radial < 3.6 mm, z in [0.128, 0.146], ring-up > 0.94 | After 0.50 s the gates loosen to 4.8 mm and ring-up > 0.90. After 1.20 s it opens anyway |
| `open` | gap > 0.090 m and radial < 2.2 mm and z in [0.128, 0.142] and ring-up > 0.94 | At 2.2 s: gap > 0.090 m, radial < 2.5 mm, z ≤ 0.144, ring-up > 0.92. At 2.8 s the weld is released without those gates |
| `retreat` | hand z within 10 mm of 1.005 m, or 0.75 s | `ASSEMBLY_DONE` once `phase_t` ≥ 0.05 s |

`lower` resets trim to `(0.0055, 0)` and then tracks `_place_hand_target` with `lead_s = 0.06`. That function, `_update_place_trim`, and `_seat_geometry` use the plant carrier's true bottle position and the ring body's simulator pose. Trim is an integral of lead and cross error, gain 0.05 at A and 0.08 at B, clamped to ±12 mm. Seat height command is `KIT_TOP_Z + 0.133` minus 6 mm at A and 10 mm at B. Station lead added along the belt is 5.5 mm at A and 2.2 mm at B.

`_try_weld` engages when `closest_ring` distance is under 16 mm and the finger gap is under 0.11 m. `engage_weld` writes the current gripper-to-ring pose into `eq_data` and activates the weld. Ring contact bits switch to carry mode (bit 4 only) until release. The jaws open in `open` while the weld is still active. `_release` clears `eq_active`.

If the bottle's encoder x passes 0.38 m the job goes to `abort`. From `carry` or `lower`, a welded ring that is still high and roughly centred goes back to `lower` (`finish-place`). From `lower`, `settle`, or `open`, a welded ring already inside a loose seat gate is opened and reported (`abort-claim`). Otherwise the ring is released, the kit is abandoned, and `ASSEMBLY_DONE` is not sent.

## 5. Pick sensing

`_ring_target` calls `LinePerception.ring_point`. Unless `--oracle-perception` is set, that is `ENTRY_X + encoder delta`, refined by a 5×5 downward ray pattern (±15 mm and ±30 mm) from 0.12 m above the prediction. Hits are kept when z is in [0.780, 0.792] m and within 45 mm of the prediction. The mission then clamps the station-local result to ±5 mm of the encoder prediction. At least two hits are required for the centroid; otherwise the encoder point is used.

## 6. Human stop and PLC retry

See `solution/HLD.md` sections 4 and 5. The position hold is the `qpos` snapshot around `mj_step` in `Mission.run`. `arm.brake` zeroes arm and finger `qvel` and does not copy `qpos` into `ctrl`.

The retry frame logged as `retry_request_hex` is built in `_drop_then_retry`. The second write is `PLCClient.command` with the same sequence, station, kit, and command. Both committed hex strings are 21-byte FC16 frames (42 hex characters) writing registers 100–103.

## 7. Video

Optional. `Mission` opens a 1920×1080 MuJoCo renderer when `--video` is set. Frames are raw RGB into `ffmpeg`. The encoder arguments are `h264_nvenc` with preset `p1` and tune `ull` when `ffmpeg -encoders` contains `h264_nvenc`; otherwise `libx264` with `ultrafast`. Per-camera files go under `<video>/../views/` for overview, station_a, station_b, top, follow, and lidar. The mosaic is also 1920×1080.

`gl_backend.prepare_egl` runs only when `MUJOCO_GL` is `egl`. On WSL ( `/dev/dxg` or `libdxcore.so` ) it sets Mesa's `d3d12` Gallium driver. Headless `cell-run` without `--video` does not need that path. `scripts/run_gpu.sh` selects `MUJOCO_GL=osmesa` for its `physics` mode and `egl` otherwise, and it calls `/opt/ars-venv`.

No frame-time or speedup figure is encoded in the controller. Do not read a millisecond-per-frame number out of these docs as a property of the code.

## 8. Placement ground truth

`carry`, `lower`, `settle`, `open`, and `retreat` call `_place_hand_target` or `_bottle_target`, which prefer `plant.by_kit[kit_id].arrival_t` and `.speed`. Seat gates call `_seat_geometry`, which reads `data.xpos` of the ring and the same true bottle position. `_update_place_trim` does the same. This is true when `trace["oracle"]` is false. Pick targeting does not use that pose; it uses the encoder and the ray ROI.
