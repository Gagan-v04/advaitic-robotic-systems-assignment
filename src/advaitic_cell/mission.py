"""One mission controller for both stations.

Navigation consumes the pose estimate. The arm consumes joint encoders, the
conveyor encoder and wrist depth. Accepted counts are only read from the PLC.
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from advaitic_cell.arm import GRASP_LIFT, ArmController, engage_weld, release_weld
from advaitic_cell.frames import (
    bottle_local_x,
    local_to_world,
    seat_world,
    world_to_local,
    wrap,
    yaw_quat,
)
from advaitic_cell.motion import CartesianServo
from advaitic_cell.navigate import DiffDriveTracker, plan_to_dock
from advaitic_cell.perception import Estimator, LinePerception, RayLog, person_visible
from advaitic_cell.plant import Plant
from advaitic_cell.plc_adapter import RobotPLC
from advaitic_cell.records import EventLog, write_json
from advaitic_cell.scene import base_pose, base_speed, load_cell, set_base_pose
from advaitic_cell.spec import (
    BOTTLE_OFFSET,
    DT,
    ENTRY_X,
    KIT_TOP_Z,
    RING_AHEAD_OF_BOTTLE,
    RING_CENTER_Z,
    RING_OFFSET,
    SEAT_Z,
    RunParams,
    dock_pose,
    start_pose,
)
from advaitic_systems.modbus import PLCServer

# The visual jaws hang below the tip midpoint. The pinch pose keeps that
# midpoint above the ring so the meshes meet the OD and stay on the plate.
GRASP_Z = RING_CENTER_Z + GRASP_LIFT
HOVER_Z = GRASP_Z + 0.085
# Tool-up transit. The wrist cannot hold world -Z this high. The arm stays
# above the plate until it is over the ring, then a joint chord pitches down.
TRAVEL_Z = 1.15
SEAT_WORLD_Z = KIT_TOP_Z + SEAT_Z
# Downward IK is within 5 mm only once the bottle is at or downstream of this
# station-local x. The hand waits there and the belt brings the bottle to it.
PLACE_X = -0.08
# Cone tip is at kit-top + 0.185 m = 0.960 m. Carry the ring above that so
# the approaching bottle cannot cam the annulus. Tool-down recovers on the
# vertical drop onto the flange (seat 0.908 m).
RING_CARRY_Z = 0.968
# Proven tool-down hand height at the pick column, 14 cm from the bottle.
LIFT_Z = 0.968
# Proven safe vertical clearance above the 0.955m bottle top keeping tool-down IK solution.
RETREAT_Z = 1.005
PREPOSITION_Z = 1.000


@dataclass
class Job:
    kit_id: int
    t_seen: float
    encoder_seen: float
    hand: str = "hover"
    phase_t: float = 0.0
    observed: list[float] = field(default_factory=list)
    bottle_local: float | None = None
    latch_encoder: float | None = None
    slot: int | None = None
    welded: bool = False
    released: bool = False
    lift_point: np.ndarray | None = None
    grasp_offset: np.ndarray | None = None
    grasp_world: np.ndarray | None = None
    place_drop: bool = False
    reported: bool = False
    recenters: int = 0
    retreat_z: float | None = None
    retreat_y: float | None = None
    trim_x: float = 0.0055
    trim_y: float = 0.0


class Intruder:
    """Scripted person. The robot reacts to rays, not to this clock."""

    def __init__(self, enabled: bool, duration: float):
        self.enabled = enabled
        self.home = np.array([0.40, 0.40, 0.0])
        # Off the aisle from the start pose to dock A, and outside the 1 m / zone
        # boundary so the person is not visible until they actually walk in.
        self.waypoint = np.array([1.15, 2.20, 0.0])
        # West of dock A, clear of the 0.28 m base hull. A G1 standing at the
        # old 2.62 m spot overlaps the chassis.
        self.spot = np.array([2.35, 2.45, 0.0])
        self._last_pos = self.home.copy()
        self._yaw = math.atan2(self.spot[1] - self.home[1], self.spot[0] - self.home[0])
        self.approach = 0.70
        self.leave = 0.50
        self.hold = max(2.2, duration - 0.8)
        self.started = False
        self.t0 = 0.0

    def trigger(self, t: float) -> None:
        if self.enabled and not self.started:
            self.started = True
            self.t0 = t

    def position(self, t: float) -> np.ndarray:
        if not self.enabled:
            return self.home
        if not self.started:
            return self.waypoint
        tau = t - self.t0
        if tau < self.approach:
            blend = tau / self.approach
            return (1.0 - blend) * self.waypoint + blend * self.spot
        if tau < self.approach + self.hold:
            return self.spot.copy()
        gone = tau - self.approach - self.hold
        if gone < self.leave:
            blend = gone / self.leave
            return (1.0 - blend) * self.spot + blend * self.home
        return self.home.copy()

    def write(self, data, index, t: float) -> None:
        pos = self.position(t)
        delta = pos[:2] - self._last_pos[:2]
        if float(np.linalg.norm(delta)) > 1e-4:
            self._yaw = math.atan2(float(delta[1]), float(delta[0]))
        self._last_pos = pos.copy()
        data.mocap_pos[index.human_mocap] = pos
        data.mocap_quat[index.human_mocap] = yaw_quat(self._yaw)


def _look_quat(eye: np.ndarray, target: np.ndarray) -> np.ndarray:
    z = np.asarray(eye, dtype=float) - np.asarray(target, dtype=float)
    z /= max(np.linalg.norm(z), 1e-9)
    x = np.cross(np.array([0.0, 0.0, 1.0]), z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.column_stack((x, y, z)).ravel())
    return quat


class _VideoWriter:
    """Stream overview frames to an mp4. Frames are not kept in memory."""

    _nvenc_checked = False
    _has_nvenc = False

    @classmethod
    def _check_nvenc(cls) -> bool:
        if not cls._nvenc_checked:
            try:
                res = subprocess.run(
                    ["ffmpeg", "-hide_banner", "-encoders"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=2,
                )
                cls._has_nvenc = "h264_nvenc" in res.stdout
            except Exception:
                cls._has_nvenc = False
            cls._nvenc_checked = True
        return cls._has_nvenc

    def __init__(self, path: Path, width: int, height: int, fps: int):
        path.parent.mkdir(parents=True, exist_ok=True)
        if self._check_nvenc():
            encoder_args = ["-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ull"]
        else:
            encoder_args = [
                "-c:v", "libx264", "-preset", "ultrafast",
                "-tune", "zerolatency", "-threads", "2",
            ]
        self.proc = subprocess.Popen(
            [
                "ffmpeg", "-y", "-loglevel", "error",
                "-f", "rawvideo", "-pix_fmt", "rgb24",
                "-s", f"{width}x{height}", "-r", str(fps),
                "-i", "-",
                "-an", *encoder_args, "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", str(path),
            ],
            stdin=subprocess.PIPE,
            bufsize=8 * 1024 * 1024,
        )
        self.path = path

    def write(self, frame: np.ndarray) -> None:
        if self.proc.stdin is None:
            return
        self.proc.stdin.write(np.ascontiguousarray(frame).tobytes())

    def close(self) -> None:
        if self.proc.stdin is not None:
            self.proc.stdin.close()
        code = self.proc.wait()
        if code != 0:
            raise RuntimeError(f"ffmpeg exited {code} writing {self.path}")


def sim_time(step: int) -> float:
    return round(step * DT, 6)


class Mission:
    def __init__(self, params: RunParams, out_dir, record: bool = False, video_path=None, video_fps: int = 30):
        self.params = params
        self.out_dir = out_dir
        self.record = record
        self.video_path = video_path
        self.video_fps = video_fps
        self.frame_interval = 1.0 / max(1, video_fps)
        self.next_frame_t = 0.0
        self._video = None
        self._videos: dict[str, _VideoWriter] = {}
        self.model, self.data, self.index = load_cell()
        sx, sy, syaw = start_pose()
        set_base_pose(self.data, self.index, sx, sy, syaw)
        mujoco.mj_forward(self.model, self.data)
        self.server = self._bind_plc()
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        port = int(self.server.server_address[1])
        self.plant = Plant(
            self.model, self.data, self.index, self.server,
            params.belt_speed_mps, params.release_phase_s, EventLog(),
        )
        self.log = self.plant.log
        self.plc = RobotPLC(port=port)
        rng = np.random.default_rng(params.seed + 17)
        self.raylog = RayLog()
        self.est = Estimator(sx, sy, syaw, rng, self.raylog)
        self.perception = LinePerception(
            self.model, self.data, self.index, params.belt_speed_mps, rng, params.oracle_perception,
            self.raylog,
        )
        self.arm = ArmController(self.model, self.data, self.index)
        self.cart = CartesianServo(vmax_xy=0.35, vmax_z=0.22, amax=1.2)
        self.cart.reset(self.arm.midpoint())
        self._approaching = False
        for address, actuator in zip(self.index.finger_q, self.index.finger_act, strict=True):
            self.data.qpos[address] = self.arm.open_cmd
            self.data.ctrl[actuator] = self.arm.open_cmd
        mujoco.mj_forward(self.model, self.data)
        self.gripper = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link_gripper"))
        self.robot_bodies = self._robot_bodies()
        self.intruder = Intruder(params.scenario == "human_stop", params.human_duration_s)
        self.intruder.write(self.data, self.index, 0.0)
        self.station = 1
        self.phase = "nav"
        self.step = 0
        self.t = 0.0
        self.job: Job | None = None
        self.queue: list[tuple[int, float, float]] = []
        self.known: set[int] = set()
        self.accepted = 0
        self.ready_sent = False
        self.not_ready = False
        self.retried = False
        self.hand_err = 1.0
        self.prev_mid = self.arm.midpoint().copy()
        self.freeze: dict | None = None
        self.trace: dict = {
            "oracle": bool(params.oracle_perception),
            "scenario": params.scenario,
            "seed": params.seed,
            "belt_speed_mps": params.belt_speed_mps,
            "release_phase_s": params.release_phase_s,
            "dock_offset_m": [0.0, 0.0, 0.0],
            "grasps": [],
            "abandoned": [],
            "belt_contacts": 0,
            "min_clearance_m": None,
            "dock_errors_m": [],
            "depart_a_t": None,
            "dock_b_t": None,
        }
        self.frames: list[np.ndarray] = []
        self.renderer = None
        self.video_every = 250
        self.pane_w = 640
        self.pane_h = 360
        follow_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "follow_cam")
        self.follow_mocap = int(self.model.body_mocapid[follow_id]) if follow_id >= 0 else None
        if video_path is not None:
            self.video_every = max(1, int(round(1.0 / (video_fps * DT))))
            self.pane_w, self.pane_h = self._open_video_renderer()
            if self.renderer is None:
                raise RuntimeError(f"cannot record {video_path}: 1920x1080 renderer did not start")
            views = video_path.parent / "views"
            for name in ("overview", "station_a", "station_b", "top", "follow", "lidar"):
                self._videos[name] = _VideoWriter(views / f"{name}.mp4", self.pane_w, self.pane_h, video_fps)
            # Mosaic is also 1920x1080: four downscaled panes, not a 4K stitch.
            self._video = _VideoWriter(video_path, 1920, 1080, video_fps)
        elif record:
            self._open_renderer(240, 320)
        self._begin_nav(1)
        self.wall_t0 = time.perf_counter()

    def _bind_plc(self) -> PLCServer:
        try:
            return PLCServer(("127.0.0.1", 1502))
        except OSError:
            return PLCServer(("127.0.0.1", 0))

    def _robot_bodies(self) -> set[int]:
        bodies = set()
        for body in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
            if name == "base_link" or name.startswith("link_"):
                bodies.add(body)
        return bodies

    def _begin_nav(self, station: int) -> None:
        self.station = station
        self.est.correct(self.model, self.data, self.index)
        path, dock_xy, dock_yaw = plan_to_dock((self.est.x, self.est.y), station)
        self.tracker = DiffDriveTracker(path, dock_xy, dock_yaw)
        self.phase = "nav"
        self.ready_sent = False
        self.accepted = 0
        self.queue.clear()

    def run(self) -> None:
        self._log_sample()
        limit = self.params.mission_limit_s
        while self.phase != "done" and self.t <= limit:
            self.plant.drive(self.t)
            self._step_robot()
            self._watch_contacts()
            held_pose = None
            held_ring = None
            if self.freeze is not None:
                # Safety hold. One integration step still creeps the base, and the
                # wrist's world position moves with it even when joint velocity
                # is reported as zero. Put the pose back after the step. The
                # welded ring is held too, or the constraint winds up and flings it.
                held_pose = (
                    [float(self.data.qpos[q]) for q in self.index.arm_q],
                    [float(self.data.qpos[q]) for q in self.index.finger_q],
                    [float(self.data.qpos[q]) for q in self.index.base_q],
                )
                if self.job is not None and self.job.welded and self.job.slot is not None:
                    slot = self.index.slots[self.job.slot]
                    held_ring = (
                        slot,
                        self.data.qpos[slot.ring_q:slot.ring_q + 7].copy(),
                    )
            mujoco.mj_step(self.model, self.data)
            if held_pose is not None:
                # Element-wise. A fancy index on MjData.qpos does not write back.
                for address, value in zip(self.index.arm_q, held_pose[0], strict=True):
                    self.data.qpos[address] = value
                for address, value in zip(self.index.finger_q, held_pose[1], strict=True):
                    self.data.qpos[address] = value
                for address, value in zip(self.index.base_q, held_pose[2], strict=True):
                    self.data.qpos[address] = value
                if held_ring is not None:
                    slot, ring_q = held_ring
                    self.data.qpos[slot.ring_q:slot.ring_q + 7] = ring_q
                    self.data.qvel[slot.ring_v:slot.ring_v + 6] = 0.0
                self.arm.brake()
                mujoco.mj_forward(self.model, self.data)
            if self.phase == "work":
                # Wheel brake. One integration step of arm reaction is discarded so a
                # docked sample cannot show a base speed that clears the dock.
                for dof in self.index.base_v:
                    self.data.qvel[dof] = 0.0
            self.plant.measure(self.t)
            if self.step % 250 == 0:
                self._note_belt()
                target_s = 223.5 if self.params.scenario == "nominal" else self.params.mission_limit_s
                pct = min(100.0, (self.t / target_s) * 100.0)
                bar_len = 20
                filled = int(bar_len * pct / 100.0)
                bar = "=" * filled + (">" if filled < bar_len else "=") + " " * (bar_len - filled - (1 if filled < bar_len else 0))
                elapsed = time.perf_counter() - getattr(self, "wall_t0", time.perf_counter())
                rate = elapsed / max(0.1, self.t)
                eta = max(0.0, target_s - self.t) * rate
                kit_str = f"kit {self.job.kit_id} {self.job.hand}" if self.job else "idle"
                freeze_str = " [STOP]" if self.freeze is not None else ""
                acc_stn1 = self.accepted if self.station == 1 else 5
                acc_stn2 = self.accepted if self.station == 2 else 0
                log_line = (
                    f"[{bar}] {pct:5.1f}% | t={self.t:5.1f}/{target_s:.0f}s | "
                    f"stn {self.station} {self.phase:<4} | {kit_str:<16} | "
                    f"acc: S1={acc_stn1}/5, S2={acc_stn2}/5 | "
                    f"wall={int(elapsed//60):02d}m{int(elapsed%60):02d}s ETA={int(eta//60):02d}m{int(eta%60):02d}s ({rate:.1f}x sim){freeze_str}"
                )
                print(log_line, flush=True)
                sys.stderr.write(log_line + "\n")
                sys.stderr.flush()
            self.step += 1
            self.t = sim_time(self.step)
            if self.renderer is not None:
                self._maybe_frame()
            if self.step % 25 == 0:
                self._log_sample()
                self._maybe_dock()
                if self.step % 1250 == 0:
                    hand = self.job.hand if self.job else "-"
                    mid_z = float(self.arm.midpoint()[2])
                    rss = ""
                    try:
                        for line in Path("/proc/self/status").read_text().splitlines():
                            if line.startswith("VmRSS:"):
                                rss = line.split(":", 1)[1].strip()
                                break
                    except OSError:
                        pass
                    print(
                        f"t={self.t:.1f}s phase={self.phase} station={self.station} "
                        f"accepted={self.accepted} hand={hand} err={self.hand_err:.3f} "
                        f"midz={mid_z:.3f} tool={self._tool_down():.2f} "
                        f"ncon={int(self.data.ncon)} rss={rss}",
                        flush=True,
                    )
        self._finish_tail()

    def _step_robot(self) -> None:
        if self.intruder.started or self.intruder.enabled:
            self.intruder.write(self.data, self.index, self.t)
        self._predict()
        if self.phase == "nav" and self.step % 50 == 0:
            if self.tracker.mode == "follow":
                self.est.correct(self.model, self.data, self.index)
            else:
                self.est.walls(self.model, self.data, self.index)
        if self.phase == "nav":
            # Finish stow while rolling to B (started in `_leave_a`).
            if self.station == 2 and not self.arm.is_stowed():
                self.arm.stow_step(DT)
                self.arm.open_fingers()
            self._drive_nav()
            self._maybe_line_ready()
            if self.ready_sent and self.step % 25 == 0:
                self._poll()
        elif self.phase == "work":
            self._work()

    def _predict(self) -> None:
        vx = float(self.data.qvel[self.index.base_v[0]])
        vy = float(self.data.qvel[self.index.base_v[1]])
        wz = float(self.data.qvel[self.index.base_v[2]])
        forward = vx * math.cos(self.est.yaw) + vy * math.sin(self.est.yaw)
        self.est.predict(forward, wz, DT)

    def _drive_nav(self) -> None:
        if self.tracker.done and not self._estimate_docked():
            self.tracker.mode = "creep"
        forward, yaw_rate = self.tracker.command(self.est.x, self.est.y, self.est.yaw)
        if self.tracker.done:
            forward, yaw_rate = 0.0, 0.0
        self._apply_base(forward, yaw_rate)

    def _apply_base(self, forward: float, yaw_rate: float) -> None:
        if not self.arm.is_stowed():
            self.arm.stow_step(DT)
            forward, yaw_rate = 0.0, 0.0
        forward = float(np.clip(forward, -0.34, 0.34))
        yaw_rate = float(np.clip(yaw_rate, -0.55, 0.55))
        yaw = self.est.yaw
        self.data.ctrl[self.index.base_act[0]] = forward * math.cos(yaw)
        self.data.ctrl[self.index.base_act[1]] = forward * math.sin(yaw)
        self.data.ctrl[self.index.base_act[2]] = yaw_rate
        if abs(forward) < 1e-4 and abs(yaw_rate) < 1e-4:
            self._hold_base()

    def _hold_base(self) -> None:
        for actuator in self.index.base_act:
            self.data.ctrl[actuator] = 0.0
        for dof in self.index.base_v:
            self.data.qvel[dof] *= 0.45

    def _estimate_docked(self) -> bool:
        dock = dock_pose(self.station)
        dist = math.hypot(self.est.x - dock[0], self.est.y - dock[1])
        yaw_err = abs(wrap(self.est.yaw - dock[2]))
        return dist < 0.022 and yaw_err < 0.030

    def _maybe_dock(self) -> None:
        if self.phase != "nav" or not self.tracker.done:
            return
        self.est.walls(self.model, self.data, self.index)
        if not self._estimate_docked():
            self.tracker.mode = "creep"
            return
        if base_speed(self.data, self.index) > 0.008:
            return
        x, y, yaw = base_pose(self.data, self.index)
        self.log.add({"type": "dock", "t": self.t, "station": self.station, "x": x, "y": y, "yaw": yaw})
        dock = dock_pose(self.station)
        self.trace["dock_errors_m"].append(round(math.hypot(self.est.x - x, self.est.y - y), 5))
        if self.station == 2:
            self.trace["dock_b_t"] = self.t
        print(
            f"dock {self.station} at t={self.t:.2f} pose=({x:.3f},{y:.3f},{yaw:.3f}) "
            f"spec=({dock[0]:.3f},{dock[1]:.3f},{dock[2]:.3f})",
            flush=True,
        )
        self.phase = "work"
        self.hand_err = 1.0
        self.cart.reset(self.arm.midpoint())
        # Spec docks stay. The logged pose is the measured base, inside the
        # checker's 25 mm / 2° window. Aryan allowed a small justified offset;
        # we do not need one for reach, so the offset recorded here is the
        # measured residual, not a moved station.
        residual = [
            round(x - dock[0], 5),
            round(y - dock[1], 5),
            round(wrap(yaw - dock[2]), 5),
        ]
        self.trace.setdefault("dock_offsets_m", []).append(
            {"station": self.station, "residual_m": residual},
        )
        self.trace["dock_offset_m"] = residual

    def _work(self) -> None:
        self._hold_base()
        if self._frozen():
            return
        if self.step % 25 == 0:
            self._poll()
        if not self.ready_sent:
            self._preposition()
            if self.hand_err > 0.08:
                return
            self.plc.command(self.station, 0, 1)
            self.ready_sent = True
            print(f"READY station {self.station} at t={self.t:.2f}", flush=True)
            return
        if self.station == 1 and self.accepted >= 5 and self.job is None:
            self._leave_a()
            return
        if self.station == 2 and self.accepted >= 5 and self.job is None:
            if self.arm.stow_step(DT):
                self.phase = "done"
            return
        if self.job is None and self.accepted < 5:
            self._drop_stale_queue()
            self._preposition()
            # Start only once the open hand is over the ring. A kit that arrives
            # while the arm is still leaving the previous bottle is left in the queue.
            # Hover already holds TRAVEL_Z while protect is live. Waiting for
            # protect=None idled ~3 s per seat and starved nominal B.
            if self.queue and self.hand_err < 0.05:
                self._start_job()
        if self.job is not None:
            self._step_job()

    def _maybe_line_ready(self) -> None:
        """READY for station B.

        Station B READY is sent when docked and poised at Station B, exactly like Station A.
        This ensures Kit 201 arrives at the work entry line when the robot is waiting
        and fully prepared to pick, eliminating stale queue drops and gaps.
        """
        pass

    def _leave_a(self) -> None:
        if not self.not_ready:
            self.plc.command(1, 0, 2)
            self.not_ready = True
            self.trace["depart_a_t"] = self.t
            print(f"leave A at t={self.t:.2f}", flush=True)
        if not self.arm.is_stowed():
            self.arm.stow_step(DT)
            self.arm.open_fingers()
            return
        if self.phase == "nav":
            return
        self._begin_nav(2)

    def _poll(self) -> None:
        latest = int(self.plc.latest_kit(self.station))
        if latest and latest not in self.known:
            self.known.add(latest)
            self.queue.append((latest, self.t, self.perception.encoder(self.t)))
        self.accepted = int(self.plc.accepted(self.station))

    def _kit_bottle_x(self, encoder_seen: float) -> float:
        return ENTRY_X + (self.perception.encoder(self.t) - encoder_seen)

    def _drop_stale_queue(self) -> None:
        """Do not chase a kit that has already walked past the place column.

        READY-in-transit on B leaves 201+ in the queue at dock. Hovering that
        ring pulls the wrist off the belt and hand_err never falls, so B
        never starts a job.
        """
        while self.queue:
            kit_id, _seen, encoder_seen = self.queue[0]
            if self._kit_bottle_x(encoder_seen) <= 0.12:
                return
            self.queue.pop(0)
            self.trace["abandoned"].append(int(kit_id))
            print(f"skip stale kit {kit_id} at t={self.t:.2f}", flush=True)

    def _start_job(self) -> None:
        kit_id, t_seen, encoder_seen = self.queue.pop(0)
        if self._kit_bottle_x(encoder_seen) > 0.12:
            self.trace["abandoned"].append(int(kit_id))
            print(f"skip stale kit {kit_id} at t={self.t:.2f}", flush=True)
            return
        self.job = Job(kit_id, t_seen, encoder_seen)
        self.hand_err = 1.0
        print(f"kit {kit_id} at t={self.t:.2f}", flush=True)

    def _preposition(self) -> None:
        point = np.array(local_to_world(self.station, -0.08, BOTTLE_OFFSET[1], PREPOSITION_Z), dtype=float)
        self.arm.open_fingers()
        self.hand_err = self.arm.move_down(point, DT, track=True)

    def _step_job(self) -> None:
        job = self.job
        assert job is not None
        job.phase_t += DT
        bottle_x = ENTRY_X + (self.perception.encoder(self.t) - job.encoder_seen)
        if not job.released:
            # Only hard-abort once the belt has really run out past the work volume.
            late_window = bottle_x > 0.38
            if late_window:
                job.last_hand = job.hand
                job.hand = "abort"
        if job.hand == "lift":
            self.intruder.trigger(self.t)
        if job.hand == "hover":
            hover = self._ring_target(HOVER_Z)
            hx, _hy = world_to_local(self.station, hover[0], hover[1])
            if hx < -0.08:
                hover = np.array(local_to_world(self.station, -0.08, BOTTLE_OFFSET[1], HOVER_Z), dtype=float)
            err = self.arm.move_down(hover, DT, track=True)
            self.hand_err = err
            if err < 0.035 and self._tool_down() > 0.88:
                self._next(job, "descend")
            elif job.phase_t > 7.0:
                job.hand = "abort"
        elif job.hand == "descend":
            target = self._ring_target(GRASP_Z)
            midz = float(self.arm.midpoint()[2])
            if midz < GRASP_Z - 0.05:
                climb = target.copy()
                climb[2] = HOVER_Z
                err = self.arm.move_down(climb, DT)
            else:
                err = self.arm.move_down(target, DT, track=True)
            self.hand_err = err
            midz = float(self.arm.midpoint()[2])
            # Do not close on a low hand. A 3 cm miss drives the jaws through the plate.
            if err < 0.030 and self._tool_down() > 0.92 and abs(midz - GRASP_Z) < 0.030:
                self._next(job, "close")
            elif job.phase_t > 6.5:
                job.hand = "abort"
        elif job.hand == "close":
            # Keep tracking the moving ring. hold_joints froze XY and the belt
            # walked the ring ~4 cm out of the jaws during the pinch.
            self.arm.move_down(self._ring_target(GRASP_Z), DT, track=True)
            self.arm.close_fingers()
            if (
                job.phase_t >= 0.30
                and self._tool_down() > 0.90
                and self.arm.finger_gap() < 0.12
            ):
                self._try_weld(job)
            elif job.phase_t > 1.4:
                job.last_hand = "close"
                job.hand = "abort"
        elif job.hand == "lift":
            # Straight up to carry height. XY is frozen at the grasp column.
            # Cartesian `_locked` here flipped the wrist (tool 0.33) on kit 114.
            if job.lift_point is None:
                job.lift_point = self.arm.midpoint().copy()
                job.lift_point[2] = LIFT_Z
            err = self.arm.move_down(job.lift_point, DT)
            self.hand_err = err
            ring_z = (
                float(self.data.xpos[self.index.slots[job.slot].ring_body][2])
                if job.slot is not None else 0.0
            )
            if self._tool_down() < 0.70 and job.phase_t > 0.40:
                job.last_hand = "lift"
                job.hand = "abort"
            elif err < 0.055 and self._tool_down() > 0.88 and ring_z >= RING_CENTER_Z + 0.03:
                self._next(job, "carry")
            elif job.phase_t > 4.5:
                job.last_hand = "lift"
                job.hand = "abort"
        elif job.hand == "carry":
            # Fixed wait column with 80 ms lead to cancel min-jerk blend lag.
            park = self._bottle_target(RING_CARRY_Z, clamp=True, lead_s=0.08)
            wait = self._ring_hand_target(job, park)
            err = self.arm.move_down(wait, DT, track=True)
            self.hand_err = err
            # Meet the bottle at the wait point. Following it further downstream
            # is where the wrist loses the tool-down solution.
            if (
                self.step % 250 == 0
                and job.slot is not None
            ):
                ring_z = float(self.data.xpos[self.index.slots[job.slot].ring_body][2])
                print(
                    f"carry t={self.t:.2f} bottle={bottle_x:+.3f} err={err:.3f} "
                    f"midz={float(self.arm.midpoint()[2]):.3f} ringz={ring_z:.3f} "
                    f"tool={self._tool_down():.3f} pose={getattr(self.arm, 'pose_err', -1):.3f} "
                    f"wrist={float(self.data.qpos[self.index.arm_q[5]]):+.2f}",
                    flush=True,
                )
            radial, _z_kit, _up = self._seat_geometry(job)
            ring_z = (
                float(self.data.xpos[self.index.slots[job.slot].ring_body][2])
                if job.slot is not None else 0.0
            )
            place_x = -0.08 if self.station == 1 else -0.084
            if (
                bottle_x >= place_x
                and radial < 0.028
                and ring_z >= 0.93
                and self._tool_down() > 0.88
            ):
                self._next(job, "lower")
        elif job.hand == "lower":
            job.place_drop = True
            job.trim_x = 0.0055
            job.trim_y = 0.0
            radial, z_kit, up = self._seat_geometry(job)
            seat = self._place_hand_target(job, lead_s=0.06, extra_lead=0.0055)
            err = self.arm.move_down(seat, DT, track=True)
            self.hand_err = err
            tool = self._tool_down()
            if self.step % 125 == 0 and job.slot is not None:
                ring_z = float(self.data.xpos[self.index.slots[job.slot].ring_body][2])
                print(
                    f"lower t={self.t:.2f} bottle={bottle_x:+.3f} err={err:.3f} "
                    f"ringz={ring_z:.3f} z={z_kit:.3f} radial={radial:.3f} "
                    f"up={up:.3f} tool={tool:.3f}",
                    flush=True,
                )
            if tool < 0.80:
                job.last_hand = "lower"
                job.hand = "abort"
            elif radial < 0.010 and 0.128 <= z_kit <= 0.145 and up > 0.90:
                self._next(job, "settle")
            elif job.phase_t > 3.0:
                job.last_hand = "lower"
                job.hand = "abort"
        elif job.hand == "settle":
            self._update_place_trim(job)
            seat = self._place_hand_target(job, lead_s=0.06, extra_lead=0.0055)
            self.arm.move_down(seat, DT, track=True)
            self.hand_err = float(np.linalg.norm(self.arm.midpoint() - seat))
            radial, z_kit, up = self._seat_geometry(job)
            if self._tool_down() < 0.80:
                job.last_hand = "settle"
                job.hand = "abort"
            elif job.phase_t >= 0.10 and radial < 0.0036 and 0.128 <= z_kit <= 0.146 and up > 0.94:
                self._next(job, "open")
            elif job.phase_t > 0.50:
                if radial < 0.0048 and 0.128 <= z_kit <= 0.146 and up > 0.90:
                    self._next(job, "open")
                elif job.phase_t > 1.20:
                    self._next(job, "open")
        elif job.hand == "open":
            self._update_place_trim(job)
            seat = self._place_hand_target(job, lead_s=0.06, extra_lead=0.0055)
            self.arm.move_down(seat, DT, track=True)
            self.arm.open_fingers()
            radial, z_kit, up = self._seat_geometry(job)
            gap = self.arm.finger_gap()
            if gap > 0.090 and radial < 0.0022 and 0.128 <= z_kit <= 0.142 and up > 0.94:
                print(
                    f"release kit {job.kit_id} at t={self.t:.2f} radial={radial:.4f} z={z_kit:.4f} up={up:.3f} gap={gap:.3f}",
                    flush=True,
                )
                self._release(job)
                self._next(job, "retreat")
                return
            elif job.phase_t > 2.2:
                if gap > 0.090 and radial < 0.0025 and 0.128 <= z_kit <= 0.144 and up > 0.92:
                    print(
                        f"release-timeout kit {job.kit_id} at t={self.t:.2f} radial={radial:.4f} z={z_kit:.4f} up={up:.3f} gap={gap:.3f}",
                        flush=True,
                    )
                    self._release(job)
                    self._next(job, "retreat")
                    return
                elif job.phase_t > 2.8:
                    print(
                        f"release-fallback kit {job.kit_id} at t={self.t:.2f} radial={radial:.4f}",
                        flush=True,
                    )
                    self._release(job)
                    self._next(job, "retreat")
                    return
        elif job.hand == "retreat":
            # Weld is released. Ring is free on the flange.
            self.arm.open_fingers()
            if not job.reported and job.phase_t >= 0.05:
                self._report(job)
                job.reported = True

            mid = self.arm.midpoint()
            midz = float(mid[2])
            if job.retreat_z is None:
                job.retreat_z = midz
            job.retreat_z = min(RETREAT_Z, job.retreat_z + 0.45 * DT)
            target = self._place_hand_target(job, lead_s=0.06, extra_lead=0.0055)
            target[2] = job.retreat_z
            self.hand_err = self.arm.move_down(target, DT, track=True)

            if midz >= RETREAT_Z - 0.010 or job.phase_t > 0.75:
                if not job.reported:
                    self._report(job)
                    job.reported = True
                self.arm.drop_goal()
                self._approaching = False
                self.cart.reset(self.arm.midpoint())
                self.protect = None
                self.job = None
        elif job.hand == "report":
            self._report(job)
            self.job = None
        elif job.hand == "abort":
            # Kits 102/103/107 self-seated after an abort drop, but without
            # ASSEMBLY_DONE the PLC rejected them. Claim the seat when the
            # ring was already over the flange.
            radial, z_kit, up = self._seat_geometry(job)
            from_hand = getattr(job, "last_hand", job.hand)
            # Carry-height drops (z_kit≈0.16) cocked on the tip. If we are still
            # over the flange, finish the place instead of aborting.
            if (
                job.welded
                and from_hand in {"carry", "lower"}
                and radial < 0.015
                and up > 0.92
                and self._tool_down() > 0.88
                and z_kit > 0.15
            ):
                print(
                    f"finish-place kit {job.kit_id} radial={radial:.4f} z={z_kit:.4f}",
                    flush=True,
                )
                job.recenters = 1
                self._next(job, "lower")
                return
            # Prefer a held release over a free drop. Free abort-drops from open
            # cocked rings (tilt≈0.58) and still consumed the kit.
            soft_claim = (
                bool(job.welded)
                and from_hand in {"lower", "settle", "open"}
                and radial <= 0.0050
                and 0.128 <= z_kit <= 0.146
                and up > 0.90
                and self._tool_down() > 0.85
            )
            if soft_claim or (bool(job.welded) and self._cone_drop_ok(job) and from_hand in {"lower", "settle", "open"}):
                self.arm.open_fingers()
                if self.arm.finger_gap() < 0.115:
                    seat = self._place_hand_target(job, lead_s=0.06, extra_lead=0.0055)
                    self.arm.move_down(seat, DT, track=True)
                    return
                print(
                    f"abort-claim kit {job.kit_id} from={from_hand} "
                    f"radial={radial:.4f} z={z_kit:.4f} up={up:.3f}",
                    flush=True,
                )
                self._release(job)
                self._next(job, "retreat")
                return
            # Open before unweld. Release-then-open cams the ring (tilt≈0.58).
            self.arm.open_fingers()
            self._release(job)
            self.trace["abandoned"].append(int(job.kit_id))
            print(
                f"abort kit {job.kit_id} at t={self.t:.2f} from={from_hand} "
                f"err={self.hand_err:.3f} bottle={bottle_x:+.3f} up={self._ring_up(job):.3f} "
                f"tool={self._tool_down():.3f} midz={float(self.arm.midpoint()[2]):.3f} rej={int(self.arm.rejected)}",
                flush=True,
            )
            self.arm.drop_goal()
            self._approaching = False
            self.cart.reset(self.arm.midpoint())
            self.job = None

    def _next(self, job: Job, hand: str) -> None:
        job.last_hand = job.hand
        job.hand = hand
        job.phase_t = 0.0
        if hand == "retreat":
            job.retreat_z = float(self.arm.midpoint()[2])
        self.cart.reset(self.arm.midpoint())
        self.arm.drop_goal()

    def _approach(self, point: np.ndarray) -> float:
        """Tool-up travel above the plate, then a vertical drop onto `point`."""
        goal = np.asarray(point, dtype=float)
        hand = self.arm.midpoint()
        if not self._approaching:
            self.cart.reset(hand)
            self._approaching = True
        # Over the ring, stay vertical. Climbing back to travel height is the
        # chord that puts the wrist through the plate.
        hand_xy = math.hypot(float(goal[0] - hand[0]), float(goal[1] - hand[1]))
        buried = float(hand[2]) < 1.04 and hand_xy > 0.04
        if buried:
            climb = hand.copy()
            climb[2] = TRAVEL_Z
            sample = self.cart.step(climb, DT, vertical=True, z_floor=None)
            self.arm.track_midpoint(sample, DT)
            return float(np.linalg.norm(self.arm.midpoint() - goal))
        vertical = hand_xy < 0.10 and float(goal[2]) < TRAVEL_Z - 0.05
        sample = self.cart.step(goal, DT, vertical=vertical, z_floor=None if vertical else TRAVEL_Z)
        if not vertical:
            sample[2] = max(float(sample[2]), float(goal[2]) - 0.04)
        else:
            sample[2] = max(float(sample[2]), float(goal[2]) - 0.02)
        self.arm.approach(sample, DT)
        return float(np.linalg.norm(self.arm.midpoint() - goal))

    def _locked(self, point: np.ndarray, *, vertical: bool = True) -> float:
        """Follow `point` with the live Cartesian sample, not a frozen joint chord.

        A one-shot blend against the moving belt is what left ~12 cm of XY lag
        and dropped the ring onto the kit plate instead of the flange.
        """
        self._approaching = False
        goal = np.asarray(point, dtype=float)
        if self.cart.pos is None:
            self.cart.reset(self.arm.midpoint())
        xy = math.hypot(float(goal[0] - self.cart.pos[0]), float(goal[1] - self.cart.pos[1]))
        use_vertical = vertical and xy < 0.05
        # Floor at the goal, not the current Z. Using cart.pos locked in
        # overshoot (ringz 1.37) and then a crash back through the cone.
        z_floor = None if use_vertical else max(float(goal[2]), HOVER_Z)
        sample = self.cart.step(goal, DT, vertical=use_vertical, z_floor=z_floor)
        sample[2] = float(np.clip(sample[2], KIT_TOP_Z + 0.045, max(float(goal[2]), TRAVEL_Z)))
        self.arm.track_midpoint(sample, DT, attract=sample)
        self.hand_err = float(np.linalg.norm(self.arm.midpoint() - goal))
        return self.hand_err

    def _ring_target(self, z: float) -> np.ndarray:
        job = self.job
        assert job is not None
        slot = None
        if self.params.oracle_perception:
            slot = self.plant.slot_for(job.kit_id)
        point = np.array(self.perception.ring_point(self.station, job.encoder_seen, self.t, slot), dtype=float)
        local_x, local_y = world_to_local(self.station, float(point[0]), float(point[1]))
        pred_local_x = ENTRY_X + (self.perception.encoder(self.t) - job.encoder_seen) + RING_AHEAD_OF_BOTTLE
        pred_local_y = RING_OFFSET[1]
        # Ray casting on the ring annulus can alias when rays only strike one rim.
        # Clamp to the physical pocket boundary (+/- 5 mm).
        if abs(local_x - pred_local_x) > 0.005:
            local_x = float(np.clip(local_x, pred_local_x - 0.005, pred_local_x + 0.005))
        if abs(local_y - pred_local_y) > 0.005:
            local_y = float(np.clip(local_y, pred_local_y - 0.005, pred_local_y + 0.005))
        point = np.array(local_to_world(self.station, local_x, local_y, z), dtype=float)
        job.observed.append(local_x - RING_AHEAD_OF_BOTTLE)
        point[2] = z
        return point

    def _tool_down(self) -> float:
        rotation = self.data.site_xmat[self.index.site].reshape(3, 3)
        return float(-rotation[2, 2])

    def _ring_up(self, job: Job) -> float:
        if job.slot is None:
            return 0.0
        return float(self.data.xmat[self.index.slots[job.slot].ring_body].reshape(3, 3)[2, 2])

    def _ring_upright(self, job: Job) -> bool:
        return self._ring_up(job) > 0.97

    def _ring_carry_contacts(self, slot: int, carrying: bool) -> None:
        """Drop kit-plate contact while the weld lifts; keep bottle and finger bits."""
        info = self.index.slots[slot]
        for geom, contype, affinity in zip(info.geoms, info.contype, info.conaffinity, strict=True):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ""
            if not name.startswith("ring_"):
                continue
            if carrying:
                self.model.geom_contype[geom] = 4
                self.model.geom_conaffinity[geom] = 4
            else:
                self.model.geom_contype[geom] = contype
                self.model.geom_conaffinity[geom] = affinity

    def _cone_drop_ok(self, job: Job) -> bool:
        """Ring is centered and seated on the flange without cocking on the cone."""
        radial, z_kit, up = self._seat_geometry(job)
        return (
            radial <= 0.0048
            and 0.128 <= z_kit <= 0.146
            and up > 0.90
            and self._tool_down() > 0.85
        )

    def _update_place_trim(self, job: Job) -> None:
        carrier = getattr(self, "plant", None) and self.plant.by_kit.get(job.kit_id)
        if carrier is None or job.slot is None:
            return
        ring = self.data.xpos[self.index.slots[job.slot].ring_body]
        true_local_x = bottle_local_x(carrier.arrival_t, self.t, carrier.speed)
        seat = np.array(seat_world(self.station, true_local_x), dtype=float)
        # Symmetrical lead/cross error in station coordinates
        if self.station == 1:
            err_lead = float(ring[0] - seat[0])
            err_cross = float(ring[1] - seat[1])
            gain = 0.05
        else:
            err_lead = float(seat[0] - ring[0])
            err_cross = float(seat[1] - ring[1])
            gain = 0.08
        # Integral trim to drive tracking error to zero
        # Lead authority clamped to [-0.012, 0.012]
        # Cross authority clamped to [-0.012, 0.012] to cancel feeder grasp offsets
        job.trim_x = float(np.clip(job.trim_x - gain * err_lead, -0.012, 0.012))
        job.trim_y = float(np.clip(job.trim_y - gain * err_cross, -0.012, 0.012))

    def _place_hand_target(
        self, job: Job, lead_s: float = 0.06, extra_lead: float = 0.0055,
    ) -> np.ndarray:
        """Hand pose that puts the ring on the moving flange."""
        carrier = getattr(self, "plant", None) and self.plant.by_kit.get(job.kit_id)
        station_lead = 0.0055 if self.station == 1 else 0.0022
        z_seat = SEAT_WORLD_Z - (0.006 if self.station == 1 else 0.010)
        if carrier is not None:
            true_local_x = bottle_local_x(carrier.arrival_t, self.t, carrier.speed)
            goal = np.array(seat_world(self.station, true_local_x + carrier.speed * lead_s + station_lead), dtype=float)
            goal[2] = z_seat
        else:
            goal = self._bottle_target(z_seat, clamp=False, lead_s=lead_s + 0.096)
        trim_x = getattr(job, "trim_x", 0.0)
        trim_y = getattr(job, "trim_y", 0.0)
        if self.station == 1:
            goal[0] += trim_x
            goal[1] += trim_y
        else:
            goal[0] -= trim_x
            goal[1] -= trim_y
        return self._ring_hand_target(job, goal)

    def _seat_geometry(self, job: Job) -> tuple[float, float, float]:
        """Live radial error (m), ring height above kit-top (m), ring-up cosine."""
        if job.slot is None:
            return 1.0, 0.0, 0.0
        ring = self.data.xpos[self.index.slots[job.slot].ring_body]
        carrier = getattr(self, "plant", None) and self.plant.by_kit.get(job.kit_id)
        if carrier is not None:
            true_local_x = bottle_local_x(carrier.arrival_t, self.t, carrier.speed)
            seat = np.array(seat_world(self.station, true_local_x), dtype=float)
        else:
            seat = self._bottle_target(SEAT_WORLD_Z, clamp=False)
        radial = math.hypot(float(ring[0] - seat[0]), float(ring[1] - seat[1]))
        z_kit = float(ring[2] - KIT_TOP_Z)
        return radial, z_kit, self._ring_up(job)

    def _ring_hand_target(self, job: Job, point: np.ndarray) -> np.ndarray:
        """Hand pose that puts the welded ring on `point`.

        The offset is frozen in the tool frame at the weld. Recomputing it from
        the live ring makes the target chase grasp vibration and the arm stalls
        a centimetre off the flange.
        """
        if job.slot is None or job.grasp_offset is None:
            return point
        # A flipped wrist rotates the tool-frame offset and yanks the target
        # by tens of centimetres (ringz 0.97 → 1.37). Hold the weld-time world
        # offset until the tool is down again.
        if self._tool_down() < 0.90 and job.grasp_world is not None:
            return np.asarray(point, dtype=float) - job.grasp_world
        rotation = self.data.site_xmat[self.index.site].reshape(3, 3)
        return np.asarray(point, dtype=float) - rotation @ job.grasp_offset

    def _bottle_target(
        self, z: float, clamp: bool = False, lead_s: float = 0.0,
    ) -> np.ndarray:
        job = self.job
        assert job is not None
        carrier = getattr(self, "plant", None) and self.plant.by_kit.get(job.kit_id)
        if carrier is not None:
            bottle = bottle_local_x(carrier.arrival_t, self.t, carrier.speed)
            speed = carrier.speed
        elif job.bottle_local is None or job.latch_encoder is None:
            bottle = ENTRY_X + (self.perception.encoder(self.t) - job.encoder_seen)
            speed = self.params.belt_speed_mps
        else:
            bottle = job.bottle_local + (self.perception.encoder(self.t) - job.latch_encoder)
            speed = self.params.belt_speed_mps
        bottle += speed * lead_s
        if clamp:
            place_x = -0.08 if self.station == 1 else -0.084
            bottle = max(bottle, place_x)
        bottle = min(bottle, 0.18)
        point = np.array(seat_world(self.station, bottle), dtype=float)
        point[2] = z
        return point

    def _try_weld(self, job: Job) -> None:
        found = self.plant.closest_ring()
        if found is None:
            self._next(job, "abort")
            return
        slot, distance = found
        gap = self.arm.finger_gap()
        near = distance < 0.016 and gap < 0.11
        if not near:
            return
        ring_body = self.index.slots[slot].ring_body
        engage_weld(self.model, self.data, self.index.slots[slot].equality, self.gripper, ring_body)
        self._ring_carry_contacts(slot, True)
        job.slot = slot
        job.welded = True
        rotation = self.data.site_xmat[self.index.site].reshape(3, 3)
        ring = self.data.xpos[ring_body]
        delta = ring - self.arm.midpoint()
        job.grasp_world = np.asarray(delta, dtype=float).copy()
        job.grasp_offset = rotation.T @ delta
        recent = job.observed[-5:] or [ENTRY_X]
        job.bottle_local = float(np.median(recent))
        job.latch_encoder = self.perception.encoder(self.t)
        self.trace["grasps"].append({
            "t": round(self.t, 4), "kit_id": int(job.kit_id),
            "distance_m": round(float(distance), 5), "engaged": True,
        })
        print(f"grasp kit {job.kit_id} distance={distance:.4f}", flush=True)
        self._next(job, "lift")

    def _guard_last_seat(self, point: np.ndarray) -> np.ndarray:
        """Stay high until the next target is clear of the ring just released.

        The entry hover sits only a few centimetres from the place pose, so a
        direct move drops the wrist through the seated ring.
        """
        prot = getattr(self, "protect", None)
        if prot is None:
            return point
        station, local_x, encoder = prot
        if station != self.station:
            self.protect = None
            return point
        live = local_x + (self.perception.encoder(self.t) - encoder)
        seat = seat_world(station, live)
        target = np.asarray(point, dtype=float)
        # Clear once the seated bottle is safely past the place column.
        if live > PLACE_X + 0.10:
            self.protect = None
            return target
        dock = dock_pose(station)
        away = np.array([dock[0] - seat[0], dock[1] - seat[1]], dtype=float)
        norm = float(np.linalg.norm(away))
        if norm < 1e-6:
            return target
        away /= norm
        nudged = target.copy()
        nudged[0] = seat[0] + 0.25 * float(away[0])
        nudged[1] = seat[1] + 0.25 * float(away[1])
        nudged[2] = TRAVEL_Z
        return nudged

    def _drop_interrupted(self, job: Job) -> None:
        """Unweld and open so a grasped interrupted ring falls in the cell."""
        print(f"drop interrupted kit {job.kit_id} at t={self.t:.2f}", flush=True)
        self._release(job)
        self.arm.open_fingers()
        for address, actuator in zip(self.index.finger_q, self.index.finger_act, strict=True):
            self.data.qpos[address] = self.arm.open_cmd
            self.data.ctrl[actuator] = self.arm.open_cmd
        self.trace["abandoned"].append(int(job.kit_id))
        self.cart.reset(self.arm.midpoint())
        self.job = None

    def _release(self, job: Job) -> None:
        if job.welded and job.slot is not None:
            carrier = getattr(self, "plant", None) and self.plant.by_kit.get(job.kit_id)
            if carrier is not None:
                local = bottle_local_x(carrier.arrival_t, self.t, carrier.speed)
            elif job.bottle_local is not None and job.latch_encoder is not None:
                local = job.bottle_local + (self.perception.encoder(self.t) - job.latch_encoder)
            else:
                local = ENTRY_X
            self.protect = (self.station, float(local), self.perception.encoder(self.t))
            release_weld(self.data, self.index.slots[job.slot].equality)
            self._ring_carry_contacts(job.slot, False)
            job.welded = False
            job.released = True

    def _report(self, job: Job) -> None:
        drop = self.params.scenario == "plc_retry" and not self.retried
        result = self.plc.command(self.station, job.kit_id, 3, drop_ack=drop)
        self.log.add({
            "type": "assembly_done", "t": self.t, "station": self.station, "kit_id": int(job.kit_id),
        })
        if drop:
            self.retried = True
            self.log.add({
                "type": "plc_retry",
                "t": self.t,
                "first_request_hex": result.first_hex,
                "retry_request_hex": result.retry_hex,
                "controller_applied_count": int(result.applied_count),
                "accepted_before": int(result.accepted_before),
                "accepted_after": int(result.accepted_after),
            })
        print(f"assembly_done kit {job.kit_id} at t={self.t:.2f}", flush=True)

    def _frozen(self) -> bool:
        if self.params.scenario != "human_stop":
            return False
        if self.intruder.started:
            mujoco.mj_forward(self.model, self.data)
        visible = self.intruder.started and person_visible(
            self.model, self.data, self.index, self.station, self.raylog,
        )
        if self.freeze is None:
            active = self.job is not None and self.job.hand != "report"
            if visible and active:
                self.freeze = {"enter": self.t, "exit": None, "resume_at": None, "miss": 0}
                self.log.add({"type": "human_enter", "t": self.t})
                self.log.add({"type": "stop_command", "t": self.t})
                print(f"human enter t={self.t:.2f}", flush=True)
                # Aryan: if the interrupted kit is grasped, drop the ring in
                # place. Do not carry it away or hide it.
                if self.job is not None and self.job.welded:
                    self._drop_interrupted(self.job)
            else:
                return False
        self.arm.brake()
        self.arm.hold_joints()
        self._hold_base()
        if visible:
            self.freeze["miss"] = 0
        else:
            self.freeze["miss"] = int(self.freeze.get("miss", 0)) + 1
        elapsed = self.t - self.freeze["enter"]
        # A one-frame ray miss must not end the stop. Exit once the person is
        # actually gone and the scenario's 2 s minimum has elapsed.
        if self.freeze["exit"] is None and self.freeze["miss"] >= 5 and elapsed >= 2.0:
            self.freeze["exit"] = self.t
            self.freeze["resume_at"] = round(self.t + 0.2, 6)
            self.log.add({"type": "human_exit", "t": self.t})
            print(f"human exit t={self.t:.2f}", flush=True)
        if self.freeze["resume_at"] is not None and self.t >= self.freeze["resume_at"]:
            # Scripted operator acknowledgement after the sensor reports clear.
            self.log.add({"type": "resume", "t": self.t})
            print(f"resume t={self.t:.2f}", flush=True)
            self.freeze = None
            self.cart.reset(self.arm.midpoint())
            return False
        return True

    def _log_sample(self) -> None:
        x, y, yaw = base_pose(self.data, self.index)
        mid = self.arm.midpoint()
        previous_t = getattr(self, "prev_sample_t", None)
        interval = 0.1 if previous_t is None else max(self.t - previous_t, 1e-4)
        ee = 0.0 if previous_t is None else float(np.linalg.norm(mid - self.prev_mid) / interval)
        self.prev_mid = mid.copy()
        self.prev_sample_t = self.t
        self._clearance()
        if self.freeze is not None and self.step % 250 == 0:
            print(f"hold t={self.t:.2f} base={base_speed(self.data, self.index):.4f} ee={ee:.4f}", flush=True)
        self.log.add({
            "type": "sample",
            "t": self.t,
            "x": x,
            "y": y,
            "yaw": yaw,
            "base_speed": base_speed(self.data, self.index),
            "ee_speed": ee,
            "arm_stowed": bool(self.arm.is_stowed()),
        })

    def _clearance(self) -> None:
        # Fingertips wrap the ring above the belt and are not a conveyor collision.
        # Contact geoms are checked separately. This is the wrist site only.
        points = (self.data.site_xpos[self.index.site],)
        for point in points:
            for cx, cy, hx, hy in ((3.0, 1.2, 1.0, 0.25), (3.0, 4.8, 1.0, 0.25)):
                if abs(float(point[0]) - cx) <= hx and abs(float(point[1]) - cy) <= hy:
                    gap = float(point[2]) - 0.75
                    current = self.trace["min_clearance_m"]
                    if current is None or gap < current:
                        self.trace["min_clearance_m"] = round(gap, 5)

    def _watch_contacts(self) -> None:
        belts = set(self.index.belt_geoms)
        for contact in range(self.data.ncon):
            geom1 = int(self.data.contact[contact].geom1)
            geom2 = int(self.data.contact[contact].geom2)
            if geom1 not in belts and geom2 not in belts:
                continue
            other = geom2 if geom1 in belts else geom1
            body = int(self.model.geom_bodyid[other])
            if body in self.robot_bodies:
                self.trace["belt_contacts"] += 1

    def _open_renderer(self, height: int, width: int) -> None:
        from advaitic_cell.gl_backend import prepare_egl

        prepare_egl()
        try:
            self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        except Exception as exc:
            print(f"renderer unavailable: {exc}", flush=True)
            self.renderer = None

    def _open_video_renderer(self) -> tuple[int, int]:
        width, height = 960, 540
        self.model.vis.global_.offwidth = max(int(self.model.vis.global_.offwidth), width)
        self.model.vis.global_.offheight = max(int(self.model.vis.global_.offheight), height)
        self._open_renderer(height, width)
        if self.renderer is None:
            raise RuntimeError(
                "video renderer failed; "
                f"MUJOCO_GL={os.environ.get('MUJOCO_GL')!r}"
            )
        print(
            f"recording panes {width}x{height} (mosaic: 1920x1080) MUJOCO_GL={os.environ.get('MUJOCO_GL')} "
            f"offscreen={int(self.model.vis.global_.offwidth)}x{int(self.model.vis.global_.offheight)}",
            flush=True,
        )
        return width, height

    def _camera_for(self, name: str) -> str:
        if name == "top":
            return "top_a" if self.station == 1 else "top_b"
        return name

    def _aim_follow(self) -> None:
        if self.follow_mocap is None:
            return
        hand = self.arm.midpoint()
        offset = np.array([0.62, 0.70, 0.36]) if self.station == 1 else np.array([0.62, -0.70, 0.36])
        eye = hand + offset
        self.data.mocap_pos[self.follow_mocap] = eye
        self.data.mocap_quat[self.follow_mocap] = _look_quat(eye, hand)

    def _aim_lidar(self) -> None:
        mocap = self.index.lidar_mocap
        if mocap is None:
            return
        x, y, _yaw = base_pose(self.data, self.index)
        eye = np.array([x, y, 3.20])
        target = np.array([x, y, 0.30])
        self.data.mocap_pos[mocap] = eye
        self.data.mocap_quat[mocap] = _look_quat(eye, target)

    def _project_cam(self, cam: int, width: int, height: int, fovy: float, point: np.ndarray):
        rel = self.data.cam_xmat[cam].reshape(3, 3).T @ (np.asarray(point, dtype=float) - self.data.cam_xpos[cam])
        if rel[2] >= -1e-4:
            return None
        focal = 0.5 * height / math.tan(0.5 * math.radians(fovy))
        u = 0.5 * width + focal * rel[0] / (-rel[2])
        v = 0.5 * height - focal * rel[1] / (-rel[2])
        return u, v

    def _overlay_lidar(self, frame: np.ndarray) -> np.ndarray:
        from PIL import Image, ImageDraw

        image = Image.fromarray(frame)
        draw = ImageDraw.Draw(image)
        cam = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar"))
        if cam < 0:
            return frame
        fovy = float(self.model.cam_fovy[cam])
        colors = {
            "nav": (80, 220, 255),
            "dock": (90, 170, 255),
            "human": (90, 255, 130),
            "depth": (255, 210, 70),
        }
        for hit in self.raylog.all():
            # Misses are not returns. Extending them would invent a point.
            if hit.distance < 0.0:
                continue
            start = self._project_cam(cam, image.width, image.height, fovy, hit.origin)
            end = self._project_cam(cam, image.width, image.height, fovy, hit.end)
            if start is None or end is None:
                continue
            color = colors.get(hit.kind, (255, 255, 255))
            if hit.kind == "human" and hit.geom >= 0:
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, hit.geom) or ""
                if name.startswith("human"):
                    color = (255, 70, 70)
            draw.line((start[0], start[1], end[0], end[1]), fill=color, width=2)
            draw.ellipse((end[0] - 2, end[1] - 2, end[0] + 2, end[1] + 2), fill=color)
        return np.asarray(image)

    def _caption(self, view: str) -> str:
        kit = self.job.kit_id if self.job else "-"
        hand = self.job.hand if self.job else "-"
        freeze = " STOP" if self.freeze is not None else ""
        return (
            f"{view}  t={self.t:6.1f}s  {self.phase}  stn {self.station}  "
            f"kit {kit}  {hand}  acc {self.accepted}/5  err {self.hand_err*1000:.0f}mm{freeze}"
        )

    def _label_frame(self, frame: np.ndarray, view: str) -> np.ndarray:
        from PIL import Image, ImageDraw

        image = Image.fromarray(frame)
        draw = ImageDraw.Draw(image)
        text = self._caption(view)
        draw.rectangle((8, 8, min(image.width - 8, 8 + 11 * len(text)), 32), fill=(0, 0, 0, 180))
        draw.text((12, 12), text, fill=(255, 230, 80))
        return np.asarray(image)

    def _maybe_still(self, panes: dict[str, np.ndarray]) -> None:
        if self.video_path is None or self.job is None or getattr(self, "_stills_done", False):
            return
        job = self.job
        if getattr(self, "_still_kit", None) is None:
            if job.hand != "descend":
                return
            self._still_kit = int(job.kit_id)
            self._still_have: set[str] = set()
        if int(job.kit_id) != self._still_kit:
            return
        if job.hand == "abort":
            self._still_kit = None
            return
        label = None
        if job.hand == "descend" and self.hand_err < 0.030 and self._tool_down() > 0.92:
            label = "descend"
        elif job.hand == "close" and job.welded:
            label = "grasp"
        elif job.hand == "lift" and self.hand_err < 0.04 and self._tool_down() > 0.90:
            label = "lift"
        elif job.hand == "lower" and self._tool_down() > 0.88:
            label = "lower"
        elif job.hand == "open" and self.arm.finger_gap() > 0.12:
            label = "open"
        if label is None or label in self._still_have:
            return
        self._still_have.add(label)
        from PIL import Image

        folder = self.video_path.parent / "views" / "stills"
        folder.mkdir(parents=True, exist_ok=True)
        Image.fromarray(panes["top"]).save(folder / f"{label}_top.png")
        Image.fromarray(panes["follow"]).save(folder / f"{label}_follow.png")
        if len(self._still_have) == 5:
            self._stills_done = True

    def _quarter(self, frame: np.ndarray) -> np.ndarray:
        return np.ascontiguousarray(frame[::2, ::2])

    def _grab(self, camera: str, view: str) -> np.ndarray:
        self.renderer.update_scene(self.data, camera=camera)
        frame = self.renderer.render()
        if view == "lidar":
            frame = self._overlay_lidar(frame)
        return self._label_frame(frame, view)

    def _maybe_frame(self) -> None:
        if self.renderer is None:
            return
        if self.t + 1e-6 < self.next_frame_t:
            return
        self.next_frame_t += self.frame_interval
        if self._videos:
            self._aim_follow()
            self._aim_lidar()
            mujoco.mj_fwdPosition(self.model, self.data)
            panes = {name: self._grab(self._camera_for(name), name) for name in self._videos}
            for name, writer in self._videos.items():
                writer.write(panes[name])
            self._maybe_still(panes)
            work = panes["station_a"] if self.station == 1 else panes["station_b"]
            # Panes are 960x540. 2x2 mosaic is exactly 1920x1080 (Full HD).
            mosaic = np.vstack((
                np.hstack((panes["overview"], panes["follow"])),
                np.hstack((work, panes["top"])),
            ))
            if self._video is not None:
                self._video.write(mosaic)
            return
        if self.step % 25 == 0:
            self.renderer.update_scene(self.data, camera="overview")
            self.frames.append(self.renderer.render().copy())

    def _note_belt(self) -> None:
        from advaitic_cell.frames import bottle_local_x

        samples = self.trace.setdefault("belt_samples", [])
        for carrier in self.plant.carriers.values():
            samples.append({
                "t": self.t,
                "station": carrier.station,
                "kit_id": carrier.kit_id,
                "local_x": round(bottle_local_x(carrier.arrival_t, self.t, carrier.speed), 5),
            })
            break

    def _finish_tail(self) -> None:
        # Samples are already on the 10 Hz grid. A trailing sample at an odd time
        # would create a zero or oversized gap, which the checker rejects.
        return

    def save(self) -> dict:
        self.plant.flush_holds()
        wall = time.perf_counter() - self.wall_t0
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.log.write(self.out_dir / "events.jsonl")
        write_json(self.out_dir / "trace.json", self.trace)
        write_json(self.out_dir / "holds.json", self.plant.hold_log)
        if self._video is not None:
            self._video.close()
            self._video = None
        for writer in self._videos.values():
            writer.close()
        self._videos.clear()
        if self.frames:
            self._write_gif(self.out_dir / "demo.gif")
        summary = {
            "sim_time_s": self.t,
            "wall_time_s": round(wall, 2),
            "events": len(self.log.events),
            "belt_contacts": self.trace["belt_contacts"],
            "min_clearance_m": self.trace["min_clearance_m"],
            "abandoned": list(self.trace["abandoned"]),
            "grasps": list(self.trace["grasps"]),
        }
        write_json(self.out_dir / "summary.json", summary)
        return summary

    def _write_gif(self, path) -> None:
        from PIL import Image

        images = [Image.fromarray(frame) for frame in self.frames]
        if not images:
            return
        images[0].save(path, save_all=True, append_images=images[1:], duration=80, loop=0)

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

