"""Arm IK, smooth joint streaming, and the documented grasp weld.

The mission samples a minimum-jerk Cartesian path and this module turns each
sample into a joint command. Inverse kinematics is warm-started from the
command, not from a fresh seed, and the command is acceleration-limited so a
one-tick IK wiggle cannot stair-step the arm.

The weld is armed only after the fingertip midpoint is at the pinch pose and
the jaws have closed around the ring. `eq_data` is rewritten to the current
relative pose so the constraint does not pull a distant ring into the hand.

The visual finger meshes hang about 5 cm below the tip-body midpoint. Parking
that midpoint on the ring centre drives the meshes through the kit plate.
`GRASP_LIFT` holds the midpoint above the ring so the jaws meet the orange OD
and the meshes stay on the plate, not in it.
"""

from __future__ import annotations

import mujoco
import numpy as np

from advaitic_cell.motion import minjerk
from advaitic_cell.scene import ModelIndex

ARM_NAMES = (
    "joint_torso", "joint_shoulder", "joint_bicep", "joint_elbow",
    "joint_forearm", "joint_wrist", "joint_gripper",
)
# Comfortable reach: elbow bent, wrist down, torso free to yaw toward the belt.
POSTURE = np.array([0.0, 0.95, 0.0, 1.55, 0.0, -1.15, 0.0])
# Tool-down pose over the belt. Both docks reach it: the base yaw puts the
# belt in front, so the arm joints match. IK from the stow posture misses it.
BELT_POSE = np.array([0.54, 1.20, -1.13, 0.33, -2.13, -1.74, -2.59])
# Same arm, hand 8.5 cm lower. Refining the belt pose walks off this one.
GRASP_POSE = BELT_POSE + np.array([0.24, 0.0, -0.06, 0.54, -0.03, 0.20, 0.83])
# Torso yaw stays short so a bad seed cannot wind the column. The other joints
# keep the vendor travel: the tool-down belt pose sits near forearm -2.1 and
# gripper -2.6, and a tighter window makes that pose unreachable.
_IK_WINDOW = np.array([
    [-1.60, 1.35],
    [-2.66, 3.18],
    [-2.13, 3.71],
    [-2.05, 3.79],
    [-2.92, 2.92],
    [-1.79, 1.79],
    [-4.49, 1.35],
])
# Pinch command whose visual jaws sit on the ring OD (outer radius 40 mm).
PINCH_COMMAND = 0.60
# How far the fingertip midpoint stays above the ring centre at that pinch.
# The visual meshes hang ~51 mm below the midpoint and sag a few millimetres
# once the position servos are holding against gravity.
GRASP_LIFT = 0.056
SEEDS = (
    POSTURE,
    np.array([0.7, 1.05, 0.4, 1.45, 0.3, -1.05, 0.0]),
    np.array([-0.7, 1.05, -0.4, 1.45, -0.3, -1.05, 0.0]),
    np.array([0.4, 0.7, 0.2, 1.9, 0.0, -1.45, 0.0]),
    np.array([-0.4, 0.7, -0.2, 1.9, 0.0, -1.45, 0.0]),
    np.array([1.1, 1.2, 0.6, 1.2, 0.4, -0.9, 0.2]),
    np.array([-1.1, 1.2, -0.6, 1.2, -0.4, -0.9, -0.2]),
)


def _mat(data: mujoco.MjData, body: int) -> np.ndarray:
    return data.xmat[body].reshape(3, 3).copy()


def _quat_from_mat(mat: np.ndarray) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, mat.reshape(9))
    return quat


def relative_pose(data: mujoco.MjData, body1: int, body2: int) -> tuple[np.ndarray, np.ndarray]:
    rotation = _mat(data, body1)
    rel_pos = rotation.T @ (data.xpos[body2] - data.xpos[body1])
    q1 = _quat_from_mat(rotation)
    q2 = _quat_from_mat(_mat(data, body2))
    q1_inv = np.zeros(4)
    rel_q = np.zeros(4)
    mujoco.mju_negQuat(q1_inv, q1)
    mujoco.mju_mulQuat(rel_q, q1_inv, q2)
    return rel_pos, rel_q


def engage_weld(model: mujoco.MjModel, data: mujoco.MjData, equality: int, body1: int, body2: int) -> None:
    """Hold the ring where it already is relative to the gripper.

    Compiled welds store the relative position at ``eq_data[3:6]`` and the
    relative quaternion at ``[6:10]``. Writing that pair from the current pose
    is what keeps the constraint from dragging the ring across the cell.
    """
    rel_pos, rel_q = relative_pose(data, body1, body2)
    model.eq_data[equality, 0:3] = 0.0
    model.eq_data[equality, 3:6] = rel_pos
    model.eq_data[equality, 6:10] = rel_q
    model.eq_data[equality, 10] = 1.0
    if hasattr(data, "eq_data"):
        data.eq_data[equality] = model.eq_data[equality]
    if hasattr(model, "eq_solref"):
        model.eq_solref[equality] = (0.002, 1.0)
    if hasattr(model, "eq_solimp"):
        model.eq_solimp[equality] = (0.99, 0.999, 0.001, 0.5, 2.0)
    data.eq_active[equality] = 1


def release_weld(data: mujoco.MjData, equality: int) -> None:
    data.eq_active[equality] = 0


class ArmController:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, index: ModelIndex):
        self.model = model
        self.data = data
        self.index = index
        self.scratch = mujoco.MjData(model)
        self.ranges = np.array([model.jnt_range[_joint_id(model, i)] for i in index.arm_q])
        self.ik_ranges = np.vstack([
            np.maximum(self.ranges[:, 0], _IK_WINDOW[:, 0]),
            np.minimum(self.ranges[:, 1], _IK_WINDOW[:, 1]),
        ]).T
        self.open_cmd, self.grasp_cmd = self._calibrate_fingers()
        self.mid_offset = self._fingertip_offset()
        self.deployed = False
        self._tool_locked = False
        self._stow = np.zeros(len(index.arm_q))
        joint_ids = [_joint_id(model, qadr) for qadr in index.arm_q]
        self.arm_dof = [int(model.jnt_dofadr[joint]) for joint in joint_ids]
        self.finger_dof = [int(model.jnt_dofadr[_joint_id(model, qadr)]) for qadr in index.finger_q]
        # Orientation is part of the Cartesian servo: keep tool world -Z while
        # the trajectory generator, not a seed flip, chooses the path.
        self.orient_weight = 0.35
        self.posture_weight = 0.02
        self.stream: JointStream | None = None
        self._reseed_age = 0.0
        self._cached_q: np.ndarray | None = None
        zeros = np.zeros(len(index.arm_q))
        # Straight min-jerk in joint space. An independent joint slew bows off
        # this chord and puts the wrist through the plate.
        self._blend_q0 = zeros.copy()
        self._blend_q1 = zeros.copy()
        self._blend_t = 0.0
        self._blend_T = 0.0
        self._blend_point: np.ndarray | None = None
        self._down_q: np.ndarray | None = None
        self._arm_geoms: list[int] | None = None
        self._finger_geoms: set[int] = set()
        self._hit_name = ""
        self._kit_boxes: list[int] | None = None
        self.rejected = False
        self._told = False
        self.open_fingers()

    def _calibrate_fingers(self) -> tuple[float, float]:
        # Skip the widest open. Max-gap jaws sweep a just-seated ring on the
        # retreat / preposition path (B kits cocked at z≈0.150, tilt≈0.58).
        # Clearance open: just past the 0.12 m release gap.
        clearance = None
        clearance_gap = 1.0
        widest = 0.05
        widest_gap = -1.0
        for command in np.linspace(0.02, 1.25, 36):
            gap = self._gap_at(float(command))
            if gap > widest_gap:
                widest_gap, widest = gap, float(command)
            if gap >= 0.135 and gap < clearance_gap:
                clearance_gap, clearance = gap, float(command)
        # Fixed pinch, not the body-gap that leaves the visual jaws off the OD.
        return (clearance if clearance is not None else widest), PINCH_COMMAND

    def _gap_at(self, command: float) -> float:
        saved_q = self.data.qpos.copy()
        saved_c = self.data.ctrl.copy()
        for address, actuator in zip(self.index.finger_q, self.index.finger_act, strict=True):
            self.data.qpos[address] = command
            self.data.ctrl[actuator] = command
        mujoco.mj_forward(self.model, self.data)
        gap = self.finger_gap()
        self.data.qpos[:] = saved_q
        self.data.ctrl[:] = saved_c
        mujoco.mj_forward(self.model, self.data)
        return gap

    def _fingertip_offset(self) -> np.ndarray:
        mujoco.mj_forward(self.model, self.data)
        rotation = self.data.site_xmat[self.index.site].reshape(3, 3)
        return rotation.T @ (self.midpoint() - self.data.site_xpos[self.index.site])

    def midpoint(self) -> np.ndarray:
        return 0.5 * (self.data.xpos[self.index.tip_right] + self.data.xpos[self.index.tip_left])

    def finger_gap(self) -> float:
        return float(np.linalg.norm(
            self.data.xpos[self.index.tip_right] - self.data.xpos[self.index.tip_left]
        ))

    def set_fingers(self, command: float) -> None:
        for actuator in self.index.finger_act:
            self.data.ctrl[actuator] = command

    def open_fingers(self) -> None:
        self.set_fingers(self.open_cmd)

    def close_fingers(self) -> None:
        self.set_fingers(self.grasp_cmd)

    def hold_joints(self) -> None:
        # Hold the commanded setpoint. Copying measured qpos into ctrl drops
        # the gravity-holding pose and the wrist sags.
        if self.stream is None:
            for address, actuator in zip(self.index.arm_q, self.index.arm_act, strict=True):
                self.data.ctrl[actuator] = self.data.qpos[address]
            return
        self.stream.v[:] = 0.0
        self._apply(self.stream.q)

    def track_midpoint(self, point: np.ndarray, dt: float, attract: np.ndarray | None = None) -> float:
        """Servo the fingertip midpoint toward `point`. Returns the current error (m).

        `point` is this tick's Cartesian sample. IK warm-starts from the joint
        command so a lagging measurement cannot flip the solution, and the
        command is acceleration-limited.
        """
        rotation = self.data.site_xmat[self.index.site].reshape(3, 3)
        point = np.asarray(point, dtype=float)
        aim = np.asarray(attract, dtype=float) if attract is not None else point
        pointing = float(-rotation[2, 2])
        if not self.deployed:
            self._tool_locked = False
            self._cached_q = None
            self._reseed_age = 0.0
        # Hysteresis: a 0.85 threshold on the live pose toggled the site target
        # every tick (~5 cm XY jump) and kit 101 never finished the descend.
        if pointing > 0.90:
            self._tool_locked = True
        elif pointing < 0.70:
            self._tool_locked = False
        if self._tool_locked:
            down_R = self._down_frame(rotation)
            site_target = aim - down_R @ self.mid_offset
        else:
            site_target = aim.copy()
            site_target[2] += max(abs(float(self.mid_offset[2])), 0.04)
        self._ensure_stream()
        assert self.stream is not None
        pos_err = float(np.linalg.norm(self.midpoint() - aim))
        if dt > 0.05:
            # Large-dt snap path used by the reach unit test. The 4 ms stream
            # below is what the mission runs.
            if not self.deployed:
                self._cached_q = self._solve(site_target, prefer=POSTURE)
                self.deployed = True
            else:
                self._cached_q, _err = self._ik(site_target, self.stream.q, iterations=30, dq_lim=0.22)
            self._command(self._cached_q, dt)
            return float(np.linalg.norm(self.midpoint() - point))
        if dt <= 0.02:
            self._reseed_age += dt
        far = pos_err > 0.18
        arrived = self._cached_q is None or float(np.max(np.abs(self.stream.q - self._cached_q))) < 0.25
        # Joint-space move to a seeded pose that reaches `aim`. Tick DLS on the
        # Cartesian sample from stow winds torso onto the wrap stops.
        if not self.deployed or (far and arrived and (self._reseed_age > 0.40 or dt > 0.02)):
            # POSTURE is the unstow seed. Using it for a far belt target folds
            # a deployed arm through the kit (~27 cm error, midz through the plate).
            prefer = POSTURE if (not self.deployed or self._pinned()) else self.stream.q
            self._cached_q = self._solve(site_target, prefer=prefer)
            self.deployed = True
            self._reseed_age = 0.0
        elif not far:
            self._cached_q, _err = self._ik(site_target, self.stream.q, iterations=12, dq_lim=0.15)
        elif self._cached_q is None:
            self._cached_q = self.stream.q.copy()
        self._command(self._cached_q, dt)
        return float(np.linalg.norm(self.midpoint() - point))

    def approach(self, point: np.ndarray, dt: float) -> float:
        """Follow a nearby Cartesian sample with the tool free.

        High transit only. Orientation weight stays at zero so the wrist does
        not pitch into the plate while the arm is still crossing the kit.
        """
        point = np.asarray(point, dtype=float)
        saved_orient = self.orient_weight
        saved_posture = self.posture_weight
        self.orient_weight = 0.0
        self.posture_weight = 0.0
        try:
            self.mid_offset = self._fingertip_offset()
            rotation = self.data.site_xmat[self.index.site].reshape(3, 3)
            site = point - rotation @ self.mid_offset
            self._ensure_stream()
            assert self.stream is not None
            target, _err = self._ik(site, self.stream.q, iterations=4, dq_lim=0.05)
            self._command(target, dt)
            self.deployed = True
        finally:
            self.orient_weight = saved_orient
            self.posture_weight = saved_posture
        return float(np.linalg.norm(self.midpoint() - point))

    @property
    def locked_down(self) -> bool:
        return self._down_q is not None

    def blend_done(self) -> bool:
        return self._blend_T > 0.0 and self._blend_t >= self._blend_T - 1e-9

    def drop_goal(self) -> None:
        self._blend_point = None
        self._down_q = None
        self._blend_T = 0.0
        self._blend_t = 0.0
        self.rejected = False

    def hold_blend(self) -> None:
        """Keep the last tool-down command while the fingers move."""
        self._ensure_stream()
        assert self.stream is not None
        if self._blend_T > 0.0:
            self.stream.q = self._blend_q1.copy()
        self.stream.v[:] = 0.0
        self._apply(self.stream.q)

    def move_down(self, point: np.ndarray, dt: float, *, track: bool = False) -> float:
        """Min-jerk the straight joint chord to a tool-down solution of `point`.

        The chord is rejected when an arm mesh would pass through a kit plate.
        An independent joint-rate limit bows off this line and does exactly that.
        `track=True` retargets millimetre belt motion from the live command so
        a 6 mm freeze cannot walk the ring off the flange.
        """
        point = np.asarray(point, dtype=float)
        goal = point.copy()
        self.rejected = False
        finished = self._blend_T <= 0.0 or self._blend_t >= self._blend_T - 1e-9
        moved = True
        if self._blend_point is not None:
            delta = point - self._blend_point
            xy_lim = 0.0015 if track else 0.006
            z_lim = 0.002 if track else 0.004
            moved = float(np.hypot(delta[0], delta[1])) > xy_lim or abs(float(delta[2])) > z_lim
        # Never splice a vertical drop mid-blend. Restarting every 1.5 mm of
        # belt travel left kit 104 at z=0.145 because Z never finished.
        if finished and (self._blend_point is None or moved):
            current = self._commanded()
            if track:
                q, err, tool = self._nudge(point, current)
                if q is None or err > 0.018:
                    q, err, tool = self._snap_solve(point, current, ignore_fingers=True)
                if q is None or err > 0.018:
                    free_q, free_err, free_tool = self._refine(point, current)
                    if free_tool > 0.88 and (q is None or free_err < err):
                        q, err, tool = free_q, free_err, free_tool
            else:
                q, err, tool = self._snap_solve(point, current)
                if q is None:
                    # The seeded search can fall into a tool-up basin on a 2 cm
                    # belt step. A short step from the live command stays down.
                    q, err, tool = self._nudge(point, current)
            if q is None:
                # A long vertical chord can sweep the plate even when both ends
                # are clear. Step halfway and continue on the next retarget.
                halfway = point.copy()
                halfway[2] = 0.5 * (float(point[2]) + float(self.midpoint()[2]))
                if abs(float(halfway[2] - point[2])) > 0.02:
                    q, err, tool = self._nudge(halfway, current) if track else self._snap_solve(halfway, current)
                    if q is None:
                        q, err, tool = self._nudge(halfway, current)
                    if q is not None:
                        point = halfway
            if q is not None:
                # Do not blend toward a seed that is farther than the pose
                # already being held. That is how a 1 cm hover became a 4 cm miss.
                live_err, live_tool = self._score_pose(current, point)
                if (not track) and live_tool > 0.90 and live_err <= err + 0.002:
                    q = np.asarray(current, dtype=float).copy()
                    err = live_err
                self.pose_err = float(err)
                self._begin_blend(current, q, quick=track)
                self._blend_point = point.copy()
                self._down_q = q.copy()
                self._told = False
            else:
                self.rejected = True
                self._blend_point = point.copy()
                if not self._told:
                    print(
                        f"keep-out hold err={err:.3f} tool={tool:.2f} geom={self._hit_name}",
                        flush=True,
                    )
                    self._told = True
        if self._blend_T > 0.0:
            self._blend_t = min(self._blend_T, self._blend_t + dt)
            mix = minjerk(self._blend_t / self._blend_T)
            command = (1.0 - mix) * self._blend_q0 + mix * self._blend_q1
        else:
            command = self._commanded()
        self._apply(command)
        self._ensure_stream()
        assert self.stream is not None
        self.stream.q = np.asarray(command, dtype=float).copy()
        self.stream.v[:] = 0.0
        self.deployed = True
        return float(np.linalg.norm(self.midpoint() - goal))

    def _begin_blend(self, q0: np.ndarray, q1: np.ndarray, *, quick: bool = False) -> None:
        self._blend_q0 = np.clip(np.asarray(q0, dtype=float), self.ik_ranges[:, 0], self.ik_ranges[:, 1])
        self._blend_q1 = np.clip(np.asarray(q1, dtype=float), self.ik_ranges[:, 0], self.ik_ranges[:, 1])
        delta = float(np.max(np.abs(self._blend_q1 - self._blend_q0)))
        lo, hi = (0.06, 0.40) if quick else (0.10, 2.2)
        rate = 2.40 if quick else 1.20
        self._blend_T = float(np.clip(max(lo, delta / rate), lo, hi))
        self._blend_t = 0.0

    def _commanded(self) -> np.ndarray:
        if self.stream is not None:
            return self.stream.q.copy()
        return np.array([float(self.data.ctrl[a]) for a in self.index.arm_act])

    def _snap_solve(
        self, point: np.ndarray, current: np.ndarray, *, ignore_fingers: bool | None = None,
    ) -> tuple[np.ndarray | None, float, float]:
        """Best tool-down pose whose straight chord from `current` misses the plate."""
        seeds: list[np.ndarray] = [np.asarray(current, dtype=float)]
        if self._down_q is not None:
            seeds.append(self._down_q)
        seeds.append(BELT_POSE.copy())
        seeds.append(GRASP_POSE.copy())
        seeds.append(POSTURE.copy())
        best_q: np.ndarray | None = None
        best_err = 1.0
        best_tool = -1.0
        best_score = 1e9
        for seed in seeds:
            q, err, tool = self._refine(point, seed)
            # A short tool-down correction may brush the plate with the jaws.
            # A long chord still has to clear the plate with the fingers too,
            # which is what rejects the folded pitch.
            modest = float(np.max(np.abs(q - current))) < 1.05
            start_tool = self._score_pose(current, point)[1]
            skip_fingers = ignore_fingers if ignore_fingers is not None else (
                modest and start_tool > 0.90 and tool >= 0.88
            )
            if err > 0.045 or tool < 0.88 or not self._chord_clear(current, q, skip_fingers):
                if best_q is None and err < best_err:
                    best_err, best_tool = err, tool
                continue
            score = err + 0.2 * max(0.0, 0.96 - tool)
            if score < best_score:
                best_score = score
                best_q, best_err, best_tool = q, err, tool
            if err < 0.012 and tool > 0.96:
                break
        return best_q, best_err, best_tool

    def _refine(self, point: np.ndarray, seed: np.ndarray) -> tuple[np.ndarray, float, float]:
        """Current-frame IK on the scratch model, starting from `seed`.

        A seed that is already on the point is kept. Extra damped steps from a
        good pose walk the wrist off the belt.
        """
        point = np.asarray(point, dtype=float)
        q = np.asarray(seed, dtype=float).copy()
        err, tool = self._score_pose(q, point)
        best_q, best_err, best_tool = q.copy(), err, tool
        # A seed already on the point keeps its tool. Otherwise walk position
        # without the orientation boost, which stalls a centimetre short.
        if not (err < 0.012 and tool > 0.96):
            saved = self.orient_weight
            self.orient_weight = 0.0
            try:
                for _ in range(12):
                    self.scratch.qpos[:] = self.data.qpos
                    self.scratch.qpos[self.index.arm_q] = q
                    mujoco.mj_forward(self.model, self.scratch)
                    rotation = self.scratch.site_xmat[self.index.site].reshape(3, 3)
                    mid = 0.5 * (
                        self.scratch.xpos[self.index.tip_right]
                        + self.scratch.xpos[self.index.tip_left]
                    )
                    offset = rotation.T @ (mid - self.scratch.site_xpos[self.index.site])
                    site = point - rotation @ offset
                    q_next, _site = self._ik(site, q, iterations=3, dq_lim=0.04, orient_boost=False)
                    err, tool = self._score_pose(q_next, point)
                    if best_tool > 0.96 and tool < 0.96:
                        break
                    q = q_next
                    if err < best_err or (tool > best_tool and err < best_err + 0.002):
                        best_q, best_err, best_tool = q.copy(), err, tool
                    if best_err < 0.004 and best_tool > 0.985:
                        break
            finally:
                self.orient_weight = saved
        # The wrist still has travel at the seat. A small pitch stands the ring
        # up without giving back the position the Jacobian would trade away.
        if best_err < 0.012 and best_tool < 0.990:
            lo, hi = float(self.ik_ranges[5, 0]), float(self.ik_ranges[5, 1])
            for delta in (0.04, 0.08, 0.12, -0.04, -0.08, -0.12):
                trial = best_q.copy()
                trial[5] = float(np.clip(trial[5] + delta, lo, hi))
                err, tool = self._score_pose(trial, point)
                if tool > best_tool + 0.01 and err < 0.012:
                    best_q, best_err, best_tool = trial, err, tool
        return best_q, best_err, best_tool

    def _nudge(
        self, point: np.ndarray, current: np.ndarray,
    ) -> tuple[np.ndarray | None, float, float]:
        """Short tool-down steps from the live command. Used when a reseed flips."""
        q = np.asarray(current, dtype=float).copy()
        err, tool = self._score_pose(q, point)
        if tool < 0.90:
            return None, err, tool
        best_q, best_err, best_tool = q.copy(), err, tool
        for _ in range(5):
            self.scratch.qpos[:] = self.data.qpos
            self.scratch.qpos[self.index.arm_q] = q
            mujoco.mj_forward(self.model, self.scratch)
            rotation = self.scratch.site_xmat[self.index.site].reshape(3, 3)
            mid = 0.5 * (
                self.scratch.xpos[self.index.tip_right] + self.scratch.xpos[self.index.tip_left]
            )
            offset = rotation.T @ (mid - self.scratch.site_xpos[self.index.site])
            site = point - rotation @ offset
            q_next, _site = self._ik(site, q, iterations=2, dq_lim=0.03, orient_boost=False)
            err, tool = self._score_pose(q_next, point)
            if tool < 0.90:
                break
            q = q_next
            if err < best_err:
                best_q, best_err, best_tool = q.copy(), err, tool
            if best_err < 0.008:
                break
        if best_tool < 0.90 or best_err > 0.040:
            return None, best_err, best_tool
        if float(np.max(np.abs(best_q - current))) > 0.40:
            return None, best_err, best_tool
        if not self._chord_clear(current, best_q, ignore_fingers=True):
            return None, best_err, best_tool
        return best_q, best_err, best_tool

    def _score_pose(self, q: np.ndarray, point: np.ndarray) -> tuple[float, float]:
        self.scratch.qpos[:] = self.data.qpos
        self.scratch.qpos[self.index.arm_q] = q
        mujoco.mj_forward(self.model, self.scratch)
        mid = 0.5 * (self.scratch.xpos[self.index.tip_right] + self.scratch.xpos[self.index.tip_left])
        err = float(np.linalg.norm(mid - point))
        tool = float(-self.scratch.site_xmat[self.index.site].reshape(3, 3)[2, 2])
        return err, tool

    def _chord_clear(self, q0: np.ndarray, q1: np.ndarray, ignore_fingers: bool = False) -> bool:
        self._hit_name = ""
        for mix in np.linspace(0.0, 1.0, 16):
            if self._hits_plate((1.0 - mix) * q0 + mix * q1, ignore_fingers):
                return False
        return True

    def _hits_plate(self, q: np.ndarray, ignore_fingers: bool = False) -> bool:
        self._cache_geoms()
        scratch = self.scratch
        scratch.qpos[:] = self.data.qpos
        scratch.mocap_pos[:] = self.data.mocap_pos
        scratch.mocap_quat[:] = self.data.mocap_quat
        scratch.qpos[self.index.arm_q] = q
        mujoco.mj_forward(self.model, scratch)
        assert self._arm_geoms is not None and self._kit_boxes is not None
        for box in self._kit_boxes:
            corners = self._corners(scratch, box)
            top = max(float(corner[2]) for corner in corners)
            if top < 0.4:
                continue
            xs = [float(corner[0]) for corner in corners]
            ys = [float(corner[1]) for corner in corners]
            x0, x1 = min(xs) - 0.012, max(xs) + 0.012
            y0, y1 = min(ys) - 0.012, max(ys) + 0.012
            for geom in self._arm_geoms:
                if ignore_fingers and geom in self._finger_geoms:
                    continue
                for corner in self._corners(scratch, geom):
                    if corner[2] < top - 0.003 and x0 <= corner[0] <= x1 and y0 <= corner[1] <= y1:
                        body = int(self.model.geom_bodyid[geom])
                        self._hit_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
                        return True
        return False

    def _cache_geoms(self) -> None:
        if self._arm_geoms is not None:
            return
        geoms: list[int] = []
        fingers: list[int] = []
        boxes: list[int] = []
        for geom in range(self.model.ngeom):
            body = int(self.model.geom_bodyid[geom])
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
            gname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ""
            if name.startswith("link_") and int(self.model.geom_group[geom]) in (2, 3):
                geoms.append(geom)
                if "finger" in name:
                    fingers.append(geom)
            if gname.startswith("kit_") and gname.endswith("_box"):
                boxes.append(geom)
        self._arm_geoms = geoms
        self._finger_geoms = set(fingers)
        self._kit_boxes = boxes

    def _corners(self, data: mujoco.MjData, geom: int) -> list[np.ndarray]:
        center = self.model.geom_aabb[geom, :3]
        half = self.model.geom_aabb[geom, 3:]
        rotation = data.geom_xmat[geom].reshape(3, 3)
        origin = data.geom_xpos[geom]
        corners = []
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    local = center + half * np.array([sx, sy, sz])
                    corners.append(origin + rotation @ local)
        return corners

    def stow_step(self, dt: float) -> bool:
        self.drop_goal()
        self._command(self._stow, dt)
        error = max(abs(float(self.data.qpos[q]) - float(goal)) for q, goal in zip(self.index.arm_q, self._stow, strict=True))
        self.open_fingers()
        if error < 0.18:
            self.deployed = False
            self._tool_locked = False
            self._cached_q = None
            return True
        return False

    def is_stowed(self) -> bool:
        if self.deployed:
            return False
        return max(abs(float(self.data.qpos[q])) for q in self.index.arm_q) < 0.22

    def brake(self) -> None:
        # Do not copy qpos into ctrl. That drops the gravity-holding setpoint and
        # the arm sags through the stop window.
        for dof in (*self.arm_dof, *self.finger_dof):
            self.data.qvel[dof] = 0.0

    def _pinned(self) -> bool:
        if self.stream is None:
            return False
        for index in (0, 2, 4, 6):
            value = float(self.stream.q[index])
            lo, hi = self.ik_ranges[index]
            if value <= lo + 0.05 or value >= hi - 0.05:
                return True
        return False

    def _ensure_stream(self) -> None:
        if self.stream is not None:
            return
        current = np.array([float(self.data.ctrl[a]) for a in self.index.arm_act])
        self.stream = JointStream(current, self.ranges)

    def _apply(self, q: np.ndarray) -> None:
        for actuator, value in zip(self.index.arm_act, q, strict=True):
            self.data.ctrl[actuator] = float(value)

    def _command(self, target: np.ndarray, dt: float) -> None:
        self._ensure_stream()
        assert self.stream is not None
        # Reach/unit tests call this with a large dt and snap qpos to ctrl.
        # The acceleration stream is for the 4 ms physics tick.
        if dt > 0.05:
            self.stream.q = np.clip(np.asarray(target, dtype=float), self.ranges[:, 0], self.ranges[:, 1])
            self.stream.v[:] = 0.0
            self._apply(self.stream.q)
            return
        self._apply(self.stream.step(target, dt))

    def _down_frame(self, rotation: np.ndarray) -> np.ndarray:
        """Tool frame with world -Z, keeping the current yaw.

        The fingertip offset is only valid for the orientation we are commanding.
        Using the stowed (tool-up) frame aims the site at the wrong point, so a
        downward seed looks worse than staying upside down.
        """
        z_axis = np.array([0.0, 0.0, -1.0])
        x_axis = rotation[:, 0] - z_axis * float(np.dot(rotation[:, 0], z_axis))
        if float(np.linalg.norm(x_axis)) < 1e-6:
            x_axis = np.array([1.0, 0.0, 0.0])
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    def _solve(self, site_target: np.ndarray, prefer: np.ndarray | None = None) -> np.ndarray:
        seeds: list[np.ndarray] = []
        if prefer is not None:
            seeds.append(np.asarray(prefer, dtype=float))
        seeds.append(self.data.qpos[self.index.arm_q].copy())
        seeds.extend(SEEDS)
        best_q = seeds[0]
        best_err = 1e9
        prefer_q: np.ndarray | None = None
        prefer_err = 1e9
        for index, seed in enumerate(seeds):
            q, err = self._ik(site_target, seed, dq_lim=0.22)
            if index == 0 and prefer is not None:
                prefer_q, prefer_err = q, err
            if err < best_err:
                best_q, best_err = q, err
            if err < 0.008:
                break
        # Stay in the current configuration unless another seed is clearly better.
        if prefer_q is not None and best_err > prefer_err - 0.004:
            return prefer_q
        return best_q

    def _ik(
        self, site_target: np.ndarray, seed: np.ndarray, iterations: int = 30, dq_lim: float = 0.12,
        orient_boost: bool = True,
    ) -> tuple[np.ndarray, float]:
        scratch = self.scratch
        scratch.qpos[:] = self.data.qpos
        scratch.qpos[self.index.arm_q] = seed
        mujoco.mj_forward(self.model, scratch)
        dofs = self.arm_dof
        qadr = self.index.arm_q
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        for _ in range(iterations):
            mujoco.mj_jacSite(self.model, scratch, jacp, jacr, self.index.site)
            site = scratch.site_xpos[self.index.site]
            pos_err = site_target - site
            rotation = scratch.site_xmat[self.index.site].reshape(3, 3)
            # Prefer a downward tool without letting orientation spoil the position.
            pos_n = float(np.linalg.norm(pos_err))
            pointing = float(-rotation[2, 2])
            if orient_boost and pointing > 0.90 and pos_n < 0.22:
                # Hold tool-down across the lift (~13 cm). A 4 cm window let the
                # wrist flip as soon as the vertical segment started.
                weight = 0.45
            else:
                weight = self.orient_weight * float(np.clip(1.0 - pos_n / 0.04, 0.35, 1.0))
            z_err = np.array([0.0, 0.0, -1.0]) - rotation[:, 2]
            error = np.concatenate([pos_err, weight * z_err])
            jac = np.vstack([jacp[:, dofs], weight * jacr[:, dofs]])
            damp = 0.04
            hess = jac @ jac.T + damp * np.eye(6)
            dq = jac.T @ np.linalg.solve(hess, error)
            posture = self.posture_weight * (POSTURE - scratch.qpos[qadr])
            # Project posture into the Jacobian null space.
            jjt = jac.T @ np.linalg.solve(hess, jac)
            dq = dq + (np.eye(len(dofs)) - jjt) @ posture
            dq = np.clip(dq, -dq_lim, dq_lim)
            scratch.qpos[qadr] = np.clip(scratch.qpos[qadr] + dq, self.ranges[:, 0], self.ranges[:, 1])
            mujoco.mj_forward(self.model, scratch)
            pointing = float(-scratch.site_xmat[self.index.site].reshape(3, 3)[2, 2])
            if np.linalg.norm(pos_err) < 0.004 and pointing > 0.9:
                break
        final = float(np.linalg.norm(scratch.site_xpos[self.index.site] - site_target))
        return scratch.qpos[qadr].copy(), final


class JointStream:
    """Acceleration-limited joint command.

    The previous slew limit changed speed in a single control tick. That is a
    step in acceleration, which is the jerk visible in the recordings. This
    stream ramps speed, so a smooth Cartesian sample stays smooth in joint space
    even when the IK target chatters by a few hundredths of a radian.
    """

    def __init__(self, q: np.ndarray, ranges: np.ndarray):
        self.q = np.asarray(q, dtype=float).copy()
        self.v = np.zeros_like(self.q)
        self.ranges = ranges
        self.vmax = 2.8
        self.amax = 20.0

    def step(self, target: np.ndarray, dt: float) -> np.ndarray:
        dt = max(float(dt), 1e-4)
        target = np.clip(np.asarray(target, dtype=float), self.ranges[:, 0], self.ranges[:, 1])
        desired = np.clip((target - self.q) / dt, -self.vmax, self.vmax)
        self.v = self.v + np.clip(desired - self.v, -self.amax * dt, self.amax * dt)
        self.q = np.clip(self.q + self.v * dt, self.ranges[:, 0], self.ranges[:, 1])
        return self.q.copy()


def _joint_id(model: mujoco.MjModel, qadr: int) -> int:
    matches = np.flatnonzero(model.jnt_qposadr == qadr)
    if len(matches) != 1:
        raise RuntimeError(f"qpos address {qadr} is not a 1-DoF joint")
    return int(matches[0])
