"""Physics gates. Skipped where the MuJoCo library cannot load."""

from __future__ import annotations

import pytest

try:
    import mujoco
    import numpy as np

    from advaitic_cell.arm import ArmController, engage_weld
    from advaitic_cell.frames import ring_world, seat_world
    from advaitic_cell.scene import base_pose, base_speed, load_cell, set_base_pose
    from advaitic_cell.spec import KIT_TOP_Z, dock_pose
except (ImportError, OSError):
    pytest.skip("mujoco unavailable", allow_module_level=True)


def test_base_mass_and_speed():
    model, data, index = load_cell()
    mass = float(model.body_subtreemass[index.base_body])
    assert 40.0 < mass < 120.0
    set_base_pose(data, index, 1.0, 3.0, 0.0)
    data.ctrl[index.base_act[0]] = 0.2
    mujoco.mj_forward(model, data)
    for _ in range(250):
        mujoco.mj_step(model, data)
    assert base_speed(data, index) > 0.12
    assert base_pose(data, index)[0] > 1.05


def test_weld_holds_without_snapping():
    model, data, index = load_cell()
    mujoco.mj_forward(model, data)
    slot = index.slots[0]
    gripper = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_gripper"))
    start = data.xpos[gripper] + np.array([0.0, 0.0, 0.04])
    data.qpos[slot.ring_q:slot.ring_q + 3] = start
    data.qpos[slot.ring_q + 3:slot.ring_q + 7] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    before = data.xpos[slot.ring_body].copy()
    engage_weld(model, data, slot.equality, gripper, slot.ring_body)
    arm = ArmController(model, data, index)
    for _ in range(100):
        arm.hold_joints()
        mujoco.mj_step(model, data)
    after = data.xpos[slot.ring_body]
    assert np.linalg.norm(after - before) < 0.015


def test_ring_stays_seated_on_moving_bottle():
    model, data, index = load_cell()
    slot = index.slots[0]
    for geom, contype, affinity in zip(slot.geoms, slot.contype, slot.conaffinity, strict=True):
        model.geom_contype[geom] = contype
        model.geom_conaffinity[geom] = affinity
    seat = np.array(seat_world(1, 0.0))
    data.mocap_pos[slot.mocap] = (3.07, 1.2, KIT_TOP_Z)
    data.mocap_quat[slot.mocap] = (1.0, 0.0, 0.0, 0.0)
    data.qpos[slot.ring_q:slot.ring_q + 3] = seat
    data.qpos[slot.ring_q + 3:slot.ring_q + 7] = (1.0, 0.0, 0.0, 0.0)
    data.qvel[slot.ring_v:slot.ring_v + 3] = (0.03, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    speed = 0.03
    for step in range(500):
        t = (step + 1) * 0.004
        local = speed * t
        data.mocap_pos[slot.mocap] = (3.07 + local, 1.2, KIT_TOP_Z)
        mujoco.mj_step(model, data)
    ring = data.xpos[slot.ring_body]
    bottle = np.array(seat_world(1, speed * 2.0))
    # seat_world's z is the target center; compare xy to the bottle and z to the window.
    assert abs(ring[0] - bottle[0]) < 0.004
    assert abs(ring[1] - bottle[1]) < 0.004
    assert 0.130 <= ring[2] - KIT_TOP_Z <= 0.136


def test_arm_reaches_work_window_from_both_docks():
    from advaitic_cell.arm import POSTURE

    model, data, index = load_cell()
    arm = ArmController(model, data, index)
    worst = 0.0
    for station in (1, 2):
        set_base_pose(data, index, *dock_pose(station))
        targets = [("ring", local, np.array(ring_world(station, local), dtype=float)) for local in (-0.30, -0.10, 0.05)]
        targets += [("seat", local, np.array(seat_world(station, local), dtype=float)) for local in (-0.08, 0.0, 0.10)]
        for label, local, point in targets:
            arm.deployed = False
            arm.stream = None
            data.qpos[index.arm_q] = POSTURE
            data.ctrl[index.arm_act] = POSTURE
            mujoco.mj_forward(model, data)
            error = 1.0
            for _ in range(40):
                error = arm.track_midpoint(point, 0.4)
                data.qpos[index.arm_q] = data.ctrl[index.arm_act]
                mujoco.mj_forward(model, data)
            down = float(-data.site_xmat[index.site].reshape(3, 3)[2, 2])
            worst = max(worst, error)
            assert error < 0.080, f"station {station} {label} {local} error {error:.4f}"
            assert down > 0.70, f"station {station} {label} {local} tool down {down:.2f}"
    assert worst < 0.080


def test_arm_keepout_and_person_mesh_have_no_base_shove():
    from advaitic_cell.scene import KIT_CONTACT

    model, _data, index = load_cell()
    assert index.arm_collide_geoms, "arm collision meshes should still be identified"
    belt = index.belt_geoms[0]
    assert KIT_CONTACT in index.slots[0].contype
    geom = index.arm_collide_geoms[0]
    arm_type, arm_aff = int(model.geom_contype[geom]), int(model.geom_conaffinity[geom])
    belt_type, belt_aff = int(model.geom_contype[belt]), int(model.geom_conaffinity[belt])
    assert not ((arm_type & belt_aff) and (belt_type & arm_aff))
    human = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "human_body"))
    assert int(model.geom_type[human]) == int(mujoco.mjtGeom.mjGEOM_MESH)
    assert int(model.geom_contype[human]) == 0
    assert int(model.geom_conaffinity[human]) == 0


def test_safety_lidar_hits_the_person_mesh():
    from advaitic_cell.perception import person_visible

    model, data, index = load_cell()
    set_base_pose(data, index, *dock_pose(1))
    data.mocap_pos[index.human_mocap] = (2.62, 2.18, 0.0)
    data.mocap_quat[index.human_mocap] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    assert person_visible(model, data, index, 1)
