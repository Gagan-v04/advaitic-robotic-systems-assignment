"""Compose the customer cell around the vendored Google Robot.

The Menagerie model is a fixed-base arm. Planar drive joints are added here so the
same MJCF can dock and travel. Mesh density is zeroed because those visuals were
authored for a welded base and otherwise dominate the inertia; link inertials from
the vendor file are kept, and the base gets an explicit chassis inertia.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from advaitic_cell.spec import DT, ROBOT_DIR, ROBOT_XML, ROOT

PERSON_DIR = ROOT / "third_party" / "person"
# Kit contact bit 8 is reserved for a convex arm proxy. Vendor STL meshes
# stay identified but non-contacting after full STL contact SIGKILL'd Docker.
ARM_KIT_BIT = 8
KIT_CONTACT = 2 | ARM_KIT_BIT
ARM_COLLIDE_BODIES = {
    "link_elbow", "link_forearm", "link_wrist", "link_gripper",
}

SLOTS_PER_STATION = 6
RING_SEGMENTS = 20


@dataclass
class SlotIndex:
    slot: int
    station: int
    kit_body: int
    bottle_body: int
    ring_body: int
    ring_q: int
    ring_v: int
    mocap: int
    equality: int
    geoms: list[int] = field(default_factory=list)
    contype: list[int] = field(default_factory=list)
    conaffinity: list[int] = field(default_factory=list)


@dataclass
class ModelIndex:
    site: int
    tip_right: int
    tip_left: int
    base_body: int
    base_q: tuple[int, int, int]
    base_v: tuple[int, int, int]
    base_act: tuple[int, int, int]
    arm_q: list[int]
    arm_act: list[int]
    finger_q: list[int]
    finger_act: list[int]
    slots: list[SlotIndex]
    human_mocap: int
    belt_geoms: list[int]
    hull_geom: int
    lidar_mocap: int | None = None
    arm_collide_geoms: list[int] = field(default_factory=list)


def _look_camera(name: str, eye: tuple[float, float, float], target: tuple[float, float, float], fovy: float) -> str:
    eye_v = np.array(eye, dtype=float)
    z = eye_v - np.array(target, dtype=float)
    z /= np.linalg.norm(z)
    x = np.cross(np.array([0.0, 0.0, 1.0]), z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    axes = " ".join(f"{value:.5f}" for value in (*x, *y))
    return (
        f'<camera name="{name}" pos="{eye_v[0]:.3f} {eye_v[1]:.3f} {eye_v[2]:.3f}" '
        f'xyaxes="{axes}" fovy="{fovy}"/>'
    )


def _cone_vertices() -> str:
    """Guide above the seat. The seated ring (top near 138 mm) never touches it.

    A ring lowered a few millimetres off centre hits the narrow top first and
    slides inward before it reaches the flange.
    """
    vertices = []
    for z, radius in ((0.132, 0.022), (0.185, 0.006)):
        for i in range(16):
            angle = i * 2.0 * np.pi / 16.0
            vertices.extend((radius * np.cos(angle), radius * np.sin(angle), z))
    return " ".join(f"{value:.6f}" for value in vertices)


def _kit_lip(slot: int) -> str:
    """Shallow pocket so the free ring cannot walk off the kit before the grasp."""
    parts = []
    for i in range(8):
        angle = 2.0 * np.pi * i / 8.0
        x = 0.07 + 0.052 * np.cos(angle)
        y = 0.045 + 0.052 * np.sin(angle)
        parts.append(
            f'<geom name="kit_{slot}_lip_{i}" type="box" pos="{x:.4f} {y:.4f} 0.0015" '
            f'size="0.006 0.006 0.0015" rgba="0.55 0.42 0.25 1" mass="0.002" '
            f'friction="1.2 0.02 0.001" group="2"/>'
        )
    return "".join(parts)


def _ring_geoms(slot: int) -> str:
    parts = []
    count = RING_SEGMENTS
    for i in range(count):
        angle = 2.0 * np.pi * i / count
        parts.append(
            f'<geom name="ring_{slot}_{i}" type="box" '
            f'pos="{0.032 * np.cos(angle):.6f} {0.032 * np.sin(angle):.6f} 0" '
            f'euler="0 0 {angle:.6f}" '
            f'size="0.008 {0.032 * np.sin(np.pi / count):.6f} 0.004" '
            f'mass="{0.04 / count:.6f}" rgba="0.95 0.32 0.08 1" '
            f'friction="1.8 0.2 0.002" condim="4" solref="0.02 1" '
            f'solimp="0.95 0.99 0.001" margin="0.0008" group="2"/>'
        )
    return "".join(parts)


def build_xml() -> str:
    text = ROBOT_XML.read_text(encoding="utf-8")
    text = text.replace(
        '<geom type="mesh" contype="0" conaffinity="0" group="2" material="robot_mtl"/>',
        '<geom type="mesh" contype="0" conaffinity="0" group="2" density="0" material="robot_mtl"/>',
    )
    text = text.replace(
        '<geom type="mesh" rgba="1 1 1 1" group="3" contype="1" conaffinity="1"/>',
        '<geom type="mesh" rgba="1 1 1 1" group="3" contype="0" conaffinity="0" density="0"/>',
    )
    text = text.replace(
        '<option timestep="0.004" integrator="implicitfast"/>',
        '<option timestep="0.004" integrator="implicitfast" iterations="12" '
        'ls_iterations="8" cone="pyramidal"/>',
    )
    old = '<body name="base_link" childclass="robot" pos="0 0 0.06205">'
    joints = """
      <inertial pos="0 0 0.04" mass="55" diaginertia="1.8 1.8 0.9"/>
      <joint name="base_x" type="slide" axis="1 0 0" damping="8" frictionloss="0" armature="0"/>
      <joint name="base_y" type="slide" axis="0 1 0" damping="8" frictionloss="0" armature="0"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1" damping="4" frictionloss="0" armature="0"/>
      <geom name="base_hull" type="cylinder" pos="0 0 0.09" size="0.28 0.07"
            rgba="0.2 0.55 0.3 0.25" mass="0" group="3"/>
"""
    if old not in text:
        raise RuntimeError("vendor robot XML no longer has the expected base_link body")
    text = text.replace(old, old + joints, 1)
    # Drop the original closing tag; the cell appends the rest of the model.
    if not text.rstrip().endswith("</mujoco>"):
        raise RuntimeError("vendor robot XML is not a single mujoco model")
    text = text.rstrip()[: -len("</mujoco>")]
    return text + _cell_fragment() + "\n</mujoco>\n"


def _cell_fragment() -> str:
    cone = _cone_vertices()
    slots = []
    excludes = []
    welds = []
    for slot in range(SLOTS_PER_STATION * 2):
        station = 1 if slot < SLOTS_PER_STATION else 2
        park = -4.0 - 0.5 * slot
        slots.append(f"""
    <body name="kit_{slot}" pos="{park} -3 -2" mocap="true">
      <geom name="kit_{slot}_box" type="box" pos="0 0 -0.0125" size="0.15 0.12 0.0125"
            rgba="0.75 0.62 0.4 1" friction="1.6 0.02 0.001" group="2" mass="0.4"/>
      {_kit_lip(slot)}
      <body name="bottle_{slot}" pos="-0.07 0.045 0">
        <geom name="bottle_{slot}_body" type="cylinder" pos="0 0 0.06" size="0.03 0.06"
              rgba="0.2 0.75 0.35 1" mass="0.08" friction="1.6 0.1 0.002" condim="4" group="2"/>
        <geom name="bottle_{slot}_flange" type="cylinder" pos="0 0 0.127" size="0.030 0.002"
              rgba="0.2 0.75 0.35 1" mass="0.02" friction="1.8 0.2 0.002" condim="4" group="2"/>
        <geom name="bottle_{slot}_cone" type="mesh" mesh="seat_cone"
              rgba="0.2 0.75 0.35 1" mass="0.02" friction="0.35 0.01 0.001" condim="4" group="2"/>
      </body>
    </body>
    <body name="ring_{slot}" pos="{park} -3 -2">
      <freejoint name="ring_{slot}_free"/>
      {_ring_geoms(slot)}
    </body>""")
        excludes.append(f'<exclude body1="kit_{slot}" body2="bottle_{slot}"/>')
        welds.append(
            f'<weld name="grasp_{slot}" body1="link_gripper" body2="ring_{slot}" '
            f'active="false" solref="0.004 1" solimp="0.95 0.99 0.001"/>'
        )
        del station
    cameras = [
        _look_camera("overview", (5.55, -0.85, 4.15), (3.0, 3.0, 0.55), 42),
        _look_camera("station_a", (4.85, 2.25, 1.72), (2.95, 1.28, 0.86), 32),
        _look_camera("station_b", (4.85, 3.75, 1.72), (2.95, 4.72, 0.86), 32),
        _look_camera("top_a", (3.15, 1.72, 2.55), (3.02, 1.32, 0.86), 38),
        _look_camera("top_b", (3.15, 4.28, 2.55), (3.02, 4.68, 0.86), 38),
    ]
    # Absolute path: compiler assetdir is google_robot/assets, so a relative
    # ../../person/person.obj would resolve under menagerie/, not third_party/.
    person_file = PERSON_DIR / "person.obj"
    if not person_file.is_file():
        raise FileNotFoundError(f"vendored person mesh missing: {person_file}")
    person_mesh = person_file.resolve().as_posix()
    return f"""
  <visual>
    <global offwidth="1920" offheight="1080"/>
  </visual>
  <asset>
    <mesh name="seat_cone" vertex="{cone}"/>
    <mesh name="person" file="{person_mesh}"/>
  </asset>
  <worldbody>
    <light name="key" pos="3 3 5.5" dir="0 0 -1" diffuse="0.85 0.85 0.85" specular="0.2 0.2 0.2"/>
    <geom name="floor" type="plane" size="0 0 0.1" rgba="0.93 0.94 0.95 1" group="0"/>
    <geom name="belt_a" type="box" pos="3 1.2 0.40" size="1.0 0.25 0.35"
          rgba="0.25 0.45 0.75 1" group="0" friction="0.8 0.01 0.001"/>
    <geom name="belt_b" type="box" pos="3 4.8 0.40" size="1.0 0.25 0.35"
          rgba="0.25 0.45 0.75 1" group="0" friction="0.8 0.01 0.001"/>
    <geom name="wall_s" type="box" pos="3 -0.05 1" size="3.1 0.05 1" rgba="0.8 0.82 0.84 1" group="0"/>
    <geom name="wall_n" type="box" pos="3 6.05 1" size="3.1 0.05 1" rgba="0.8 0.82 0.84 1" group="0"/>
    <geom name="wall_w" type="box" pos="-0.05 3 1" size="0.05 3.1 1" rgba="0.8 0.82 0.84 1" group="0"/>
    <geom name="wall_e" type="box" pos="6.05 3 1" size="0.05 3.1 1" rgba="0.8 0.82 0.84 1" group="0"/>
    <body name="human" pos="0.4 0.4 0" mocap="true">
      <geom name="human_body" type="mesh" mesh="person"
            rgba="0.22 0.34 0.48 1" group="1" contype="0" conaffinity="0" density="0"/>
    </body>
    <body name="follow_cam" pos="3 3 2" mocap="true">
      <camera name="follow" fovy="38"/>
    </body>
    <body name="lidar_cam" pos="3 3 3.2" mocap="true">
      <camera name="lidar" fovy="58"/>
    </body>
    {''.join(cameras)}
    {''.join(slots)}
  </worldbody>
  <contact>
    {''.join(excludes)}
  </contact>
  <equality>
    {''.join(welds)}
  </equality>
  <actuator>
    <velocity name="base_vx" joint="base_x" kv="500" ctrlrange="-0.4 0.4" forcerange="-1500 1500"/>
    <velocity name="base_vy" joint="base_y" kv="500" ctrlrange="-0.4 0.4" forcerange="-1500 1500"/>
    <velocity name="base_wz" joint="base_yaw" kv="80" ctrlrange="-1.2 1.2" forcerange="-120 120"/>
  </actuator>
"""


def _jid(model: mujoco.MjModel, name: str) -> int:
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if index < 0:
        raise RuntimeError(f"missing joint {name}")
    return int(index)


def _aid_for_joint(model: mujoco.MjModel, joint_name: str) -> int:
    joint = _jid(model, joint_name)
    for actuator in range(model.nu):
        if int(model.actuator_trnid[actuator, 0]) == joint:
            return actuator
    raise RuntimeError(f"missing actuator for {joint_name}")


def _body(model: mujoco.MjModel, name: str) -> int:
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if index < 0:
        raise RuntimeError(f"missing body {name}")
    return int(index)


def configure(model: mujoco.MjModel, data: mujoco.MjData) -> ModelIndex:
    model.opt.timestep = DT
    model.vis.global_.offwidth = 1920
    model.vis.global_.offheight = 1080
    # Named cell geoms keep the groups assigned in XML. Robot meshes were already
    # taken out of collision. Finger capsules still need to meet the ring and bottle.
    finger_bodies = {
        _body(model, "link_finger_right"),
        _body(model, "link_finger_left"),
        _body(model, "link_finger_tip_right"),
        _body(model, "link_finger_tip_left"),
    }
    for geom in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ""
        body = int(model.geom_bodyid[geom])
        if name.startswith("ring_"):
            model.geom_contype[geom] = 7
            model.geom_conaffinity[geom] = 7
        elif name.startswith("bottle_"):
            model.geom_contype[geom] = 4
            model.geom_conaffinity[geom] = 4
        elif name.startswith("kit_"):
            model.geom_contype[geom] = KIT_CONTACT
            model.geom_conaffinity[geom] = KIT_CONTACT
        elif name.startswith(("belt_", "wall_", "floor", "base_hull")):
            model.geom_contype[geom] = 1
            model.geom_conaffinity[geom] = 1
        elif name.startswith("human"):
            # Mocap intruder is a ray target. A colliding capsule shoves the docked base.
            model.geom_contype[geom] = 0
            model.geom_conaffinity[geom] = 0
        elif body in finger_bodies and int(model.geom_group[geom]) != 2:
            # Group 3 stays out of the navigation lidar. Contact bits still meet the ring.
            model.geom_group[geom] = 3
            model.geom_contype[geom] = 6
            model.geom_conaffinity[geom] = 6
    slots: list[SlotIndex] = []
    for slot in range(SLOTS_PER_STATION * 2):
        station = 1 if slot < SLOTS_PER_STATION else 2
        ring_joint = _jid(model, f"ring_{slot}_free")
        kit = _body(model, f"kit_{slot}")
        geoms = []
        for geom in range(model.ngeom):
            owner = int(model.geom_bodyid[geom])
            owner_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, owner) or ""
            if owner_name in {f"kit_{slot}", f"bottle_{slot}", f"ring_{slot}"}:
                geoms.append(geom)
        slots.append(SlotIndex(
            slot=slot,
            station=station,
            kit_body=kit,
            bottle_body=_body(model, f"bottle_{slot}"),
            ring_body=_body(model, f"ring_{slot}"),
            ring_q=int(model.jnt_qposadr[ring_joint]),
            ring_v=int(model.jnt_dofadr[ring_joint]),
            mocap=int(model.body_mocapid[kit]),
            equality=int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"grasp_{slot}")),
            geoms=geoms,
            contype=[int(model.geom_contype[g]) for g in geoms],
            conaffinity=[int(model.geom_conaffinity[g]) for g in geoms],
        ))
    arm_collide_geoms: list[int] = []
    for geom in range(model.ngeom):
        body = int(model.geom_bodyid[geom])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
        if body_name in ARM_COLLIDE_BODIES and int(model.geom_group[geom]) == 3:
            # Record the meshes. Contact stays off: full STL vs kit contact
            # blew the Docker job (SIGKILL). The Cartesian z-floor is the
            # plate keep-out; kit bits still reserve ARM_KIT_BIT for a later
            # convex proxy.
            arm_collide_geoms.append(geom)
    # Parked props stay out of the contact graph until the feeder enables a slot.
    for slot_index in slots:
        for geom in slot_index.geoms:
            model.geom_contype[geom] = 0
            model.geom_conaffinity[geom] = 0
    arm_names = [
        "joint_torso", "joint_shoulder", "joint_bicep", "joint_elbow",
        "joint_forearm", "joint_wrist", "joint_gripper",
    ]
    finger_names = ["joint_finger_right", "joint_finger_left"]
    index = ModelIndex(
        site=int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripper")),
        tip_right=_body(model, "link_finger_tip_right"),
        tip_left=_body(model, "link_finger_tip_left"),
        base_body=_body(model, "base_link"),
        base_q=tuple(int(model.jnt_qposadr[_jid(model, name)]) for name in ("base_x", "base_y", "base_yaw")),
        base_v=tuple(int(model.jnt_dofadr[_jid(model, name)]) for name in ("base_x", "base_y", "base_yaw")),
        base_act=tuple(_aid_for_joint(model, name) for name in ("base_x", "base_y", "base_yaw")),
        arm_q=[int(model.jnt_qposadr[_jid(model, name)]) for name in arm_names],
        arm_act=[_aid_for_joint(model, name) for name in arm_names],
        finger_q=[int(model.jnt_qposadr[_jid(model, name)]) for name in finger_names],
        finger_act=[_aid_for_joint(model, name) for name in finger_names],
        slots=slots,
        human_mocap=int(model.body_mocapid[_body(model, "human")]),
        belt_geoms=[
            int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "belt_a")),
            int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "belt_b")),
        ],
        hull_geom=int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "base_hull")),
        lidar_mocap=int(model.body_mocapid[_body(model, "lidar_cam")]),
        arm_collide_geoms=arm_collide_geoms,
    )
    # Stock position gains stall about 3 cm short of a contact pose. Four times
    # the vendor kp lets the wrist finish the seat without exceeding forcerange
    # on the large joints for the errors we actually command.
    for actuator in index.arm_act:
        model.actuator_gainprm[actuator, 0] *= 4.0
        model.actuator_biasprm[actuator, 1] *= 4.0
    return index


def load_cell() -> tuple[mujoco.MjModel, mujoco.MjData, ModelIndex]:
    path = ROBOT_DIR / "cell_scene.xml"
    path.write_text(build_xml(), encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    index = configure(model, data)
    return model, data, index


def set_base_pose(data: mujoco.MjData, index: ModelIndex, x: float, y: float, yaw: float) -> None:
    data.qpos[index.base_q[0]] = x
    data.qpos[index.base_q[1]] = y
    data.qpos[index.base_q[2]] = yaw


def base_pose(data: mujoco.MjData, index: ModelIndex) -> tuple[float, float, float]:
    return (
        float(data.qpos[index.base_q[0]]),
        float(data.qpos[index.base_q[1]]),
        float(data.qpos[index.base_q[2]]),
    )


def base_speed(data: mujoco.MjData, index: ModelIndex) -> float:
    vx = float(data.qvel[index.base_v[0]])
    vy = float(data.qvel[index.base_v[1]])
    return float(np.hypot(vx, vy))
