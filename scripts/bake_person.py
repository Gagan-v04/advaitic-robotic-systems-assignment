"""Bake the Unitree G1 stand pose into third_party/person/person.obj.

The checkout this reads is mujoco_menagerie unitree_g1 at
98a70c569c4c1b6da467a20012f766ee1f8dc521, mounted at /g1 when run in Docker.
Evaluation loads only the baked OBJ. The person stays a static mocap mesh:
a live humanoid would fall and shove the docked base.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = Path("/g1/g1.xml")
OUT = ROOT / "third_party" / "person" / "person.obj"
PITCH = 0.007


def _decimate(vertices: np.ndarray, faces: np.ndarray, pitch: float) -> tuple[np.ndarray, np.ndarray]:
    keys = np.round(vertices / pitch).astype(np.int64)
    _, index, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    reduced = vertices[index]
    remapped = inverse[faces]
    keep = (
        (remapped[:, 0] != remapped[:, 1])
        & (remapped[:, 1] != remapped[:, 2])
        & (remapped[:, 0] != remapped[:, 2])
    )
    return reduced, remapped[keep]


def _write(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# Baked Unitree G1 stand pose. See LICENSE and docs/PROVENANCE.md.\n")
        for vertex in vertices:
            handle.write(f"v {vertex[0]:.5f} {vertex[1]:.5f} {vertex[2]:.5f}\n")
        for face in faces:
            handle.write(f"f {int(face[0]) + 1} {int(face[1]) + 1} {int(face[2]) + 1}\n")


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SRC))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if key < 0:
        raise RuntimeError("G1 stand keyframe missing")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)

    chunks_v: list[np.ndarray] = []
    chunks_f: list[np.ndarray] = []
    head: list[np.ndarray] = []
    offset = 0
    for geom in range(model.ngeom):
        if int(model.geom_group[geom]) != 2 or int(model.geom_type[geom]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            continue
        mesh = int(model.geom_dataid[geom])
        if mesh < 0:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ""
        vadr = int(model.mesh_vertadr[mesh])
        vnum = int(model.mesh_vertnum[mesh])
        fadr = int(model.mesh_faceadr[mesh])
        fnum = int(model.mesh_facenum[mesh])
        local = np.array(model.mesh_vert[vadr : vadr + vnum])
        faces = np.array(model.mesh_face[fadr : fadr + fnum])
        rotation = data.geom_xmat[geom].reshape(3, 3)
        world = local @ rotation.T + data.geom_xpos[geom]
        if "head" in name:
            head.append(world.copy())
        chunks_v.append(world)
        chunks_f.append(faces + offset)
        offset += vnum

    vertices = np.vstack(chunks_v)
    faces = np.vstack(chunks_f)
    vertices, faces = _decimate(vertices, faces, PITCH)
    vertices[:, 2] -= float(vertices[:, 2].min())
    feet = vertices[vertices[:, 2] < 0.04]
    anchor = feet if len(feet) else vertices
    vertices[:, 0] -= float(anchor[:, 0].mean())
    vertices[:, 1] -= float(anchor[:, 1].mean())

    OUT.parent.mkdir(parents=True, exist_ok=True)
    for stale in OUT.parent.glob("person_*.obj"):
        stale.unlink()
    _write(OUT, vertices, faces)
    shutil.copyfile("/g1/LICENSE", OUT.parent / "LICENSE")
    span = vertices.max(axis=0) - vertices.min(axis=0)
    head_x = float(np.vstack(head)[:, 0].mean()) if head else float("nan")
    print(
        f"wrote {OUT.name} verts={len(vertices)} faces={len(faces)} "
        f"size={span[0]:.3f}x{span[1]:.3f}x{span[2]:.3f} head_x={head_x:.3f}"
    )


if __name__ == "__main__":
    main()
