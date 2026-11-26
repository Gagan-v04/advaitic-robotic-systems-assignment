# Provenance

## This repository

The starter (`src/advaitic_systems/`, `spec/customer_cell.json`, `docs/CUSTOMER_PACKET.md`, `docs/ACCEPTANCE.md`, `docs/INTERFACES.md`, and the evidence checker) is Advaitic's assignment package, Apache-2.0. It was not modified to change a score. Candidate code lives in `src/advaitic_cell/`.

## Google Robot

The mobile manipulator is the Google Robot from MuJoCo Menagerie.

- Source: https://github.com/google-deepmind/mujoco_menagerie
- Path: `google_robot/`
- Pinned commit: `98a70c569c4c1b6da467a20012f766ee1f8dc521` (2026-11-02)
- Licence: Apache-2.0, copied at `third_party/mujoco_menagerie/google_robot/LICENSE`
- Recorded in `third_party/mujoco_menagerie/PINNED_COMMIT.txt`

The cell scene is generated next to that model at runtime and is gitignored. Visual meshes keep their Menagerie files. Arm-link collision meshes are rewritten to density 0 and `contype`/`conaffinity` 0. `ARM_KIT_BIT` (bit 8) is reserved on the kit geoms for a later convex proxy; those arm meshes stay out of contact. Finger capsules that are not visual group 2 get `contype` and `conaffinity` 6 so they meet the ring. Base collision STLs stay out of the graph; the cell adds its own hull. A 55 kg base inertia replaces the mesh-density mass that otherwise made the chassis immovable.

## Worker mesh

The human-stop intruder is a baked stand pose of the Unitree G1 from the same Menagerie pin, vendored at `third_party/person/person.obj`.

- Source: https://github.com/google-deepmind/mujoco_menagerie `unitree_g1/`
- Pinned commit: `98a70c569c4c1b6da467a20012f766ee1f8dc521`
- Licence: BSD 3-clause, Unitree Robotics, copied at `third_party/person/LICENSE`
- Bake: `scripts/bake_person.py` (not used at evaluation). Feet on z=0, visual meshes only, 7 mm vertex weld.

The body is a mocap geom named `human_body`, group 1, so the existing person rays still hit it. Contact is off so the mesh cannot shove the docked base. It does not walk: a dynamic humanoid falls, and a gait is not required for the 2–4 s stop. The scripted path slides this stand pose through the aisle.

## Other runtime pieces

- MuJoCo 3.8.1 and NumPy, from PyPI, pinned by `uv.lock` for the physics group.
- Pillow, used only to write `demo.gif`.
- No paid model API and no assets downloaded at evaluation time.
