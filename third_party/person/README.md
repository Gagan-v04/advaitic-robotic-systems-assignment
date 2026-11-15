# Cell intruder mesh

Baked stand pose of the Unitree G1 (MuJoCo Menagerie `unitree_g1` at commit `98a70c569c4c1b6da467a20012f766ee1f8dc521`).

- Licence: BSD 3-clause, Unitree Robotics (`LICENSE`)
- Bake script: `scripts/bake_person.py`
- The cell loads `person.obj` only. It does not fetch Menagerie at evaluation time.
- Contact: none. The geom is a lidar/ray target on a mocap body so the person cannot shove the docked base.
- Motion: the mesh is a fixed stand pose. The mission slides it; it has no walk cycle.
