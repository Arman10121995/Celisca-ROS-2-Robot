# R9.1: declared-dependency license inventory (2026-10-12)

`scripts/license_inventory.py` resolves the license of every dependency
declared in a workspace `package.xml` (depend, build, exec and test
depends) or in a `setup.py` `install_requires`. Each license is read from
the dependency's own installed metadata, never from a name list:

| Dependency kind | Where the license is read |
|---|---|
| Workspace package | its `package.xml` |
| ROS package | `share/<pkg>/package.xml` under `AMENT_PREFIX_PATH` |
| rosdep system key | the Debian `/usr/share/doc/<pkg>/copyright` |
| Python package | `importlib.metadata` |

It replaces `r9_1_provenance_framework.py`'s substring table, which had
marked 152 of 174 entries unknown and MuJoCo as proprietary (MuJoCo is
Apache-2.0).

## Result (this host, ROS 2 Humble)

[licenses.md](licenses.md) has one row per dependency;
[licenses.json](licenses.json) has the same rows with their evidence path.

| | Count |
|---|---:|
| Declared dependencies | 125 |
| Resolved from installed metadata | 118 |
| ROS 1 keys in vendored Unitree description manifests (catkin, roscpp, roslaunch, rviz, gazebo); not built or run here | 5 |
| **Declared but not installed** | 2: `python3-rtree`, `python3-shapely` |

Workspace packages are Apache-2.0 (19), MIT (7) and BSD-3-Clause
(robot_lab_algorithms). Thirteen manifests said "Apache 2.0" and
robot_lab_msgs said "Apàche 2.0"; all fourteen now use the SPDX
identifier `Apache-2.0`.

## Findings to act on

- **Copyleft dependencies.**
  - `eigen`: MPL-2, with BSD and LGPL parts.
  - `nav2_amcl`: LGPL-2.1-or-later.
  - `slam_toolbox`: LGPL.

  These are used as installed shared libraries and processes. Shipping
  modified copies would bring their source obligations.
- **GPL-3.0 through ORB-SLAM3.** `orbslam3` (`src/ORB_SLAM3`, the
  zang09 ROS 2 wrapper) declares MIT. It links the ORB_SLAM3 library at
  `/workspace/molar/ORB_SLAM3`, which is GPL-3.0 (its `LICENSE`), so the
  built wrapper is distributable only under GPL-3.0.
- **Missing installs (corrected 2026-10-13).** `shapely` (planar_mesh,
  native mobile sensors) and `rtree` were reported as not installed. They
  are installed: `scripts/install_geometry_dependencies.sh` puts the
  pinned wheels (`scripts/requirements-geometry.txt`) under
  `robot_lab_runtime/python_deps/geometry`, and `scripts/ssd_env.sh` adds
  that to `PYTHONPATH`. The inventory above ran without sourcing it. Run
  with the SSD environment sourced
  ([ssd-env-2026-10-13/](ssd-env-2026-10-13/licenses.md)), **120 of 125**
  resolve: shapely 2.1.2 ("BSD License" classifier) and Rtree 1.4.1 (MIT). The 5 left
  are the unused ROS 1 keys.

## Clean-host reproduction

`r9_1_provenance_framework.py` used to write three passing clean-host
tests (container build, container test, bare-metal mission) that had
never been run. It now records them as **not run**. No clean host was
available in this session.
