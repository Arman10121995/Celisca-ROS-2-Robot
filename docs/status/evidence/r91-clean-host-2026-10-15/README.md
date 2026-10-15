# R9.1: clean-host build, tests and mission in a fresh container (2026-10-15)

`scripts/clean_host_reproduction.sh` reproduces one revision from scratch
on the Jetson, in a container built from the official ROS 2 Humble image
([docker/clean_host/Dockerfile](../../../../docker/clean_host/Dockerfile)).
Nothing comes from the reference host's install tree, caches, user site or
SSD wheels.

1. `git archive` of the revision. The 211 git-LFS paths are smudged to
   their content, as a clone with git-lfs would have them.
2. `rosdep install` from the packages' manifests, plus the pinned Python
   packages in
   [requirements.txt](../../../../docker/clean_host/requirements.txt).
3. `colcon build` of the whole workspace. ORB_SLAM3 is skipped: it needs
   Pangolin, which is not packaged, and no integrated algorithm uses it.
4. `scripts/test_tiers.sh fast` inside the container.
5. `scripts/sim_nav_mission.sh mujoco bumperbot nav_obstacle` inside the
   container: a headless MuJoCo Bumperbot Nav2 mission (detour goal,
   cancel and stop, resume, near-obstacle goal), judged against simulator
   truth.

## Result (`1d819a8`, 21:22-21:47)

| Step | Result |
|---|---|
| Image | ros:humble-ros-base-jammy `sha256:860e5379…`, linux/arm64 ([image.txt](image.txt)) |
| Dependencies | rosdep installed 163 ROS packages ([list](rosdep-installed-ros-packages.txt)); pip installed the 15 pinned packages ([pip-freeze.txt](pip-freeze.txt)) |
| Build | **25 packages finished** in 2 min 11 s ([tail](colcon-build-tail.log)) |
| Fast test tier | **1035 passed, 26 skipped** ([tail](fast-tier-tail.log)) |
| Mission | **passed** ([report](mission_mujoco_bumperbot_nav_obstacle.json)): detour goal 0.061 m from truth after resume, near-obstacle goal 0.121 m, stopped 0.14 s after the cancel |
| Wall time | image 19.4 min, tests 1.8 min, everything 24.8 min ([result.json](result.json)) |

One manifest key, `urdf_parser_py` (robot_lab_utils), had no rosdep
definition; its package came in through another dependency. The key is now
`urdfdom_py` (resolved in the rerun below).

## Rerun with a golden comparison (`d99c5ea`, 00:31-00:55, [rerun-d99c5ea/](rerun-d99c5ea/))

The script now also runs the seeded R7.2 perception benchmark
(`scripts/r7_2_perception_benchmark.py`) twice from the same archive: in
the container, and on the host with the runtime's `/usr/bin/python3`. It
then compares every accuracy figure, leaving out timings.

| Step | Result |
|---|---|
| Dependencies | every rosdep key resolved: `urdfdom_py` now replaces `urdf_parser_py` |
| Build | 25 packages finished in 1 min 12 s |
| Fast test tier | 1036 passed, 26 skipped |
| Mission | passed: 0.085 m and 0.095 m from truth, stopped 0.14 s after the cancel |
| Golden comparison | **all 25 accuracy figures identical** between container and host ([golden_comparison.json](rerun-d99c5ea/golden_comparison.json)): laser obstacle detection, ground removal and clustering |

## Earlier attempts, kept as recorded

Each attempt found something the reference host had been hiding.

1. **Revision 6845b8c: stopped before building.** `git archive` keeps
   git-LFS files as pointers, so meshes such as the OAK-D model would have
   reached the container as three lines of text. The script now smudges
   each LFS path.
2. **Revision 54915b1: build and tests passed; the mission never started**
   ([result](attempts/54915b1-result.json)). The mission ran under
   `xvfb-run` as the container's first process, which never saw Xvfb's
   ready signal. It waited 2 h 15 min until it was killed (exit 137). The
   mission now starts Xvfb itself under an init process, within a 30-minute
   bound.
3. **Revision 70cabab: build and tests passed; the mission failed**
   ([result](attempts/70cabab-result.json),
   [report](attempts/70cabab-mission.json)). The controller server could
   not load Regulated Pure Pursuit
   ([log line](attempts/70cabab-controller-server-fatal.txt)). No package
   manifest declared the Nav2 planner and controller plugins the
   configurations load. The reference host had them only because the whole
   navigation2 stack was installed. Declared in `1d819a8`:
   - robot_lab_navigation: nav2_costmap_2d, nav2_navfn_planner,
     nav2_smac_planner, nav2_regulated_pure_pursuit_controller,
     nav2_mppi_controller, dwb_core, dwb_plugins, dwb_critics;
   - robot_lab_bringup: pointcloud_to_laserscan and imu_filter_madgwick,
     which the algorithm dispatch starts;
   - robot_lab_localization: nav2_rviz_plugins, for its RViz
     configurations;
   - robot_lab_utils: `python3-trimesh-pip`, the trimesh key that resolves.

## Limits

- The host is the same Jetson (linux/arm64, the same kernel). The
  container isolates software state, not hardware: another machine type or
  a desktop GPU was not tried.
- One mission in one simulator (MuJoCo). Gazebo needs the GPU for its
  sensors; Isaac and PX4 are installed outside the workspace and were not
  part of this reproduction.
- The heavy optional Python tools (torch, open3d, pymeshlab) are not in
  the image. Only policy provisioning and offline asset tools use them,
  and the fast tier does not need them.
- The images stay local (about 14 GB each, mostly shared layers); nothing
  was pushed to a registry.
