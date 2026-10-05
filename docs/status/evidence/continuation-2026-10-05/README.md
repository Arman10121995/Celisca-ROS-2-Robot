# October 5 measured continuation evidence

Source baseline: `700b94e`; working-tree stages are distinguished below.
Host: Jetson AGX Orin, ROS Humble. Wheel backends: Gazebo Fortress 6.18,
MuJoCo, PyBullet, native Isaac Sim 6.0.1 / CPU PhysX. Flight: Gazebo Harmonic
8.15, native PX4 v1.16.2, revision
`54f0455ffcd755534539a7cf33a09a20bf71d29d`.

Compact JSON here is copied unchanged from bounded live probes. Reports are
indexed with SHA-256 in [runtime-evidence-index.yaml](../../runtime-evidence-index.yaml).
Raw launches, stamped body traces, RTAB-Map databases, SLAM grids and generated
PGM/YAML remain at:

```text
/workspace/molar/robot_lab_runtime/r56-qualification-2026-10-05/
```

Physics runs were serial, on separate ROS domains and simulator partitions.
Cleanup targeted the owned launch/process groups. Test/build logs are in the
same SSD directory. No catalog counts, generated trajectories, assumed resets
or random benchmark scores are used as robot mission evidence here.

## Producer stages

- `px4-altitude-source-hashes.json`: original GUI altitude screen in `nav_empty`;
  four relevant source files captured before that trial. The original report
  predates body-message recording and the final FCU-state field.
- `runtime-source-hashes.json`: continuation control/reset/mapping snapshot.
  Clear-map continuous-critic and earlier workflow results precede this snapshot;
  they are historical working-tree screens, not a fresh execution of every
  final file. The shared control modules were subsequently changed for obstacles.
- `final-control-source-hashes.json`: footprint/terminal-approach obstacle stage,
  before the Gazebo/PyBullet/Isaac 0.4 m inflation batch.
- `worlds-flight-obstacle-source-hashes.json`: captured before the mistakenly
  attributed Isaac 0.65 m inflation trial. The launch used an undeclared
  argument, so its 11,410 body samples describe the default Ackermann mode,
  not crab. It is excluded from crab support.
- `px4-nav-obstacle-source-hashes.json`: captured before the additional native
  flight, including the actual PX4 adapter modules and body-recording probe.
- `gui-save-source-hashes.json`: first RGB-D workflow/save attempt, also with
  the incorrect mode argument; exporter permissions failed. Retain as negative
  evidence, not a passing crab workflow.
- `mode-verified-source-hashes.json`: captured before the true Isaac crab
  repeat with 0.65 m inflation. The query proves the steering mode; Nav2 action
  succeeds but independent body heading −5.63° fails the 5° limit.
- `corrected-save-source-hashes.json`: captured before the corrected MuJoCo
  crab RGB-D workflow and actual GUI save. Mode, post-reset tracking, intact
  nine-keyframe database and 7,255 finite PCD points pass.
- `final-heading-source-hashes.json`: final controller arrival-window repeat;
  controller yaw tolerance is 0.015 rad, independent body acceptance stays 5°.

Do not attribute an earlier stage's numerical result to every later source
change. The source manifests, report hashes and containing commit complement
the baseline revision; third-party/model pins are recorded separately.

## Reproduce a strict obstacle screen

From the repository root, with no other physics qualification running:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
trial="$ROBOT_LAB_RUNTIME_ROOT/repeat-4ws"
mkdir -p "$trial"
export ROS_DOMAIN_ID=194 TIMEOUT=400
export CHECK_ARGS="--offset-x 4 --offset-y 3 --goal-yaw-deg 90 \
  --max-position-error .15 --max-yaw-error-deg 5 \
  --trace-out $trial/body.trace.json \
  --world-file src/robot_lab_maps/maps/nav_obstacle/worlds/nav_obstacle.world \
  --robot-radius .34 --min-route-clearance .02 --require-detour"
timeout 520 bash scripts/sim_nav_check.sh isaac four_wheel_steer_car \
  nav_obstacle "$trial/result.json" steering_mode:=crab \
  spawn_x:=-7 spawn_y:=-7
```

Use a fresh output directory/domain for each repeat; record current hashes
before execution. Position and heading come from `/odom/ground_truth`, not
the Nav2 estimate. Swept clearance covers static boxes and a conservative
circular robot envelope, not measured contact events or furnished mesh worlds.

For Drive/mapping/reset, use `scripts/sim_workflow_check.sh` with `loc`, `slam`
or `3d_slam`; `CHECK_ARGS=--lateral` enables both strafe directions. For flight,
launch native `px4_x500` Flight with the selected map, then run
`xvfb-run -a python3 scripts/px4_ros_check.py --gui-altitude --map-name MAP --out OUT`.
Keep that launch alive until the probe finishes, then stop its owned group.
The flight probe records actual Tk button actions, FCU state, independent
body phases and a separate `.truth.json` trace.

For the GUI RGB-D save check, run an active RTAB-Map launch with database
`$ROBOT_LAB_RTABMAP_DIR/nav_empty_four_wheel_steer_car.db`, map `nav_empty`,
then use `scripts/gui_map_save_check.py` through Xvfb with a fresh `--target`
and `--out`. Only the destination picker is automated; the real GUI button,
ROS backup, SQLite integrity/node counts and finite PCD data are exercised.

## Negative evidence retained

`mujoco-crab-workflow-before.json` records the backwards-clock/stale-reset
failure. `pybullet-mecanum-2d-spacing.json` and
`pybullet-mecanum-2d-clock-fixed.json` record accepted map observations but
stale estimator/mapping history after reset. The final 2D SLAM report records
actual upstream graph restart, fresh odometry and successful resumed motion.

`mujoco-crab-obstacle-corrected.json` records an aborted corner-cutting route
and negative physical clearance. `isaac-crab-obstacle-final.json` records the
aborted 0.4 m inflation run, despite a clear measured circular envelope.
`isaac-crab-obstacle-inflated.json` is a successful default-mode repeat with
the wrong manual launch argument; it is not crab evidence.
`isaac-crab-mode-verified.json` is the true crab repeat and fails its independent
heading limit despite Nav2 success. `mujoco-3d-current-workflow.json` and
`mujoco-3d-gui-save.json` retain the first incorrect-mode/permissions failure;
their corrected counterparts measure the actual passing crab/save workflow.
Other exploratory precision/soft-gate failures and the early mesh-segment
projection failures remain in the raw SSD directory. These failed runs are
not erased or converted into passing support cells.

## Final verification and support subset

- `release-fast-final.log`: `scripts/test_fast.sh`, 756 passed / two skipped,
  registry cross-reference PASS on `e69d1b2` plus recorded updates.
- `direct-fast-final.log`: fixed direct runner without inherited source/package
  paths, 755 passed / two skipped / one integration case deselected. The real
  mecanum xacro check remains in the integration tier and passes separately
  (`mecanum-integration-final.log`). The old GitHub runner omitted the utility
  source path; `test_fast.sh` previously masked that through inherited paths.
- `release-focused-final.log`: 35 world-reader/heightfield/PCD/support-guard
  tests pass. `gui-43-final.log`: 43 Tk command/Drive tests pass on their named
  source stage; full-tab live Save Map and flight trials are separate.
- The final Isaac mode-verified repeat passes the unchanged body limits:
  0.074 m / −4.21°, 0.255 m swept clearance, 14,487 body samples.
- [Current support subset](support-matrix-current.md) indexes seven individual
  pre-trial-hashed reports, with five latest exact support cells. Older
  post-trial snapshots and wrong-argument trials are retained outside this
  index. Absence from the subset does not erase older named evidence.

Full-release gates remain blocked by unfinished task acceptance, including
the new extensions. `--require-release-ready` must return a nonzero status
until those prerequisites actually pass.

`gui-worlds-check.json` exercises the installed full GUI's actual Generate and
Stop buttons. It records the owned ROS CLI/generator/Gazebo/service process
tree, cancellation before export, and a subsequent real map with 23,170 free
/ 1,608 occupied cells. The original `/proc/*/children` observer missed the
descendants; its failed reports remain beside the corrected `ps` observer.
`gui_worlds_check.py` archives that producer; copy it to a fresh SSD run
directory before reproducing rather than writing generated maps into docs.
