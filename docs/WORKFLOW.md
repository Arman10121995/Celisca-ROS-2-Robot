# Robot Lab operational workflow

This is the current source/evidence workflow for the repository. It separates
static checks, isolated runtime probes, scenario trials and promotion claims.
The machine-readable [status ledger](status/platform-status.yaml) is authoritative
for task state and ownership; [ROADMAP.md](../ROADMAP.md) is authoritative for
acceptance criteria.

## 1. Establish the revision and scope

Record the source revision and whether the tree is dirty before running a
claim-producing check:

```bash
cd /home/molar1/bumperbot_ws
git rev-parse --short HEAD
git status --short --branch
```

A clean revision is preferred for recorded evidence. If the tree is dirty,
save the diff or identify the exact working-tree patch; do not label the result
as a clean-checkout result. Record the host, ROS distribution, Python
interpreter, simulator/engine versions, package build options and relevant
hardware. Never infer portability from one arm64/Jetson host.

## 2. Run the fast development gate

The fast tier is hermetic: it does not initialize a shared ROS graph or spawn a
simulator. Run it before live work:

```bash
scripts/test_fast.sh
```

At revision `e7a8cda` the recorded result is **465 passed, 1 skipped**, with
registry cross-reference validation passing. The result is a source/static
gate, not a clean build, GUI mission or simulator qualification. For a
ROS-dependent check, source the intended environment first:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash -c 'source /opt/ros/humble/setup.bash && scripts/test_tiers.sh integration'
```

Optional physics tests must report an explicit skip when an engine is absent;
a skipped engine is not a passing engine result.

## 3. Validate composition before starting ROS

Use the source registry for a quick inventory check:

```bash
export PYTHONPATH="$PWD/src/robot_lab/robot_lab_registry${PYTHONPATH:+:$PYTHONPATH}"
python3 -m robot_lab_registry.cli validate \
  -c src/robot_lab/robot_lab_registry/config --cross-references
```

A resolved manifest is the unit of execution. The CLI and GUI use the same
typed resolver/validator and must agree on aliases, algorithm slots, planner
plugins, world paths, reset policy and seed. The `launch` subcommand is safe
dry-run by default; omit `--execute` when inspecting the command. It must not
start a process in that mode:

```bash
python3 -m robot_lab_registry.cli launch \
  -c src/robot_lab/robot_lab_registry/config \
  --composition '{"robot_id":"bumperbot","environment_id":"nav_empty","simulator":"mujoco","algorithm_ids":{"localization":"amcl"}}'
```

Use the exact IDs and capabilities in the current catalogs. A successful
cross-reference check is not proof that a robot has a class-appropriate
controller, sensor or task.

## 4. Isolate a live simulator run

Use an unused `ROS_DOMAIN_ID`, non-conflicting simulator transport and an
artifact directory outside the source tree. Never kill all ROS or simulator
processes to clean up another run. Prefer the repository's task-specific probe
and capture both stdout/stderr and machine-readable output.

A minimal built-workspace launch is:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=loc simulator:=mujoco robot_model:=bumperbot \
  map_name:=nav_empty gui:=false start_rviz:=false
```

Before sending a command, verify readiness and the expected clock, truth,
joint-state, sensor, TF and command topics. Use a stop sequence and verify that
owned processes exit. A process that starts, an offline fallback, or a topic
that exists does not establish readiness.

## 5. Measure a class-specific task

For every trial record:

- robot/model, controller or policy/checkpoint, backend, world, task and seed;
- initialization and reset result, command windows and stop/watchdog behavior;
- simulator time and wall time, readiness/launch outcome and process deaths;
- independent truth, estimator inputs, frame/timestamp convention and TF owner;
- task metrics with units, contact/fall/tilt/effort limits and missing metrics;
- raw logs, JSON/rosbag paths, manifest and checksums.

A task is successful only when its declared outcome condition is met. A wait,
a node staying alive, zero reported collisions without a contact source, or a
single successful seed is insufficient for qualification.

## 6. Use negative evidence deliberately

Failed trials are part of the result. Preserve the exact command, revision,
configuration, failed metric and likely hypothesis. Separate these cases:

- **Implementation failure:** command, controller, topic, asset or process error.
- **Physics/contact mismatch:** compare masses, geometry, friction, contacts,
  timestep, actuator dynamics and reset with a bounded trace.
- **Policy fixed point:** command remains live while action/effort converges to
  a constant target and motion decays; do not keep tuning unrelated rates
  without a new measured hypothesis.
- **Task/terrain failure:** preserve the first failed contact or ledge and do
  not promote the result to a navigation or terrain qualification.

For example, R5.3 fixed duplicate MuJoCo ground contacts, but the common BHL
policy still stalled; that fix is a plant-parity correction, not walking
qualification.

## 7. Promote only exact cells

Promotion requires a manifest, pinned source revision, named host/backend,
commands, seed protocol, readiness/contract checks, raw artifacts, outcome and
measured metrics. Update `ROADMAP.md`, `docs/status/platform-status.yaml`, the
support matrix and the relevant evidence README together. Keep failed and
partial cells explicit. Hardware HIL remains a separate, explicitly authorized
milestone; this workflow does not authorize physical motion.
