# Robot Lab test tiers and evidence

Updated October 9, 2026. The runner and manifests live in
`scripts/test_tiers.sh`; `scripts/test_fast.sh` adds source compilation and
registry checks. [Current status](status/CURRENT_STATUS.md) records measured
robot work separately from software checks.

## Implementation-first handoff

The current user priority defers test campaigns and physical validation until
initial implementation is in place. Nine changed packages built for the
[October 9 candidates](status/implementation-2026-10-09.md); no new mission,
GUI suite or CI pass is claimed. Validate policy contracts, native Servo/
sequence/object interactions, Stretch sensor/map/Nav2 composition, actual
static drone route tracking, coplanar collision/speed fidelity and two real
concurrent plants after initial implementation. Keep earlier source-specific
results separate and preserve all existing tiers/acceptance requirements.

## Tiers

| Tier | Scope | Environment and limits |
|---|---|---|
| `fast` | Numerical, schema, configuration and source contracts | Plain Python; no shared ROS graph. ROS-import checks belong in integration. |
| `integration` | Xacro, launch, ROS imports and adapter contracts | Source ROS and the installed workspace. Missing optional dependencies/assets have explicit skips. |
| `physics` | Headless real-engine stepping and backend contracts | Optional engines must be installed; missing engines skip explicitly. This is separate from a robot mission. |
| GUI | Actual Tk widgets, command/autofill, selection and layout | Requires a real display or Xvfb; native preview additionally needs OpenGL/GLX. |
| Live mission | Named simulator/robot/world/controller outcome | Isolated ROS/transport, actual truth/sensors, artifacts and owned cleanup. Follow each task's producer. |
| Hardware | Physical robot/device work | Full HIL qualification remains open; software skips are not hardware evidence. |

Three unchanged ROS-import contracts moved from fast into the sourced
integration manifest after the recorded `dc39922` CI failure. The numerical
assertions and required CI failure conditions remain. The current workflow
builds the core workspace, excluding optional `orbslam3`, then runs all three
required tiers and registry validation.

## Run

From the workspace root:

```bash
source scripts/ssd_env.sh
bash scripts/test_fast.sh
bash scripts/test_tiers.sh --list
bash scripts/test_tiers.sh fast
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/test_tiers.sh physics
bash scripts/test_tiers.sh integration
```

For actual GUI checks on this Jetson:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 xvfb-run -a python3 -m pytest -q src/robot_lab_gui/test
```

Use the intended sourced/package environment and keep output under
`$ROBOT_LAB_RUNTIME_ROOT`. Build changed packages and compare installed files
with source before mission-producing trials. Run only checks appropriate to
the change; retain failures and declared skips.

## Latest recorded results

| Stage | Result | Evidence |
|---|---|---|
| October 8 current fast | 1005 passed, one skip/four integration deselections | October 8 checkpoint publication receipt; software only |
| October 8 current GUI | 104 passed, two PyOpenGL deprecation warnings | Drone mirrored controls, target editing and complete existing GUI suite |
| October 8 core build / physics / integration | 25 packages; 27 physics passed; 147 integration passed under dedicated Xvfb | Desktop integration initially failed compact layout; isolated and full Xvfb repeats pass, original failure retained |
| October 7 local GUI | 94 passed, two PyOpenGL deprecation warnings | [GUI evidence](status/evidence/gui-controls-column-2026-10-07/README.md) |
| Final focused real-Tk layout | Five passed | Same evidence; compact setup access, wide restoration, controls/limits and neutral inputs |
| Preceding local control-column stage | 936 fast passed, one skip/four deselections; 146 integration passed | Exact source stages and logs in the GUI manifest |
| Published `091d388` CI | Core build: 25 packages; fast: 933 passed/four skips/four deselections; physics: 20 passed; integration: 134 passed/12 skips; registry validation passed | [Exact CI report/log hashes](status/evidence/ci-extensions-2026-10-07/README.md) |

Counts differ by source stage and host availability. Keep local display checks,
CI skips and real simulator missions separate. A green build or generic physics
case does not close the full robot/map/mode/backend matrix. Historical test
counts remain in their dated reports. Use [Workflow](WORKFLOW.md),
[the checklist](status/CHECKLIST.md) and [the ledger](status/platform-status.yaml)
for qualification and remaining acceptance.
