# Official TIAGo source adapter — November 14, 2026

R5.9 remains **partial**. This implements a complete second source robot's
portable Display control adapter; it is not an autonomous manipulation,
SLAM, Nav2, all-map, all-posture or hardware certificate.

The ordinary GUI selects `asset_pal_tiago`, grouped as **PAL Robotics TIAGo**,
and fills the command. Existing Drive and Mobile arm controls expose the
source base and twelve independent position channels. Five official PAL
Humble repositories are pinned; all five root licenses are Apache-2.0.
`integration.json` records their revisions, resource hashes, original source
mass, Home endpoint, inertial defects and declared lab adapters.

## Actual final control screens

Jetson / ROS 2 Humble; robot `asset_pal_tiago`; world `nav_empty`; domain 114;
one owned physics launch at a time. Each final command comes from the normal
GUI preview, with only simulator/RViz viewers disabled and the selected
portable backend substituted. Final cases share exactly `profile.json`'s
model path; state/model/source hashes and actual values remain in the JSON.

Five original channels are explicitly targeted: head pan 0.2 rad, left/right
fingers 0.015/0.025 m, torso 0.02 m and arm joint 4 at 0.1 rad. All twelve
channels publish measured feedback. After settling, the probe requests and
acknowledges articulation Stop before Drive, matching the GUI page handoff.
It requests 0.12 m/s for four **simulated** seconds, then zeros Drive and
requests Stop; stopped speed is checked after two simulated seconds.

| Backend | Actual forward travel | Final body vx after Stop | Actual RGB frames | Drive-phase real-time factor |
|---|---:|---:|---:|---:|
| MuJoCo | 0.297010 m | 0.000000043 m/s | 55 | 0.973 |
| PyBullet | 0.448637 m | −0.0000937 m/s | 56 | 0.505 |
| Isaac / PhysX | 0.377070 m | 0.0015641 m/s | 46 | 0.400 |

The measured real-time factor is the odometry stamp interval divided by the
wall interval of that drive phase, not a general performance benchmark.
Steady body speed differs between engines (about 0.079 / 0.121 / 0.106 m/s
at the final moving sample); tracking and dynamics remain uncalibrated.
The three final launch processes returned zero after owned cleanup.

Acceptance for these short screens is unchanged: head/arm target errors
below 0.02 rad; fingers/torso below 0.005 m; travel above 0.10 m; final
absolute body vx below 0.025 m/s; actual RGB frames. Neither a process return
code nor the source/GUI checks alone establish these outcomes.

## Source and backend implementation

Original URDF link/joint transforms, axes, limits, inertia declarations,
collision geometry and public-simulation fixed low-friction casters are
retained. Position PD/rates and the panoramic scan/ideal root IMU/reduced-rate
source-head RGB-D kit are declared lab adapters. Fingers stay independent.
Home is the source motion's final arm/torso endpoint plus declared finger
opening, not a collision-checked timed trajectory.

Source inertia sum is **84.66799184112 kg**. MuJoCo compiles
84.668099800 kg after existing tiny-mass bounds and inertia regularization;
PyBullet imports 84.667992841 kg, including its 1 mg floating root frame.
Two antenna tensors are indefinite, two tool-frame tensors are zero.
MuJoCo also excludes three overlapping rest-pose pairs. These are declared
import approximations, not equivalent or calibrated dynamics.

Moving URDF cameras use a surviving rigid carrier rather than a fixed root
offset across movable joints. Isaac's profile opts into actual-pose PhysX
generalized gravity compensation every physics step. Only joint columns are
used after the floating tensor's six root-wrench columns; source-bounded
feedforward reserves from the same actuator effort budget. Invalid native
feedback fails the control path. Other robots retain their defaults.

Isaac's explicit source contact port binds scalar friction on fixed/fused/
instanced source colliders. The wheel-loss port maps original 2 Nm Coulomb
friction and 1 Nm/(rad/s) damping to native axis effort/degree units instead
of the importer's dimensionless joint-bearing coefficient. Motor limits
remain 6 Nm. It retains rotor armature and source velocity bounds.

**125 focused checks pass**, including actual source envelopes, moving camera
FK, source-preservation checks, arm GUI/schema, fixed/floating force layout,
finite/effort budgets, instance-proxy bindings, wheel torque/viscous units,
existing articulation guards and actual Tk documentation rendering. The
four affected packages build: utils, MuJoCo, bringup and Isaac. Checks and
builds are separate from the physical cases above. The installed normal GUI
also verifies family/command/12-channel selection without launching motion.

## Retained initial and failed work

Raw artifacts and reproducers stay on SSD:
`/workspace/molar/robot_lab_runtime/tiago-source-20261114/`.
`manifest.json` binds raw files and final runtime source hashes.

| Preserved case | Finding and correction |
|---|---|
| `control-result`, `pybullet-result` | Earlier head/finger-only screens, before final damping and additional torso/arm targets; not replacements for the final cases |
| `first-checks.log` | Test fixture used caster `_1_link` instead of actual contact `_2_link`; fixture corrected |
| `isaac-startup-interlock-result` | Command arrived before startup settling/heartbeat readiness; keep interlock and wait for measurements |
| `isaac-slide-only-gravity-result` | Lift-only feedforward left arm sag/oscillation; base remained correctly inhibited |
| `isaac-wrong-root-layout-result` | Initial native wrapper use misindexed root forces as joint forces; source-bound fault stopped the trial; explicit raw-tensor mapping and regressions fix it |
| `isaac-caster-default-friction-result` | Correct joint targets but almost no base travel; source caster contact settings had been lost |
| `isaac-proxy-binding-result` | Missing arm/head colliders were instance proxies; bind their editable instance roots with descendant precedence |
| `isaac-no-panel-handoff-result` | 0.0357 m in a wall-timed interval; use the actual GUI Stop handoff and measured simulation duration |
| `isaac-wheel-coefficient-units-result` | 0.0831 m in four simulated seconds failed the original travel gate; native source torque-loss units fix the importer mismatch |
| `mujoco-concurrent-stop-*.log` | Another simulator appeared; only this owned check was interrupted and its launch group cleaned. Probe signal cleanup initially published after ROS shutdown, then was repaired. This interrupted case is not a passing measurement |

No source effort/range, startup gate, base/articulation interlock or travel
criterion was relaxed to pass these screens.

Remaining R5.9 implementation: TIAGo source-specific collision planning,
grasp/custody/object pipeline, payload footprints and mapping/navigation
workflows; general perception and other object sources. Gazebo Fortress
needs its separate source port. Broader mission/fidelity qualification waits
for the roadmap validation queue. [Operator/source guide](../../../tutorials/tiago.md).
