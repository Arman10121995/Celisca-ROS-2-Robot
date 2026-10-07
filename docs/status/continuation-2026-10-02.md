# Measured robot workflows and remaining gaps — October 2

> Historical source-stage record. Use [current status](CURRENT_STATUS.md),
> [the ledger](platform-status.yaml) and [the checklist](CHECKLIST.md) for the
> published October 7 work and remaining qualification. Measurements and
> original claims below retain their dates; later audited corrections apply.

This continues baseline `2a0aae2` on the Jetson/ROS Humble workspace. Source
hashes and exact commands accompany the trial artifacts. The completion audit
still applies: a catalog entry, launch option or passing source test does not
qualify all robots/maps/backends. Drone SITL flight integration is now measured;
the overall platform remains partial.

## Test the current GUI

Rebuild changed packages and restart Robot Lab GUI to load current installed
modules. Some Python files on this host are copied despite symlink installation.
The **Health** tab opens the current ledger and has **PX4 Flight Guide** and
**Verified Robot Trials** buttons. Health reports completed, partial and active
tasks without converting planned features to successful missions.

| GUI selection | Available behavior | Recorded live result |
|---|---|---|
| `px4_x500` / Gazebo / `nav_empty` / PX4 Flight | Takeoff, Hold, Land, Drive/keyboard/joystick, strafe, 3D goals, measured pose/rotor RViz | Native FCU mission and ROS service/Drive trial pass; command loss lands |
| `four_wheel_steer_car` / MuJoCo / `nav_empty` / Navigation | Opposite-phase, crab, in-phase, pivot; Smac2D/DWB defaults; pattern included in command/manifest | All four patterns reach recorded goals; crab/in-phase/pivot diagonal goals request a 90° heading |
| `four_wheel_steer_car` / PyBullet or Gazebo / `nav_empty` / Navigation / crab | Translation or pivot constrained by DWB critic | Diagonal goal with requested 90° heading succeeds |
| `four_wheel_steer_car` / Isaac / `nav_empty` / Navigation / crab | Lateral command reaches the steering allocator after bridge repair | Goal succeeds in 76.1 s wall time; truth position error 0.210 m, heading error 16.13° |
| `four_wheel_steer_car` / Gazebo / Localization / pivot | Exact tangent steering and wheel rolling, body truth | 0.994 rad/s on 1.0 commanded; bounded 0.014 m translation |
| `mecanum_car` / Gazebo / `nav_empty` / Localization or Navigation | Physical passive rollers, GUI Strafe, Omni AMCL/DWB | 0.278 m/s lateral on 0.3 commanded; recorded forward Nav2 goal passes |
| `mecanum_car` / MuJoCo or PyBullet / `nav_empty` / Localization | Current collision proxy, physical Strafe and stop | Lateral truth 0.253/0.276 m/s on 0.3 commanded; stopped tails ≤0.001 m/s |
| Bumperbot/Labbot, BHL/Go2 and the three car profiles | Existing supported GUI modes and policy toggles retained | Earlier scoped evidence remains valid; see the ledger and its linked artifacts |

The ground wheel profiles expose Localization, 2D SLAM, RGB-D SLAM and
Navigation through implemented sensor/control paths. The new wheel models
now contain an actual OAK-D camera. The complete mission matrix for these
modes is still being qualified. Only compatible modes/planners are selectable;
X500 does not claim ground Nav2 or SLAM. Physical joystick qualification is
pending even though neutral-input GUI tests pass.

## Fixes and evidence

Four-wheel inverse kinematics now computes each contact's rolling direction
and signed speed, including inner/outer wheels, ±90° lateral steering and true
pivot tangents. Wheels wait for steering transitions, and stopping holds the
pod angles. Wheel odometry solves rolling plus no-side-slip constraints.

Unconstrained RPP crab navigation aborted on a diagonal route. DWB now loads
`robot_lab_controller::ParallelSteeringCritic` for parallel steering. It rejects
simultaneous translation/yaw, so its predicted trajectory is physically
admissible; final-heading control pivots separately. The three other car
profiles retain their curvature-aware planning restrictions.

Crab navigation now has successful clear-map screens on all four simulators.
The first Isaac trial moved only forward because its bridge discarded `vy` for
four-wheel steering. The corrected trial succeeds with independent body truth.
Its heading is 73.87° against a requested 90°; Nav2 accepts its localization
estimate, but this does not qualify precise final body heading. Earlier screens record
the requested heading and accumulated turn, rather than exact final yaw.

The original 5,844-face mesh on each of 60 rollers stalled Gazebo physics.
A deterministic 68-face convex contact proxy keeps its directional support
within 0.42 mm; full visual meshes remain. Gazebo body pose is published
independently of wheel odometry. Native Nav2 plugins and stack AMCL were
restored after duplicate YAML dispatch keys replaced them with educational
nodes; standalone educational AMCL callback crashes were fixed and its output
was namespaced to avoid competing with Nav2 AMCL.

See [wheel/navigation evidence](evidence/r56-functional-2026-10-02/README.md)
and [flight evidence](evidence/r54-flight-2026-10-02/README.md). Large raw logs
and `.ulg` stay on `/workspace/molar/robot_lab_runtime`; compact measured
reports and failed cases are preserved in the repository.

## Remaining tasks before broader completion

1. **Four-wheel/mecanum matrix (R5.6):** complete remaining Gazebo, PyBullet and
   Isaac steering-pattern missions beyond the named crab screens; obstacle routes, precise repeated final heading,
   reset, publisher loss, localization and 2D/3D mapping. Qualify MuJoCo/PyBullet
   mecanum Nav2 and obstacle routes beyond the current-proxy movement repeats.
   The named Isaac crab mission does
   not qualify its other patterns, mecanum, maps or mapping modes.
2. **Differential bases/maps (P3/R6.1):** remaining Bumperbot/Labbot furnished-
   floor cells and routes beside furniture; actor maps, cancellation/second goal,
   reset/history and geometry/occupancy alignment. The 20% Celisca correction
   and earlier six short corridor goals are retained. Furnished PGMs still
   require furniture-aware occupancy or measured local-costmap avoidance.
3. **Timing/input (P2/P4):** physical joystick neutral/release/disconnect trial,
   measured heavy-map real-time factor and sensing/collision cost. Preserve
   disabled effort catch-up and measured BHL/Go2 feedback cadence.
4. **Legged/humanoid (R5.2/R5.3/P7):** BHL held-turn/fixed-point stalls and Go2
   terrain/recovery failures; deployable model-specific contracts for other
   policies. Flat-ground bounded walks do not qualify stairs, recovery or Nav2.
5. **Aerial extension:** obstacle-aware 3D planning/SLAM, additional maps,
   long-hover estimator drift, concurrent FCUs and broader failsafes. R5.4's
   named SITL flight acceptance is done; no hardware flight is authorized.
6. **Environments/algorithms/release (R6/R7/R8/R9):** actual sensor-degradation,
   dynamic/multi-floor/terrain missions, matched-input algorithm comparisons,
   real concurrent processes, clean-host reproduction, measured tutorials and
   artifact-backed release matrix. October 1 generated/random metrics and
   metadata reports remain withdrawn as completion evidence. Guard their
   generators so rerunning them cannot overwrite the audit or promote synthetic
   output to completion again.

Legacy R6/R7 verification scripts were also moved from the root into `scripts/`
in the existing working tree. Some still assume their old root location. Their
path repair and demonstration guards remain open; use the exercised
`scripts/test_tiers.sh` and task-linked live probes for current verification.

## Continuing-agent procedure

Read AGENTS, handoff, ledger, audit, storage and the patch guide. Claim a task
before editing. Source `scripts/ssd_env.sh`; keep builds/trials on the SSD.
Build changed packages, verify installed content, run focused/source tests,
then one isolated physics trial at a time (`ROS_DOMAIN_ID < 233` and unique
Gazebo partition). Use the GUI-generated command and save the resolved manifest.

For wheel Nav2 screening, use `scripts/sim_nav_check.sh` with
`CHECK_ARGS='--via-topic --offset-x 1.5 --offset-y 1.5 --goal-yaw-deg 90'` and
the selected `steering_mode:=`. Record both odometry and independent truth,
terminal action state, final heading, clearance and cleanup. Short clear-map
screens do not substitute for obstacle or mapping missions. Preserve aborted
routes and phase timeouts. For drone setup/trials, follow the PX4 Flight Guide.

Promote a task only when its actual roadmap acceptance passes. Update ledger,
roadmap, GUI-visible status and evidence together; do not erase earlier failures.
