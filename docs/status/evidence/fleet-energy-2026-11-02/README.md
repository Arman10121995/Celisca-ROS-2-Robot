# Fleet actuator energy, localization frames and native RMF continuation

Recorded November 2, 2026, on master from `375143ff` plus this delivery's working
changes, Jetson AGX Orin / ROS Humble / Gazebo Fortress 6.18. Project and R10.2
remain partial. These are named simulation cases, not universal qualification.

## Actual cases

| Case | Actual observation | Boundary |
|---|---|---|
| Source effort readback | Bumperbot: 729/729 complete joint samples, maximum effort 0.032978 Nm, positive mechanical power 0.060844 W, body excursion 0.232067 m. Labbot: 728/728 samples, 0.110488 Nm, 0.168317 W, 0.232000 m | One forward/reverse/Stop screen per source in fleet_facility; no motor efficiency/chemistry calibration |
| Energy and berth | Bumperbot settled at the authored dock; modeled fraction rose from 0.798518667 to 0.799562167. Departure removed charging, load drained energy, actual joint-state broadcaster deactivation made SOC unknown; restarting the broadcaster restored power samples but retained unknown SOC | 11.69 wall seconds; pose-berth charging is an authored 120 W model, not a measured charger/connector. Labbot charging was not exercised |
| RMF door/dock | Actual Open-RMF patrol crossed the physical main door, then a second patrol invoked the native dock callback and reached independently settled berthing near [-4.5220, -2.9655], yaw -0.00990, linear/angular speed zero | 155.24 wall seconds; one Bumperbot, two named tasks. No multi-robot negotiation/loss or multi-floor RMF mission claim |

A repeat with the new floor-frame tree physically rode L0-L1-L0 in 70.13 wall seconds; actual returned body [3.662945, -0.000002, 0.000001] m, speed zero, and both observed landing maps loaded. See `floor-transition-result.json`; its driver is the unchanged earlier floor receipt.

Results are in `*-result.json`; executable drivers and session requests are
included here. Full native source/observations/trace logs are on SSD at
`/workspace/molar/robot_lab_runtime/completion-20261102-QUo52j/`. The source-power
result records the resolved session directory and actual frame parents:
`map -> alpha/map -> alpha/odom` and the corresponding beta chain. AMCL owns
planar map-to-odom; infrastructure supplies only vertical floor placement.

Energy source certificates in `infrastructure.yaml` bind generated source
URDF SHA-256 and the actual readback library SHA-256. They do not qualify another
robot or changed runtime. The source of JointState effort was cross-checked
against the official Humble gz_ros2_control read method, which projects joint
transmitted torque onto the source axis. Observed power uses actual wheel
rates/efforts, explicit efficiency/hotel load and simulated SI time. Missing
coverage remains unknown; no regeneration or default RMF SOC is substituted.

## Native upstream repair and retained failures

The first actual RMF startup crashed in `rmf_traffic_ros2::convert`, confirmed by
GDB and Python faulthandler. Humble's upstream graph serializer dereferences
`entry_event` when an exit event exists. The extracted Apache-2.0 source in
`src/robot_lab_utils/vendor/rmf_graph` uses `exit_event`, and fixes the adjacent
exit-orientation lookup. Its source pin/hash/license are preserved. Only the
Robot Lab RMF process preloads the correction; system/SDK libraries stay intact.
The corrected native mission above ran through the real schedule and official
door supervisor. An exited RMF owner now shuts down its owned launch.

The coarse RMF lattice initially missed the narrow door portal. Source-declared
approaches now add clearance-tested connecting segments without changing body
clearance. A test's initial assumption that a coarse vertex already lay inside
the portal was corrected with an explicit inside-portal fixture. Failed graph
checks, native crash logs/backtrace, and initial missing-header build diagnostics
are retained in the SSD run. No failure was relabeled as a passing mission.

## Reproduction and checks

Source `scripts/ssd_env.sh`, ROS, workspace and `scripts/rmf_env.sh`.
Build the changed packages and run `scripts/build_rmf_graph_fix.sh` for an
existing RMF overlay. Run `shared_world.launch.py` with `power-session.yaml`,
`navigation:=true gui:=false rviz:=false` in an isolated ROS domain/partition.
Set `CASE_OUTPUT` to an SSD directory and execute `power_probe.py` or
`energy_trial.py` using `/usr/bin/python3 -s` in the same ROS domain.
For RMF use `rmf-session.yaml`, add `rmf:=true`, and run `rmf_trial.py`.
Each driver is bounded and sends Stop/disable; terminate and check the owned
launch descendants afterwards. Do not run these cases on the same ROS domain
concurrently. Certificate changes require a new source-bound readback case.

The post-run source hashes and driver identities are in `manifest.json`.
This is not an immutable full-build manifest. The late removal of obsolete
unused battery bookkeeping does not change the energy algorithm exercised.
Focused tests/builds, installed GUI status and the floor-transform continuation
are recorded with the delivery; their results do not establish broader missions.
ROS imports and actual missions worked with the system Python/SSD environment;
the previous blanket GLIBCXX blocker did not reproduce here. No system library
replacement was required. The separate aerial owner and PX4 issue are unchanged.

Remaining: automatic RMF lift-session graph missions, multi-robot conflicts/loss,
other robot/floor/map/backend transport, actual connector/contact sensing,
calibrated energy and RMF energy scheduling, broader extension and old roadmap.

The native RMF trial wrote its default schedule journal in the repository. That generated change was archived to SSD and the pre-run tracked file restored. Future schedule nodes run in their resolved SSD session directory; this directory-only change does not refresh the recorded mission.

Final local checks: five changed packages built; 43 focused fleet/energy/graph checks pass, installed Tk status/Facility rendering is neutral and current, and registry cross-references pass. New pure energy/graph checks join the fast CI tier; the ROS infrastructure checks join integration.

The broad fast attempt was interrupted after 214.12 seconds: 311 passed, one skipped, four deselected, three failures; no full-tier PASS is claimed. Those failures identified absent Run sections in six newer tutorials and two terrain registry image fields pointing to YAML instead of PGM. Useful existing Run instructions now have explicit sections, both registry fields name their actual image, and all eleven checks in the affected documentation/provenance classes pass. The failures and focused correction log are retained. No test threshold was weakened.
