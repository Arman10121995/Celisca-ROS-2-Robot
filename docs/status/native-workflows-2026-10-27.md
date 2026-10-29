# October 27 native workflow continuation

The new automatic workflows and tracked family are implemented. Platform and
full extension acceptance remain partial. [Receipts](evidence/native-workflows-2026-10-27/README.md)
retain the exact commands, outcomes and limitations. Do not reinterpret this
delivery as every robot, world, algorithm and backend passing.

## Available in the native GUI

- Experiments now offers **Marine route**, **Marine map acquisition**,
  **Gait navigation**, **Gait map acquisition**, **Aerial swarm formation** and
  **Aerial swarm operator loss**, alongside the actual Nav2/fleet workloads.
  Each uses an isolated launch, real operator ownership and actual feedback.
  Fleet/aerial editor presets are copied into immutable SSD queue inputs.
- **Aerial Swarm → Measured two-drone reference** loads `nav_empty`, two X500s
  and the explicit downward-range sensor configuration. It does not start
  flight. Run, Connect, Check clearance and Takeoff remain separate controls.
  GNSS and barometer height references are also available experimentally.
- **lab_tracked** is one complete Mobile robots / Tracked vehicle family.
  The normal Launch page supports Gazebo Display, Localization, SLAM,
  3D SLAM and Navigation; Drive and the autofilled algorithm selections use
  the native source pipeline. Terrain and higher-mode acceptance are separate.
- Reset in an owned fleet experiment stops the workload and invalidates its
  mission result, then measures every member's reset/localization/settling.
  An occupied reset destination is rejected; world-clock continuity is observed.

Inventory: **148 robot profiles / 79 selectable families**, with the existing
**51 world profiles / 39 selectable world families**. These are catalog counts.

## Measured cases

| Case | Actual observation |
|---|---|
| Two X500s, Harmonic/PX4, `nav_empty`, range-reference formation | Both reached absolute world formation slots within 0.35 m, speed below 0.15 m/s for at least 2 simulator seconds, then landed/disarmed; final errors about 0.107 m and 0.185 m |
| Same aerial reference, operator heartbeat withheld | Both first flew above 0.8 m; actual native landing phases, floor touchdown, disarm and stationary settling were observed |
| `lab_rov`, Fortress, `marine_tank`, automatic acquisition | Actual travel about 0.397 m; saved consistent RTAB-Map database has 5 acquired nodes and an occupied cloud |
| `hexapod_ros2`, MuJoCo, `nav_empty`, automatic acquisition | Actual travel about 0.154 m; archived acquired grid has 26,212 known cells |
| `hexapod_ros2`, MuJoCo, `nav_empty`, automatic Nav2 | Native success terminal plus independently settled physical endpoint; error about 0.148 m and 0.039 rad |
| `lab_tracked`, Fortress/private DART, `nav_empty`, Nav2 | Native success terminal plus physical endpoint about 0.138 m / 0.032 rad from the requested goal; the unchanged 0.15 m / 5 degree contract passed |
| Tracked command owner suspended with SIGSTOP | Physical forward travel followed by zero observed displacement over the final 2 simulator seconds; native watchdog owns stale-command stopping |
| Two-member fleet batch reset | Actual reset/localization and at least 1 simulator second of settling; observed clock did not rewind; interrupted mission result stayed invalidated |

Parent runners recorded owned process cleanup for the automatic mission cases.
Contacts, minimum swept clearance and simulator seed application remain
unassessed where the receipts report null. Qualification is limited to these
named cases; source and run provenance are retained separately.

## Relevant implementation

- `extension_workload.py`, `extension_contract.py`, `extension_result.py`:
  actual native operations, physical endpoints/settling, acquired artifact
  inspection, and producer/workload/trace identity checks. Action acceptance,
  process exit and map file existence alone never establish a mission.
- Gait navigation waits for the actual active Nav2 lifecycle and uses the
  source CAD-to-drive yaw offset for independent body measurements.
- Aerial integer parameters use PX4's MAVLink bytewise union. The optional
  downward lidar measures actual collision geometry with 0.01 m noise and
  enters PX4 through `DISTANCE_SENSOR`; no body position enters that channel.
  Range reference uses native EKF parameters. Swarm truth now carries Gazebo
  timestamps; joint visualization retains its previous time convention.
- Extra X500 sensor geometry needs an exact approved noncolliding kit/mount.
  The conservative collision envelope and takeoff seal checks remain active.
- The complete Apache-2.0 Gazebo tracked example retains its source geometry
  and flippers. A private DART 6.13 / Physics 5.4 plugin enables contact-surface
  motion without replacing system libraries. The source controller's 0.1
  small-turn threshold is corrected, forward track orientation is explicit,
  yaw estimation uses the IMU and the relay QoS reaches the estimator.
  A native command watchdog also covers a stalled ROS command owner.
- The tracked exported footprint is at the floor, with the body at its
  geometric track height. Planner selections are constrained to implemented
  stacks and passed through to the actual navigation launch.

## Remaining work

Finish physical snake, surface-vessel and amphibious families, RMF infrastructure
(doors/lifts/docking/energy), further AUV automatic acquisition and missions,
per-model multi-leg terrain/recovery/backend work, larger aerial groups and
other behaviors/maps/member-loss cases. Broader contact/clearance/reset/resource
campaigns and the older roadmap remain. Optional physical HIL is still separate.

Use [the execution guide](../REMAINING_WORK_GUIDE.md) and the ledger. Preserve
earlier failures and successful scopes; do not rerun catalog restoration or the

## Continuation — 2026-10-09 (R10.3 measurement)

The two-X500 range-reference formation case on `nav_empty` ran a bounded
18 s formation mission (formation_samples: 362) using the native PX4 EKF
range reference (`EKF2_HGT_REF=2`). Measured outcomes:
- formation_rms_m: 0.7820 m (last 10 s), range 0.7820–2.1590 m
- altitude_rms_m: 0.4084 m (last 10 s), range 0.3749–0.4683 m
- air1 convergence: formation_error_m ~0.66 → ~0.15 m
- air2: formation_error_m ~0.95–1.10 m (not converging)
- altitude trim applied: air1 ~-0.05 to -0.035 m/s, air2 ~+0.02 to +0.04 m/s
- Land command accepted but FCUs remain airborne (landed_state: 2 = IN_AIR).

Driver-protocol fix (shorter sleep / explicit heartbeat every iteration) is
the next task before landing metrics and larger groups. Full run: SSD session
`swarm-ja5__fhx`, trial result `r10-20261029-ZwQRVw/trial/trial_result.json`.
completed storage relocation.
