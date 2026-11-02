# Automatic native floor transport — November 2, 2026

One **Bumperbot / fleet_facility / Gazebo Fortress 6.18** completed autonomous
L0 → L1 → L0 in **319.913 wall seconds**. The Facility request drove the real
Nav2 approach, boarding, native force-driven car/gates, observed body-height
floor classification, member map load/costmap reset and settled exit. The
operator issued destination requests; it did not steer, teleport or write
plant poses/velocities. This is the native transport workflow, separate from
the optional Open-RMF task engine.

| Exit | Actual body x/y/z (m) | Actual yaw (rad) | Measured planar rates | Active map |
|---|---|---|---|---|
| L1 | 2.223619 / −0.003286 / 3.000002 | 0.013864 | 0 / 0 | map_L1.yaml |
| L0 | 2.382130 / 0.020322 / 0.000002 | −0.005018 | 0 / 0 | map.yaml |

The unchanged task gate required independent position error <0.15 m, yaw error
<0.15 rad, speed <0.03 m/s and angular speed <0.05 rad/s for one simulated
second. Nav2 success alone did not finish transport. Nominal exits are L1
[2.2, 0, 0] and L0 [2.3, 0, 0]. Full source/feedback is in
[the result](native-roundtrip-result.json), [frozen driver](native-roundtrip-driver.py)
and [post-run source manifest](native-source-manifest.json). The source snapshot
precedes the later SDK edits; it is not an immutable whole-build certificate.

Reproduction uses a one-member fleet session with Bumperbot `alpha` initially
at [2.5, 0, 0.1, yaw 0], map `fleet_facility`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
ros2 launch robot_lab_bringup shared_world.launch.py \
  session:=/absolute/ssd/session.yaml navigation:=true gui:=false rviz:=false
```

Run the frozen driver on the same isolated ROS domain with `CASE_OUTPUT` set
to a new SSD directory. The normal GUI equivalent is **Fleet & Swarm → Facility
→ Transport selected robot**, once the GUI lease is acknowledged.

The source correction adds fixed landing gates, 65 kg gravity compensation,
3 mm lift leveling, source-matched thin landing/car walls in the upper occupancy
grid, unknown space outside physical support, floor-map costmap acknowledgments
and planar AMCL scan gating during vertical travel. Raw obstacle scans remain
live. Infrastructure owns vertical floor placement; AMCL retains planar pose.
No truth XY/yaw is injected into localization. Door force/speed (300 N / 0.25 m/s)
and lift limits remain unchanged.

Earlier failures remain on SSD: missing upper support/stale obstacles, platform
step, biased upper-floor localization, a strict-JSON unobserved timestamp, and
a first-request operator-lease race. The final successful case still contains
Nav2 TF extrapolation/recovery warnings during scan/map epochs. It establishes
this mission, not warning-free robustness, other robots, concurrent passengers,
additional maps/backends or new Stop/loss physical qualification.

Raw trace and launch log:
`/workspace/molar/robot_lab_runtime/completion-20261102-QUo52j/automatic-transport-thin-wall/`
and `transport-thin-wall-launch.log` in its parent. Frozen source is
`native-auto-passed-source/` there. The subsequent SDK cases follow below.

## Real Open-RMF SDK round-trip

A subsequent **Bumperbot / fleet_facility / Fortress** mission completed native
SDK patrols L0 → L1 → L0 in **359.063 wall seconds**. The official Humble building
parser supplied LiftSessionBegin/Move/DoorOpen/SessionEnd events; the real door
and lift supervisors communicated with physical joint feedback. Per-member
floor observations selected the SDK logical map and actual Nav2 occupancy map.
Inside-car graph vertices were not offered as operator destinations.

| Exit | Actual body x/y/z (m) | Actual yaw / final SDK requested yaw (rad) | Rates |
|---|---|---|---|
| L1 | 2.208814 / −0.000915 / 3.000002 | −3.127266 / −3.131121 | 0 / 0 |
| L0 | 2.399371 / 0.011131 / 0.000002 | 3.126496 / 3.123904 | 0 / 0 |

The observer required the correct observed floor, inactive completed route,
idle member, each planar error component <0.15 m, heading error <0.15 rad and
measured rates <0.03 m/s / <0.05 rad/s for two wall seconds. Actual Euclidean
errors were 0.00886 m and 0.09999 m. Requests were two separate SDK patrols;
the later GUI two-place **RMF floor round-trip** macro is an implemented
convenience, without a separate mission qualification from this case.

[Result](rmf-roundtrip-result.json), [frozen driver](rmf-roundtrip-driver.py),
[installed-source readback manifest](rmf-source-manifest.json).
Raw trace, building YAML, session and launch log are on SSD in
`completion-20261102-QUo52j/rmf-lift-hold/`, the referenced resolved fleet-session
directory and `rmf-lift-hold-launch.log`. Use the native command above with
`rmf:=true` after provisioning the SDK/serializer correction. Run the frozen
driver on the same isolated ROS domain and a new SSD `CASE_OUTPUT`.

The [first SDK negative case](rmf-first-negative-result.json) reached and
exited L1, then failed during return. Once a successful SDK door-open event
stopped repeating requests, the car target followed measured position and
ratcheted physical disturbances. The corrected adapter retains its commanded
height even after the session ends; an active fresh SDK lease retains the
requested floor. Stop/loss instead holds measured targets and invalidates the
enable epoch. Door/landing interlocks and force/speed/level limits were not
weakened. The source regression covers request cessation and open-gate travel.

The successful run still retained transient peer-feedback/TF recovery and
replanning messages. It does not establish warning-free behavior, other robots,
multi-passenger conflicts, calibrated charging, larger fleets or other backends.
Source files added later do not inherit a new whole-build mission certificate.

## Active SDK ownership loss

The [actual process-suspension case](rmf-loss-result.json), run with the
[frozen loss driver](rmf-loss-driver.py), passed in **130.734 wall seconds**.
The driver explicitly enabled control and requested L1, then suspended the
owned SDK process while the physical car was between floors. Native joint,
clock and body feedback remained live. The facility invalidated the old epoch
and held measured targets. After resuming the SDK, control stayed disabled,
fixture hold stayed active and the body was settled; no new enable request
was sent. The declared 10 mm held-position window passed: 77 observations
spanned only 0.000000275 m after settling. Final measured car height was
0.563919 m. This is a safe mid-shaft hold, not automatic recovery to a floor.

Raw trace: `completion-20261102-QUo52j/rmf-loss/`; the same loaded target-hold
source as the successful SDK trip was used. The driver always resumes its owned
process during cleanup. ALL STOP was sent and the owned shared-world launch was
subsequently shut down. Larger fleet/member/contact losses remain separate.

Source checks: 30 focused transport/infrastructure/energy/building-graph regressions
passed. Changed packages built. The installed native GUI rendered actual recorded
floor/lease data and hid car-internal endpoints without creating ROS publishers.
These checks do not extend the mission scope above.

An additional fleet/GUI composition/layout batch on the existing desktop display
reported 37 passed and one narrow-window visibility failure. That exact layout
case passed unchanged under its documented isolated Xvfb environment (1 passed,
5.74 seconds); both logs remain in the SSD run directory. Registry cross-reference
validation passed. Required CI is green on the earlier cache/dependency repair
`d064eece`; the new fleet revision `d6638c11` started its own separate workflow.
