# TIAGo joint Plan / Execute — November 14, 2026

R5.9 remains **partial**. The normal MuJoCo **Mobile arm** page now offers
**Plan targets**, **Plan Home** and separate **Execute plan** for independent
source position joints. This is joint-target planning, not Cartesian IK,
grasping, payload transport, Nav2 or all-map/backend qualification.

## Actual normal GUI measurements

Robot `asset_pal_tiago`, MuJoCo, `nav_empty`, Jetson / ROS 2 Humble, domain 114.
The normal installed GUI selects the robot and fills its command. Its public
main-launch API starts that command with only the simulator/RViz viewers
disabled. The actual Mobile arm widgets send Plan, Execute and Stop through
the original owned-launch/unique-publisher/source identity checks. Physics
uses the original source servos; the live robot/body/object poses are never
written by the planner or the probe.

`gui-trial-result.json` records a completed normal GUI Plan → Execute of head
pan 0.2 rad, torso 0.02 m, arm joint 4 at 0.1 rad and independent fingers
0.015/0.025 m. All twelve channels are included in the joint plan; the other
seven keep their measured coordinates. Planning itself holds the robot.

`gui-stop-result.json` is the final run with published compiled-scene and
snapshot digests. It completes that route, then plans another head/torso/arm
route and interrupts it with the actual GUI **Stop / hold** button. A new
plant generation is observed, the whole plan is revoked and measured joints
hold over a two-wall-second observation. The owned launch returns zero after
cleanup. Its exact before/plan/execution/Stop states are retained, not a
fabricated PASS flag.

The final run's selected target errors are approximately:

| Source channel | Measured error |
|---|---:|
| Head pan | 0.000000068 rad |
| Torso | 0.0000089 m |
| Arm joint 4 | 0.0000048 rad |
| Left finger | 0.0000069 m |
| Right finger | 0.000184 m |

These are ideal simulator state errors, not calibrated hardware precision.
`measurements.json` keeps the exact values and separates metres/radians for
planning motion and post-Stop drift. There is no mixed-unit physical score.
The stopping check retains its original 3 mm linear / 0.01 rad angular
drift limits. No source force, position or velocity ceiling was increased.

## Implementation and scope

- Planning owns a cloned `MjModel` and `MjData`. Existing bounded
  `connect_controls` supplies direct/RRT-Connect configuration routes.
- Edges inspect independently clipped joint-rate motion, including channels
  that finish before others. Samples are <=25 ms, <=0.005 rad and <=1 mm;
  search is capped at eight seconds / 12,000 collision inspections / 64
  execution waypoints.
- Planned rates stay below the source ceilings and account for servo
  `kd/kp` lag. The same rates are passed to the existing finite-force
  articulation contract. Its original manual rates remain the default.
- A separate Execute requires matching plant/model/URDF/plan identities,
  unchanged measured start/scene and a fresh operator lease. Ready plans
  expire after ten seconds. Each waypoint must settle for 0.2 simulated
  seconds before the next is commanded.
- Live robot contacts and the next servo reference are checked against the
  compiled scene. New/worsening penetration, tracking divergence, changed
  geometry/world joints/mocap/equalities/base/held joints, faults or deadlines
  stop execution. Source resting contacts and existing import exclusions are
  preserved; this is sampled checking, not continuous-collision certification.
- Planning's internal settled hold preserves the currently authorized lease.
  Operator Stop still revokes plant generation and heartbeat. Direct manual
  intervention in a pending/active plan holds the whole route and requires
  observing/retrying against the new generation; an old queued target cannot
  restart it.
- Flex scenes report planning unavailable. PyBullet/Isaac retain direct
  controls without claiming a planner. Cartesian/TCP IK, predictive moving
  scenes, grasp/custody and payload/map adapters remain R5.9 work.

The source collision-detour test uses a separate two-slide MuJoCo fixture.
An independent dense narrow-phase audit checks its path; actual finite-force
fixture servos execute another route. These are useful implementation checks,
**not a TIAGo obstacle mission**. Source/geometry/plant/lease/expiry/Stop,
manual takeover, reduced-rate-only and real Tk widget checks are included.
The focused core batch passes **92 checks**; the final expanded batch passes
**100 checks**, including documentation and actual source-preservation checks.
Utils, MuJoCo and GUI packages build.

## Retained failures and reproducing

Raw logs, all GUI traces and probe scripts are on SSD:
`/workspace/molar/robot_lab_runtime/articulation-planning-20261114/`.
`manifest.json` records the exact source/model/world hashes and raw artifacts.
The two successful earlier runs without the final scene-ID fields are kept
separately; they do not replace the final run's provenance.

The first unit run exposed an implementation error: using operator Stop for
an internal planning hold revoked the lease needed by the plan. A distinct
settled, authorized internal hold fixes this while keeping Stop revocation.
The second unit run caught excessive servo lag with source-maximum ramping;
planned rates were reduced rather than relaxing the tracking guard.

`gui-startup-interlock-result.json` records the initial GUI request arriving
while source joints were still settling. The backend correctly refused it.
The Plan buttons now follow measured readiness, and the successful probe
waits for startup settling. No controller readiness threshold was relaxed.

Use the [operator and agent guide](../../../tutorials/tiago.md#plan-joint-targets-in-mujoco)
and continue the single [roadmap](../../../../ROADMAP.md). Earlier portable
Display screens have their own [source receipt](../tiago-2026-11-14/README.md)
and revision scope; they do not qualify this new planner on other backends.
