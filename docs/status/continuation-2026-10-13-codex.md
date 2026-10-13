# October 13 Codex continuation

Reviewed clean, published `master` at `4e93bcc`. Claude's existing live walking
robot queues remain running and preserved. Earlier October 9 continuation
features are already committed and subsequently checked; this work continues
the remaining source-profile bicycle-controller gap as `R7.8.bicycle`.

## Existing delivered work reviewed

- Persistent Drone/Drive click latches, Stretch task editing/execution,
  selected-world takeoff clearance and measured Nav2 workload contracts are
  committed. The later continuation receipt records actual build/GUI/source
  checks; those are scoped separately from full missions.
- Current October 12/13 evidence records Celisca missions, car/4WS/mecanum
  screens, walking-robot mapping/navigation, the tf2 repair and real algorithm
  comparisons. See [current status](CURRENT_STATUS.md) for exact named cells,
  failures and limitations; do not extrapolate all-map support.
- RGB-D aerial mapping/save/reload was added after the initial October 9
  batch. Dynamic flight avoidance, mapped-obstacle planning and other aerial
  backends remain open.

## New implementation

Five affected packages built in 12.9 s; the final algorithm rebuild took
4.69 s. Ten numerical checks, nine actual installed Tk command selections
and three real ROS server constructors completed on isolated graphs.
Installed source/launch byte comparisons match. No plant was launched by
these checks. [Receipt](bicycle-implementation-receipt-2026-10-13.json).

Three selectable experimental car controls now serve a separate actual Nav2
FollowPath action: bicycle pure pursuit, Stanley with rear-steer adaptation,
and constrained bicycle steering-rate MPC. They retain source geometry/rate
and full footprint, fresh TF/odom/costmap guards, owned cancellation/preemption,
solver deadlines, stopping sweeps and forward-only DUBIN planning. Existing
car and differential defaults are preserved. Read [the operator guide](../tutorials/bicycle-control.md).

This is implementation delivery, not a physical car mission certificate.
Numerical tests check lateral correction, curvature/slew limits, invalid
states, solver fallback, corner collisions, unknown cells and boundaries.
The initial short MPC preview failed two lateral-correction checks; a longer
prediction interval fixes them without weakening the criteria. Raw negative
and final logs remain on the SSD under
`/workspace/molar/robot_lab_runtime/bicycle-implementation-2026-10-13/`.

## Remaining work

Other source-specific robots/backends and sensor/controller integrations;
Go2 terrain/get-up and other legged cells; full generic arm planning,
dexterous hands, arbitrary-object/grasp and other mobile-manipulator adapters;
dynamic/mapped-obstacle aerial planning and other flight backends; provider,
actor and terrain parity; applied simulator seeds, contact metrics, concurrent
mission qualification, more algorithm/robot/map comparisons and clean-host
reproduction. The separate owners and exact acceptance remain in
[the ledger](platform-status.yaml), [checklist](CHECKLIST.md) and [roadmap](../../ROADMAP.md).
