# R6.1 Nav2 goal clock and RViz path, 2026-09-30

Host: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress through `ign gazebo`.
Source revision before this change: `a6ded59`. These runs use the uncommitted
working tree with the goal relay, dedicated RViz goal topic and 5% Celisca
mesh/map enlargement. Each run used an isolated ROS domain and partition.

Reproduction from a built/source workspace:

```bash
TIMEOUT=150 CHECK_ARGS='--via-topic --distance 1.0' ROS_DOMAIN_ID=97 \
  bash scripts/sim_nav_check.sh gazebo bumperbot celisca_floor_1 /tmp/bumperbot.json
TIMEOUT=150 CHECK_ARGS='--via-topic --distance 1.0' ROS_DOMAIN_ID=98 \
  bash scripts/sim_nav_check.sh gazebo labbot celisca_floor_1 /tmp/labbot.json
```

The probe publishes a map-frame `PoseStamped` on `/robot_lab/goal_pose`, the
topic selected by the RViz `2D Goal Pose` tool. The relay replaces the sender's
timestamp with its simulation clock and forwards one `NavigateToPose` action.
It chooses a free cell 1 m ahead from the published map and checks the action
terminal status and final localized distance.

| Robot | Action result | Wall time | Final estimated goal error |
| --- | --- | ---: | ---: |
| Bumperbot | succeeded | 3.7 s | 0.257 m |
| Labbot | succeeded | 3.5 s | 0.239 m |

The same relay was then used for a 1 m straight goal in `nav_empty`, with
`SmacPlannerHybrid` (Reeds-Shepp) and regulated pure pursuit selected for each
car. Their separate navigation overlays retain the front/rear footprint
positions; the common curvature-aware plugin family reflects their shared
wheelbase and steering limit.

| Car | Action result | Wall time | Final estimated goal error |
| --- | --- | ---: | ---: |
| `ackermann_car` | succeeded | 3.6 s | 0.259 m |
| `rear_steer_car` | succeeded | 5.1 s | 0.262 m |
| `anti_ackermann_car` | succeeded | 6.4 s | 0.270 m |

The first attempted topic was Nav2's standard `/goal_pose`; it caused a direct
goal and the relay goal to be active at once. The negative run records a
preemption, and its checker incorrectly treated an executing status as a final
result. The dedicated topic plus corrected terminal-status check produced the
two successes above. No clock extrapolation was observed in either success.

Scope: one short straight goal per robot in Gazebo. It does not qualify
turning/slalom/reverse missions for the cars, longer routes, furnished maps,
the other three simulators, physical robots, or PX4/legged navigation modes.
