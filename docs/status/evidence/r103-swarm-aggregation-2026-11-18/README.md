# R10.3: distributed aggregation gathers three robots (November 18, 2026)

**Status: implemented, experimental.** Ground aggregation, the R10.3 item
that had not passed, gathers three Bumperbots from spread-out starts. Each
member uses only its own scan (the ROS2swarm port in `swarm_behaviours.py`).

## What was wrong: members froze against walls and peers

The trace of the first arena runs showed the members still for the last
80 s.

1. **Against walls.**
   - Aggregation's explore state drives straight ahead, as in ROS2swarm.
   - The fleet controller's scan guard stops a member that would translate
     into its stopping corridor (radius + 0.12 m + a speed term: 0.41 m for
     a Bumperbot at 0.35 m/s), and it stops rotation too.
   - The ROS2swarm protection layer reaches only radius + 0.18 m, so it
     never acted.
   - The member kept asking to drive into the wall and stayed frozen.
2. **Against a peer its scan does not show.**
   - A Labbot's lidar passes over a Bumperbot.
   - The pose-based separation guard held the Labbot, and its behaviour
     kept driving at a robot it could not see.

**Fixes:**

- **Behaviour** (`LocalBehaviour._turn_short_of_corridor`): after the
  protection layer, a forward command with an obstacle inside the guard's
  stopping corridor for that speed turns in place, away from the obstacle.
  `fleet_motion.stopping_distance` is now shared by the guard and the
  behaviours.
  - The first version missed obstacles slightly to the right on the
    Bumperbot: its scan angles plus the π mounting yaw run from 0 to 2π.
    Bearings are now wrapped, and a test covers the turned scan.
- **Controller:** when the separation guard holds a local-swarm member's
  forward command, the member turns in place away from the nearest member,
  still under both guards. The reason reads "swarm member turning from a
  peer its scan does not show".

## New world `swarm_arena`

A 5 × 5 m walled arena (`scripts/generate_swarm_arena.py`; map,
`sim_maps.yaml` and registry entries). ROS2swarm finds robots within 3.5 m
(its burger parameters), and its experiments use small arenas. In the
14 m `nav_empty`, dispersed members never found each other (run 1).

## Measured (Gazebo, Nav2 localization, `swarm_probe.py` from `swarm-distributed-2026-11-14`)

"Together" means all three within 1.5 m of each other.

| Run | World, members | Start | Code | Mean distance (start → end) | Together | Result |
|---|---|---|---|---|---|---|
| 1 | `nav_empty`, 2 Bumperbots + Labbot | after 25 s dispersion | before the fixes | 3.8 → 10.8 m | — | failed: lost each other |
| 2 | arena, mixed | after dispersion | before | 0.7 → 4.1 m | 3 % | failed: froze at walls |
| 3 | arena, mixed | after dispersion | behaviour fix, unwrapped bearings | 0.8 → 2.3 m | 3 % | failed: Bumperbots still froze; Labbot held by a peer it cannot see |
| 4 | arena, mixed | after dispersion | both fixes | 0.7 → 1.0 m | 15 % | partial: no freezing; the Labbot never sees the Bumperbots, so it never joins |
| 5 | arena, 3 Bumperbots | after dispersion (the arena walls pushed them together: 0.94 → 0.44 m) | both fixes | 0.4 → 1.9 m | 56 % | stays together; not a gathering test |
| **6** | arena, 3 Bumperbots | **spread: 3.6–4.0 m apart, beyond sensing range** | both fixes | **3.9 → 1.0 m** | **56 %** (50 % in the second half) | **gathers** |

**Run 6 in detail:**

- All three explore until they sense each other; they are within 2 m at
  19.9 s and within 1.5 m at 21.9 s.
- They then cycle through stay, leave and join, as ROS2swarm's
  probabilistic aggregation does. They end all within 1.1 m, after 180 s.
- The closest pair was 0.39 m (radii 0.24 m), with no contact stop.
- The separation turn fired 14 times.

Each `aggregation-run-*.json` is the probe's summary. Full traces and
launch logs are on SSD under `robot_lab_runtime/swarm-agg-20261118`
(launch-log hashes here).

## Limits

- One gathering run. These runs share the host with Go2 policy training.
- Mixed groups only partly aggregate: a Labbot cannot sense a Bumperbot
  with its lidar, and this behaviour uses nothing else.
- Dispersion in a small arena gathers instead of spreading: its 3.5 m
  potential field is dominated by the walls.
- Air swarms, attraction and flocking remain (R10.3 done-when).
