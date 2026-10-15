# P7 / R5.2: walking robots navigate with leg odometry (2026-10-13)

Status: **implemented, experimental**. Quadrupeds and the G1 humanoid walk
Nav2 missions on their rl_sar policies, localized from leg kinematic
odometry: **every MuJoCo and Isaac cell passes** (Go2, A1, B2, G1). The
Go2 in Gazebo now stands and walks but does not pass the mission. One
seeded mission per cell.

## What was built

- **Leg kinematic odometry** (`robot_lab_algorithms.leg_odometry`).
  - Base velocity from the stance feet: `v = -(J q̇ + ω × p)`, using
    forward kinematics from the URDF and the IMU's angular rate.
  - A foot is in stance when it is within 3 cm of the lowest foot **and**
    its implied base velocity is near the previous estimate
    (0.15 m/s + 0.3 |v|). Without the velocity gate, swing feet near the
    ground counted as stance and the estimate read 20-40% slow.
  - 20 ms low-pass; published on `/leg_odometry`, which feeds the EKF as
    `odom0`.
- **Walking in slam/nav.** In MuJoCo, Isaac and Gazebo, slam and nav modes
  select the robot's provisioned policy automatically; the capability
  check grants `velocity_base` to a policy robot. The GUI fills in the
  policy for these modes.
- **Navigation overlays** (`src/robot_lab_navigation/config/robots/`) for
  Go2, A1, B2 and G1:
  - polygon footprints for the dogs (a 0.38 m circle around the Go2 left
    no room in nav_obstacle's corridors) and a 0.3 m radius for the G1;
  - Regulated Pure Pursuit at 0.35-0.5 m/s, never slower than the gait
    actually walks: minimum approach speed 0.2 m/s for Go2 and A1, 0.3
    for the G1, 0.4 for B2;
  - `max_angular_accel: 20`: RPP ramps an in-place turn up from the
    measured yaw rate, and gaits that ignore 0.1 rad/s never ramped up;
  - Go2 does not rotate to the goal heading (himloco does not spin in
    place in MuJoCo or Gazebo), so its yaw tolerance is open.
- **Gazebo start.** The Go2 fell limp from its spawn before its PD
  controller was active and landed on its side. The world now starts
  paused and `gazebo_unpause_when_ready.py` unpauses it once the
  controller is configured (see round 3 for the first version's
  deadlock).

## Mission

`scripts/sim_nav_mission.sh <sim> <robot> nav_obstacle` in nav mode:

1. A detour goal behind the obstacle row.
2. A cancel after 1 m; the robot must stop.
3. A resume to the same goal.
4. A second goal next to an obstacle.

Each goal must succeed within 0.30 m of simulator truth. The goal heading
is not checked (`--max-yaw-error-deg 0`). The checker measures from the
trunk (`--base base`; `pelvis` for the G1).

The leg odometry screen (`scripts/leg_odometry_check.sh`, slam mode)
walks forward, arcs both ways, slows and stops (about 9 m), and reports
how far the leg odometry and the EKF end from truth.

## Rounds

| Robot | Simulator | Round 1 | Round 2 | Round 3 | Round 4 |
|---|---|---|---|---|---|
| Go2 | MuJoCo | fail | **pass** | **pass** (0.14 / 0.13 m) | - |
| Go2 | Isaac | fail | **pass** (0.06 / 0.16 m) | - | - |
| Go2 | Gazebo | fail | fail (fell at spawn) | fail (no controller) | fail (walks; lost localization) |
| A1 | MuJoCo | fail | fail (stalled turning) | **pass** (0.11 / 0.01 m) | - |
| A1 | Isaac | - | - | **pass** (0.22 / 0.02 m) | - |
| B2 | MuJoCo | - | fail (stalled turning) | fail (stalled at 0.2 m/s) | **pass** (0.13 / 0.02 m) |
| G1 | MuJoCo | fail | **pass** (0.16 / 0.11 m) | **pass** (0.10 / 0.10 m) | **pass** (0.18 / 0.03 m) |
| G1 | Isaac | - | - | fail (stood still, 0.49 / 0.58 m) | **pass** (0.14 / 0.07 m) |

The pairs are the final truth errors of the resumed detour goal and the
near-obstacle goal. "-" means not run in that round.

**Round 1** (`v1/`, 03:10-03:31, before `0bf1aef`): every mission failed.

- The leg odometry read 20-40% slow, so AMCL and the EKF lagged the
  robot. In the diagnosis walk (`diagnosis/walk.json`, MuJoCo Go2) it
  read 0.22 m/s for a true 0.35 m/s and ended 2.1 m off after 7.7 m.
- The 0.38 m circular Go2 footprint triggered "collision ahead" stops
  (25 in MuJoCo, 31 in Isaac; 10 for the A1).
- In Gazebo and Isaac the planner later found the start "in lethal space"
  and gave up.
- Leg odometry drift over the screen: 8.6-22% of the distance walked.

**Round 2** (`v2/`, 06:54-07:15, after `0bf1aef` and `71b2dbb`: velocity
gate, polygon footprints): 3 of 6 pass.

- Leg odometry drift fell to 1.3-7.1% (B2 1.3%, G1 3.0%, Go2 MuJoCo 3.8%,
  A1 4.5%, Gazebo Go2 5.9%, Isaac Go2 7.1%).
- A1 and B2 stalled while turning in place: RPP's turn ramp never rose
  above what their gaits respond to.
- The Gazebo Go2 fell on its side at spawn and never moved.

**Round 3** (`v3/`, from 10:58 at `f36bc5e`; turn ramp removed, paused
Gazebo start): 4 of 7 pass. The A1 now passes in MuJoCo and Isaac.

- **Gazebo Go2:** the paused start deadlocked. A controller switch
  completes only in the controller manager's update loop, which a paused
  world never runs, and the manager's other services waited behind it.
  Both spawners timed out after 3 × 10 s, the unpause node gave up after
  60 s, and the Go2 never got its controller. The unpause node now steps
  the paused world a few iterations per poll while it waits
  (`8f3bc2c`).
- **B2:** it walked the detour, then stood still for 20 s at the 0.2 m/s
  approach command, 0.19 m and then 0.40 m short of its goals, until the
  progress checker aborted. Its minimum approach speed was first raised
  to 0.3 m/s (`8f3bc2c`); the check below showed that is not enough.
- **G1 in Isaac:** Nav2 reported both goals reached, but truth was 0.49
  and 0.58 m away. The traces show why: at the 0.2 m/s approach command
  the G1 stood still (truth speed 0.001-0.011 m/s in each 5 s window)
  while its leg odometry crept forward, so the map estimate ended
  0.27-0.35 m ahead of the robot. The G1 also stands still at 0.2 m/s in
  MuJoCo, for about 36 s in total in that mission; it passed there (0.10 m)
  only because the creeping estimate entered the goal window while the
  robot was already close.
- The same check over all round-3 traces: B2 also stands still at 0.3
  m/s (leg odometry screen) and stalls for seconds at 0.35. Go2 and A1
  were never held at a low command.
- G1's minimum speeds are now 0.3 m/s, where it walks 0.29 m/s; B2 is
  commanded at 0.5 m/s and never below 0.4 (`336dc46`).

**Round 4** (`v4/`, from 11:55 at `59c6ca6`; stepped paused start,
walking speeds): 3 of 4 pass.

- **B2 and G1** pass in MuJoCo, and the G1 in Isaac. They are also
  faster: G1 MuJoCo reached its goals in 25 s and 12 s instead of 60 s
  and 47 s. In Isaac the G1's estimate ended 0.07-0.09 m from truth,
  against 0.27-0.35 m in round 3.
- **Gazebo Go2: the start works.** The unpause node stepped the paused
  world 20 iterations (20 ms of simulated time). Both controllers were
  active before it unpaused, and the Go2 stood and walked 6.6 m.
- **Gazebo Go2: the mission still fails.**
  - The localization error stayed within 0.16 m for 22 s.
  - Then the Go2 surged to about 0.5 m/s, faster than its 0.32-0.40 m/s
    command, and its estimate fell 0.63 m behind truth. Two seconds
    earlier it had walked 0.27-0.30 m from the west wall.
  - The planner then found the estimated start "in lethal space" and gave
    up on both goals.
  - The cause of the surge is not established. A brush with the wall is
    one candidate; another is that the leg odometry's velocity gate
    rejected stance feet during a sudden speed change.

**Round 5** (`v5/`, 16:3x, `e0e496e`): Gazebo Go2 with 2 ms physics.

- The R4.3-style worlds step physics at 10 ms, so the Go2's 500 Hz PD
  loop had run at 100 Hz. That matches the lurching in round 4, while the
  Go2 walks cleanly in nav_empty (2 ms). The Go2 profile now declares
  `gazebo_max_physics_step_s: 0.002`, and coarser worlds launch as a
  capped copy.
- The gait no longer surged, and localization stayed within 0.1 m all run.
- The mission still failed, in the start corner:
  - after the cancel, the Go2 faced the west wall;
  - RPP stopped 23 times for "collision ahead";
  - the recovery spins could not turn it, because himloco does not turn
    on the spot in Gazebo.
- Next for this cell: no spin recovery for robots that cannot turn in
  place, and a less cramped start.

**Round 6** (`v6/`, 17:58, `4e93bcc`): Gazebo Go2 with the back-up recovery
tree (`a9c6587`).

- The first goal again aborted beside the start block, and the resumed
  detour was aborted 3.8 m short.
- The **near-obstacle goal succeeded (0.09 m)**: the first goal the Go2
  has reached in Gazebo.

## Repeats (`repeats/`, seeds 2 and 3, 17:28-17:58)

The mission with two more goal seeds for every cell that had passed seed
1:

| Robot | MuJoCo, seeds 1 / 2 / 3 | Isaac, seeds 1 / 2 / 3 |
|---|---|---|
| A1 | pass / pass / pass | pass / pass / pass |
| G1 | pass / pass / pass | pass (round 4) / pass / fail |
| Go2 | pass / fail / fail | pass (round 2) / pass / fail |
| B2 | pass (round 4) / fail / fail | not run |

The A1 passed all six runs, and the G1 five of six. Every failure but one
began at the start, a 0.5 m gap below the corner of a 2.5 m block:

- **Go2** (MuJoCo seed 2, Isaac seed 3): RPP arced it toward the block
  ("collision ahead"). It cannot turn on the spot in MuJoCo, and the
  0.15 m/s back-up is inside its reverse dead zone. The first goal aborted,
  and later goals failed the same way.
- **B2** (both seeds): turning on the spot, its 1.1 m footprint would hit
  the block, so the first goal aborted. Both later goals then succeeded
  (0.13-0.31 m; 0.309 m is just over the 0.30 m limit).
- **G1 Isaac seed 3:** the planner found the start "in lethal space" on the
  first goal, though the start is 0.65 m from any wall. Both later goals
  succeeded. Not explained.
- **Go2 MuJoCo seed 3** got past the start and reached the detour goal,
  then arced into an obstacle on the near-obstacle goal.

This led to the forward-arc motion model (`7a8a3cf`): Go2 (MuJoCo,
Gazebo) and B2 (MuJoCo) now plan forward-only Hybrid-A* curves with their
footprint, and never spin or back up.

## Files

- `v1/`: round 1 missions and leg odometry screens.
- `v2/`: round 2.
- `v3/`: round 3 missions.
- `v4/`: round 4 missions.
- `v5/`, `v6/`: Gazebo Go2 rounds 5 and 6.
- `repeats/`: seeds 2 and 3.
- `diagnosis/`: the MuJoCo Go2 walk and mission that exposed the slow
  leg odometry, before the velocity gate.

The `/tf` bags, traces and launch logs stay on the SSD under
`robot_lab_runtime/claude-legged-nav*`.

## Limits

- Three seeds per cell at most, on one map (nav_obstacle), flat ground.
- Leg odometry drifts a few percent of the distance walked; AMCL
  corrects it against the map. Mapping (slam mode) with leg odometry is
  measured separately
  ([p7-legged-slam-mapping-2026-10-13](../p7-legged-slam-mapping-2026-10-13/README.md)).
- Isaac B2 overshoots and drifts after stop (2026-10-12) and is not run.
- Go2 himloco cannot turn in place in MuJoCo or Gazebo; it turns while
  walking.
