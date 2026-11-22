# V1: vacuum jobs on MuJoCo, PyBullet and Isaac (November 21, 2026)

**Status: 8 of 9 screens passed after the November 22 docking fix** (5 of 9 on November 21, when PyBullet docked off target). `run_jobs.sh`
replays the November 10 Gazebo start requests: the office rooms job, the
(2, 2) spot job, and the keep-out zone with a no-go box and a virtual wall.
It runs `oomwoo` in `small_house` through `scripts/sim_vacuum_mission.sh`,
one job at a time. `cleaning_screen` judges each report:
- the job completed;
- 85 % or more of the target was cleaned;
- the robot is docked within 0.10 m and 10° by truth;
- no-go zones and virtual walls were respected along the truth trail.

| Backend | Rooms (office) | Spot | Keep-out zone |
|---|---|---|---|
| MuJoCo | **failed**: "Stuck", 46 % cleaned | passed (91 %, docked 0.03 m) | passed (99 %, 0.02 m) |
| PyBullet | **failed**: 92 %, docked 0.18 m off | **failed**: 94 %, 0.15 m off | **failed**: 98 %, 0.14 m off (passed on Nov 10) |
| Isaac | passed (96 %, 0.04 m) | passed (91 %, 0.04 m) | passed (99.5 %, 0.03 m) |
| Gazebo (today's reference) | — | — | passed (99.3 %, 0.02 m) |

Six of these cells are new in the support matrix (now 348 measured cells):
the rooms and spot jobs on the three backends.

## PyBullet docking

All three PyBullet jobs clean fully and come back. Each then ends 0.14–0.18 m
to the dock's +y side, still reporting "Docked"; its own estimate is
0.09–0.12 m off. In the truth trail the align phase turns the robot in
place to about −0.08 rad. While it drives in, the heading keeps rising to
+0.48 rad, and the short 0.25 m entry cannot recover. Gazebo and Isaac
drive straight in at about −0.1 rad.

Ruled out, each reverted, with runs in `runs-friction/`:
- Floor friction 1.0 instead of Bullet's 0.5 for undeclared static shapes:
  still 0.159 m.
- Leaving out the sub-ground floor and carpet meshes, which became world
  meshes when the November 20 small_house projection resolved its
  furniture: still 0.159 m.
- No charger body is spawned at the dock.
- The PyBullet wheel force limit is unchanged since October.

The remaining suspect is the robot's in-place rotation overshoot on
PyBullet after the align phase. It needs a closer look at the spinning
wheels' braking at that moment.

### Fixed on November 22: all three PyBullet jobs pass

The cause was in the shared docking logic, not in PyBullet. The entry phase
steers toward the dock and switches to the final turn once the bearing
error exceeds 1.2 rad. A robot passing beside the dock reaches that angle
and then logged "Docked" wherever it was. On PyBullet the align spin was
still coasting when the entry began, so the robot curved in. Commit
42f6a13d changes two things:
- the align phase now waits for the heading rate to fall below 0.05 rad/s;
- an entry that misses by more than 0.06 m backs off and aligns again, up
  to three times, then reports "Could not dock".

| Job | Coverage | Dock offset by truth | Retries |
|---|---|---|---|
| rooms_office | 91.8 % | 0.037 m, −2.5° | 1 (missed by 0.11 m) |
| spot | 94.0 % | 0.036 m, 2.4° | 0 |
| zone_keepout | 97.1 % | 0.029 m, −4.2° | 1 (missed by 0.13 m) |

Before the fix the same jobs ended 0.14–0.18 m off. The re-runs are in
`runs-docking-fix/`, indexed by `docking-fix-batch.yaml`, which now
controls the three PyBullet cells. With them, the round passes 8 of 9: all
of Isaac and PyBullet, and MuJoCo 2 of 3. The Isaac and Gazebo cells ran before the fix
and have not been re-run with it.

The three MuJoCo jobs were re-run with the fix (`docking-fix-mujoco-batch.yaml`):
- spot passed (90.6 %, docked 0.037 m);
- keep-out passed (99.6 %, docked 0.038 m);
- the rooms job froze again, at 27 % in the kitchen, at a different place
  (9.13, 0.07). Its truth did not change by even 1 mm for 25 s while all
  three freeing manoeuvres commanded reverse and turn-in-place. No cliff,
  bumper or safety stop was logged.

The robot is jammed in the MuJoCo scene, not stopped by its controller.
`runs-mujoco-stuck/` repeats the job with static world flex culling off
and on. Both froze at x = 9.13:
- culling off: at (9.13, 0.22) after 26.9 %;
- culling on: at (9.13, 0.41) after 91.8 %.

So culling is not the cause. Three of four MuJoCo rooms runs froze within
0.4 m of x = 9.13, between the kitchen table and the window wall. What
holds the robot there in the MuJoCo scene is still open; an offline mesh
probe did not reproduce the scene's include poses, so it is not used. The
MuJoCo rooms cell remains failed.

## MuJoCo rooms job

The rooms job stopped after 283 s with "Stuck – move the robot to open
floor and resume" (3 freeing attempts), at (6.62, 1.88) in the office.
It is kept.

Launch logs and the job stores are on SSD under
`robot_lab_runtime/v1-vacuum-20261121`. Reports are indexed with
`vacuum-batch.yaml`.
