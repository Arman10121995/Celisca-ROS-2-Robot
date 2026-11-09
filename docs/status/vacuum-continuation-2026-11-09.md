# Continuation plan — November 9, 2026 (paused at the user's request)

Work stopped mid-validation of the robot-vacuum cleaning system. Nothing
is running. The working tree is committed. Pick up here.

## 1. Robot-vacuum cleaning system (OOMWOO-class): state

**Implemented (experimental):**
- `robot_lab_vacuum_cleaning`: the core, the node and 16 unit tests.
- `oomwoo` Gazebo model with LiDAR, bumper, cliff sensors and IMU.
- `simulated_room_vacuum.launch.py controller:=cleaning`.
- The Vacuum workspace app (9 GUI tests).
- `scripts/sim_vacuum_mission.sh` (one recorded job) and
  `scripts/vacuum_run_report.py` (metrics, keep-out check against truth,
  dock error, figure).
- Guide: [robot-vacuum.md](../tutorials/robot-vacuum.md).
- Commits: 61b58bcb, e9ecb1d4, 39780794, then this commit.

**Measured in Gazebo, oomwoo, small_house, `nav` mode, at e9ecb1d4.**
Raw reports and figures are in
`/workspace/molar/robot_lab_runtime/claude-v63/{runs,report}`; they are
not yet copied into `docs/status/evidence/`.

| Run | Result | Coverage | Area | Time | Dock error (truth) | Notes |
|---|---|---|---|---|---|---|
| smoke zone (61b58bcb build) | completed | 91.0% | 7.6 m² | 157 s | 0.02 m, 3° | `claude-v63/smoke_zone.json` |
| rooms: office, edges | completed | 94.4% | 9.5 m² | 306 s | 0.035 m, −1.7° | cliff sensors read the floor at 4.9 mm; localisation error mean 0.056 m |
| zone + no-go + virtual wall | completed | 83.3% | 8.2 m² | 231 s | 0.034 m, 3.4° | **fail:** robot centre came 0.111 m from the no-go box, so the body entered it about 5 cm. No wall crossings |
| spot at (2, 2) | completed | 58.9% | 2.0 m² | 112 s | 0.065 m, −2.7° | **fail:** low coverage next to a furniture bar |

**Fixed after those runs; unit-tested, not re-run in a simulator:**
- Keep-outs get an extra 0.12 m berth beyond the robot radius and margin.
  Offline, the zone job gives 86%.
- The follower's progress index advances only along a short stretch of
  path ahead, and spot/edge/lane paths use shorter lookaheads.

**Still open:**
- Spot coverage is still 60% offline on that spot. Check the square spiral
  generation and splitting next to obstacles. Reproduce with the
  kinematic harness:
  `/workspace/molar/.tmp/claude-1000/-workspace-molar-ros-ws-bumperbot-ws/7b33972f-1908-4335-b9c0-2b5aaf81d855/scratchpad/dbg_spot.py`.
  If the scratchpad is gone, rebuild it from `test/test_cleaning_core.py`
  (`World`, `run`) on the small_house map.

## 2. Next steps, in order

1. Raise spot coverage (target ≥ 85% of the reachable square). Add a
   unit test on a map with an obstacle beside the spot.
2. **Rebuild before any run.**
   `colcon build --packages-select robot_lab_vacuum_cleaning` installs
   copies, so edits are not live until rebuilt. Then re-run the queue
   (one simulation at a time, quiet host), in this order:
   1. zone + keep-out; it passes only with no body entry and no wall crossings;
   2. spot;
   3. bedroom with recharge and resume (`BATTERY_SCALE=60`, interrupted
      this time, set aside in `claude-v63/interrupted`);
   4. MuJoCo zone (backend portability);
   5. SLAM mapping run (`MODE=slam`, `explore`);
   6. whole home (about 60 minutes).

   The queue script is in the scratchpad (`queueT9.sh`). The same calls
   can be made directly with `scripts/sim_vacuum_mission.sh`.
3. Bumper evidence: add an object that is not on the map (a box in a
   world copy, or Gazebo spawn) and confirm `/bumper` contacts and the
   detour.
4. Live GUI check: launch from Setup, start a rooms job from the map,
   pause/resume, dock; screenshot. So far only fed-message tests and
   screenshots exist.
5. Record the evidence:
   - `docs/status/evidence/vacuum-2026-11/` (reports, figures, README);
   - a batch YAML, indexed with `scripts/r9_3_index_missions.py` (new
     task_type, e.g. `vacuum_cleaning_job`);
   - regenerate the matrix.
6. Add **R13 Robot-vacuum cleaning system** to ROADMAP.md, the ledger
   (`docs/status/platform-status.yaml`, state partial, owner claude),
   CURRENT_STATUS and CHECKLIST. Do this only once evidence is recorded.
7. Later (not implemented): carpet detection/boost, mopping, a camera
   obstacle classifier, Home Assistant/MQTT bridge, Nav2 FollowPath
   executor option, MuJoCo/PyBullet/Isaac bumper and cliff emulation.

## 3. Other work waiting from the November 5 queues (not yet recorded)

Raw data is in `/workspace/molar/robot_lab_runtime/`. None of it is
indexed yet.

- **Round 59, car course** (`claude-v59/course`, `--align-heading`, 15
  runs): full pass 5/15. Gazebo rear-steer r1, MuJoCo anti-Ackermann and
  PyBullet anti-Ackermann abort in the slalom. The other failures park
  0.15–0.18 m off.
- **Round 62, car course with the depth trim** (`claude-v62/course`):
  full pass **10/12**, parking 0.003–0.079 m and ≤4.4°. Failures: slalom
  aborts for Gazebo rear-steer and Isaac anti-Ackermann.
- **Still to apply:** the RPP
  `max_allowed_time_to_collision_up_to_carrot: 0.5` change in
  `navigation.launch.py` CAR_PARAMS.
- **Go2 in Isaac with go2_walk_r2** (`claude-v59/go2`, `go2-arcs`): to analyse.
- **Training** (`claude-v59/train`, `claude-v62/train`): go2_walk_r3
  (actuation delay), go2w_walk_r1, g1_walk_r1 and go2_rough_r1, with
  their drive checks. All ran (exit 0); still to analyse.
- **Calibration** (`claude-v60/calibration`): centre check in four
  backends, to analyse. GUI suite at that point: 205 passed.
- **PX4 manual flight in 8 worlds** (`claude-v61/px4`): all failed
  because the drone never armed (phase idle, 55 s flight timeout). This is
  likely the newer takeoff-check gate versus `qualify_px4_drive_gui.py`.
  Fix the script; do not bypass the gate.
- **Still open:** MOLAR sensors need the user's real sensor layout.
