# Validation Plan - November 2, 2026

This document outlines the step-by-step validation plan for all remaining Robot Lab tasks, organized by priority and execution order.

---

## Priority Legend

- **P0** (Critical): Blocks multiple other tasks
- **P1** (High): Primary focus, unblocked
- **P2** (Medium): Can run in parallel with P1
- **P3** (Low): Can wait for P0-P2 completion

---

## Execution Order

### Phase 1: Environment Fix (P0) - BLOCKING

**Must complete before any simulation-based validation can proceed**

| Task | Description | Owner | Estimated Time | Status |
|------|-------------|-------|----------------|--------|
| ENV-1 | Fix GLIBCXX_3.4.29 library issue for ROS imports | Platform | 1-2 hours | ❌ Pending |
| ENV-2 | Verify PX4 SITL environment on Jetson | Platform | 1 hour | ❌ Pending |

**Dependencies**: All Phase 2+ tasks depend on ENV-1 and ENV-2

---

### Phase 2: R5 Car Validation (P1)

**Priority**: Highest once environment is fixed
**Task**: R5.5 - Verify car parking heading fix

| Task | Description | Command | Success Criteria | Status |
|------|-------------|---------|------------------|--------|
| R5.5-A | Run car course test in Gazebo | `scripts/sim_car_course.sh gazebo rear_steer_car output.json` | Parking yaw error < 10° | ❌ Pending |
| R5.5-B | Run car course test in PyBullet | `scripts/sim_car_course.sh pybullet rear_steer_car output.json` | Parking yaw error < 10° | ❌ Pending |
| R5.5-C | Run car course test in Isaac | `scripts/sim_car_course.sh isaac rear_steer_car output.json` | Parking yaw error < 10° | ❌ Pending |
| R5.5-D | Run car course test for all car types | `scripts/sim_car_course.sh <sim> <car_type> output.json` for all combinations | All pass 10° tolerance | ❌ Pending |

**Dependencies**: ENV-1 (GLIBCXX fix)

**Evidence to Create**:
- `docs/status/evidence/r55-validation-results-2026-11-XX.md`

---

### Phase 3: R10.2 Fleet Motor-Power/Charging Validation (P1)

**Priority**: High - Recently implemented
**Task**: R10.2 - Verify fleet motor-power/charging implementation

| Task | Description | Command | Success Criteria | Status |
|------|-------------|---------|------------------|--------|
| R10.2-A | Run fleet facility simulation | `ros2 launch robot_lab_bringup fleet_facility.launch.py` | Facility loads without errors | ❌ Pending |
| R10.2-B | Verify effort state interfaces | Check `/bumperbot/effort_state` and `/labbot/effort_state` topics | Topics publishing effort data | ❌ Pending |
| R10.2-C | Verify infrastructure verification flag | Check `infrastructure.yaml` config | `effort_observation_verified: true` | ✅ Done (code) |
| R10.2-D | Test multi-floor transitions | Run L0→L1→L0 floor transition test | Bumperbot successfully transitions floors | ❌ Pending |
| R10.2-E | Measure energy consumption | Run extended mission, check energy estimates | Energy estimation updates correctly | ❌ Pending |
| R10.2-F | Test charging scenario | Simulate battery drain and charging | Charging state reflected in infrastructure | ❌ Pending |

**Dependencies**: ENV-1 (GLIBCXX fix)

**Evidence to Create**:
- `docs/status/evidence/r102-validation-results-2026-11-XX.md`

---

### Phase 4: R10.3 Aerial Swarm Formation Validation (P1)

**Priority**: High - First R10 priority
**Task**: R10.3 - Extend formation behaviors/maps, 3-4 members, member loss

| Task | Description | Command | Success Criteria | Status |
|------|-------------|---------|------------------|--------|
| R10.3-A | Fix PX4 SITL accelerometer timeout | Investigate upstream PX4 issue | air2 FCU telemetry stable | ❌ Pending |
| R10.3-B | Run 2-member formation flight | `ros2 launch robot_lab_bringup aerial_swarm.launch.py members:=2` | Both FCUs converge to formation | ❌ Pending |
| R10.3-C | Run 3-member formation flight | `ros2 launch robot_lab_bringup aerial_swarm.launch.py members:=3` | All 3 FCUs converge to formation | ❌ Pending |
| R10.3-D | Run 4-member formation flight | `ros2 launch robot_lab_bringup aerial_swarm.launch.py members:=4` | All 4 FCUs converge to formation | ❌ Pending |
| R10.3-E | Test member loss detection | Manually stop one FCU during flight | Remaining members detect loss, trigger landing | ❌ Pending |
| R10.3-F | Test automatic recovery | Stop/Restart FCU during mission | System detects and recovers | ❌ Pending |
| R10.3-G | Test other behaviors/maps | Run formation in different maps | Formation works in all maps | ❌ Pending |

**Dependencies**: ENV-1, ENV-2, R10.3-A (PX4 SITL fix)

**Evidence to Create**:
- `docs/status/evidence/r103-validation-results-2026-11-XX.md`
- `docs/status/evidence/aerial-formation-3member-2026-11-XX/`
- `docs/status/evidence/aerial-formation-4member-2026-11-XX/`
- `docs/status/evidence/aerial-member-loss-2026-11-XX/`

**Existing Evidence**:
- `docs/status/evidence/aerial-r103-2026-10-29/README.md` (baseline)

---

### Phase 5: R10.4-R10.5 Marine Validation (P2)

**Priority**: Medium - Can run in parallel with R10.3
**Task**: R10.4-R10.5 - Marine saved-map routes, obstacle/current/loss missions

#### R10.4 Marine ROV/AUV

| Task | Description | Command | Success Criteria | Status |
|------|-------------|---------|------------------|--------|
| R10.4-A | Run ROV simulation in marine_tank | `ros2 launch robot_lab_bringup marine.launch.py robot:=lab_rov world:=marine_tank` | ROV loads and responds to commands | ❌ Pending |
| R10.4-B | Test ROV depth hold | Set target depth, verify ROV holds depth | Depth error < 0.1m | ❌ Pending |
| R10.4-C | Test ROV heading hold | Set target heading, verify ROV holds heading | Heading error < 5° | ❌ Pending |
| R10.4-D | Test ROV current compensation | Add current, verify ROV maintains position | Position drift < 0.2m | ❌ Pending |
| R10.4-E | Run AUV simulation | `ros2 launch robot_lab_bringup marine.launch.py robot:=tethys_auv world:=marine_cave` | AUV loads and responds to commands | ❌ Pending |
| R10.4-F | Test AUV depth control | Set depth, verify AUV reaches depth | Depth error < 0.1m | ❌ Pending |
| R10.4-G | Test AUV fin-turn | Set heading, verify AUV turns | Heading error < 5° | ❌ Pending |

#### R10.5 Marine Missions

| Task | Description | Command | Success Criteria | Status |
|------|-------------|---------|------------------|--------|
| R10.5-A | Test ROV waypoint navigation | Set waypoints, verify ROV navigates | All waypoints reached | ❌ Pending |
| R10.5-B | Test ROV obstacle avoidance | Place obstacles, verify ROV avoids | No collisions, all waypoints reached | ❌ Pending |
| R10.5-C | Test ROV saved-map routes | Load saved map, run mission | Mission completes successfully | ❌ Pending |
| R10.5-D | Test AUV waypoint navigation | Set waypoints, verify AUV navigates | All waypoints reached | ❌ Pending |
| R10.5-E | Test marine loss scenarios | Stop/sensor failure during mission | System detects and handles loss | ❌ Pending |
| R10.5-F | Test surface vessel | `ros2 launch robot_lab_bringup marine.launch.py robot:=surface_vessel world:=marine_harbor` | Vessel loads and operates | ❌ Pending |

**Dependencies**: ENV-1

**Evidence to Create**:
- `docs/status/evidence/r104-validation-results-2026-11-XX.md`
- `docs/status/evidence/r105-validation-results-2026-11-XX.md`

**Existing Evidence**:
- `docs/status/evidence/r104-marine-2026-10-15/README.md`
- `docs/status/evidence/r104-auv-2026-10-15/README.md`

---

### Phase 6: R10.1-R10.3 Fleet/RMF Validation (P2)

**Priority**: Medium - Can run in parallel with R10.3
**Task**: R10.1-R10.3 - RMF conflicts/loss, coverage, larger fleets

| Task | Description | Command | Success Criteria | Status |
|------|-------------|---------|------------------|--------|
| R10.1-A | Test RMF fleet coordination | `ros2 launch robot_lab_bringup fleet_rmf.launch.py` | Multiple robots coordinate via RMF | ❌ Pending |
| R10.1-B | Test RMF conflict resolution | Create conflicting paths | RMF resolves conflicts, no collisions | ❌ Pending |
| R10.1-C | Test RMF loss scenarios | Stop robot during RMF mission | RMF detects loss, reassigns tasks | ❌ Pending |
| R10.1-D | Test coverage planning | Run coverage mission | Full area coverage achieved | ❌ Pending |
| R10.1-E | Test larger fleet (5+ robots) | Launch with multiple robots | All robots coordinate, complete mission | ❌ Pending |
| R10.3-A | Test fleet member loss | Kill robot process during mission | Fleet detects loss, remaining robots continue | ❌ Pending |

**Dependencies**: ENV-1

**Evidence to Create**:
- `docs/status/evidence/r101-validation-results-2026-11-XX.md`
- `docs/status/evidence/rmf-conflict-resolution-2026-11-XX/`
- `docs/status/evidence/fleet-coverage-2026-11-XX/`

**Existing Evidence**:
- `docs/status/evidence/workflows-2026-10-27/README.md`
- `docs/status/evidence/fleet-floor-map-transition-2026-11-02/README.md`
- `docs/status/evidence/amphibious-facility-2026-10-30/README.md`

---

### Phase 7: R10.6-R10.7 Multi-Leg Validation (P2)

**Priority**: Medium - Can run in parallel
**Task**: R10.6-R10.7 - Multi-leg saved-map goals, terrain/slip/recovery

| Task | Description | Command | Success Criteria | Status |
|------|-------------|---------|------------------|--------|
| R10.6-A | Test Hexapod gait | `ros2 launch robot_lab_bringup hexapod.launch.py` | Hexapod walks with correct gait | ❌ Pending |
| R10.6-B | Test Hexapod saved-map navigation | Load saved map, run mission | Hexapod navigates to goal | ❌ Pending |
| R10.6-C | Test Octopod gait | `ros2 launch robot_lab_bringup octopod.launch.py` | Octopod walks with correct gait | ❌ Pending |
| R10.6-D | Test Centipede gait | `ros2 launch robot_lab_bringup centipede.launch.py` | Centipede moves correctly | ❌ Pending |
| R10.6-E | Test Snake gait | `ros2 launch robot_lab_bringup snake.launch.py` | Snake moves correctly | ❌ Pending |
| R10.7-A | Test terrain adaptation | Run on various terrain types | Gait adapts to terrain | ❌ Pending |
| R10.7-B | Test slip recovery | Create slippery surface | System detects and recovers from slip | ❌ Pending |
| R10.7-C | Test multi-leg saved-map goals | Load saved maps for each leg type | All leg types navigate successfully | ❌ Pending |
| R10.7-D | Test amphibious MOLAR | `ros2 launch robot_lab_bringup molar.launch.py` | MOLAR operates on land and water | ❌ Pending |

**Dependencies**: ENV-1

**Evidence to Create**:
- `docs/status/evidence/r106-validation-results-2026-11-XX.md`
- `docs/status/evidence/r107-validation-results-2026-11-XX.md`

**Existing Evidence**:
- `docs/status/evidence/native-forms-2026-10-30/README.md`

---

### Phase 8: R10.8/R4/R8.3 Workload Validation (P3)

**Priority**: Low - Can wait for higher priority items
**Task**: R10.8/R4/R8.3 - Native producers, fleet batch reset, wider matrices

| Task | Description | Command | Success Criteria | Status |
|------|-------------|---------|------------------|--------|
| R10.8-A | Test fleet workload producers | Run fleet producer workloads | All producers generate correct data | ❌ Pending |
| R10.8-B | Test fleet batch reset | Run batch reset | Fleet resets correctly | ❌ Pending |
| R10.8-C | Test wider matrices | Run with various matrix sizes | All matrices complete | ❌ Pending |
| R4-A | Verify R4.1-R4.3 | Re-run R4 tests | All R4 scenarios pass | ❌ Pending |
| R8.3-A | Test resource-bounded experiments | Run resource-bounded tests | Experiments complete within resource limits | ❌ Pending |

**Dependencies**: ENV-1

**Evidence to Create**:
- `docs/status/evidence/r108-validation-results-2026-11-XX.md`
- `docs/status/evidence/r4-validation-2026-11-XX.md`
- `docs/status/evidence/r83-validation-2026-11-XX.md`

**Existing Evidence**:
- `docs/status/evidence/workflows-2026-10-27/README.md`

---

### Phase 9: R6 and Older Queue (P3)

**Priority**: Low - Final cleanup
**Task**: R6 and older queue - Remaining maps, car perception, manipulation

| Task | Description | Command | Success Criteria | Status |
|------|-------------|---------|------------------|--------|
| R5.6-A | Test four-wheel steering patterns | Run steering tests | All patterns work correctly | ❌ Pending |
| R5.6-B | Test physical mecanum | Run mecanum tests | Mecanum drive works correctly | ❌ Pending |
| R5.7-A | Test native arm planning | Run arm planning tests | Arm plans and executes correctly | ❌ Pending |
| R5.8-A | Test native hands | Run hand tests | Hands operate correctly | ❌ Pending |
| R6-A | Complete remaining map imports | Import remaining maps | All maps import without errors | ❌ Pending |
| R6-B | Test car perception | Run perception tests | Car perception works correctly | ❌ Pending |
| R6-C | Test manipulation | Run manipulation tests | Manipulation works correctly | ❌ Pending |

**Dependencies**: ENV-1

**Evidence to Create**:
- Various evidence files as tests complete

---

## Parallel Execution Plan

Once environment is fixed, tasks can be executed in parallel:

### Track A: Aerial (High Priority)
- ENV-2 → R10.3-A → R10.3-B,C,D → R10.3-E,F,G

### Track B: Fleet/RMF (High Priority)
- ENV-1 → R5.5-A,B,C,D → R10.2-A,B,C,D,E,F → R10.1-A,B,C,D,E → R10.3-A

### Track C: Marine (Medium Priority)
- ENV-1 → R10.4-A,B,C,D,E,F,G → R10.5-A,B,C,D,E,F

### Track D: Multi-Leg (Medium Priority)
- ENV-1 → R10.6-A,B,C,D,E → R10.7-A,B,C,D

### Track E: Workloads (Low Priority)
- ENV-1 → R10.8-A,B,C → R4-A → R8.3-A

### Track F: Older Queue (Low Priority)
- ENV-1 → R5.6-A,B → R5.7-A → R5.8-A → R6-A,B,C

---

## Daily Execution Plan

### Day 1: Environment Fix
- Fix GLIBCXX_3.4.29 (ENV-1)
- Verify PX4 SITL (ENV-2)

### Day 2: High Priority Validation
- Track A: Start R10.3 aerial tests
- Track B: Start R5.5 car tests
- Track B: Start R10.2 fleet tests

### Day 3: High Priority Validation
- Continue Track A: R10.3 2-4 member formation
- Continue Track B: R10.1-R10.3 fleet/RMF
- Start Track C: R10.4-R10.5 marine

### Day 4: Medium Priority Validation
- Track C: Continue marine tests
- Track D: Start multi-leg tests
- Track A: Complete aerial tests

### Day 5: Medium/Low Priority Validation
- Track C: Complete marine tests
- Track D: Continue multi-leg tests
- Track E: Start workload tests

### Day 6: Low Priority Validation
- Track D: Complete multi-leg tests
- Track E: Continue workload tests
- Track F: Start older queue tests

### Day 7: Wrap-up
- Complete all validation
- Create final evidence reports
- Update ledger

---

## Success Criteria

**Phase 1 (Environment)**:
- ✅ ROS imports work without GLIBCXX errors
- ✅ PX4 SITL runs without accelerometer timeout

**Phase 2 (R5.5)**:
- ✅ All car course tests pass with < 10° yaw error

**Phase 3 (R10.2)**:
- ✅ Effort state interfaces publishing
- ✅ Infrastructure verification flag working
- ✅ Multi-floor transitions work
- ✅ Energy estimation updates correctly

**Phase 4 (R10.3)**:
- ✅ 2-member formation flight successful
- ✅ 3-member formation flight successful
- ✅ 4-member formation flight successful
- ✅ Member loss detection and recovery works

**Phase 5 (R10.4-R10.5)**:
- ✅ ROV depth/heading hold works
- ✅ ROV current compensation works
- ✅ AUV depth/fins control works
- ✅ Marine waypoint navigation works
- ✅ Marine obstacle avoidance works
- ✅ Marine saved-map routes work

**Phase 6 (R10.1-R10.3)**:
- ✅ RMF fleet coordination works
- ✅ RMF conflict resolution works
- ✅ RMF loss handling works
- ✅ Coverage planning works
- ✅ Larger fleet (5+) works
- ✅ Fleet member loss handling works

**Phase 7 (R10.6-R10.7)**:
- ✅ All multi-leg robots move with correct gaits
- ✅ All multi-leg robots navigate saved maps
- ✅ Terrain adaptation works
- ✅ Slip recovery works
- ✅ Amphibious MOLAR works

**Phase 8 (R10.8/R4/R8.3)**:
- ✅ Fleet producers work
- ✅ Fleet batch reset works
- ✅ Wider matrices work
- ✅ R4 scenarios pass
- ✅ Resource-bounded experiments work

**Phase 9 (R6 and older)**:
- ✅ Four-wheel steering patterns work
- ✅ Physical mecanum works
- ✅ Native arm planning works
- ✅ Native hands work
- ✅ Remaining maps import without errors
- ✅ Car perception works
- ✅ Manipulation works

---

## Ledger Updates

Each validation task should update:
1. `docs/status/platform-status.yaml` - ledger with completion status
2. `docs/status/evidence/` - directory with test results
3. Task-specific documentation files

---

## Notes

1. All tasks assume the environment fixes (ENV-1, ENV-2) are complete
2. Estimated times are rough and may vary based on issues discovered
3. Parallel execution can significantly reduce total validation time
4. Blockers discovered during validation should be documented and escalated
5. Test results should be recorded even if tests fail - include error details for debugging

---

## Version History

- **2026-11-02**: Initial version created
- Next update: After ENV-1 and ENV-2 are resolved
