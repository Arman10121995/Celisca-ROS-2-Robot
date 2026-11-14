# P4: MuJoCo flex culling, sensor thread and tick pacing (2026-11-14)

Jetson AGX Orin, MuJoCo 3.17, headless, nav mode. Implemented, experimental.

The [October work](../p4-mujoco-step-2026-10-12/README.md) removed the
midphase refit. This pass profiled what was left with the new
`profile_period_s:=<s>` spawner option, which logs where the physics thread
spends its time.

## What was slow

Labbot on celisca_floor_1, before ([profile_before.txt](profile_before.txt)).
A physics tick is two 2 ms steps:

| Phase | Share of the thread | Per call |
|---|---:|---:|
| `mj_step` ×2 | 64 % | 3.70 ms |
| Python around the step | 12 % | 0.69 ms |
| Camera render (RGB + depth, 5 Hz) | 9 % | 25 ms |
| Scan (5 Hz) | 4.5 % | 13 ms |

Native timers on the staged world split one step (2.5 ms) into:

- 1.17 ms of flex kinematics, where MuJoCo updates all 55,635 vertices and
  70,029 element boxes every step;
- 1.16 ms of broadphase over all 71 static flexes.

## Changes

| Change | Effect | How it was checked |
|---|---|---|
| `world_collision.StaticFlexCulling` (`world_flex_culling:=true`, default). A static world flex collides only while a moving geom's bounding sphere plus 0.5 m reaches its box. | Broadphase 1.16 → 0.14 ms. Step 2.55 → 1.66 ms. | 4 flights at 3 m/s into celisca_floor_1 walls, 544–1020 flex contacts each: identical final `qpos` bit for bit and identical contact counts every step ([culling_identity.json](culling_identity.json), `scripts/mujoco_flex_culling_check.py`). Unit test `test_static_flex_culling_keeps_contacts_and_disables_far_walls`. Scans are identical (rays use the separate proxies). |
| Tick pacing (`_tick_pacing`). The tick lasts exactly the physics it advances. | 240 Hz ticks were paced at 4.17 ms of wall time for 4 ms of physics, so even an empty world was capped at RTF 0.96. The drive ramp also integrated with the wrong `dt`. | `test_tick_lasts_exactly_the_physics_it_advances` |
| Oversleep paid back (`_pacing_sleep`). Late wake-ups of `time.sleep` come out of the next sleep, bounded like catch-up. | nav_obstacle 0.913 → 0.983 while the thread slept half of every tick | `test_oversleep_is_paid_back_from_the_next_sleep` |
| Sensor thread (`sensor_thread:=true`, default). Scan and camera run from a snapshot of the state they are due at, on their own `MjData`, stamped with that time. MuJoCo releases the GIL, so physics keeps stepping. | Camera and scan leave the physics thread | A stationary Bumperbot gives identical scans at all 72 shared stamps with the thread on and off ([scan_worker_true.json](scan_worker_true.json), [scan_worker_false.json](scan_worker_false.json)) |

The camera pose comes from kinematics on the stepped state, so it now
matches its stamp. Before, it lagged one 2 ms step behind it. A newer
request supersedes one still waiting, so a frame is dropped rather than
queued.

## Real-time factor (live, `scripts/sim_rtf_check.sh`, 20 s window)

| Robot / map | Oct 12 | Before this pass | Final ([rtf_final.txt](rtf_final.txt)) |
|---|---:|---:|---:|
| Labbot / celisca_floor_1 | 0.675 | 0.651 | **0.806** |
| Bumperbot / celisca_floor_1 | 0.693 | | **0.832** |
| Labbot / celisca_floor_2_furniture | 0.842 | | **0.989** |
| Labbot / dataset_hospital | 0.344 | | **0.431** |
| Bumperbot / nav_obstacle | 0.877 | | **0.983** |

Labbot on celisca_floor_1 with only the pacing fix (culling and sensor
thread off) gave 0.661. Intermediate results before the oversleep fix are in
[rtf_before_oversleep_fix.txt](rtf_before_oversleep_fix.txt), and the
per-phase profiles in [profiles_final.txt](profiles_final.txt).

## Not reached: RTF ≥ 1 on the large worlds

On celisca_floor_1 the physics thread is busy 90 % of the time. `mj_step`
takes 3.74 ms of each 4 ms tick, and most of that is MuJoCo's per-step
update of every static flex vertex and element box. That cost is not
conditional on collision flags: in `mj_flex`, only the BVH refit is skipped
when the midphase is off. The hospital (194k elements) takes 7.7 ms per tick.

Below real time, the remaining options change geometry:

- fewer collision triangles;
- SDF or convex-decomposition collision for static world geometry.

Either would change contacts, so it needs its own identity and mission
checks.
