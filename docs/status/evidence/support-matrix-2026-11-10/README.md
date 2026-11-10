# R9.3: support matrix with the robot-vacuum runs (2026-11-10)

`scripts/r9_3_index_missions.py` appended 8 records from
[vacuum-batch.yaml](../vacuum-2026-11/vacuum-batch.yaml) and 1 from
[px4-batch.yaml](../px4-2026-11-10/px4-batch.yaml) to
[runtime-evidence-index.yaml](../../runtime-evidence-index.yaml), which now
holds 696. Each is judged by the new `cleaning_screen`: the job completed with
≥ 85 % of its reachable target cleaned, the robot ended on its dock within
0.10 m / 10° by truth, and the no-go and virtual-wall rules held on the truth
trail.

| | November 9 (687 reports) | November 10 (696 reports) |
|---|---|---|
| Measured cells | 310 | **318** |
| Passed screens | 298 | **306** |
| Failed screens | 11 | 11 |
| Unverified | 1 | 1 |

There are seven new cells (oomwoo × small_house), all passed:
- Gazebo: `vacuum_rooms`, `vacuum_zone_keepout`, `vacuum_spot`,
  `vacuum_zone_low_obstacle` (the b3a63ae7 re-run controls this cell),
  `vacuum_recharge_resume`, `vacuum_whole_home`;
- MuJoCo: `vacuum_zone_keepout`.

PX4 X500 manual flight on `aerial_course` (`flight_manual_screen`): passed
after the qualification script gained the takeoff clearance check. Clearance
came back clear with a 3.0 m rise; the climb measured 2.95 m. The report is
`report-measured.json`, which adds the body-trace summary from the raw trace,
as for October 8.

See the [vacuum evidence](../vacuum-2026-11/README.md). One run per cell;
unlisted cells remain untested.
