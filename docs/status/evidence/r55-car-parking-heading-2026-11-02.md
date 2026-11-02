# R5.5 Car Parking Heading Fix — 2026-11-02

## Issue

The car course mission (scripts/sim_car_course.sh) was failing parking tests due to final heading errors exceeding the 10° tolerance. In round 57 (2026-10-19), parking ended 11.6-14.3° off target in Gazebo, PyBullet, and Isaac backends (MuJoCo passed at -3.42°).

The sim_car_course.py script has a `HeadingShuffle` implementation that can correct the final heading by shuffling forward/reverse after Nav2 reports success, but this feature was never enabled because the calling script (sim_car_course.sh) did not pass the `--align-heading` flag.

## Root Cause

In scripts/sim_car_course.sh, line 31:
```bash
python3 "$HERE/sim_car_course.py" --robot "$ROBOT" --course "$COURSE/course.json" --timeout "${TIMEOUT:-600}" \
  --trace-out "$OUT.trace.json" ${CHECK_ARGS:-} > "$OUT" 2> "$OUT.err"
```

The `--align-heading` flag was missing, so the `args.align_heading` in sim_car_course.py defaulted to False, and the HeadingShuffle correction was never invoked.

## Fix

Added `--align-heading` flag to scripts/sim_car_course.sh:
```bash
python3 "$HERE/sim_car_course.py" --robot "$ROBOT" --course "$COURSE/course.json" --timeout "${TIMEOUT:-600}" \
  --trace-out "$OUT.trace.json" --align-heading ${CHECK_ARGS:-} > "$OUT" 2> "$OUT.err"
```

This enables the HeadingShuffle correction logic in sim_car_course.py, which:
1. After Nav2 reports success for the parking goal
2. Uses the localization estimate to shuffle forward/reverse
3. Aligns the heading within the configured tolerance (default: 10°)
4. Reports both the Nav2 arrival and the alignment result

## Implementation Details

The sim_car_course.py already had the complete implementation:
- `parser.add_argument('--align-heading', action='store_true', ...)`
- `parking_alignment='shuffle' if args.align_heading else 'nav2 only'`
- `align_heading` parameter in the `section()` function
- HeadingShuffle class from robot_lab_utils.car_alignment
- Default tolerance: 10° (matches course requirement)
- Default align_tolerance: 3° (for the shuffle to stop)

The fix simply enables this existing functionality.

## Expected Impact

With this fix:
- Car course parking should now pass the 10° heading tolerance in all backends
- The parkin section will report both Nav2 arrival and alignment results
- Previous failures due to 11.6-14.3° yaw errors should be resolved

## Files Changed

- `scripts/sim_car_course.sh`: Added `--align-heading` flag

## Verification

The fix needs to be verified by re-running the car course tests:
```bash
scripts/sim_car_course.sh <simulator> <robot> <output.json>
```

Expected: parking section should now show `passed: true` with final yaw error within 10°.

**Note**: Full end-to-end verification was not possible in the current environment due to ROS library compatibility issues (GLIBCXX_3.4.29 requirement). However, the code change is correct and enables the existing HeadingShuffle implementation in sim_car_course.py.

## References

- ROADMAP.md R5.5 acceptance criteria
- docs/status/evidence/rounds-2026-10-19/README.md (round 57 results showing the 11.6-14.3° failures)
- scripts/sim_car_course.py (HeadingShuffle implementation)
- robot_lab_utils.car_alignment.HeadingShuffle
