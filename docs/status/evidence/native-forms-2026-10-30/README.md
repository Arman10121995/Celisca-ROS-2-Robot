# Actual native-form receipts

[Commands and reproduction boundaries](CHECKS.md),
[installed/source parity and owned cleanup](installed-parity.json).

Recorded October 30, 2026, continuation from `db14385` on `master` with a dirty
implementation tree. [Delivery and limits](../../native-forms-2026-10-30.md).
[Manifest](manifest.json) names raw SSD directories, copied SHA-256 identities
and post-run source hashes. Run records preserve commands, domain/partition,
requested seeds, resource capture and owned cleanup.

| Case | Native robot / map / backend | Actual observation |
|---|---|---|
| [Snake 2D acquisition](snake-2d-map/extension-mission-result.json) | `lab_snake` / `nav_empty` / MuJoCo | 0.268217 m body travel; 47,073 known / 937 occupied cells; populated archived YAML/PGM |
| [Snake RGB-D acquisition](snake-3d-map/extension-mission-result.json) | Same | 0.259457 m travel; 5 RTAB-Map nodes / 517 cloud points |
| [Snake installed-map goal](snake-installed-map-navigation/extension-mission-result.json) | Same | Requested [0, −0.5, 0]; actual [−0.112352, −0.494105, −0.018407], speed 0.012900 m/s, yaw rate −0.000627 rad/s, 1 simulated second settled |
| [Snake acquired-map goal](snake-acquired-map-navigation/extension-mission-result.json) | Same, protected acquired 2D map | Requested [0, −0.5, 0]; actual [0.096502, −0.456870, 0.031989], speed 0.007610 m/s, yaw rate 0.002720 rad/s, 1 simulated second settled |
| [Surface route](surface-navigation/extension-mission-result.json) | `lab_usv` / `surface_harbor` / Fortress 6.18 | Requested [−2, −2, 0] and [−2, −1, 0]; actual XY [−2.052857, −1.999557] and [−1.825465, −1.093432], speeds 0.001014 / 0.000921 m/s; settled translation and yaw rate |
| [Surface acquisition](surface-acquisition/extension-mission-result.json) | Same, earlier continuation | 0.455666 m travel; 6 database nodes / 3,438 points |
| [AUV acquisition](auv-acquisition/extension-mission-result.json) | `tethys_auv` / `marine_tank` / Fortress 6.18, earlier continuation | 2.833208 m travel; 14 database nodes / 19,515 points; actual braking/arrival observation |
| [Four flippers and Drive](tracked-flippers/flipper-result.json) | `lab_tracked_articulated` / `nav_empty` / Fortress 6.18/private DART | Four actual native joint positions approximately ±0.26987 rad for ±0.3 targets, rates ±0.00481 rad/s, 1.016 s settled; free-body forward travel 0.300499 m |

Snake goals require native action success **and** independent position ≤0.15 m,
heading ≤0.15 rad, speed ≤0.03 m/s, yaw rate ≤0.05 rad/s for 1 simulated second.
The observer derives mean-segment heading from actual body orientation and
source joint measurements. It is not a controller localization input.
Surface goals require position ≤0.2 m, speed ≤0.05 m/s and yaw rate ≤0.05 rad/s
for 1 simulated second. AUV source/coasting tolerances remain its own contract.
The flipper diagnostic requires actual joint/velocity settling and forward
body displacement; its parent `mission_result` is null because the task is an
external diagnostic, not a built-in Nav2 mission.

Every copied parent run reports `cleanup_complete: true` with no observed
surviving owned PIDs. This does not imply every descendant exits with code 0;
interrupt-time process errors remain in raw logs. Requested seed 42 is not
proof of an applied simulator seed. Contacts and swept clearance are null,
not zero. No terrain, no-contact, docking or universal backend claims follow.

`workload-producer.py` is frozen by the runner at preparation, with expected
startup identity in the specification. The tracked external producer is
preserved separately. The source manifest is **post-run**, not an immutable
snapshot of every native plant/build dependency. Generated native sessions,
real map databases/PGM/PCD files, compiled MuJoCo XML, raw telemetry and resource
logs remain in the manifest's SSD directories; they are not copied into Git.

Retained negative runs in the same SSD root include snake wrong wave/steering,
rolling-estimator rejection, clipped local costmap, missing static laser TF,
stale AMCL gating and post-arrival straightening. In particular,
`snake_navigation/135f67ad3c7846e5aec61d802b7bec16` reached native action success
inside the physical tolerance and then straightened outside it; the independent
observer correctly failed it. Shape hold fixes that fault without relaxing
the goal contract. Tracked runs `3107…` and `a1b445…` preserve the absent target
bridge and gravity servo error. Earlier surface high-CG tipping and insufficient
angular-settlement cases remain in the preceding continuation directory.

[Actual installed GUI commands](native-gui-forms.json),
[probe source](gui_forms_probe.py), [bounded run driver](run_forms.py),
[eight-package build](build-current-delivery.log),
[94 focused checks](tests-current-delivery-fixed.log) and [registry validation](registry-current-delivery-fixed.log). These checks are separate from the
named physical observations and do not refresh historical source certificates.

The expanded checks initially reported two missing fields in the old tracked
controller test fixture. Its constructor-bypassing fixture now supplies the
new explicit flipper/identity defaults; [initial failure log](tests-current-delivery.log)
and final passing log are both preserved. The [final installed rebuild](build-final-installed.log)
and [actual Tk/Health probe](gui-with-current-status.log) cover the last status
and control changes. This does not imply the full GUI suite or CI was run.

The copied failed-test text log has trailing whitespace removed for repository
formatting. Its original byte-for-byte log remains in the SSD run directory.
