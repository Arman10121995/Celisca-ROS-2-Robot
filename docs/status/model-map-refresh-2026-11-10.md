# R11.1 — MOLAR/CELISCA model refresh

Developed natively on molar1; source `/workspace/molar_voice`, active `jetson_devel`.
Do not overwrite the independently owned cleaning work or claim broader R11 acceptance.

The complete MOLAR and ProBOT-L variants now use the photo-informed hood, covered
wheels and tower controls, with visible tyres rotating on the actual wheel joints.
The manual transfer interval is 0.440–1.100 m; the photo-based raised resting pose
is 0.750 m. The controlled lift offset is −0.310..+0.350 m. Executed Fortress
hard stops retain the existing 5 mm lower margin so the servo can leave the stop.
Upper plate, fasteners, load stops and orange guards follow the coupled top slide;
lower carriage stays fixed horizontally. Two 0.140 m slides remain estimates.

`provision_molar_hri.py` now refreshes the existing profiles instead of silently
leaving an earlier profile installed. GUI slider limits and displayed deck height
use this same rest-relative reference. The original 14 moving joints remain;
three fixed scanner/barcode frames and, for MOLAR, two camera frames are added.

| Frame | Parent and translation (m) | Status |
| --- | --- | --- |
| laser_link | base_link: .340,0,.130 | CAD optical window; manual nominal scan plane about .120, survey pending |
| rear_scanner_link | base_link: −.302,0,.085; yaw pi | Descriptive frame only, no second simulated scan stream |
| barcode_link | conveyor_carriage: −.0511,0,−.033 | Follows carriage; no simulated barcode recognizer |
| camera_link | base_link: .2704,0,1.649 | MOLAR CAD RealSense body centre, not a measured extrinsic |
| camera_optical_link | camera_link; RPY −pi/2,0,−pi/2 | Optical +Z along body +X |

MOLAR has a simulated 640×480, 10 Hz RGB-D stream, 1.211 rad horizontal FOV,
0.1–10 m clipping. Scan settings are 540 samples, 10 Hz, 0.05–12 m. These
simulation settings are explicit estimates. The existing cross-backend scan
contract remains 360°; it does not represent the hardware scanner's protective
zones/certified field. Sensor-frame numerical inertials are 1e−6 kg to prevent
importers silently adding 1 kg per frame; aggregate hardware mass remains estimated.
No IMU hardware or calibrated odometry has been invented. Modes remain Display/
Drive; SLAM/Nav2 missions and calibrated sensor acceptance remain separate.

CELISCA detailed F1/F2 retain original building floors 2/3, source transforms,
full furniture and bathroom visuals, and existing checked spawn points. The new
8-part charging cabinet appears twice on floor 2 (Charge_01/02) and once on floor 3
(Charge_03), with conservative cabinet/contact colliders. Offset from cached FMS
robot docking pose to cabinet centre is 0.93 m, an un-surveyed illustration.
Native full-detail visual caches are content-addressed, and original source hash
pins are unchanged. The old generated map trees/MJCF were preserved under
`/workspace/molar/robot_lab_runtime/r11_1_asset_refresh_20261110/before-*`.

## Reproduce / checks

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
python3 scripts/provision_molar_hri.py --source /workspace/molar_voice/simulation/robot
# Existing map refresh requires clean generated predecessors; first back them up:
python3 scripts/provision_celisca_detailed.py --source /workspace/molar_voice/simulation/celisca --refresh-reviewed-assets --skip-register
colcon build --packages-select robot_lab_robots robot_lab_maps robot_lab_bringup robot_lab_registry robot_lab_gui --symlink-install --executor sequential
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 xvfb-run -a python3 -m pytest src/robot_lab_robots/test/test_molar_asset_refresh.py src/robot_lab_bringup/test/test_molar_robot.py src/robot_lab_gui/test/test_molar_controls.py -q
```

62 native model/frame/installed-source/GUI/PyBullet-FK tests passed. Builds passed for the
five affected packages. HRI per-host tests and hash/settings receipts are in
`/workspace/molar_voice/tmp/*model_map*20261110*` (59 passed, 7 optional-tool skips
on each Jetson). Windows separately executed MuJoCo/PyBullet/OpenUSD tests;
OpenUSD schema checks do not establish Isaac PhysX runtime acceptance.

The bounded normal-GUI PyBullet/nav_empty run **passed** in isolated ROS domain
169: forward 0.627 m; simultaneous forward/turn 0.479 m with 0.725 rad yaw;
lift offset 0.350 m (1.100 m surface), both stages 0.140 m, total tip 0.280 m;
roller 2.000 rad/s; extended-base inhibition, Stop and watchdog; clean launch exit.
Producer, complete report, raw trace and screenshot are under runtime
`r11_1_asset_refresh_20261110/gui_pybullet_v3`.

Native PyBullet forward-kinematics checks also verify the installed model at
its **0.440 m minimum**, 1.100 m maximum, and 0.280 m nested extension.
The MuJoCo normal-GUI run reached those upper lift/slide limits and published
scan/RGB/depth/calibration streams. `sensor_streams_mujoco.json` records actual
frame IDs, dimensions and message counts, using the normal Robot Lab `/oakd/`
topic aliases for this CAD-mounted RealSense simulation. This topic alias does
not claim physical OAK-D hardware.

The first MuJoCo end-to-end run **did not pass**: a fresh-feedback guard stopped
the return descent at 0.188 m offset during concurrent heavy software rendering.
No guard was weakened. Preserve its trace under `gui_mujoco`. A control-only
repeat uses domain171 / partition `molar_r11_1_mj2_20261110`, no extra RViz window,
and retains simulated camera rendering, exact model identity, Stop and heartbeat
gates. Final result must be read from `gui_mujoco_v2/report.json`.
Gazebo/Isaac runtime missions, sensor calibration and broader R11 remain open.
Earlier probes also exposed the old GUI lift envelope and a test harness that
did not select the redesigned Launch workspace; preserve that failure evidence.

No physical robot, FMS/AIC request, door, lift, charger or microphone was operated.

## Final result

The control-only **MuJoCo repeat passed**: lift offset 0.350 m, both slides
0.140 m, total 0.280 m; Stop held position, watchdog stopped roller, and the
return to rest passed. The owned launch exited cleanly. It used the normal
GUI Start/control workflow with `start_rviz:=false`; simulated camera rendering
remained active. The earlier heavy-render feedback hold remains valid failure
evidence, not erased or counted as a pass. No heartbeat/freshness/identity
check was relaxed. Exact model/controller/map provenance and bounded results
are in `model-map-refresh-2026-11-10.json`. R11.1 asset-refresh acceptance is
complete; broader R11 navigation and Gazebo/Isaac runtime missions remain separate.
