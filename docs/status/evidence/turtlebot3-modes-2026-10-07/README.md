# TurtleBot3 localization, mapping and navigation, October 7

Official Burger, Waffle and Waffle Pi pass **48 actual named mode screens**:
each robot on Gazebo Fortress, MuJoCo, PyBullet and native Isaac runs
localization/reset/resume in `nav_empty`, 2D SLAM/reset/resume with a real GUI
map save in `nav_empty`, and Nav2 in both `nav_empty` and `nav_obstacle`.
The original source LDS has 360 rays, 0.12–3.5 m range and a 5 Hz lab cadence.
The separate [twelve Drive/source-sensor screens](../turtlebot3-sensors-2026-10-07/README.md)
remain distinct evidence. No RGB-D stream or 3D SLAM is fabricated.

The actual Tk producers select robot/backend/map/mode, inspect compatible
algorithm defaults and the filled command, Run, physically drive or give an
RViz-style goal, then Stop/Close. The qualification catalog references the
same executed URDF, drive/sensor/spawn/runtime settings as the normal install.
Independent engine body truth validates movement, both turns, publisher loss,
monotonic reset and resumed estimator/mapping feedback. All twelve SLAM
exports contain actual YAML/PGM data and exit cleanly.

Navigation retains fresh physical final limits **0.15 m / 5°** after one
simulation second of settling and **0.02 m** minimum swept static clearance.
Obstacle screens require a blocked direct path and an actual detour. Burger's
envelope is 0.185 m; Waffle/Pi use 0.26 m. Every final producer, launch root
and checker exits zero; owned plant shutdown is checked in each producer.
These are bounded routes, not repeated-goal or contact-force qualification.

Initial Waffle Pi MuJoCo and Isaac obstacle routes aborted in lethal costmap
space; the source-stage reports and traces remain under `negative/nav-stage1`
on the SSD. The body had static clearance but the route skirted obstacles
with approximately 0.10 m localization offset. Pi's profile-local inflation
was enlarged from 0.35 to 0.65 m; all eight Pi clear/obstacle navigation screens
were rerun and pass the unchanged physical limits. Earlier passing 0.35 m
routes also remain historical. No outcome flag was edited to pass.

[Final batch](modes-final-batch.json), per-cell real reports/measured workflows,
pre-trial source/installed hashes, executed model/controller hashes and
[artifact manifest](manifest.json) record the exact stages. The four batch
logs retain initial failures and later measured reruns. The
[collector](collect_modes.py) validates numeric reports and does not run or
simulate missions. Named world/map bytes omitted from the initial source
sweep are checked against recorded HEAD and installed bytes at collection;
generation fingerprints captured at collection are labeled accordingly.

Only after these reports pass are Display, Localization, SLAM and Navigation
enabled in normal Launch on all four engines. Actual
[normal GUI selection proof](normal-gui/report.json) checks all 48 robot/
backend/mode/default/autofill combinations. A separate normal consolidated-
GUI Waffle Pi MuJoCo obstacle repeat measures **0.009 m / -0.75°**, exits zero
and is archived under `normal-current-waffle-pi-mujoco-obstacle`. Native preview
with a live Burger/PyBullet plant is separately measured in
[embedded viewer evidence](../registry-3d-2026-10-07/README.md).

Runtime guards cover executed geometry, drive/sensor/spawn configuration,
controllers, backend stepping and actual estimator/algorithm settings. Exact
GUI snapshots/commands remain in each report; presentation-only catalog/3D
changes are followed by separate normal-GUI and physical checks. The full
launcher hash is not a TurtleBot3 mode guard. This does not transfer a
controller certificate between native and URDF source alternatives.

Other maps, longer/repeated routes, noisy hardware localization, original RGB
rendering, OpenCR firmware, docking and 3D SLAM remain open. Non-Gazebo
estimation uses ideal engine odometry; swept static geometry clearance does
not establish absence of measured contacts. Raw traces, failed stages and
source snapshots remain on the SSD at the manifest paths.
