# R14.1 source counterparts — November 12, 2026

## TurtleBot4 Lite cleaning derivative

`asset_turtlebot4_lite_cleaning` is a separate installed variant of the pinned official
TurtleBot4/Create3 description. Original profiles and certificates are unchanged.
Source chassis, wheels, spring suspension and hazard geometry are preserved. The lab
restores the source bumper plus four native GPU cliff scans at 20 Hz, bridges their
actual source names and pauses cleaning on missing/stale cliff feedback. The 0.08 m
cliff threshold comes from the source Create3 sensor contract. Cleaning, battery and
coverage remain laboratory models; the robot has no physical vacuum brush.

Actual run: Gazebo Fortress, `nav_empty`, source compliant suspension with a 2 ms
maximum step, AMCL/Nav2 and one zone `[-1,-1,0.2,0.2]`, detector disabled. It completed
and docked: 96.8% reported target coverage, 2.08 m² reported cleaned area, 13.2 m
reported travel, 99.6 s cleaning duration. One stall was detected and recovered.
All four physical cliff scans were fresh (0.04 s age) at the end. Bumper advertised,
but no contact occurred; this does **not** qualify bumper impact detection.

[Measured summary](turtlebot4-cleaning.json) preserves independent body samples and
final truth separately from estimated pose. [Source receipt](turtlebot4-provision.json)
pins original/derived bytes and dependencies. Raw trajectory/logs remain on the SSD:
`/workspace/molar/robot_lab_runtime/r14-implementation-20261112/`.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_robot_counterparts.py --robot turtlebot4_cleaning
ROS_DOMAIN_ID=144 TIMEOUT=220 bash scripts/sim_vacuum_mission.sh gazebo \
  asset_turtlebot4_lite_cleaning nav_empty /workspace/molar/robot_lab_runtime/new-cleaning-run.json \
  '{"mode":"zones","zones":[[-1,-1,0.2,0.2]],"edge":false}' \
  allow_experimental_assets:=true vacuum_detector:=none
```

The GUI Vacuum Setup exposes the installed source variant and an explicit experimental
opt-in; launch autofills the command and synchronizes the session mode. Fresh runs keep
the shared installed assets visible while isolating their cleaning history directory.
Other maps/backends, bumper contact, cliff edges and learned-camera cleaning remain
unqualified. This is one measured zone mission, not completion of R14.1.
