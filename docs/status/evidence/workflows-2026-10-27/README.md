# Native extension workflow evidence — October 27, 2026

Raw SSD root: `/workspace/molar/robot_lab_runtime/extension-completion-20261027-au5q8yp7`.
Work started at `f75dc60` on `master`; checkpoints `8e411cb` and `a224a85`
recorded intermediate implementation. Subsequent source publication is captured
in `source-manifest.json`. Existing historical source certificates are retained.

[Machine-readable measurements](measurements.json),
[actual fleet result](fleet-result.json),
[native GUI checks](native-gui.json),
[current status and scope](../../workflows-2026-10-27.md).

## Actual observations

| Workflow | Actual artifact / result | Boundary |
|---|---|---|
| Catalog | Seven restored IDs; 147 robot profiles / 78 selectable families; 51 worlds / 39 families; native GUI produces candidate commands with default algorithms | Resource/default-command checks; changed-source missions remain experimental |
| Fleet conflict | `fleet-conflict-061825`: both opposing operator tasks succeed after bounded traffic yielding/resumption | Two bodies, Gazebo/nav_empty; no inferred contact-free or coverage-completeness claim |
| RMF | `fleet-native-061035`: actual operator endpoints and several RMF waypoint terminals/progress | Full conflicting/infrastructure RMF acceptance remains open |
| Real fleet batch | `fleet-workload-110841/8cb98a89e9f7450db982f4b2e630b1b8`: both actual goals complete; independent endpoint/speed/one-simulated-second settling contract passes; 2,737 truth samples per body; owned cleanup leaves no surviving PIDs | Contact, swept clearance and communication-loss metrics remain unassessed |
| Aerial | `aerial-native-113233`: distinct FCUs 1/2, UDP 14580/14581; actual takeoff, manual/group movement and landed/disarmed endpoints | Measured centre distance stays at least 2.4149 m. During group motion body height ranges 2.834–3.120 m / 3.719–4.344 m; maximum FCU/body height differences 0.261 / 0.473 m. Altitude/formation/convergence and loss acceptance remain open |
| ROV map | `marine-slam-105919`: actual RGB-D RTAB-Map backup exports a 9-node, 1,196,032-byte database and 17,202 acquired points | Native Fortress/tank; this is an actual populated map, not a catalog or process-start claim |
| ROV reload/route | `marine-reload-111205`: protected source database unchanged, valid localization, completed estimated-pose route and independent body trace | Source-world XYZ endpoint difference is 0.0851 m for this diagnostic; broad localization/contact/obstacle/current acceptance remains separate |
| AUV | `marine-auv-115308`: actual source propeller/fin plant, RGB-D data, forward transect and signed-thrust braking reach completion | Body endpoint difference to target XYZ is 0.4012 m within the existing 0.6 m AUV arrival envelope. This is a forward cruise task, not arbitrary stop-and-pivot navigation. Subsequent zero-thrust/coast behavior has its own physical limits |
| Hexapod map | `gait-slam-073808`: actual walk and acquired occupancy export | Horizontal body and contact-odometry travel agree within approximately 2 mm over this short straight diagnostic; terrain/slip/recovery unqualified |
| Hexapod navigation | `gait-nav-110409`: acquired map reloaded unchanged, AMCL/Nav2 runs, actual Nav2 result is status 4 with about 0.306 m body travel | A Nav2 terminal does not establish every physical arrival/terrain contract |
| Octopod | `gait-3d-111321`: actual motion/RGB-D produces a flushed 9-node, 704,512-byte 3D database and 340 acquired occupied points | Acquired map export; saved localization/navigation and coverage completeness remain separate |
| Centipede | `gait-centipede-115443`: actual twelve-leg motion, contact odometry and occupancy Save; last grid observation has 32,787 known cells | Flat nav_empty mapping; terrain, recovery, full navigation and other backends remain open |

The endpoint-coordinate comparisons use the configured source-world/map prior;
they are diagnostic comparisons, not general localization RMSE certificates.
Raw reports, states, source subsets, commands and file hashes are indexed in
`measurements.json`. It also lists the retained trial archive, including failures.

## Defects retained and corrected

- Full/scoped provisioning omitted installed models. Merge preservation and
  entry-scoped repair restore profiles without silently refreshing certificates.
- Fleet spawn vertices merged onto one RMF location; progress and scheduled
  departures/replanning were incomplete. Conflicting peers lacked a checked
  retreat. The repaired runtime retains the original task through bounded yield.
- Aerial Check acknowledged asynchronous job admission before geometry finished.
  FCU startup height changes invalidated seals. The worker now waits for actual
  clearance and landed-velocity/FCU settling; startup frame calibration is
  explicit, and flight integration follows FCU time. Earlier altitude-drift
  traces remain. ROS cleanup exceptions previously skipped PX4 process cleanup.
- ROV body callbacks did not store the observed pose; DVL initialization/motion
  feedback had been disconnected and sonar published zero velocity samples.
  These regressions are repaired. The named native DVL plugin is unavailable in
  Fortress; sensing is explicitly emulated from the actual plant.
- Map export copied an unflushed database. Saving now waits for RTAB-Map's
  actual backup reply and verifies populated SQLite content and acquired cloud.
- AUV IMU yaw and physical spawn yaw differed by pi; camera clouds/DVL travel
  were rotated. The estimator now uses one configured heading alignment.
  Copied-world texture paths are preserved. Ambiguous registration no longer
  pulls the AUV sideways; pure geometric registration rejects degenerate ICP.
- Passive AUV coast overshot the original endpoint envelope. Arrival now keeps
  the source velocity controller active for bounded signed-thrust braking before
  Stop. The operator's Stop still removes force immediately.
- Gait KDL solvers lost their referenced chains, yielding zero estimated travel.
  Chains are retained and FK failures suspend output. SLAM and navigation
  costmaps now both subscribe to the correct namespaced acquired map.
- Two diagnostics briefly shared a ROS clock and were stopped/restarted in
  separate domains. An abandoned owned Fortress process from that interruption
  was identified by partition/group/source log and terminated. Published
  producers reuse the GUI's owned-launch cleanup, which also handles an exited
  wrapper with surviving children.

No failed logs were removed or rewritten into successes. Older map-save,
takeoff-check, locking, odometry, costmap and AUV arrival failures remain on SSD.

## Reproduction and source boundaries

Use `source scripts/ssd_env.sh` and `source install/setup.bash`. Set
`LAB_EXTENSION_RUN` to a new persistent SSD directory and use separate ROS
domains/Gazebo partitions for independent sessions. The published
[diagnostic entry points](producers) require the matching installed packages
and reference explicit saved-map paths where applicable. Set
`LAB_GAIT_MAP` / `LAB_SAVED_MARINE_DATABASE` to acquired artifacts for reloads.

Native controls are described in the [operator status](../../workflows-2026-10-27.md).
The fleet workload can run through **Experiments → Fleet point goals**;
its result binds actual owned tasks, independent body samples, producer,
specification, trace and cleanup. The old simulated-completion producer is gone.

Published producer scripts include later cleanup/diagnostic improvements; raw
run files preserve the executed commands and captured source subsets. The
publication source inventory does not imply that every earlier run executed
every final byte. In particular, later sensor-reference/braking and GUI status
changes do not refresh earlier certificates. These are scoped development
measurements; full roadmap support cells remain partial until their exact
source/model/map/backend/task acceptance is met.

The seven changed packages build. The final focused check log records **62
passing checks**; native GUI checks separately cover restored-model defaults,
autofill, aerial command, ahead-waypoint axes and real fleet workload parsing.
These software checks are separate from the physical rows above.

Upstream interfaces reused include [RTAB-Map's node/backup API](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_slam/doc/rtabmap.md),
its [calibrated depth projector](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_util/src/nodelets/point_cloud_xyz.cpp),
and the [official Open-RMF fleet adapter pattern](https://github.com/open-rmf/fleet_adapter_template).
