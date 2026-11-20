# Loose cargo on the MOLAR fleet deck (2026-11-20)

A shared Gazebo session can start MOLAR with a 0.50 kg, 0.20 × 0.15 ×
0.10 m reference box. The box is an independent dynamic body spawned after
the member's base controller starts. It rests on the existing stowed deck
through ordinary collision and friction. No attachment or scripted
following moves it. The original arm and belt controls remain a separate
single-robot workflow.

The fleet controller compares Gazebo's independent 3D box odometry with
the measured base pose and the deck joint transform. It requires all box
corners to remain inside the original deck's plan area, the lower surface
to be near the deck, and the deck to be stowed. The box lies inside the
existing fleet clearance envelope. Before retention is observed the
loaded member cannot drive. Loss of retention or stale feedback after
that first observation latches fleet Stop and sends a facility Stop request. Inspect
the box and restart the session to rearm; per-member reset is unavailable
because it would move the robot out from under the loose cargo.

In the Fleet editor, choose **molar_hri**, enable **Start with 0.5 kg box
on MOLAR deck**, and add or update the member. **Run** uses the generated
session and reports cargo state with the member's live feedback. The
reproducible [session](session.yaml) can also be loaded in Fleet. It uses
`fleet_facility`, one MOLAR, and a level spawn.

63 focused fleet checks passed; the utils, bringup and GUI packages built.

Two Gazebo attempts on `fleet_facility` observed the loose box retained
from initial seating until the launch was stopped:

| Run | Last MOLAR position | Retained samples | Max shift from first measured box center |
|---|---|---:|---:|
| 1 | Boarding approach, x = 3.787 m | 141/141 | 6.5 mm |
| 2 | In the car at L0, x = 3.881 m | 184/184 | 5.0 mm |

After each attempt a separate simulator started and the watcher shut down
only this launch. Neither reached a raised floor. This measures loaded
approach and boarding, with no cargo fault; loaded lift travel, the loaded
round trip, intentional cargo-loss response and operation with a fleet peer
remain to be measured. The separate [unladen round trip](../r11-molar-rmf-return-2026-11-20/README.md)
does not qualify those cases.

The [manifest](sha256.txt) records the source changes and raw paths under
`/workspace/molar/robot_lab_runtime/molar-fleet-cargo-20261120`, based on
`a585cfb8`. The runs used ROS domain 115 and independent Gazebo partitions.

## Reproduce

In both terminals, source ROS Humble, the workspace setup, `scripts/ssd_env.sh`
and `scripts/rmf_env.sh`. Use the same isolated ROS domain and a fresh SSD
output directory. Start the shared session in one terminal and the probe
in the other:

```bash
ros2 launch robot_lab_bringup shared_world.launch.py \
  session:="$PWD/docs/status/evidence/r11-molar-fleet-cargo-2026-11-20/session.yaml" \
  navigation:=true rmf:=true gui:=false rviz:=false
```

```bash
export CASE_OUTPUT="$ROBOT_LAB_RUNTIME_ROOT/molar-fleet-cargo-new"
mkdir -p "$CASE_OUTPUT"
/usr/bin/python3 -s docs/status/evidence/r11-molar-rmf-2026-11-20/roundtrip.py "$CASE_OUTPUT" 370
python3 docs/status/evidence/r11-molar-fleet-cargo-2026-11-20/check_cargo.py "$CASE_OUTPUT"
```

The probe sends two SDK patrol requests and verifies the physical pose,
stopped wheels and active map at both landings. The cargo audit also
requires the box to rise by at least 2.9 m and remain on the deck in
every recorded sample after initial retention. These checks are scoped
to the one reference payload and robot, without an active fleet peer.
