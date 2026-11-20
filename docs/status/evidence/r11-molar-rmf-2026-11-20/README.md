# MOLAR Open-RMF lift graph and cabin turn (2026-11-20)

**Implemented, experimental; upward leg measured.** On Jetson, ROS 2 Humble
and Gazebo Fortress, one MOLAR completes the SDK trip from L0 to L1,
including entry, the physical ride, map switch, in-car turn and settled
exit. The SDK return cannot yet complete. The building graph consumes the
authored lift transport layout. For `molar_hri` in `fleet_facility`, the
L0 approach is (1.75, 0) m, the car anchor is (3.95, 0) m, and the exit
destination on both floors is (2.10, 0) m. Default Bumperbot and Labbot
graphs without MOLAR keep their existing position and name conventions.
The graph uses the largest member's radius plus 0.15 m for occupancy checks.

Only a directed approach lane goes into the lift, and only a directed exit
lane comes out. Both use the SDK's native lift events. If two explicit model
layouts disagree, graph generation refuses to guess a shared layout. Native
transport continues to use its existing individual model overrides.

The SDK's in-car heading waypoint now dispatches Nav2's **Spin** behavior
when it matches the declared forward exit. The controller requires fresh
localization, measured proximity to the boarding point, the observed floor,
and open car and landing doors. It retains the original waypoint for the
physical settling check and continues using the fleet motion and Stop
gates. The scheduled Nav2 exit remains a separate waypoint.

| Check | Result |
|---|---|
| Focused graph, mission, infrastructure and transport checks | 43 pass; `robot_lab_utils` builds |
| Native Humble SDK graph probe | Plans both floor directions, each with a waypoint in the car on both floors and four SDK events |
| Gazebo run 1, graph fix before Spin dispatch | MOLAR boarded at x ≈ 3.87 m, rode to L1 and switched to `map_L1.yaml`. It repeatedly turned while trying to reacquire the in-car XY waypoint. The probe stopped before a settled floor arrival |
| Gazebo run 2, with Spin dispatch | Stopped at startup when another agent started a hand simulation. No motion verdict |
| Gazebo run 3, with Spin dispatch | Upward leg settles on L1 at (2.1467, 0.0031, 3.0000) m, yaw −3.127 rad and zero measured rates, 114.9 wall s from probe start. The turn moved the center by about 0.10 mm between the first and last observed samples. The return stalls at (2.142, 0.004, 3.0) m while turning before the closed L1 gate. The scan footprint guard holds zero motion; the operator stops the probe |
| Farther L1 approach candidate (x = 1.75 m) | Rejected by the graph clearance check: 0.70 m bitmap clearance versus the 0.64 m fleet profile plus a 0.071 m raster margin. Reverted without changing the map or guard |

The SDK inserts a turn toward the closed gate before its door event on the
return. The native transport opens that gate before sending the boarding goal.
Continue with a source-reviewed pre-turn admission step that preserves RMF
lift ownership, or a wider landing with regenerated collision and occupancy
geometry. Keep the scan guard and the independent arrival check unchanged.
A mixed fleet and payload carriage through the lift remain separate.
The [native fleet transport receipt](../r11-molar-transport-2026-11-20/README.md)
is a different workflow.

## Reproduce

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
source scripts/rmf_env.sh
export ROS_DOMAIN_ID=115
export CASE_OUTPUT="$ROBOT_LAB_RUNTIME_ROOT/rmf-molar-new"
mkdir -p "$CASE_OUTPUT"
/usr/bin/python3 -s docs/status/evidence/r11-molar-rmf-2026-11-20/sdk_probe.py "$CASE_OUTPUT"
```

With those environments in both terminals, run the launch and then the
measurement probe. The probe sends each SDK patrol separately so the
intermediate floor arrival can be independently measured.

```bash
ros2 launch robot_lab_bringup shared_world.launch.py \
  session:="$PWD/docs/status/evidence/r11-molar-transport-2026-11-20/molar-transport-session.yaml" \
  navigation:=true rmf:=true gui:=false rviz:=false
/usr/bin/python3 -s docs/status/evidence/r11-molar-rmf-2026-11-20/roundtrip.py "$CASE_OUTPUT" 330
```

Runtime baseline was `431945df` plus the scoped changes in [the manifest](sha256.txt).
Raw launch logs, plans, floor maps, session references and independent body
traces are on the SSD under
`/workspace/molar/robot_lab_runtime/rmf-molar-20261120`.
