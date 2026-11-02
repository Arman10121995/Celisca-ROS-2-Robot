# Fleet lift and floor-map transition

Recorded November 2, 2026 on `master`, base revision `b9bd4ca4ebe148e9569a7a379189ae8eff735467` plus the uncommitted changes listed in the worktree. This is one experimental Gazebo Fortress 6.18 Bumperbot case, not full R10.2 qualification.

## Measured case

- Session: `fleet_facility`, one `alpha` Bumperbot, spawn `[2.5, 0.0, 0.1]`, yaw `0`; generated session `session-gzfvva8b`.
- Physical sequence: stationary spawn/neutral, explicit lift request to L0, actual car-door open, bounded `0.12 m/s` manual boarding command under the existing fleet watchdog, measured settling, then L0 → L1 → L0. No entity-pose writes or simulated pose adjustments were used.
- L1 arrival: Gazebo body observation `[3.662950, -0.000021, 2.984001] m`, speed `0`; robot floor `L1`; active map `map_L1.yaml`; map status `ready`.
- L0 return: Gazebo body observation `[3.662950, -0.000021, 0.000002] m`, speed `0`; robot floor `L0`; active map `map.yaml`; map status `ready`.
- Total trial wall time: `70.23 s`. Result: actual body rode L0-L1-L0; observed robot floor and namespaced Nav2 map changed at both landings.
- Battery energy remained unknown. No charging or motor-power acceptance is claimed.

## Implementation exercised

The infrastructure adapter classifies each robot's floor from fresh body observations; lift joint position alone does not move its cached body pose. Per-member Nav2 map servers load the floor map asynchronously, global costmaps subscribe to that member's map, and that member remains command-held until the map service succeeds. The adapter publishes the floor-specific `map -> <member>/odom` transform after the measured transition.

The generator now provides a traversable L0 lift platform and L1 landing with planar map origin, places the lift-door panel east of the static shaft edge, narrows it inside the cabin, and uses the measured `2.15 m` open target. The lift-door controller uses a higher position gain while retaining the `300 N` effort and `0.25 m/s` velocity limits. The closed-door safety interlock was not weakened.

## Reproduction and artifacts

Start the isolated world and trial with:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=221
export GZ_PARTITION=robot_lab_floorcheck_single_20261102_i
export IGN_PARTITION="$GZ_PARTITION"
ros2 launch robot_lab_bringup shared_world.launch.py \
  session:=docs/status/evidence/fleet-floor-map-transition-2026-11-02/floor-map-session.yaml \
  navigation:=true gui:=false rviz:=false
```

In a second sourced shell, set `ROBOT_LAB_EXPERIMENT_OUTPUT` to an SSD run directory and execute `floor_map_trial.py` from this folder. The successful raw `trace.jsonl`, `mission-result.json`, and launch log are retained outside Git at `/workspace/molar/robot_lab_runtime/fleet-floor-map-live-oWr3d4/mission/` and `/workspace/molar/robot_lab_runtime/fleet-floor-map-live-oWr3d4/launch.log`.

Resolved session source SHA-256: `fe28859d63e99cbfd2f7ecbf2f2dcfde10bcffbd629d4983451fe2c09c1f6ed8`. Installed world SHA-256: `9a586431de76b033490824c800d61f4814709d20e40548c31efca92b68e7710a`. Trial driver SHA-256: `019c26230c4b55aa1f7349afa7a04363f5b15192807e406cd437bc1b94c65536`.

Focused verification: `test_fleet_infrastructure.py` 11 passed; `test_fleet_contract.py` and `test_fleet_missions.py` 22 passed; `robot_lab_maps`, `robot_lab_utils`, `robot_lab_bringup`, and `robot_lab_gui` built successfully. No full CI or clean-host run was performed.

## Retained failures and limits

Earlier runs are preserved under the SSD `fleet-floor-map-live-*` directories. They include the L0 map initially marking the shaft occupied, the L1 map yaw incorrectly set to `3`, controller/map startup failures from overlapping process groups, and lift-door stalls caused by panel overlap with the shaft floor/cabin walls. These failures motivated the present source fixes; they are not successful mission evidence.

The case does not qualify robot transport for another model, multiple robots, additional floors, RMF negotiation/conflict/loss, charging, battery energy, contact instrumentation, or other backends. R10.2 remains partial.
