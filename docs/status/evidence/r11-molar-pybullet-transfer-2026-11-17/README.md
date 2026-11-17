# R11 MOLAR payload transfer on PyBullet: not passed (November 17, 2026)

**Status:** the PyBullet deck belt is implemented and unit-tested; the
MOLAR transfer mission on PyBullet does **not** pass yet.

## Setup

`scripts/molar_station_transfer_check.py --simulator pybullet` (new mode)
writes a `nav_empty` SDF copy containing:

- the payload as a loose 0.5 kg box on the deck;
- the station as a static 0.05-friction box at the extended deck end.

Both are placed from the Gazebo-measured deck geometry, as for MuJoCo, and
the launch uses `world_path:=`. Truth comes from `/simulation/free_bodies`
and `/odom/ground_truth`.

**The belt** (`robot_lab_pybullet/pybullet_belt.py`, using the shared
`robot_lab_utils/belt.py`) attaches to `telescope_stage2` and the payload.

## What happens (runs 1–4)

**The lift and the full telescope extension work.** The payload rides
along on the deck.

**The roller command is refused.** The articulation contract is in
`fault/ownership/watchdog hold` with no fault set, and the roller target
stays 0:

| Run | Contract base-motion limits | Base speed peak at the roller command | Result |
|---|---|---|---|
| 3 | 0.025 m/s, 0.05 rad/s (default) | 0.022 m/s, 0.018 rad/s at `base_link`; the contract's centre-of-mass check tripped | refused |
| 4 | 0.05 m/s, 0.10 rad/s (trial) | 0.035 m/s, 0.127 rad/s | refused |

**Root cause:** starting the roller rocks and yaws MOLAR's base in
PyBullet, and the base-motion interlock correctly holds the articulation.

**Not kept:** the trial limits were reverted. Raising them further would
only weaken the interlock.

**Next:** find why PyBullet's MOLAR base yaws at roller start (the roller
motor's reaction, or caster damping), instead of loosening the interlock.

Gazebo (3/3) and MuJoCo (4/4) transfers from 2026-11-15 are unchanged.
