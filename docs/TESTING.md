# Robot Lab Test Tiers

Separation of test tiers (task **R1.2**). The authoritative runner and
manifests live in `scripts/test_tiers.sh`; the fast PR suite is
`scripts/test_fast.sh`.

## Tier taxonomy

| Tier | Runs | Requires ROS 2? | Spawns subprocesses / physics? | Failure policy |
|---|---|---|---|---|
| `fast` | Unit, numerical, config/catalog checks | No — plain Python only | Never | Must pass everywhere, always |
| `integration` | xacro expansion of robot models, launch-contract, robot qualification suites | Yes (sourced `ros2`, `xacro`) | Spawns `xacro` subprocesses | Explicit skip if `ros2` unavailable |
| `physics` | Headless stepping of optional engines (PyBullet, MuJoCo, Isaac) | No (engines importable) | Real engine API, headless | Explicit `skip` when an engine is not installed |
| `hardware` | In-flab HIL missions (task R9) | — | Real hardware | Not yet implemented |

## Why this matters (R1.2 acceptance)

- **The unit suite runs without accessing a shared ROS graph**: `fast` tests
  never call `rclpy.init()`, never publish/subscribe, and never spawn
  subprocesses. Algorithm logic is pure math (see `DeadReckoning` split into a
  pure integrator + `DeadReckoningNode` ROS wrapper).
- **Subprocess-bearing tests are kept out of the unit tier**: robot-profile
  xacro expansion moved from `test_sim_profiles.py` to
  `test_xacro_expansion.py` (integration tier), so the fast config tests stay
  hermetic.
- **Optional engines produce explicit skips**: `test_simulator_backends.py`
  skips per-engine when `pybullet`/`mujoco`/`isaacsim` is absent.
- **Numerical assertions remain strong**; tiering only moved which runner
  executes what, not weakened any tolerance.

## Running

```bash
scripts/test_fast.sh          # fast tier + compile + registry cross-refs (default PR check)
scripts/test_tiers.sh --list            # show every test file per tier
scripts/test_tiers.sh fast              # unit tier only
scripts/test_tiers.sh physics           # engine backends (skips per engine)
bash -c 'source /opt/ros/humble/setup.bash && scripts/test_tiers.sh integration'
scripts/test_tiers.sh all               # fast + physics + integration
```