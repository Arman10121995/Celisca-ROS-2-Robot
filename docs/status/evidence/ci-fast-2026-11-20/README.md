# November 20 CI fast-tier repair

GitHub run 44811370492 on `3b285add` reached the fast tier and failed
ten cases. A prior run on `fdec0619` failed the same cases.
[GitHub run](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/44811370492).

- `terrain_stairs_flight`, `terrain_stairs_over` and `terrain_step_wide`
  ship without saved 2D grids. The map matrix now checks that Loc and Nav
  require a grid for these three fixtures. Other Bumperbot mode assertions
  still run for each terrain fixture.
- Two MuJoCo tick pacing checks import `rclpy` through the spawner.
  The Fleet transport configuration check also imports a ROS node. The
  car lookahead check imports an installed navigation launch module.
  All four assertions run in the sourced integration tier.

The unsourced map matrix passed 225 cases with 45 marine skips. The
focused sourced run passed those four checks and the existing Nav2
frame check in the same file. Full GitHub workflow verification is
pending.
