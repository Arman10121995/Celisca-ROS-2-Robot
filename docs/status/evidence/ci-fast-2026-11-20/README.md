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

The complete unsourced fast tier passed 1,502 tests with 64 skips and
eight deselections. The focused sourced run passed the four moved checks
and the existing Nav2 frame check. The complete headless integration tier
passed 345 tests with 30 skips using the CI-pinned MAVLink package.
The Fleet queue fixture now binds the production per-model transport
configuration helper. The architecture layout and native 3D framebuffer
checks both passed separately under Xvfb.
[Published repair run](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/44815298868).

The later hosted run passed the fast tier and reached five failures in
the MuJoCo effort-budget fixture. The fixture now supplies the coupling
and actuator lookup used by the runtime. It checks the existing independent
joint and a passive follower whose bias must count toward the same shaft
effort limit. The full local physics tier passed 257 tests.
[Hosted failure](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/44818951392).
