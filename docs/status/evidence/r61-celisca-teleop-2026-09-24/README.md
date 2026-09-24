# Celisca size and GUI drive (2026-09-24)

The 2026-09-24 `a4cfa49` resize corrected a roughly 3× oversized Celisca
building, but the user found the resulting display slightly too small. This
change enlarges all six Celisca meshes by 10%: plain and actor worlds use
`1.1`, furnished worlds use `0.6325`. The Gazebo floor grows from 40 × 25 to
44 × 27.5 m and MuJoCo's generated plane follows it. The occupancy-grid
resolution and origin, floor-2 furniture offset, and spawn/initial poses are
scaled by the same factor. The PGM pixels and robot geometry are unchanged.

The GUI Drive pad now publishes `/key_vel` at 10 Hz. Its two displayed values
are **increments per 0.1 s** (default 0.025 m/s and 0.08 rad/s), rather than
the final commanded speed. Holding a pad button, WASD key, or Linux joystick
axis ramps the command toward 1 m/s and 2 rad/s caps; releasing it ramps down.
The Stop button zeros the command immediately and disables keyboard/joystick
input. The checkbox enables WASD and the first readable `/dev/input/js*`;
keyboard input is ignored while typing into a text or selection control.

Checks on this source tree:

- `gen_mjcf_worlds.py --check`: all generated worlds current.
- `test_mjcf_worlds.py` and `test_sim_profiles.py`: 205 passed, including
  actual PGM spawn clearance for all six Celisca maps.
- GUI command and drive tests: 27 passed under Xvfb.
- MuJoCo Bumperbot, furnished floor 1, 1.5 m Nav2 goal: **timed out** at
  120 wall seconds after moving 0.22 m. The stack localized at `(0, 1.1)`
  and accepted the goal, but this slow run is not navigation success.
- PyBullet Bumperbot, same furnished map, 0.75 m Nav2 goal: **succeeded**;
  final simulator truth `(0.49, 1.09)`, map-frame goal error 0.254 m.

Both live checks used `scripts/sim_nav_check.sh` in isolated ROS domains,
headless, with RViz off. Their JSON results and full launch logs are here.
No physical joystick was connected during testing; its Linux event decoder
was exercised with synthetic axis events and the GUI ramp was exercised in Tk.
