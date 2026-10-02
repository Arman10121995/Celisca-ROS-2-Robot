# FUJI mecanum collision assets

Source: https://github.com/DaiGuard/fuji_mecanum
Revision: `646431a5e107448e0e2bdeeba4575db9aa3665ff`
License: MIT (copyright 2022 DaiGuard), preserved in LICENSE.

`roller.stl` and `wheel.stl` are unmodified upstream collision meshes.
The adjacent `urdf/mecanum_rollers.xacro` adapts the upstream
`rollers.xacro` joint placement and handedness, scales the 205 mm wheel to
100 mm diameter, and removes ROS 1 transmissions. Each of the 15 rollers
remains a physical passive joint. Wheel axes are rotated from upstream x
to Robot Lab's y axis. The four driven hubs keep their existing joint names.

No Gazebo-only velocity/pose override implements sideways movement.
All backends import the same roller collision model. Live acceptance is
recorded separately from model availability.


`roller_collision.stl` is a convex contact proxy derived from the upstream
roller (MIT). Its 68 faces replace 5,844 triangles per passive roller, while
keeping its bounds. With seed 5402, 4,000 support directions measure a maximum
inward surface error of 0.4193 mm at the robot's 100 mm wheel scale. The full
upstream mesh remains the visual. This follows the convex interpretation
already used by the other rigid-body importers; lateral motion must still be
verified per backend. Recreate it with `scripts/simplify_mecanum_collision.py`.
