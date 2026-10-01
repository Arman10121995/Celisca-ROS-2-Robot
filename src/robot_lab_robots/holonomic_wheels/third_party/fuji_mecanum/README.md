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
