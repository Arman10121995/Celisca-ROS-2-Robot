# Camera-derived manipulation targets — November 14, 2026

R5.9 now has a camera-marker pose input and a source/scene/plant-bound native
task consumer. The GUI offers Simulator geometry or Camera fiducials for
unheld object reaches. [Operator procedure](../tutorials/camera-object-targets.md).

The detector consumes only RGB pixels, CameraInfo, image-time camera TF,
declared marker/object geometry and task identity. It does not read the
simulator object pose. OpenCV detection and planar IPPE recover marker pose;
the known rigid marker mount produces the object pose in `native_world`.
Reprojection, image size, conditioning, ambiguity, uncertainty, identity,
source geometry and both simulation/wall-time freshness are checked.
RGB and calibration can arrive in either order; one latest pending image is
paired without an unbounded queue. Duplicate marker IDs are refused.

The existing lab fixture now has a 24 mm printed marker on its unchanged
35 g, 30 mm cube, with a collision-free, zero-mass visual. Marker UVs initially
mirrored the code; that failure is retained. The corrected actual MuJoCo render
detects marker 7 and measures **0.002523 m** object position error. The pose
solver receives the rendered pixels and camera calibration/transform; the
known object pose is used only afterward for error comparison. This is a
rendered fixture, not a complete source-robot manipulation mission.

A separate actual ROS image/CameraInfo/TF transport test uses explicitly
synthetic perspective-projected pixels and checks the resulting estimate.
Fifteen initial camera/GUI/source checks pass; a later 28-check regression
batch includes the RGB-before-CameraInfo case and the existing native source
kinematic/scene contracts. Four changed packages build. Exact commands/logs
and rendered artifacts remain on SSD under
`/workspace/molar/robot_lab_runtime/manipulation-camera-20261114/` and the
overlapping source checks under `panda-object-task-20261114/`.

Only unheld approach/grasp point targets use this camera estimate in the local
pick recipe. Instrumented lift/contact/custody and collision checking retain
their declared native measurements. Missing perception never switches to a
truth target. The 0.5 px uncertainty floor is a stated model assumption;
camera/marker calibration accuracy is not inferred from reprojection RMS.

This completes the initial fiducial target integration, not unmarked object
recognition, a camera-only full grasp mission or a second mobile-manipulator
source. Those remain in ROADMAP.

[Rendered fixture, measured estimate and source manifest](evidence/manipulation-2026-11-14/README.md).
