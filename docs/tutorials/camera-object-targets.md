# Camera-derived object targets

## Run

The native Stretch task builder can use a printed camera marker to locate a
known box, alongside its existing simulator-geometry targets. This uses
[OpenCV ArUco and calibrated PnP](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).
It is a fiducial workflow; unmarked object recognition remains separate.

Provision the bench fixture and rebuild the changed packages:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_mobile_manipulation_scene.py
colcon build --packages-select robot_lab_utils robot_lab_mujoco robot_lab_bringup robot_lab_gui
source install/setup.bash
ros2 run robot_lab_gui robot_lab_gui
```

Select original Stretch or Stretch 3, MuJoCo, Display and
`mobile_manipulation_lab`; enable Native mobile controls. Run the autofilled
command. The map's marker binding starts the detector automatically and uses
a 640×480 camera; other worlds retain their prior camera dimensions. The
original 35 g/30 mm cube now carries a non-colliding printed top marker in the
MuJoCo fixture. Robot mechanics and the cube's physical mass are unchanged.
Gazebo's fixture remains unmarked; this camera-target integration is MuJoCo.

Aim the source camera at the cube using the native head controls. In the
mobile task panel choose the free cube and opposing finger pads, then choose
**Camera fiducials** under **Unheld object reach targets**. Wait for the named
camera target. Add an object reach or local pick recipe, review it, then Run.
Editing these controls only builds steps. An invisible, clipped, small,
ambiguous or stale marker cannot silently fall back to simulator geometry.

In the pick recipe, the approach and grasp target use fresh camera estimates.
The lift after closing retains the instrumented native geometry/contact
workflow; the GUI task records this distinction. Grasp, support, slip,
collision checking and payload receipts still use their measured plant
contracts. This is not a camera-only grasp or localization certificate.

## Inputs and association

`fiducial_object_pose.py` reads raw RGB/BGR/mono images, matching CameraInfo,
image-time TF to `native_world`, and source/scene/plant identity from
`/mobile_task/state`. It does not consume the simulator object pose. The
configuration maps marker ID to a named body, physical size, box dimensions
and the rigid **object_from_marker** transform `[xyz, wxyz]`.

```yaml
schema: robot_lab.object_markers.v1
dictionary: DICT_4X4_50
markers:
- id: 7
  object: environment_pickup_cube
  marker_size_m: 0.024
  object_size_m: [0.03, 0.03, 0.03]
  object_from_marker: [0, 0, 0.015075, 1, 0, 0, 0]
```

Place the YAML path in a map profile's `object_markers` field. An explicit
`object_markers:=/workspace/.../markers.yaml` overrides it;
`object_markers:=none` opts out. Auto binding is limited to enabled native
mobile controls. The ordinary seven algorithm selections are unchanged.

The detector publishes `/robot_lab/object_estimates` with image time, camera
frame, binding checksum, PnP quality, observation ID and transformed object
pose. The task requires matching identities, current image/receipt time and
the same physical source box dimensions. Only point targets enter the reach
planner; optional pad orientation remains an independent explicit control.

Minimum marker edge is 12 pixels; accepted fits have ≤1.5 px reprojection
RMS, bounded conditioning and translation/rotation uncertainty. The uncertainty
calculation declares a 0.5 px corner-noise floor; it is not calibrated accuracy.
The object-offset bound must be ≤2 cm. Source camera intrinsics, extrinsics
and marker dimensions remain declared inputs. Camera/TF calibration error is
not measured by the PnP residual.

The initial actual rendered-box check measured 2.523 mm position error; a
separate real ROS transport check uses explicitly synthetic image pixels.
Neither is a complete source-robot manipulation mission.
[Implementation scope](../status/camera-object-targets-2026-11-14.md).
