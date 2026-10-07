# Inspect robots and worlds inside the GUI

## Run the embedded viewer

Open **Registry**, choose **Robots** or **Environments**, select an entry and
click **Preview 3D**. The geometry appears in the Registry's **3D Preview**
pane inside Robot Lab GUI. The same button in **Worlds** and
**Installed Extensions** opens this pane.

![Furnished Celisca floor 2 in the embedded Registry viewer](../status/evidence/registry-3d-2026-10-07/controls-column-final/celisca_floor_2_furniture/gui.png)

| Control | Action |
|---|---|
| Left mouse drag | Orbit around the model |
| Right mouse drag | Pan the camera |
| Mouse wheel | Zoom |
| Fit | Frame the model or building |
| Top / Front / Side | Inspect an axis-aligned view |
| Whole scene | Include remote authored objects in Fit |

The preview reads the installed source geometry in metres: URDF joint and
visual frames, native MuJoCo nominal/keyframe geometry, and SDF world/model
includes. It is a static inspection view. Use Run and Live Monitor for actual
physics, sensor and controller feedback. Previewing an asset preserves the
current launch selection and command; it also works while an owned simulation
is running. Large meshes prepare in the background, with buffers on the SSD.
Declared colors are displayed; embedded mesh textures are not reconstructed.

Robot and map selectors now show parent families. **Model/source variant**
and **World variant** select the exact executable profile. For example,
TurtleBot3 contains Burger/Waffle/Waffle Pi, Celisca floor 2 contains shell,
furniture and actors, and R2 contains complete source revisions and retained
subassemblies. Search still finds the original profile or registry ID.

Expand a Registry family to inspect its variants and components. A forearm
can be previewed, but **Open in Launch** is disabled for an isolated component.
Complete R2 and Valkyrie source assemblies already contain their corresponding
parts. Unattached grippers stay in a component library until a compatible
mount is defined; the GUI does not guess mounting transforms. Changing a
source variant retains that profile's controller and backend restrictions.

If preparation or rendering fails, the pane reports the error and Console
contains the loader details. The viewer uses Linux Tk/GLX, PyOpenGL and
Trimesh; ROS package dependencies declare those libraries. The checked
installation supports both native MJCF and downloaded URDF/SDF assets.

The imported two-floor hospital has six fixtures with extreme source poses.
Default Fit frames the main building and reports the remote geometry;
**Whole scene** exposes the full raw extent. This does not repair that world's
spawn, collision or navigation behavior. Those checks remain in R6.6.

[Actual rendered scenes, camera-input checks and live-plant regression](../status/evidence/registry-3d-2026-10-07/README.md)
record the tested scope. Preview availability does not qualify a robot mission.
