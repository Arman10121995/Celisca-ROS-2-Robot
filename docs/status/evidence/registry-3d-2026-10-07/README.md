# Consolidated Registry and native embedded 3D, October 7

The normal Tk GUI now contains an OpenGL viewport in Registry. No RViz window,
ROS preview node, simulator or movement publisher is started by the inspector.
Reviewed parent families replace duplicate/fragment entries in Launch,
Installed Extensions and Worlds. Exact executable IDs, source variants,
saved selections, commands and controller/backend gates are preserved.

`asset_groups.yaml` supplies reviewed ownership rather than filename heuristics.
Full upstream R2 and Valkyrie assemblies are preferred over isolated limbs.
[Assembly audit](assembly-topology.json) measures containment of six retained
subassemblies by source link names and internal parent/child joint topology;
this does not establish identical mesh, inertia or controller behavior.
Unattached grippers and biological references remain inspection-only libraries.
Map variants remain attached to the complete room/floor they describe.

[Actual producer](probe_embedded.py) constructs the full normal GUI and renders
seven source scenes on Jetson AGX Orin, Linux Tk/GLX and Xvfb/Mesa llvmpipe:

| Scene | Scope |
|---|---|
| Bumperbot | Original URDF visual frames |
| Celisca floor 2 furniture | Full authored building mesh, scale preserved |
| Native Panda | Actual compiled native nominal/keyframe geometry |
| R2 left forearm | Nested component inspection |
| R2 c6 | Complete upstream body assembly |
| PX4 X500 | Source SDF geometry without starting PX4 |
| Two-floor hospital | Included world geometry and source-pose defect flag |

Every case reads the actual rendered framebuffer, saves a full GUI screenshot,
changes the image by mouse orbit, checks wheel zoom/right-drag pan/Fit and
verifies launch selection/command/process are unchanged. All seven loaders
and the producer exit zero; GUI closure releases the GLX context and buffers.
[Report](report.json), per-scene screenshots/request/geometry metadata,
[artifact hashes](manifest.json) and [archive-time source hashes](source-hashes.json)
are retained. Scene source/mesh hashes come from the actual loader. The
archive-time implementation hashes are explicitly not pre-trial fingerprints.
Large full meshes and GPU buffers remain at the SSD directories in the manifest.

The separate [live Burger/PyBullet report](live-burger-pybullet/report.json)
opens the embedded viewer during a normal owned `nav_empty` Display launch.
There are **zero movement commands during preview**, 0.00020 m measured x drift
and 90 framebuffer colors. The same plant then passes the original GUI
neutral enable, W/S/A/D, Stop, publisher loss, joint/TF and source-LDS checks.
Its launch and producer exit zero. The actual
[producer](live-burger-pybullet/producer.py), pre-trial source manifest, raw
trace fingerprints and [live artifact manifest](live-plant-manifest.json)
are preserved separately from static rendering.

Negative stages remain: initial Tk/GLX BadDrawable, an empty Mesa GL_FRONT
readback despite visible on-screen rendering, X500's unnecessary FCU import,
and hospital Fit dominated by remote source fixtures. The final viewport uses
a matching GLX window and back-buffer readback before presentation; X500
inspection reads SDF directly. Six hospital fixtures have source z positions
around -754,989,000 m. Default Fit excludes these remote bounds while retaining
all geometry and authored poses. Whole scene reveals their raw extent; no
physics-world repair or hospital mission is claimed.

Full source geometry is retained. The furnished Celisca scene has about
7.94 million triangles, and software GL can pause the UI for buffer upload or
deletion. Hardware-GPU performance, textures, animated actors and broad
per-model visual/physics qualification remain open. These rendering/input
screens do not qualify controllers, localization or navigation.

## Grouped worlds and the redesigned Registry

The [grouped-world button repeat](grouped-worlds-final/report.json) inspects
furnished Celisca floor 2 and static/dynamic Room 2; the latter add two unique
source scenes, giving nine unique assets across these recorded checks.
[Final control-column GUI repeat](controls-column-final/report.json) exercises
the actual Preview button and orbit/pan/zoom/Fit on the same three worlds after
the redesigned Registry pane. All loaders/producer exit zero, selections and
commands stay unchanged, and each scene retains actual framebuffer and source
metadata. These are static geometry/camera checks; actors and missions remain
unqualified. Full meshes remain on SSD.
