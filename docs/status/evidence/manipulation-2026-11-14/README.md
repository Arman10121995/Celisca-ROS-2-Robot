# Manipulation implementation receipts — November 14

The source manifest binds the changed camera/Panda/hand modules. The repository
base revision alone is not the whole working-tree revision. Each retained
runtime report names its actual robot, world, backend, source and plant.

`render-corrected.png` is an actual MuJoCo fixture render; marker 7 is detected
from its pixels. `render-result.json` records IPPE pose and the independently
compared authored fixture position: error **0.002522894 m**, reprojection RMS
**0.167824 px**. `focused-final.log` records 15 initial source/camera/GUI checks.
The ROS transport case uses explicitly synthetic pixels, not this render.

![Actual marked cube fixture](render-corrected.png)

The raw camera artifacts remain at the SSD path in `manifest.json`, including
the initial mirrored marker that was not detected. This fixture does not
establish a full camera-only robot grasp mission or calibration accuracy.

Panda raw artifacts and source-specific failures remain on SSD under
`panda-object-task-20261114/`: zero appended world key coordinates, repeated
static collider removal, centre-pinch bench clearance, redundant attachment
REMOVE, hidden Gait focus Stop, unreachable 10 cm lift and post-release
single-seed retreat. Retaining these failures is separate from the latest run.

`panda-run.json` and `panda-journal.jsonl` record one actual normal-GUI launch:
native Menagerie Panda / MuJoCo / `mobile_manipulation_lab`, explicit free-box
selection, no legacy grasp fixture. Two cycles complete **38 measured stages**
and the launch returns zero. Each includes original IK/OMPL/actions,
acknowledged attachment, approximately 0.5 N per opposing finger, one simulated
second of unsupported custody, supported zero-finger-force release and retreat.
Both measured lift displacements are recorded in the JSON; original motors,
geometry and 1 cm planning padding remain. Frozen placement poses prevent
compliant-object drift from changing retreat; bounded alternate IK seeds are
revalidated and OMPL still starts at measured joints.

This is one single-source/box/map/backend session, not full R5.7 acceptance.
Failed trials remain preserved. The 10 cm lift failed IK; operator-selected
lift/approach clearance defaults to 5 cm and is bounded to 2–15 cm. No physical
object pose was written or welded to the gripper.

Hand/source checks establish protocol binding, original actuator preservation,
interruption, relative-pose custody and actual Robotiq supported release.
UMI transport protocol tests explicitly declare their observations; they are
separate from the following physical component trial. Full source/backend
missions remain ROADMAP acceptance.

## UMI source continuation

`umi-component.json` records a later pinned UMI source-component trial. Its
original source motors operate the free 50 g cube: opposing forces about
0.711 N, **49.0038 mm** measured lift, unsupported holding, a supported return
with **1.516 mm** seating travel, then sustained supported release with no
finger contacts. Gains and control bounds were measured unchanged. Raw trace
hash/path and controller/source hashes are retained in the JSON.

The source adapter now closes in the positive tendon direction, places the
fixture between distal orange finger meshes and guards additional effort
above the source's spring/damping load. Six source placement motors receive
bounded position-control gravity feedforward, including free-cube weight only
under measured unsupported custody. No gains, force limits, object poses or
generalized-force arrays are changed. Return seating is limited to 3 mm at
2 mm/s and still requires actual support.

The initial reversed-closure/gravity-sag, passive-load guard, holder-centred
fixture and above-support-return failures remain on SSD. Thirty-eight hand
checks pass, including a real source component grasp/lift/return/release
regression and original-motor checks. This is not a normal GUI UMI mission,
another object, source hand, backend or hardware certificate. The earlier
Panda/camera manifest remains a separate source snapshot.
