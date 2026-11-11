# R13.3 camera implementation — November 11, 2026

Baseline: `9f3b54ba` on existing `master`, plus the source hashes in this receipt.
Pre-existing charger-placement, generated map and ledger changes are preserved.
No broad robot/backend mission campaign was run.

Implemented:

- Installed learned detector executable and exclusive segmentation/learned/none
  launch selection; GUI Setup checkpoint picker; missing-file rejection and owned
  launch shutdown if the selected learned process exits.
- RGB channel/padding/byte-order fixes, approximate image-time pairing and camera
  dimensions; no inference-time backbone download; class-map validation and
  class-aware NMS. Learned nominal cable extent covers the complete source cable.
- Timestamped JPEG object crops and bounded GUI Objects gallery.
- Read-only, finite simulator RGB/semantic dataset capture with scene/split
  metadata; independent training/validation subsets and matching input range.
- Generated-world footprint manifests and report-only truth inputs; profile body
  radius, fresh truth through the return leg, and swept-body clearance gate.

The ideal segmentation detector's November 10 physical mission is retained in
[its own receipt](../vacuum-2026-11/README.md). This patch does not claim a new
trained checkpoint, learned recognition accuracy, physical object avoidance or
finished R13/platform acceptance. Those remain R13.3 work.

Focused software and transport checks are recorded in `checks.json`; actual ROS
transport results and launch selection are copied beside it. The transport
producer is `camera_transport_probe.py`, executed on isolated domains 129/130/131
with **explicitly synthetic images and no robot/world/physics**. It checks actual
installed ROS message delivery and decodable crop pixels, not a robot mission.
Cases two and three also exercise actual bounded dataset capture from those frames.
Case three includes the final capture-image hashes.
The raw synthetic dataset/logs remain on SSD under
`/workspace/molar/robot_lab_runtime/r133-camera-20261111`.

`source-sha256.json` identifies the implementation; `artifact-sha256.json` records
retained log/result hashes. Historical policy/mission certificates are unchanged.
One launch-selection diagnostic initially passed an already expanded string to
`perform_substitutions`; it failed in the diagnostic before executing a launch.
Normalizing substitutions corrected the probe. No product check was bypassed.

[Whole-tree review and fixes](whole-tree-review.md) cover the charger assets as well.

Operator and next-agent instructions: [camera guide](../../../tutorials/vacuum-camera.md).
