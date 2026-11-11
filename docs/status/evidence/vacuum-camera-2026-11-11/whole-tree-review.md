# Full working-tree review — November 11

The user confirmed one active owner and requested checking **all** edits. This
review includes the pre-existing charger source pins, generated floors and
receipts, as well as the camera implementation. No changes were discarded.

Findings corrected:

1. **Copied map bytes changed by Git.** Content-addressed adapters contain
   upstream CRLF. Before the fix, `git hash-object --path` and `--no-filters`
   differed for both collision OBJs. A scoped `.gitattributes` rule preserves
   all copied adapter bytes; the same checks now match for OBJ, XML and USDA.
   Existing LFS filters remain in place. Copied adapters were explicitly reindexed
   under the new rule; all 126 staged adapter blobs or LFS object IDs match
   their recorded SHA-256 values. Much of the textual diff restores retained
   CRLF, rather than changing geometry. Normal whitespace checks remain enabled;
   CR is recognized as part of the preserved line ending.
2. **BGR tensor input could fail.** Reversing RGB channels produced negative
   NumPy strides, which torch rejects. BGR decoding now makes a contiguous
   copy, with a regression assertion.
3. **Stale detailed-model expectations.** Two map tests expected the old 96
   detail parts, although both HEAD and current photo-refined manifests have
   148/122. Actual source GLB `Detail_` node counts independently confirm
   148/122. Strict per-floor expectations now match the reviewed model pins;
   no geometry, hash or collision threshold was relaxed. The first run's two
   failures are retained in `whole-tree-review-tests.log` on SSD.
4. **GUI launch parsing.** During implementation, the empty optional checkpoint
   generated an invalid `vacuum_detector_model:=` argument. It is now omitted
   unless set, and GUI checks parse the exact default command with ROS launch.
5. **Gallery lifecycle.** A new launch clears the old gallery. Malformed object
   JSON and oversized/invalid snapshot data are ignored by the gallery.

Independent generated-asset review:

- The original F1/F2 source GLB accessors, transforms and retained binary data
  pass the existing source-retention verifier against the new pinned exports.
- Collision and visual OBJ face lists and vertex counts are identical to HEAD.
  Only charger vertices change, by 0.35 m (text-export rounding under 20 µm).
  F1: 32 collision / 4,904 visual vertices moved. F2: 16 / 2,452.
- All 63 copied adapter SHA-256 values per floor match their provenance.
  Cached full GLB and full visual DAE hashes match too.
- After rebuilding maps, all 68 checked installed assets per floor match source,
  including the adapters, native collision, map, projection and world.
- Existing source-triangle/winding, source-normal, occupancy/spawn and MOLAR
  model/backend checks pass: **52 passed in 4.32 s**, including camera layout
  checks. The final detector/gallery GUI subset passes **2 checks** after the
  complete **13-check GUI** run. The combined vacuum/support suite has **105
  passing checks**; its later training guard subset passes 3.
- Four packages rebuild successfully in **11.1 s**. The existing setuptools
  `tests_require` warning remains; the build completes.

See `whole-tree-map-review.json`, `charger-geometry-review.json`, `checks.json`
and `artifact-sha256.json` for the retained scope/artifacts. Logs/datasets stay
under `/workspace/molar/robot_lab_runtime/r133-camera-20261111`.

These are implementation/source/import/transport checks. A trained detector,
new physical cleaning mission, broader docking acceptance and full project
qualification remain pending. No robot hardware commands were sent.
