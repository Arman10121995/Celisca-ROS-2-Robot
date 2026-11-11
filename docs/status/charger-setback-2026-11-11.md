# Charger display setback — 11 November 2026

R11.3 is complete. Three charger assemblies are moved 0.35 m away from their
cached dock goals along their existing axes, for a 1.28 m total display offset.
Two detailed maps, content-addressed visuals, static adapters and occupancy
projections were regenerated. Original floor geometry/registration and spawns
are retained. The map package builds and three installed/source/frame/FK
checks pass. Ten current configuration/GUI files retain their hashes; robot
model files and real FMS/AIC goals were not changed. No hardware commands or
new mission acceptance. Prior maps are preserved at the receipt's SSD path.

See [receipt](charger-setback-2026-11-11.json) and the HRI checkout's
`docs/CHARGER_DISPLAY_SETBACK_2026-11-11.{md,json}`. Reload the map to consume
the new assets; this is unsurveyed display geometry, not docking calibration.

Whole-tree review by Codex: source GLB geometry/transforms are retained; changed OBJ vertices
move only 0.35 m and faces are identical. All 126 copied adapter hashes and 136 installed
assets match. Git now preserves the copied files' original line endings. An old test still
expected the historical 96 details; the actual source GLBs and manifests both contain
148/122, and the strict expectations now match those reviewed pins. The focused review suite
passes 52 checks. [Review](evidence/vacuum-camera-2026-11-11/whole-tree-review.md).
