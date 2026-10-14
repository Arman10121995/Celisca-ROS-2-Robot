# R5.8 native hand evidence

This records initial implementation and bounded diagnostics, not broad hand/
object/backend mission qualification. Read [the scope](../../native-hands-2026-10-14.md)
and [receipt](../../native-hands-receipt-2026-10-14.json).

The compact `robotiq-object-report.json` preserves the actual normal GUI
command, measured contact/support/slip snapshots and cleanup outcome. The
object starts clear of the hand and loses finger contact on Release. Contact
overshoot is disclosed; neither stable lift nor hard contact-force regulation
was established. `dexee-motion-report.json` is an earlier source stage's
normal GUI F0 movement observation.

Raw JSONL, producer code, earlier successful/failed stages, source snapshots,
SDK checks and build/launch logs remain on the SSD at
`/workspace/molar/robot_lab_runtime/native-hands-2026-10-14/`. The receipt hashes
actual files; no synthetic scores or source certificate promotions are used.
