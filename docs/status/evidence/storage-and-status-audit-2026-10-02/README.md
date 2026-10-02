# Storage relocation and completion audit — 2026-10-02

Baseline source revision: `2a0aae2`. Host: Jetson AGX Orin, Ubuntu 22.04,
ROS 2 Humble. Scope: inactive PX4 trial artifacts and source/evidence audit.
No flight or robot mission was launched by this audit.

- `relocation-manifest.json`: all 65 original/destination paths, SHA-256 hashes,
  sizes, ownership, permissions, modification times, source/destination device
  numbers and before/after filesystem usage.
- `relocate_px4.py`: exact one-off relocation script. It refuses active source
  references, pre-existing destinations and special files; verifies a full
  copy before replacing the original path. Do not rerun against an already
  relocated path set.
- `claim-probes.json` and `probe_claims.py`: three read-only demonstrations of
  unsupported completion reporting, plus hashes of inspected generators.
- `summary.json`: measured storage result and current check outcomes.
- `verification.log`: 685 fast passes, 1 skip and registry validation pass.
- `gui-verification.log`: 36 command/Drive test passes.
- `gui-build.log`, `gui-health-smoke.log`: package rebuild and installed Health
  display of corrected states, audit and storage guide pass.
- `storage-guard-check.log`: internal-disk override rejected; a separate
  tempfile smoke confirmed the helper directs temporary files to the SSD.
- `source-sha256.json`: hashes of the final rules, storage helper, audit,
  ledger, documentation entry points and GUI Health implementation.

Persistent raw artifacts:
`/workspace/molar/robot_lab_runtime/px4/legacy-tmp/`.
The runtime manifest/script/log also remain at
`/workspace/molar/robot_lab_runtime/storage-migration-2026-10-02/`.

Result: 16,957,130,259 bytes (15.79 GiB) moved, all copies verified before
removing their original internal content. All original paths remain symlinks
to SSD copies. Internal available space: 674,746,368 → 17,630,314,496 bytes
(0.63 → 16.42 GiB); `df -h /` shows 99% → 70% used.

PX4's original source/build tree was already on the SSD; its installed binary's
`-h` still works and confirms `-d` disables the unattended pxh shell. The Unix
socket was intentionally left in `/tmp`. No active simulator/FCU was present.

See [audit](../../audit-2026-10-02.md) for retained positives and remaining work,
and [storage guide](../../../STORAGE.md) for preventing another internal-disk fill.
