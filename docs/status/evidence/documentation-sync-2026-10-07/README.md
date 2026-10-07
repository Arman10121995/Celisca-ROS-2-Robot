# Current documentation reconciliation, October 7

The user requests all documentation to reflect actual current status. Runtime
checkpoint `091d388` is published on `master`; this follow-up changes maintained
documentation and status artifacts. Runtime source/installed byte equality is
preserved. No new robot mission is claimed by this documentation work.

[Current status](../../CURRENT_STATUS.md), [the checklist](../../CHECKLIST.md)
and [the roadmap](../../../../ROADMAP.md) now share the published GUI, inventory,
measured robot scopes, exact CI and remaining work. README, architecture,
workflow/testing, both agent guides, Claude entry point, robot/map package
READMEs, every current tutorial and status entry point have been reviewed.
Dated historical status records retain their bodies/measurements and link to
current status; the R7 completion report and unmeasured benchmark tables are
clearly historical/unqualified. Upstream license/source assets and raw mission
reports remain preserved.

Actual metadata/documentation checks:

- [58-page audit](links-and-consistency.json): **557 relative links, zero broken
  targets**, YAML/evidence consistency, required tutorial Run sections and
  twelve source/installed runtime hashes. The initial three broken historical
  links are retained in `initial-links-and-consistency.*` and corrected.
- [Existing documentation/evidence contracts](contracts-and-support.log):
  **42 passed**; [final documentation contracts](final-documentation-contracts.log):
  **14 passed** after the remaining text edits. These are software checks.
- [Source CLI checks](source-cli-checks.json): summary, cross-reference validation
  and default dry-run launch all exit zero using the corrected module paths.
  The original source-only README path failed to import `robot_lab_utils`; that
  observation was captured in the tool transcript, not recaptured as a trial log.
- [Installed Health formatter](installed-health-view.txt) reads the current
  repository ledger, published revision, documentation/CI fields and partial
  project state. It does not start a ROS node or plant.
- [Installed inventory](installed-inventory.json) records actual profiles and
  family/variant/component counts. Inventory remains host-specific metadata.
- The unchanged mission index revalidates **122 records / 114 measured cells**
  against the reconciled ledger, with full-release gates blocked.

[Published CI](../ci-extensions-2026-10-07/README.md),
[actual GUI/Drive proof](../gui-controls-column-2026-10-07/README.md) and
[Panda physical proof](../panda-cartesian-controls-column-2026-10-07/README.md)
retain separate source/host/mission scopes and failures. The producer and
manifest retain exact metadata checks/log hashes; these do not qualify other
robots, maps, algorithms, hardware, concurrency or clean-host missions.
