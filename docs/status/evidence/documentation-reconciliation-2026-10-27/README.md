# Documentation and remaining-work reconciliation — October 27, 2026

Source baseline `f75dc60`. This receipt records read-only source/inventory and
existing-artifact review plus documentation/status changes. It does **not**
record new physics trials, tests, flight, SLAM or navigation qualification.

The user requested updated README/roadmap instructions for continuing agents.
[The execution guide](../../../REMAINING_WORK_GUIDE.md) supplies the ordered
queue, implementation locations, prerequisites and completion/handoff rules.

## Findings

- The active manifests contain 31 core + 109 external robot profiles, grouped
  into 75 selectable families, and 34 core + 17 external world profiles,
  grouped into 39 selectable families. Referenced external robot description
  and native model files exist; inventory does not establish controller support.
- Six official mobile registrations are absent despite cached asset directories:
  TurtleBot3 Burger/Waffle/Waffle Pi, TurtleBot4 Standard/Lite and Husky. The
  historical `menagerie_franka_emika_panda` profile is also missing. Restore
  matching later control metadata, not only old Display metadata.
- `f75dc60` adds only `terrain_heightfield` to the previous map ID set. It
  registers existing 3D entries; it does not deliver fourteen new worlds.
- Aerial runtime/controller/model/worker files exist; no corresponding aerial
  swarm bringup launch or native GUI wiring was found at this baseline.
- Existing October 20 artifacts include two completed queued fleet goals, an
  RMF traffic stall, one completed bounded ROV route and failed marine map saves.
  These keep their own source-hash/robot/map/backend scope.
- New reset, disturbance, terrain and metadata tests are useful software checks.
  The full R6/R7/R8/R9/R10/R11/R12 acceptance does not follow from those counts.
  Existing exact-host Isaac and other qualified cells are preserved.
- Underwater simulated estimation/SLAM/routes and Isaac runtime contact are
  software/simulator tasks. Physical sensor calibration and optional HIL remain
  separate; water-world work does not require terrain-provider setup.

## Changes and review

README, ROADMAP, the agent guide, handoff entry points, current status/checklist
and machine-readable task state are reconciled. Runtime owners and historical
artifacts remain. Premature `done` states return to `partial`, R10.5 remains
active, and the next concrete task is R3.6 catalog restoration. GUI Health reads
the same ledger, so its task state follows this documentation correction.

Validation for this revision is limited to YAML parsing, state/owner invariants,
relative documentation links, anchors and whitespace checks. No robot mission
is promoted and no reported prior test count is presented as a new test run.

Recorded documentation checks:

- YAML parses with duplicate-key detection; all runtime task owners are unchanged.
- R8.2 and R9.4 records are unchanged. R10.5 retains its active runtime owner;
  its prerequisite is the R10.4 vehicle contract, without the unrelated terrain gate.
- Ten unique implementation queue entries remain; catalog repair comes first.
- All nineteen new or changed Markdown link targets and their anchors resolve;
  the checked documents also have no broken pre-existing link targets.
- `git diff --check` succeeds. All eleven changed/added files are documentation
  or status; runtime source and installed manifests are unchanged.
