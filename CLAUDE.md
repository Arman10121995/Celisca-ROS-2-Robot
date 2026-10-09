# Robot Lab continuation

Updated October 9, 2026. Read the [implementation checkpoint](docs/status/implementation-2026-10-09.md); work on `master`. Earlier runtime source has
successful required CI and separate measured GUI/Drive/Panda artifacts. The
project remains partial; start with [current status](docs/status/CURRENT_STATUS.md),
[the checklist](docs/status/CHECKLIST.md) and [the roadmap](ROADMAP.md).

The user's latest priority is **implement everything remaining first, test and
validate afterward**. Codex/Claude should deliver reusable controllers and
complete operator workflows rather than repeatedly run mission matrices.
Follow `implementation_first` in the ledger. Keep necessary build diagnostics
and actual Stop/limits/watchdog code. Mark new integrations implemented and
experimental, preserving historical measured source certificates until later
validation. Follow-up validation owners are currently unassigned.

Follow [AGENTS.md](AGENTS.md) and [the handoff](docs/AGENT_HANDOFF.md). Read the
ledger, October 2 audit and execution guides before changing shared files;
claim ownership and preserve the independently owned occupancy/terrain lanes.
The latest GUI uses a right-hand Drive/Arm/Hand/Drone control column, grouped
source variants/components, native Registry 3D and robot category/type filters.
Preserve exact source/controller gates and normal neutral/Drive/Panda checks.

Continue the new extensions before the older roadmap: asset/controller/world/
occupancy/terrain work, flight-world checks, Servo/object/hand/backends and
mobile manipulation; then remaining wheeled/legged missions, comparisons,
concurrency and clean-host reproduction. Use the
[extension guide](docs/ASSET_EXTENSION_GUIDE.md) and
[patch guide](docs/PATCH_EXECUTION_GUIDE.md) for acceptance.

Work directly on `master`, commit/push there when authorized, and do not create
another branch/worktree. Keep large artifacts on `/workspace`, source
`scripts/ssd_env.sh`, and read [Storage](docs/STORAGE.md). Preserve historical
measurements and failures. Catalog counts, generated scores, static previews
and software checks do not qualify a robot mission. Update all current docs
and the GUI-visible ledger together when advancing work.
