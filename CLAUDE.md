# Robot Lab continuation

Updated October 7, 2026. Runtime checkpoint: **`091d388` on `master`**, with
successful required CI and separate measured GUI/Drive/Panda artifacts. The
project remains partial; start with [current status](docs/status/CURRENT_STATUS.md),
[the checklist](docs/status/CHECKLIST.md) and [the roadmap](ROADMAP.md).

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
