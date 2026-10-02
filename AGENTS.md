# Robot Lab agent instructions

Read `docs/AGENT_HANDOFF.md`, `docs/status/platform-status.yaml`,
`docs/status/audit-2026-10-02.md` and `docs/PATCH_EXECUTION_GUIDE.md` before
changing the platform. Preserve existing work and claim task ownership in the
ledger. Follow the user's current priorities.

The user requests a single primary branch in this existing repository. Work
directly on `master`; do not create another branch or worktree unless the user
explicitly asks. Commit there and push to `origin/master` when requested.

On this Jetson, `/` and `/tmp` are on the 64 GB internal eMMC; `/workspace`
is the 1 TB SSD. Keep all large source checkouts, builds, trial logs, bags,
caches and generated assets on the mounted SSD. Read `docs/STORAGE.md` and
source `scripts/ssd_env.sh`. Explicit `/tmp/name` redirection ignores TMPDIR.
Use a persistent SSD run directory instead. Do not copy archived PX4 logs back
to eMMC or rerun the completed one-off relocation. The preserved `/tmp/px4*`
paths are compatibility symlinks, not the preferred location for new runs.

Use PX4's `-d` when launching its binary directly unattended; the interactive
pxh shell produced multi-gigabyte prompt logs. Retain required model/startup
arguments, bound trials, and monitor log growth. Coordinate drone work as its
own task; arming does not establish flight.

Never mark a runtime task done from catalog counts, metadata checks,
hard-coded/simulated PASS flags or randomly generated metrics. The October 1
R6/R7/R8/R9 demonstration reports do not satisfy full roadmap acceptance.
Keep their historical artifacts, preserve real scoped measurements, and
match each support claim to the exact robot/map/backend/task/revision and
measured artifacts. Tests and framework availability are separate from a
working robot mission. Update roadmap, ledger and GUI status consistently.
