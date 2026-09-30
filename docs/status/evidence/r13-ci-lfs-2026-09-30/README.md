# R1.3 source CI without LFS downloads, 2026-09-30

The latest pushed GitHub run before this change, `36703588194` at
`9ff8921`, stopped in `actions/checkout@v4` before any build or test:
GitHub's LFS batch endpoint reported that the repository had exceeded its
LFS budget. Retrying checkout did not change the result.

Both CI workflows now skip LFS smudging and fetching. Their test tiers use
source/configuration files, occupancy maps, xacro expansion and synthetic
box-physics scenes; none needs the large robot/world meshes. To preserve
the map geometry and spawn checks in CI, all 30 `.pgm` occupancy maps were
converted from LFS pointers to normal Git blobs. They total about 35 MB
uncompressed and about 337 kB when gzip-compressed together. Mesh LFS
tracking is unchanged, and mesh-based simulator missions still require the
actual assets outside these CI workflows.

This was checked in a separate `GIT_LFS_SKIP_SMUDGE=1` worktree at
`9ff8921`, with the new occupancy map blobs copied into it and all mesh
paths left as LFS pointer files:

| Check | Result |
| --- | --- |
| `bash scripts/test_tiers.sh fast` | 639 passed |
| `bash scripts/test_tiers.sh integration` | 118 passed |
| `bash scripts/test_tiers.sh physics` | 6 passed |
| `colcon build --symlink-install --packages-skip orbslam3` | 25 packages finished |

The first no-LFS fast run, before replacing the map pointers, failed 18
map-content checks and passed 621 others. That negative result is why the
PGM conversion is part of this CI fix. A new GitHub-hosted run is still
needed to verify checkout and dependency installation on its runner.
