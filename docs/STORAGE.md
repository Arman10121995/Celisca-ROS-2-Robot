# Jetson storage: keep Robot Lab artifacts on the SSD

This operating guide remains current for runtime checkpoint `091d388`
(October 7, 2026). The relocation below is a completed historical operation;
do not rerun it. See [current project status](status/CURRENT_STATUS.md) for
published work and remaining tasks.

The Jetson's internal eMMC is mounted at `/`; the 1 TB NVMe SSD is mounted at
`/workspace`. Source, builds, models, trial outputs, bags and large logs belong
on that SSD. `/tmp` is on the internal eMMC. A path under `/home/molar1` is
not automatically on the SSD: check its resolved destination.

## Verified relocation on 2026-10-02

65 dormant PX4 paths under `/tmp` were copied to
`/workspace/molar/robot_lab_runtime/px4/legacy-tmp/`. Every regular file's
SHA-256, size, permissions, ownership and modification time matched before
its internal copy was removed. Original paths are compatibility symlinks.
No PX4/simulator writer was active during the move. The runtime Unix socket
was left in place; the PX4 source/build tree was already on the SSD.

The moved data totals **16,957,130,259 bytes (15.79 GiB)**. Internal available
space increased from **0.63 GiB to 16.42 GiB**; `df -h /` changed from 99% used
to 70% used. No trial data was discarded.

[Full manifest, hashes and relocation script](status/evidence/storage-and-status-audit-2026-10-02/README.md)
are retained. Larger raw logs remain outside Git on the SSD.

Examples of preserved paths:

```text
/tmp/px4_form -> /workspace/molar/robot_lab_runtime/px4/legacy-tmp/px4_form
/tmp/px4_build3.log -> /workspace/molar/robot_lab_runtime/px4/legacy-tmp/px4_build3.log
```

These `/tmp` links provide compatibility with old trial scripts. `/tmp` cleanup
or a reboot can remove them; use the persistent SSD paths for new runs. If an
old script requires a missing link, restore that link from the manifest after
checking that no unrelated file now occupies its original path.

## Run new work on the SSD

From the workspace root:

```bash
source scripts/ssd_env.sh
mkdir -p "$ROBOT_LAB_RUNTIME_ROOT/px4/runs"
run_dir=$(mktemp -d "$ROBOT_LAB_RUNTIME_ROOT/px4/runs/trial-XXXXXX")
printf '%s\n' "$run_dir"
```

The helper verifies the workspace is mounted on a different filesystem from
`/`, creates SSD directories, and sets `TMPDIR`, `TMP`, `TEMP` and
`ROS_LOG_DIR` for the current shell. It does not change the user's login
profile. Those environment variables do not redirect a script that explicitly
writes `/tmp/name`; change such trial output paths to `$run_dir`.

PX4 is installed at `/workspace/molar/px4/PX4-Autopilot`. Keep its build and
working directories there. Large outputs must explicitly use `$run_dir`:

```bash
some_trial_command > "$run_dir/controller.log" 2>&1
```

The old multi-gigabyte PX4 logs consist largely of repeated `pxh>` shell
prompts. The installed binary's `-h` confirms that **`-d` disables the pxh
shell**. Use that option when launching the FCU directly without an interactive
terminal, while retaining the required model/environment/startup arguments.
Build without starting a simulator when only a build is intended; do not treat
`make px4_sitl gz_x500` as a build-only command. Bound trial duration and check
log size during long runs. This storage guide does not establish drone flight.

## Check storage and paths

```bash
df -h / /workspace
findmnt -T /workspace/molar/robot_lab_runtime
readlink -f /tmp/px4_form/px4_run.log
/workspace/molar/px4/PX4-Autopilot/build/px4_sitl_default/bin/px4 -h
```

Move inactive directories by copy → content/metadata verification → symlink
replacement → removal of the verified old copy. Do not relocate active Unix
sockets, system libraries, drivers or package-managed system directories as
part of a Robot Lab artifact cleanup.
