# R1.3 hosted-runner Firefox repair — November 11

Actual baseline failures at `9f3b54ba`:

- [Scheduled full run 43031665805](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/43031665805),
  job `130559995302`.
- [CI run 43028412341](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/43028412341).

Both fail during **Install ROS 2 Humble**, before build or test steps.
`prior-jobs.json` is the actual GitHub response for the scheduled job;
`prior-log-excerpt.txt` retains the relevant actual log lines. `setup-ros`
invokes `sudo apt-get upgrade -y`; Jammy's Firefox transition invokes Snap
installation. The store is unreachable, retries take approximately 50 minutes,
and dpkg exits 100. This is not evidence that a Robot Lab test failed.

Both workflows now hold the already installed Firefox package **on the ephemeral
GitHub runner**, before `setup-ros`. Missing Firefox needs no action. ROS setup
and the dependency/build/fast/physics/integration/registry gates remain enabled.
No Jetson package configuration changes were executed. The upstream action's
[declared inputs](https://github.com/ros-tooling/setup-ros/blob/0.7.16/action.yml)
provide no upgrade-exclusion input, so the workaround is scoped to its unrelated
browser package instead of replacing ROS setup or hiding failure exits.

Both workflow YAML files parse and every embedded shell step passes `bash -n`.
The repaired published revision still needs its actual GitHub run. **R1.3 remains
partial pending that result**; the earlier local robot/camera checks are separate
from CI, and an environment repair does not establish a green build.

The camera and full working-tree review are committed at `d3e89023`; their source
hashes describe that revision. This follow-up changes only CI and status docs.
