# Real queued and concurrent experiments

October 9 adds an implemented subprocess coordinator. Physical concurrency,
overload and mission qualification follow the initial feature implementation.
Historical generated R8 demonstration reports are separate.

## Use the GUI

Choose an exact robot/map/backend/mode and options in Launch, then open
**Benchmark → Concurrent experiments**. Set capture duration, per-job RSS,
CPU and artifact budgets. **Queue Launch selection** snapshots the current
literal run command and selection. Optional task commands run in that job's
ROS domain after actual readiness. They are executable arguments, without
shell evaluation. Queue more selections, choose **Parallel** 1–4, then **Run
queue**. Start at one on this Jetson; concurrent heavy plants can exceed memory
or CPU budgets. GUI/RViz and hardware joystick are disabled in queued launches.

The queue leases separate ROS domains and Gazebo partitions. Each job has its
own output directory, logs, resource trace, actual ROS telemetry and optional
bag. Startup is bounded; capture begins only after two actual `/clock`
messages, or FCU `/px4/odometry` for PX4. No task command means a stationary
capture, not an implied robot mission. Task exit zero means process completion,
not physical success.

**Cancel owned runs** signals only tracked owned processes/groups. **Reset
selected run** calls its actual domain-scoped `/robot_lab/reset` and saves the
response; unsupported reset fails explicitly. Closing the GUI cancels its
coordinator. The implementation retains PID birth identities, bounded signal
escalation and a surviving-process/cleanup result. This still needs an actual
simultaneous-plant campaign.

PX4 instance 0 shares MAVLink port 14580: run multiple X500 jobs with
**Parallel=1**. A native instance lease prevents overlap with the main GUI's
PX4 launch. Queued flight logs stay under that job's artifact budget. Automatic
takeoff is not added; an explicit task client must perform the flight workflow.

## CLI specification

A JSON array supplies literal commands. Use a selected generated command in
place of this simple stationary example:

```json
[
  {
    "command": ["ros2", "launch", "robot_lab_bringup", "simulated_robot.launch.py",
      "robot_model:=bumperbot", "simulator:=pybullet", "mode:=display",
      "map_name:=nav_empty", "gui:=false", "start_rviz:=false", "enable_joystick:=false"],
    "duration_s": 30,
    "startup_timeout_s": 300,
    "max_rss_mb": 4096,
    "max_cpu_percent": 800,
    "max_log_mb": 256,
    "readiness_topic": "/clock",
    "selection": {"robot": "bumperbot", "map": "nav_empty", "backend": "pybullet", "mode": "display"},
    "seed": 42,
    "record_bag": false,
    "task_command": []
  }
]
```

Save the specification/output on SSD, source the usual environment and run:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_lab_benchmark robot-lab-concurrent \
  /workspace/molar/robot_lab_runtime/experiments/jobs.json \
  --output /workspace/molar/robot_lab_runtime/experiments/run-001 --parallel 1
```

Use `readiness_topic: /px4/odometry` for PX4. Duration is limited to 3600 s,
startup to 600 s. CPU is aggregate process CPU, so one full core is approximately
100%; GPU usage remains unavailable. Artifact limits cover the job directory,
including bags/ROS logs and queued PX4 logs. Shared source/model/cache stores
remain outside these per-job artifacts.

## Read the actual results

`run.json` records base revision, working-tree state/diff digest, coordinator/
recorder hashes, literal workload, selection, requested seed/budgets, actual
capture/process/resource state and cleanup. A requested seed is exposed to the
workload; simulator-specific application still requires its own explicit contract.

`telemetry.jsonl` contains actual clock, joint, scan and body/FCU position
messages. `telemetry-summary.json` has topic counts, clock-based real-time
factor when available and measured trace distances. FCU estimates stay
separate from independent `/px4/odometry_truth`. Scan range is not robot
footprint clearance. Contact counts, footprint clearance and mission result
remain null until an actual dedicated producer measures them. Reset can create
pose jumps in a trace; use the saved reset events when evaluating movement.
Do not present raw trace distance as a qualified route metric without review.

For fair comparisons, implement explicit workload success, simulator seed,
matched input and independent geometry/contact contracts. Then validate two
plants, failed startup, overload, scoped Stop/reset and orphan cleanup. Use
[the implementation handoff](../status/implementation-2026-10-09.md) and
[robot readiness](../AI_ROBOT_READINESS_GUIDE.md); the queue does not replace
the physical acceptance criteria.
