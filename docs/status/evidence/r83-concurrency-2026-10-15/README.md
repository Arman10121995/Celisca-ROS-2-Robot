# R8.3: two simulations at once, with Nav2 goals, budgets and cancel (2026-10-15)

Status: **partial**. This is the first actual campaign of the October 9
concurrent coordinator (`ros2 run robot_lab_benchmark robot-lab-concurrent`)
with its built-in Nav2 workload. That workload had never run: it failed on
three bugs, one after another, which are now fixed and tested.

## What ran

Two jobs ([pair.json](pair.json)): Bumperbot in `mode:=nav` on
nav_obstacle, one in MuJoCo and one in PyBullet, both on wheel odometry.
Each job had:

- a 4096 MB / 800% CPU budget;
- the built-in workload: Nav2 goals (-3.8, -3.3, 0°) then (-5.5, -7.0, 180°)
  from the start at (-7, -7);
- a pass contract of 0.15 m and 5° against simulator truth, stopped for
  one simulated second.

The coordinator gave each job its own ROS domain and Gazebo partition,
output folder and resource trace. After each run, a process listing
checked for anything left behind ([survivors.txt](survivors.txt)).

## Results (rounds 33-34, `c33f221`, 18:01-18:23)

| Run | Job | State | Goal errors (truth) | Wall | Real-time factor | Peak RSS / CPU |
|---|---|---|---|---|---|---|
| A, one at a time | MuJoCo | **success** | 0.080 m, 0.7° / 0.022 m, 1.8° | 91 s | 0.87 | 1617 MB / 957% |
| | PyBullet | **success** | 0.045 m, 0.9° / 0.067 m, 1.8° | 205 s | 0.36 | 1233 MB / 984% |
| B, both at once | MuJoCo | **success** | 0.074 m, 1.4° / 0.053 m, 1.9° | 88 s | 0.88 | 1689 MB / 933% |
| | PyBullet | **success** | 0.104 m, 2.2° / 0.043 m, 0.7° | 207 s | 0.36 | 1219 MB / 847% |
| C, MuJoCo budget 300 MB | MuJoCo | **stopped: resource_budget_exceeded** at 741 MB | none | 5 s | | 741 MB / 299% |
| | PyBullet | **success** | 0.044 m, 0.9° / 0.059 m, 0.8° | 210 s | 0.36 | 1241 MB / 998% |
| D2, SIGINT to the coordinator at 150 s | MuJoCo | **success** (finished at 88 s) | 0.048 m, 0.5° / 0.016 m, 2.0° | 89 s | 0.88 | 1678 MB / 889% |
| | PyBullet | **canceled** on its second goal | 0.070 m, 0.6° / none | 152 s | 0.36 | 1227 MB / 826% |

Sources: [summary_rounds33.json](summary_rounds33.json) (A-C) and
[summary_cancel.json](summary_cancel.json) (D2).

- **Simultaneous runs did not slow each other.** Both plants kept their
  real-time factors (0.87-0.88 and 0.36) and their goal errors. The two
  jobs together took 207 s against 296 s one at a time. The Jetson has 12
  cores; heavier plants (Gazebo, Isaac) were not paired.
- **A budget stops only its own job.** The 300 MB MuJoCo job was stopped
  4.8 s after start, and the PyBullet job beside it went on to pass.
- **Cancel reaches the running workload.** The PyBullet goal was aborted
  and the job marked canceled within 3 s, while the finished MuJoCo
  result stood. Signalling the `ros2 run` wrapper (round 33's run D) did
  not reach the coordinator; the coordinator process itself has to be
  signalled.
- **Nothing survived.** After every run (A-D2, and the four failed rounds
  below), no simulator, Nav2 or launch process remained.
- In every PyBullet job, Nav2 rejected the first goal three times before
  the stack became active; MuJoCo's once. Each job then drove 11.4-11.5 m.

## The three workload bugs, in the order they appeared

1. **Round 30 (`867e6b3`).** The workload stored its goal handle as
   `self.handle`, which shadows `rclpy.Node.handle`. Every workload died
   while being constructed ("handle cannot be modified after node
   creation")
   ([summary_round30_handle_bug.json](summary_round30_handle_bug.json)).
   Even so, the overload budget stopped the MuJoCo job, and no process
   survived any of the four runs.
2. **Round 31 (`9f1157b`).** bt_navigator's action server answers before
   the stack is active and rejects goals until it is. The workload failed
   on that first rejection, 1.5 s of sim time in
   ([summary_round31_rejected.json](summary_round31_rejected.json)). The
   first goal is now retried every 2 s within the 180 s startup deadline,
   and the result records the rejections.
3. **Round 32 (`c33f221`).** The accepted goal's uuid is a NumPy uint8
   array, which json refused, so the first accepted goal failed the
   workload ([summary_round32_uuid.json](summary_round32_uuid.json)).

`src/robot_lab/robot_lab_benchmark/test/test_nav2_workload_node.py` now
constructs the node and feeds it an accepted goal.

## Remaining

- Simulator-applied seeds: the seed is passed to the job, not applied by
  the simulator.
- Contacts and swept clearance stay unassessed.
- Pairs with Gazebo or Isaac, and more than two jobs.
- A reset during a workload: the coordinator invalidates the result.
- Repeat runs.
