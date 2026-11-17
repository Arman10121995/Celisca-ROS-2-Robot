# Go2 get-up with roll-over shaping (R5.2/R12) — November 17, 2026

**Status: not passed** (implemented, experimental).

## What changed from the first get-up run

Task `get_up_roll` (`legged_training.py`) is the first `get_up` task plus one
reward term:

- **`roll_over`:** a potential-based shaping term, `upright_rate = -d(g_z)/dt`,
  weight 2.0.
- Its integral is the net change in uprightness, so rolling towards upright
  pays immediately and rocking back and forth earns nothing.
- It targets the first policy's failure: it never rolled back from poses
  tilted past 90°
  ([first run](../go2-get-up-2026-11-14/README.md)).

## Training

The settings match the first run, so the two are comparable
(`run.json`):

- `--robot unitree_go2 --task get_up_roll --envs 512 --iterations 1500 --episode-s 6 --seed 1`;
- 18.4 M environment steps on CPU.

Results:

- Mean return rose to about 2.4 (first run: peak 0.82, final 0.22).
- The `roll_over` term stayed positive through training.

## Evaluation

Command: `legged-train evaluate --task get_up`, run through
`LeggedPolicy`. It uses 10 settled collapsed starts (seed 3), the same
procedure and stand test as the first run: gravity z < −0.9 and height
> 0.8 × standing height after 5 s. Results are in `evaluate_get_up.json`.

| Policy | Stood |
|---|---|
| first `get_up`, iteration 1500 | 1 of 10 |
| **`get_up_roll`, iteration 1500** | **2 of 10** |

The other 8 trials end in the same stable side-lying pose as before:
gravity z 0.158, height 0.405 × standing.

## Next

The shaping helps with returns but does not get the robot off that pose.
Options:

- more diverse or curriculum starts that begin near that pose;
- longer training;
- a vendor recovery policy run as a separate controller.
