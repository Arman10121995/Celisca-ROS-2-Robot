# Go2 get-up policy: first training (R5.2/R12)

November 14, 2026, Claude. **Not passed.**

Training: `legged_train train --robot unitree_go2 --task get_up --envs 512 --iterations 1500
--episode-s 6 --seed 1` ([run.json](run.json)). It ran 18.4 M environment steps in 65 minutes.

Mean return rose to 0.53 at iteration 1055 and ended at 0.22.

`legged_train evaluate --task get_up` now reports where each episode ended.
A trial counts as standing when gravity z < −0.9 and height > 0.8 × standing
height after 6 s. Ten trials from settled collapsed poses (seed 3):

| Policy | Stood | Typical failure |
|---|---|---|
| final, iteration 1500 ([evaluate_get_up.json](evaluate_get_up.json)) | 1 of 10 | 8 trials end at gravity z 0.16, height 0.40 × standing |
| iteration 1050 ([evaluate_get_up_01050.json](evaluate_get_up_01050.json)) | 1 of 10 | the same pose |

The policy recovers from one collapsed pose but not the recurring one that
lies tilted past 90°. Next:

- make the evaluation starts more diverse;
- shape the reward for rolling over;
- train longer, then try again.
