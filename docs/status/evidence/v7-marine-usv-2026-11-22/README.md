# V7: the catamaran USV on four backends, repeated (November 22, 2026)

**Status: 8 of 8 runs reproduce the November 20 round.** `run_round.sh`
repeats the [November 20 receipt](../r107-usv-backends-2026-11-20/README.md)
twice per backend at revision 41c57e0c:
- `lab_usv` runs its unchanged surface controller on `marine_tank`;
- the probe is the same `rov_probe.py`: hold, surge at 0.2 m/s for 10 s,
  coast for 6 s, yaw at 0.3 rad/s for 6 s, hold, a 0.1 m/s current for
  15 s, then operator loss.

`run_case.py` starts each launch in its own process group and stops that
group afterwards.

| Phase | Gazebo | MuJoCo | PyBullet | Isaac | November 20 (G / M / P / I) |
|---|---|---|---|---|---|
| Surge 10 s | 1.895, 1.891 m | 1.862, 1.871 m | 1.875, 1.875 m | 1.880, 1.886 m | 1.893 / 1.873 / 1.871 / 1.882 m |
| Coast 6 s | 0.099, 0.101 m | 0.122, 0.122 m | 0.125, 0.119 m | 0.118, 0.118 m | 0.098 / 0.121 / 0.122 / 0.118 m |
| Yaw 6 s | 0.875, 0.885 rad | 0.850, 0.862 rad | 0.856, 0.852 rad | 0.831, 0.828 rad | 0.882 / 0.845 / 0.848 / 0.830 rad |
| Current drift 15 s | 0.853, 0.852 m | 0.878, 0.870 m | 0.874, 0.875 m | 0.892, 0.895 m | 0.851 / 0.891 / 0.880 / 0.896 m |
| Operator loss | disarms ×2 | disarms ×2 | disarms ×2 | disarms ×2 | disarms |

Every value is within 2 % of November 20. The ported plants keep their
small offsets from Gazebo: about 1 % slower surge, about 20 % longer
coast, and 3–6 % less yaw. No process of any launch was left behind
(`leftover-pids.txt` in each SSD case folder).

## Limits

- One water world and one probe. The surface controller does not hold
  station against a current; the drift is free drift, as before.
- The ROV, Tethys AUV and amphibious rover rounds are not repeated here.

## Reproduce

```bash
docs/status/evidence/v7-marine-usv-2026-11-22/run_round.sh
```

Results are copied here as `<backend>-<run>-result.json`. Launch and probe
logs and traces are on SSD under `robot_lab_runtime/v7-usv-20261122`.
Their hashes are in `runtime-sha256.txt`.
