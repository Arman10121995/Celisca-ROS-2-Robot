# V7: swarm flocking with a peer search (November 28, 2026)

**Status: the group now stays together (implemented, experimental).
Flocking is still not passed, because the group does not travel.**

**Cause.** In the ROS2swarm flocking port, a member with no peer in its
3.5 m scan range drove straight on. In open space the members therefore
lost each other and drifted apart:
- 2.5 → 7.6 m on November 18;
- 2.5 → 11.1 m on November 22
  ([round](../v7-swarm-attraction-2026-11-22/README.md)).

**Change** (`robot_lab_utils/swarm_behaviours.py`). With no peer in range,
the member turns in place for one full turn to look for the group. If it
still sees none, it drives on for 3 s and then searches again. The other
flocking zones are unchanged. A unit test covers the search; the two
existing tests that expected `forward` with no peer now expect `search`.

**Measured** (Gazebo `nav_empty`; three Bumperbots in a 2.5 m triangle;
120 s `local_flocking`; same probe and session as November 22; revision
0e6cd7bd plus this change):

| Run | Mean pair distance start → end | Median / max over the phase | All pairs within sensing range | Group centroid travel |
|---|---|---|---|---|
| 1 | 2.49 → 1.51 m | 1.93 / 2.52 m | 92 % | 0.80 m |
| 2 | 2.49 → 2.04 m | 1.70 / 2.49 m | 100 % | 0.24 m |
| 11-22 (before) | 2.49 → 11.1 m | 9.73 / 11.1 m | 24 % | 2.82 m |

- **Cohesion holds.** The members stay together, with no pair closer than
  0.47 m.
- **No collective motion.** The group's centre moved less than 1 m in 120 s:
  members spend their time turning and searching rather than moving as a
  flock. A migration term, or alignment from sensed peer motion, is still
  needed. This remains open.

**Tried and reverted: steering while moving.** Separation and cohesion
turns were given half the cruise speed forward, so that members drift
while correcting (`steer-*-reverted-result.json`):

| Run | Mean pair distance end | Median / max | Within sensing range | Centroid travel |
|---|---|---|---|---|
| steer 1 | 2.20 m | 2.07 / 2.49 m | 100 % | 0.40 m |
| steer 2 | 4.32 m | 1.99 / 4.32 m | 79 % | 3.82 m |

Only one run travelled, and that run lost cohesion. The change is
reverted; the peer search alone stays.

Metrics are from `flock_metrics.py` (`flock-metrics.jsonl`). Traces are on
SSD under `robot_lab_runtime/v7-swarm-flocking-20261128`. Their hashes are
in `runtime-sha256.txt`.
