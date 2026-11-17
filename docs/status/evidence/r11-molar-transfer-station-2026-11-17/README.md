# R11 MOLAR payload transfer: PyBullet passes; station clearance corrected (November 17, 2026)

**Status: implemented, experimental.**

The MOLAR payload transfer (lift, extend the telescope, run the deck belt,
retract onto a passive station) now passes on PyBullet. It was **not
passed** that morning
([receipt](../r11-molar-pybullet-transfer-2026-11-17/README.md)).

With the final checker it passes once on each backend:

| Backend | Result | Payload centre ahead of base | Retract drag |
|---|---|---|---|
| PyBullet (run 15) | passed | 0.687 m | 1.0 mm |
| MuJoCo (2) | passed | 0.672 m | 0.0 mm |
| Gazebo (1) | passed | 0.678 m | 0.0 mm |

All three are on `nav_empty`, using `scripts/molar_station_transfer_check.py
--simulator <sim> --time-scale 2` (PyBullet: 4). A Go2 training job was
loading the CPU, and PyBullet ran at about 0.25× real time.

## What was wrong

### 1. The test's station overlapped MOLAR's extended deck frame

At full extension (0.14 m), `telescope_stage2` has a frame under the deck
plate that reaches 0.587 m from the base:

- that is 19.5 mm past the plate end (0.5675 m);
- its top is at 0.843 m, 3 mm below the plate.

The station was placed 1 cm past the plate end, with its top 5 mm below the
deck (0.844 m).

**Effect:**

- The extending telescope ran into the station and stopped at 0.130 m.
- The telescope servo (up to 250 N) kept pushing against the station.
- When the contract held and then resumed (at the roller command), that
  push came back at once. The reaction shoved the base backward (up to
  0.086 m/s and 0.17 rad/s) and the base-motion interlock correctly held
  the arm.

This was the "base yaws at roller start" seen in the morning. It came from
the test's geometry, not from the roller.

**Fix:** the station's near edge is now 1 cm past the deck's front-most
frame (`DECK_FRONT_REACH`), on every backend. With the station clear, the
base moved at most 0.5 mm/s at the roller start (run 15).

**Gazebo and MuJoCo** passed before with the overlapping station; their
contacts let it penetrate.

### 2. PyBullet ignored the declared friction of static surfaces

`sdf_world.extract_static_shapes` returned no friction, so PyBullet gave
every static shape Bullet's default of 0.5. The station declares 0.05 (a
passive roller table). Against the payload's 0.8, it held about 1 N, and
the payload stalled half-way onto the station (runs 7 and 10).

**Fix:**

- static collision shapes now report their SDF `mu`;
- PyBullet applies it when it is below 1.

**Why only below 1:** Bullet multiplies the two coefficients, while ODE
takes the smaller. Grounds declare `mu` 100, and a product with 100 would
make every wheel grip a hundredfold. Higher values keep Bullet's default,
so existing PyBullet driving is unchanged. Furniture models that declare
`mu` below 1 now get it too.

### 3. Pass rule for the new geometry

The old rule needed the payload's centre within 2 cm of being fully on the
station (`near_edge + 0.08`). With the station clear of the frame, the gap
under the payload is 3 cm. A belt with a passive station pushes only until
the payload's rear leaves the deck, so MuJoCo then stopped 5 mm short of
the old rule (run `mujoco1`, kept).

The rule is now:

- the payload has left the robot: its rear edge is past the deck end;
- it rests on the station, with its centre at least 5 cm onto it, flat at
  the station's height and inside its width;
- the retraction dragged it less than 1 cm (unchanged).

## Run log (PyBullet; all kept)

| Run | Code / setup | Result |
|---|---|---|
| 5 | morning code, `--trace` | roller refused: base spike 0.022 m/s |
| 6, 8, 9 | trial: interlock judged base motion over 0.25 s, as Isaac does | roller accepted in 6, then a real 0.04–0.09 m/s jolt; held |
| 7 | same | roller ran; payload stalled 2.4 cm short (friction 0.5) |
| 10 | + telescope stopped 5 mm short (`--extend-margin 0.005`) | no jolt; payload stalled (friction) |
| 11 | + static friction fix | **passed** (still with the margin) |
| 12 | no margin | jolt, held |
| 13 | trial: PyBullet hard stops 5 mm beyond the limits | jolt, held (hypothesis rejected; reverted) |
| 14 | station clear of the deck frame, no margin | **passed** |
| 15 | same, the 0.25 s motion-window trial reverted | **passed**; base peak 0.5 mm/s at roller start |

Both trials (the 0.25 s motion window and the wider hard stops) were
reverted. The interlock is unchanged.

**Added and kept:**

- checker options `--trace` (base twist, payload and roller at 20 Hz, by
  step) and `--extend-margin`;
- the PyBullet belt logs which robot links each loose body touches.

PyBullet traces are thinned to 5 Hz here; the full JSONs and launch logs are
on SSD under `robot_lab_runtime/molar-pybullet-yaw-20261117`.

## Limits

- One passing run per backend with the final checker, all under CPU load.
- The earlier results (Gazebo 3/3, MuJoCo 4/4 on 15 Nov; CELISCA floor on
  17 Nov) used the overlapping station and the old rule. They are kept as
  they were and were not re-run on CELISCA.
- The Isaac belt and multi-floor operation remain.
