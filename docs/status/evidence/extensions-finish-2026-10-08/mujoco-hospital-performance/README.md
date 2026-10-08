# MuJoCo hospital timing and lossless camera packing

This is a partial P4 result. Native model, camera resolution/rates, scan,
actuators, collision geometry, control timing and navigation gates are unchanged.
Large-map physics performance remains open.

The original [actual clock window](report.json) measures 0.130493× real time:
1.892 simulated seconds over 14.499 monotonic wall seconds, nine scans and ten
depth frames. The [after-packing window](clock-after.json) measures 0.130892×:
1.956 simulated seconds over 14.944 wall seconds, nine scans and nine depth
frames. Both observe normal Labbot/MuJoCo/hospital navigation with the same
first/last clock-arrival algorithm and bounded 15-second window. Different
route phases and run variability remain; these values establish no sustained
or meaningful whole-simulator speed improvement.

The new window also receives a real 320×240 `32FC1` optical-frame image:
76,800 finite depth pixels, 0.392–10.346 m, expected stride and byte length.
No observer publishes velocity or goals. [Producer](after-producer.py) and
[artifact hashes](after-artifacts.json) retain the measurements.

## Camera message construction

Humble's generated `Image.data` setter checks a byte string element by element
twice in Python. It accepts a `uint8` array directly. The shared helper now
uses that supported typed array while retaining identical serialized data.

| Actual installed 320×240 builder | Before median (ms) | After median (ms) |
|---|---:|---:|
| RGB `rgb8` | 30.051 | 0.060 |
| Depth `32FC1` | 40.019 | 0.070 |

These are 20 actual construction samples per encoding, from the same
[microbenchmark producer](image-packing-producer.py), with source and wire
hashes in [before](image-packing-before.json) and [after](image-packing-after.json).
Both complete ROS serialized messages are byte-identical. This is a message
construction result, separate from simulator real-time factor. Four
[ROS serialization cases](image-wire-tests.log) also preserve contiguous and
strided RGB/depth, NaN payload bits, infinities, signed zero, header and stride.

## Actual native profiling and route repeat

The opt-in [timing hook](sitecustomize.py) profiles only the normal spawner's
physics thread; no production code/rate/model setting is changed. It records
the compiled scene hash on SSD. Instrumentation overhead remains in both
[before](profile-before.json) and [after](profile-after.json) reports; compare
these separately from uninstrumented clock observations.

Both use MuJoCo 3.12, 191 geoms / 64 meshes / 200 flexes, no viewer, unchanged
5 Hz scan/RGB-D and 320×240 camera. The before loop takes 39.181 s over 4.364
simulated seconds, including 23.599 s in native `mj_step` and 14.041 s in the
camera callback. The after loop takes 31.246 s over 4.328 simulated seconds,
with 23.404 s in `mj_step` and 6.320 s in the camera callback. Removing the
per-byte assertions also removes substantial profiling overhead, so this
profile difference is not a general speed benchmark. Native step remains
the largest measured cost. Collision, constraints and integration inside
that native call need finer profiling and validated optimization.

The normal GUI [post-change hospital route](../navigation/camera-packing/labbot-dataset_hospital-mujoco/report.json)
passes the unchanged 0.15 m / 5° fresh-body/floor/settle gates: 0.046 m / +1.97°,
Nav2 success in 79.0 action wall seconds, 419 body samples, zero root exit.
This is a new exact source stage. The other seven hospital cells retain their
earlier source stages; no whole matrix or higher mapping mission is rerun by
this sensor-packing change. System joystick child shutdown errors remain in
raw logs. Original [baseline artifacts](profile-before-artifacts.json) stay intact.

The full sourced integration tier passes **151 checks** with two retained
OpenGL deprecation warnings; [complete log](integration-after.log).
The [source/profile comparison](source-stage.json) verifies that only
`camera_msgs.py` changes runtime bytes; map/robot profiles remain identical.
[76 actual normal GUI selections](gui-report.json) preserve measured
Husky/TurtleBot3/4 modes and autofilled commands, and Health displays P4 as
partial with the real measurements. No robot is launched by that GUI check.
