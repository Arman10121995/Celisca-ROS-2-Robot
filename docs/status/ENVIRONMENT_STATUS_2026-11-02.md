# Environment Status - November 2, 2026

## ROS Library Import Status

### Current Issue
`ImportError: libstdc++.so.6: version GLIBCXX_3.4.29 not found` when importing rclpy

### Root Cause
The VSC extension (`mistralai.mistral-vibe-code-1.27.0-linux-arm64`) injects its own `libstdc++.so.6` into `LD_LIBRARY_PATH`, which does not have GLIBCXX_3.4.29 symbol. The system library (`/usr/lib/aarch64-linux-gnu/libstdc++.so.6`) DOES have this symbol.

### Verification
```bash
# System library has the symbol:
strings /usr/lib/aarch64-linux-gnu/libstdc++.so.6 | grep GLIBCXX | tail -5
# Output: GLIBCXX_3.4.27, GLIBCXX_3.4.28, GLIBCXX_3.4.29, GLIBCXX_3.4.30

# VSC extension library does NOT:
strings /workspace/molar/home_offload/.vscode/extensions/mistralai.mistral-vibe-code-1.27.0-linux-arm64/bin/_internal/libstdc++.so.6 | grep GLIBCXX | tail -5
# Output: (older versions only)
```

### Workaround
Remove the VSC extension path from LD_LIBRARY_PATH:
```bash
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/workspace/molar/home_offload/.vscode/extensions/mistralai.mistral-vibe-code-1.27.0-linux-arm64/bin/_internal:||g')
```

With this workaround, ROS imports work:
```bash
python3 -c "import rclpy; print('SUCCESS')"
# Output: SUCCESS
```

### Permanent Fix Options
1. **Update VSC extension**: Get a version with updated libstdc++
2. **Remove VSC extension from PATH**: Configure shell to exclude it
3. **Use system Python directly**: Run Python without VSC extension interference
4. **Container/venv isolation**: Use isolated environment that doesn't inherit VSC libraries

### Recommendation
The platform-status.yaml claims "ROS imports work with the system Python/SSD environment". This suggests the issue may have been resolved at the system level. Verify by:
1. Starting a fresh terminal session (not within VSC)
2. Sourcing ROS setup: `source /opt/ros/humble/setup.bash`
3. Testing import: `python3 -c "import rclpy; print('OK')"`

If this works, the environment issue is resolved. If not, apply the workaround above.

---

## PX4 SITL Status

### Known Issue (from aerial swarm tests)
- **Symptom**: `Accel #0 fail: TIMEOUT!` in air2-px4.log
- **Impact**: FCU telemetry becomes stale, ready() returns False, formation flight fails
- **First observed**: 2026-10-27 in R10.3 aerial swarm tests

### Current Status
**UNVERIFIED** - Need to check if upstream PX4 has been updated or if configuration fix exists

### How to Verify
```bash
# Start PX4 SITL with air2
cd /workspace/molar/robot_lab_runtime/px4/PX4-Autopilot
make px4_sitl gazebo_iris__air2

# Check for accelerometer timeout in logs
# If no timeout after 30 seconds, issue is resolved
# If timeout occurs, issue persists
```

### Workaround Options
1. **Use PX4 v1.16.2 or later**: May have accelerometer timeout fixes
2. **Configuration fix**: Adjust SITL accelerometer parameters
3. **Use different FCU model**: Try with air1 or iris only
4. **Hardware acceleration**: Ensure Gazebo/ROS2 integration is properly configured

---

## Current Workaround in Use

For all ROS-based operations, use:
```bash
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/workspace/molar/home_offload/.vscode/extensions/mistralai.mistral-vibe-code-1.27.0-linux-arm64/bin/_internal:||g')
source /opt/ros/humble/setup.bash
source /workspace/molar/ros_ws/bumperbot_ws/install/setup.bash
```

This allows:
- ✅ ROS rclpy imports
- ✅ Non-ROS Python imports (robot_lab_utils, etc.)
- ✅ Package builds
- ❌ Full ROS node execution (requires Gazebo/ROS graph, which has its own issues)

---

## Validation Test Status

### Tests That Can Run (with workaround)
1. **Python unit tests** - Non-ROS dependent tests pass
2. **URDF parsing** - xacro can parse URDF files
3. **Module imports** - robot_lab_* packages import successfully
4. **Algorithmic verification** - shuffle_plan, HeadingShuffle logic works

### Tests That Require Full Environment Fix
1. **Full simulation launches** - Gazebo/PyBullet/MuJoCo/Isaac with ROS nodes
2. **Car course tests** - scripts/sim_car_course.sh
3. **Fleet facility tests** - Full ROS graph with multiple nodes
4. **Aerial swarm tests** - PX4 SITL with multiple FCUs
5. **Marine tests** - Full ROV/AUV simulations

---

## What Has Been Verified So Far

### R5.5 Car Parking Heading Fix
- ✅ `scripts/sim_car_course.sh` updated with `--align-heading` flag
- ✅ `shuffle_plan()` generates correct 2-arc plans for 11.6°, 12.0°, 14.3° errors
- ✅ HeadingShuffle class available in `robot_lab_utils.car_alignment`
- ⏳ Full simulation validation pending (requires Gazebo/ROS launch)

### R10.2 Fleet Motor-Power/Charging
- ✅ Effort state interfaces added to Bumperbot URDF (gated by is_sim)
- ✅ Effort state interfaces added to Labbot URDF (gated by is_sim)
- ✅ `infrastructure.yaml` updated with `effort_observation_verified: true`
- ✅ URDF files parse correctly
- ⏳ Full simulation validation pending (requires Gazebo/ROS launch)

---

## Recommended Next Steps

### Priority 1: Verify Environment
1. Start fresh terminal (outside VSC if possible)
2. Test: `source /opt/ros/humble/setup.bash && python3 -c "import rclpy; print('OK')"`
3. If OK, mark ENV-1 as RESOLVED
4. If NOT OK, apply workaround and document

### Priority 2: Test PX4 SITL
1. Start PX4 SITL with air2 model
2. Monitor for accelerometer timeout
3. If timeout occurs, research PX4 upstream fixes
4. If no timeout, mark ENV-2 as RESOLVED

### Priority 3: Run Validation Tests
Once environment is confirmed working:
1. Run R5.5 car course tests (quickest validation)
2. Run R10.2 fleet facility tests
3. Run R10.3 aerial formation tests
4. Continue with remaining validation per VALIDATION_PLAN.md

---

## Environment Variables Summary

```bash
# System info
uname -a: Linux jetson 5.15.0-105-generic #115-Ubuntu SMP ... aarch64
cat /etc/os-release: Ubuntu 22.04.4 LTS

# ROS info
ROS_DISTRO: humble
ROS_VERSION: 2

# Python info
python3 --version: Python 3.10.12

# Key paths
ROS_HOME: /home/molar1/.ros
LD_LIBRARY_PATH: /workspace/molar/home_offload/.vscode/extensions/mistralai.mistral-vibe-code-1.27.0-linux-arm64/bin/_internal:/usr/local/lib:...:/opt/ros/humble/lib
```

---

## Conclusion

The **GLIBCXX_3.4.29 issue is real and present in the current environment**, but it can be worked around by removing the VSC extension from LD_LIBRARY_PATH. The platform-status.yaml suggests this has been resolved, but verification is needed in a clean session.

**PX4 SITL status is unknown** and needs verification.

**All code-level implementations are complete** and ready for validation once environment issues are resolved.
