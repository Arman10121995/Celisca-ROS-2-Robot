"""Inspect authored native actuator contracts without ROS dependencies."""
import numpy as np


def position_channels(model, robot_joints=None, allow_mobile_base=False):
    """Inspect, rather than guess, bounded position-servo command contracts."""
    import mujoco
    if not allow_mobile_base and np.any(model.jnt_type[:robot_joints] == mujoco.mjtJoint.mjJNT_FREE):
        return []  # Free-base articulation needs its own balance/base controller.
    channels = []
    for index in range(model.nu):
        gain = float(model.actuator_gainprm[index, 0])
        bias = model.actuator_biasprm[index]
        if (not model.actuator_ctrllimited[index]
                or model.actuator_dyntype[index] != mujoco.mjtDyn.mjDYN_NONE
                or model.actuator_gaintype[index] != mujoco.mjtGain.mjGAIN_FIXED
                or model.actuator_biastype[index] != mujoco.mjtBias.mjBIAS_AFFINE
                or not np.isfinite(gain) or gain <= 0 or bias[1] >= 0
                or not np.all(np.isfinite(bias[:3]))):
            continue
        lower, upper = map(float, model.actuator_ctrlrange[index])
        if not np.isfinite(lower + upper) or lower >= upper:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        if not name or name.startswith('environment_'):
            continue
        joint_name, units = '', 'source control'
        transmission = model.actuator_trntype[index]
        if transmission == mujoco.mjtTrn.mjTRN_JOINT:
            joint = int(model.actuator_trnid[index, 0])
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if (model.jnt_type[joint] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
                    and np.allclose(model.actuator_gear[index], [1, 0, 0, 0, 0, 0])
                    and np.isclose(gain, -bias[1]) and np.isclose(bias[0], 0)):
                units = 'm' if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_SLIDE else 'rad'
                if model.jnt_limited[joint]:
                    lower = max(lower, float(model.jnt_range[joint, 0]))
                    upper = min(upper, float(model.jnt_range[joint, 1]))
        elif transmission != mujoco.mjtTrn.mjTRN_TENDON:
            continue
        if lower >= upper:
            continue
        rate = min(upper - lower, .5) if units == 'rad' else min(upper - lower, .05) if units == 'm' else (upper - lower) * .2
        channels.append(dict(index=index, name=name, joint=joint_name or '', units=units,
                             limits=[lower, upper], max_rate=rate))
    return channels
