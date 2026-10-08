"""Side-group velocity targets for a physical skid-steer base.

All wheels remain separate physics joints. This ideal encoder model cannot
measure lateral scrub: retain independent body truth and qualify turn/slip
on each backend rather than treating encoder yaw as physical tracking.
"""
import math

from .drive_kinematics import DiffDrive, DriveTargets


class SkidSteerDrive:
    kind = 'skid_steer'
    steer_joints = ()

    def __init__(self, config):
        self.left_wheels = self._group(config, 'left_wheel_joints')
        self.right_wheels = self._group(config, 'right_wheel_joints')
        names = self.left_wheels + self.right_wheels
        if len(set(names)) != len(names):
            raise ValueError('Skid-steer wheel joints must be unique across both sides')
        self.left, self.right = self.left_wheels[0], self.right_wheels[0]
        for key in ('wheel_radius', 'wheel_separation'):
            if not math.isfinite(float(config[key])) or float(config[key]) <= 0:
                raise ValueError(key + ' must be finite and positive')
        self._ideal = DiffDrive(self.left, self.right, config['wheel_radius'],
            config['wheel_separation'], **{key: config.get(key) for key in
            ('max_speed', 'max_accel', 'max_angular_speed', 'max_angular_accel')})
        self.radius, self.track = self._ideal.radius, self._ideal.track
        self._yaw_feedback = config.get('skid_yaw_rate_feedback')
        self._yaw_integral = 0.
        self._linear_integral = 0.
        if self._yaw_feedback is not None:
            if not isinstance(self._yaw_feedback,dict):
                raise ValueError('Skid yaw feedback must declare bounded gains')
            for key in ('kp','ki','max_correction','max_wheel_speed'):
                value = self._yaw_feedback[key]
                if isinstance(value,bool) or not math.isfinite(value) or value<=0:
                    raise ValueError('Positive finite skid yaw feedback '+key+' required')
            if 'linear_kp' in self._yaw_feedback:
                for key in ('linear_kp','linear_ki','max_linear_correction'):
                    value=self._yaw_feedback[key]
                    if isinstance(value,bool) or not math.isfinite(value) or value<=0:
                        raise ValueError('Positive finite skid linear feedback '+key+' required')

    @staticmethod
    def _group(config, key):
        group = config.get(key)
        if (not isinstance(group, (list, tuple)) or len(group) < 2
                or any(not isinstance(name, str) or not name.strip() for name in group)):
            raise ValueError(key + ' must name at least two physical wheel joints')
        return tuple(group)

    @property
    def wheel_joints(self):
        return list(self.left_wheels + self.right_wheels)

    def reset(self):
        self._ideal.reset()
        self._yaw_integral = 0.
        self._linear_integral = 0.

    def targets(self, vx, wz, dt=None, measured_wz=None, measured_vx=None):
        target = self._ideal.targets(vx, wz, dt)
        rates = {joint: target.velocity[self.left] for joint in self.left_wheels}
        rates.update({joint: target.velocity[self.right] for joint in self.right_wheels})
        feedback = self._yaw_feedback
        if feedback is not None and measured_wz is not None:
            if (not all(math.isfinite(value) for value in (vx,wz,measured_wz))
                    or not dt or not math.isfinite(dt) or dt<=0
                    or ('linear_kp' in feedback and
                        (measured_vx is None or not math.isfinite(measured_vx)))):
                self.reset()
                return DriveTargets({joint:0. for joint in self.wheel_joints},{},(0.,0.))
            # Neutral/watchdog commands cannot retain an integral yaw bias.
            # The existing velocity ramp still controls physical deceleration.
            if vx==0 and wz==0:
                self._yaw_integral = 0.
                self._linear_integral = 0.
            else:
                error = target.twist[1]-measured_wz
                bound = feedback['max_correction']
                self._yaw_integral = max(-bound,min(bound,
                    self._yaw_integral+feedback['ki']*error*dt))
                correction = max(-bound,min(bound,feedback['kp']*error+self._yaw_integral))
                wheel_bias = correction*self.track/(2*self.radius)
                common_bias=0.
                if 'linear_kp' in feedback:
                    linear_error=target.twist[0]-measured_vx
                    bound=feedback['max_linear_correction']
                    self._linear_integral=max(-bound,min(bound,
                        self._linear_integral+feedback['linear_ki']*linear_error*dt))
                    common_bias=max(-bound,min(bound,
                        feedback['linear_kp']*linear_error+self._linear_integral))/self.radius
                cap = feedback['max_wheel_speed']
                for joint in self.left_wheels:
                    rates[joint] = max(-cap,min(cap,rates[joint]-wheel_bias+common_bias))
                for joint in self.right_wheels:
                    rates[joint] = max(-cap,min(cap,rates[joint]+wheel_bias+common_bias))
        return DriveTargets(rates, {}, target.twist)

    def body_twist(self, measured_rates):
        """Ideal encoder estimate using every independently measured wheel."""
        if not isinstance(measured_rates, dict) or any(
                name not in measured_rates or not math.isfinite(measured_rates[name])
                for name in self.wheel_joints):
            raise ValueError('Finite measured rates required for every skid-steer wheel')
        left = sum(measured_rates[joint] for joint in self.left_wheels) / len(self.left_wheels)
        right = sum(measured_rates[joint] for joint in self.right_wheels) / len(self.right_wheels)
        return self._ideal.body_twist(left, right)


def wheel_force_limit(config, default=5.):
    """Optional simulation motor cap; preserve existing profiles' defaults."""
    value = config.get('wheel_force_limit', default)
    if isinstance(value, bool):
        raise ValueError('wheel_force_limit must be a finite positive torque')
    force = float(value)
    if not math.isfinite(force) or force <= 0:
        raise ValueError('wheel_force_limit must be a finite positive torque')
    return force


def wheel_friction_axes(config):
    """Optional Bullet link-axis friction ratios from the declared tire model."""
    values = config.get('pybullet_anisotropic_wheel_friction')
    if values is None:
        return None
    if (not isinstance(values,(list,tuple)) or len(values)!=3
            or any(isinstance(v,bool) or not isinstance(v,(int,float))
                   or not math.isfinite(v) or v <= 0 for v in values)):
        raise ValueError('Wheel anisotropic friction needs three finite positive axis ratios')
    return list(values)


def bullet_contact_parameters(config):
    """Explicit per-plant friction solver choice; legacy profiles are unchanged."""
    cone = config.get('pybullet_enable_cone_friction')
    if cone is None:
        return {}
    if not isinstance(cone,bool):
        raise ValueError('pybullet_enable_cone_friction must be a boolean')
    return {'enableConeFriction':int(cone)}
