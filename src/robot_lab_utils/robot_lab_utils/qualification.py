"""Acceptance checks over measured simulation reports, never generated scores."""
import math


def inspect_drive_mode(node, drive_node, expected_mode):
    """Read the running adapter's actual config before attributing a trial."""
    import json
    import rclpy
    from rcl_interfaces.srv import GetParameters
    report={'node':drive_node,'expected_mode':expected_mode,'passed':False}
    client=node.create_client(GetParameters,drive_node.rstrip('/')+'/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Running drive parameter service unavailable')
        request=GetParameters.Request();request.names=['drive_config']
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('Running drive configuration query timed out')
        config=json.loads(future.result().values[0].string_value)
        report['configuration']=config
        report['passed']=config.get('type')=='four_wheel_steer' and config.get('steering_mode')==expected_mode
    except (ValueError,RuntimeError,IndexError) as exc:
        report['error']=str(exc)
    finally:
        node.destroy_client(client)
    return report


def navigation_acceptance(report, position_m, yaw_deg):
    checks = {'goal_succeeded': report.get('outcome') == 'succeeded'}
    for name, key, limit in (
            ('truth_position', 'final_error_truth_m', position_m),
            ('truth_heading', 'final_yaw_error_truth_deg', yaw_deg)):
        value = report.get(key)
        checks[name] = (isinstance(value, (int, float)) and
                        math.isfinite(value) and abs(value) <= limit)
    age = report.get('truth_age_wall_s')
    checks['fresh_truth'] = (isinstance(age, (int, float)) and
                             math.isfinite(age) and 0 <= age < 1.0)
    return {'passed': all(checks.values()), 'checks': checks,
            'limits': {'position_m': position_m, 'yaw_deg': yaw_deg,
                       'truth_age_wall_s': 1.0}}
