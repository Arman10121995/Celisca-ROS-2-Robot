"""One Harmonic world, distinct leased PX4 instances and native ROS controls."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def setup(context):
    from robot_lab_adapter.aerial_swarm_model import prepare_session
    def value(key):
        return LaunchConfiguration(key).perform(context)
    python = value('python')
    if not Path(python).is_file():
        raise ValueError('Provision the existing PX4 Python environment first: '+python)
    session = prepare_session(value('session'), value('map_name'), px4_root=value('px4_root'))
    print('[robot_lab aerial swarm] '+session['resolved_file'])
    runtime = ExecuteProcess(cmd=[python, '-m', 'robot_lab_adapter.aerial_swarm_runtime',
        '--session', session['resolved_file'], '--px4-root', value('px4_root'), '--gui', value('gui')],
        output='screen', sigterm_timeout='25', sigkill_timeout='5')
    def exited(event, context):
        if event.returncode:
            raise RuntimeError('Aerial shared-world runtime failed; see its owned session logs, exit='+str(event.returncode))
        return [EmitEvent(event=Shutdown(reason='Aerial shared-world runtime exited'))]
    return [runtime, RegisterEventHandler(OnProcessExit(target_action=runtime,on_exit=exited))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session', default_value=str(Path(share('robot_lab_bringup'))/'config/aerial_swarm_demo.yaml')),
        DeclareLaunchArgument('map_name', default_value='', description='Installed world; empty uses session map'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('python', default_value=os.environ.get('PX4_PYTHON', '/workspace/molar/robot_lab_runtime/px4/venv/bin/python3')),
        DeclareLaunchArgument('px4_root', default_value=os.environ.get('PX4_ROOT', '/workspace/molar/px4/PX4-Autopilot')),
        OpaqueFunction(function=setup),
    ])
