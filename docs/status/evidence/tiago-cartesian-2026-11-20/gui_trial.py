"""Normal GUI TIAGo tool-pose screen. Set CASE_OUTPUT to an SSD run directory."""
import json
import math
import os
from pathlib import Path
import shlex
import time

from robot_lab_gui.launcher import SimulationLauncherGui
from robot_lab_gui.process_control import stop_owned_launch


def run(output):
    app = SimulationLauncherGui()
    process = None
    panel = app.urdf_arm_controls
    report, trace = {}, []

    def wait(seconds, predicate=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            app.update()
            if process is not None and process.poll() is not None:
                raise RuntimeError(f'Owned launch exited {process.returncode}')
            state = panel.state
            if state and (not trace or state['state_monotonic'] != trace[-1]['state_monotonic']):
                trace.append(state)
            if predicate and predicate():
                return
            time.sleep(.01)
        if predicate:
            raise RuntimeError(panel.plan_feedback.get() + ' · ' + panel.status.get())

    def phase(*names):
        return panel.planning().get('phase') in names

    try:
        app.robot_var.set('asset_pal_tiago')
        app.simulator_var.set('mujoco')
        app.map_var.set('nav_empty')
        app.mode_var.set('display')
        app._update_from_selection()
        app.update()
        app.show_tab('Mobile arm')
        app.update()
        report['gui_command'] = shlex.split(app.command_var.get())
        command = [arg for arg in report['gui_command'] if not arg.startswith(('gui:=', 'start_rviz:='))]
        command += ['gui:=false', 'start_rviz:=false']
        report['actual_command'] = command
        app.start_launch_command(command)
        process = app.process
        wait(50., lambda: panel.measured_tool() and not panel.state['base_inhibited'])
        wait(4.)
        wait(10., lambda: panel.plan_pose_button.instate(['!disabled']))
        panel.copy_tool_button.invoke()
        report['initial'] = panel.state
        panel.jog_tool(2, 1)
        report['goal'] = panel.current_pose()
        panel.plan_pose_button.invoke()
        wait(12., lambda: phase('ready', 'failed'))
        assert phase('ready'), panel.planning()
        report['planned'] = panel.state
        assert panel.plan_matches()
        old_x = panel.tool_xyz[0].get()
        panel.jog_tool(0, 1)
        assert not panel.execute_plan()
        panel.tool_xyz[0].set(old_x)
        panel.update_buttons()
        assert panel.execute_button.instate(['!disabled'])
        panel.execute_button.invoke()
        wait(45., lambda: phase('succeeded', 'failed'))
        report['finished'] = panel.state
        assert phase('succeeded'), panel.planning()
        error = math.dist(panel.planning()['tool']['position'], report['goal']['position'])
        report['measured_relative_position_error_m'] = error
        assert error < .006, error
        report['success'] = True
    except BaseException as exc:
        report['success'] = False
        report['error'] = repr(exc)
        raise
    finally:
        (output/'result.json').write_text(json.dumps(report, indent=2) + '\n')
        (output/'trace.json').write_text(json.dumps(trace, indent=2) + '\n')
        for control in (panel, app.arm_tab, app.hand_tab):
            if control is not None:
                control.close()
        if process is not None:
            stop_owned_launch(process)
            process.wait(timeout=20)
        for job in app.tk.call('after', 'info'):
            app.tk.call('after', 'cancel', job)
        app.destroy()


if __name__ == '__main__':
    run(Path(os.environ['CASE_OUTPUT']))
