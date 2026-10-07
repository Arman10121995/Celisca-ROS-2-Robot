"""Exercise real normal-installed Tk selectors/defaults/autofill after proof.

These are UI checks. Physical mapping/navigation and Drive trials are archived
separately. No provisional catalog or experimental policy flag is used here.
"""
import hashlib
import json
import os
from pathlib import Path
import shlex

from robot_lab_gui.launcher import SimulationLauncherGui

out = Path(os.environ['PROBE_NORMAL_OUT'])
out.mkdir(parents=True, exist_ok=True)
app = SimulationLauncherGui()
app.update()
rows, compatibility = [], []
try:
    for robot in ('asset_turtlebot3_burger', 'asset_turtlebot3_waffle', 'asset_turtlebot3_waffle_pi'):
        profile = app.robot_profiles[robot]
        for backend in ('gazebo', 'mujoco', 'pybullet', 'isaac'):
            assert profile['supported_modes_by_simulator'][backend] == ['display', 'loc', 'slam', 'nav']
            compatibility.append(dict(robot=robot, backend=backend, modes=profile['supported_modes_by_simulator'][backend],
                unavailable='3d_slam: original simulation cameras are RGB-only; no qualified depth/export workflow'))
            for mode in ('display', 'loc', 'slam', 'nav'):
                app.robot_var.set(robot)
                app.simulator_var.set(backend)
                app.map_var.set('nav_empty')
                app.mode_var.set(mode)
                app._update_from_selection()
                app.update()
                command = shlex.split(app.command_var.get())
                algorithms = {key: value.get() for key, value in app.slot_vars.items()}
                assert (app.robot_var.get(), app.simulator_var.get(), app.mode_var.get()) == (robot, backend, mode)
                assert app.start_button.instate(['!disabled']), app.validation_var.get()
                for token in ('robot_model:='+robot, 'simulator:='+backend, 'mode:='+mode, 'map_name:=nav_empty',
                              'spawn_x:=-4.0', 'spawn_y:=-4.0', 'spawn_yaw:=0.0'):
                    assert token in command, (token, command)
                assert not any(token.startswith('enable_experimental_') for token in command)
                if mode in ('loc', 'nav'):
                    assert 'amcl' in algorithms.values(), algorithms
                if mode == 'slam':
                    assert 'slam_toolbox' in algorithms.values(), algorithms
                if mode == 'nav':
                    assert 'a_star_planner' in algorithms.values() and 'pure_pursuit' in algorithms.values(), algorithms
                rows.append(dict(robot=robot, backend=backend, mode=mode,
                    command=app.command_var.get(), algorithms=algorithms, run_enabled=True))
            # A named-map default must not leak when an operator changes maps.
            app.map_var.set('celisca_floor_1')
            app.mode_var.set('display')
            app._update_from_selection()
            app.update()
            command = shlex.split(app.command_var.get())
            assert 'map_name:=celisca_floor_1' in command
            assert not any(token.startswith('spawn_') for token in command), command
    assert len(rows) == 48
finally:
    (out/'producer.py').write_bytes(Path(__file__).read_bytes())
    (out/'report.json').write_text(json.dumps(dict(
        scope='Actual normal-installed Tk selectors, compatible algorithm defaults, command and Run state; physical missions separate',
        rows=rows, compatibility=compatibility,
        producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()), indent=2)+'\n')
    app._on_close()
print('Normal installed TurtleBot3 GUI selections checked:', len(rows))
