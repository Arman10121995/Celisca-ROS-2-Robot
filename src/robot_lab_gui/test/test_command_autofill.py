"""Exercise command autofill through real Tk controls (run with xvfb-run)."""

import os
from pathlib import Path
import shlex
import sys
import sqlite3
from subprocess import CompletedProcess
from unittest.mock import Mock, patch

import pytest

pytest.importorskip("tkinter")
pytest.importorskip("ament_index_python")

SRC = Path(__file__).resolve().parents[2]
for package in (SRC / "robot_lab_gui", SRC / "robot_lab_adapter",
                SRC / "robot_lab" / "robot_lab_registry"):
    sys.path.insert(0, str(package))

from robot_lab_gui import launcher  # noqa: E402
from robot_lab_gui.gui_composition import get_registry  # noqa: E402

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="Requires Xvfb or a display"),
]


def test_named_map_spawn_is_visible_in_command_and_does_not_leak_to_other_map(app):
    app.robot_profiles['bumperbot']['spawn_by_map'] = {'nav_empty': {'x': -4., 'y': -4.}}
    app.robot_var.set('bumperbot'); app.simulator_var.set('mujoco')
    app.map_var.set('nav_empty'); app.mode_var.set('display'); app._update_from_selection()
    arguments = shlex.split(app.command_var.get())
    assert 'spawn_x:=-4.0' in arguments and 'spawn_y:=-4.0' in arguments
    assert 'map_name:=nav_empty' in arguments
    app._set_command(['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'spawn_x:=1.5'])
    assert [v for v in app._prepared_command if v.startswith('spawn_x:=')] == ['spawn_x:=1.5']
    app.map_var.set('celisca_floor_1'); app._update_from_selection()
    arguments = shlex.split(app.command_var.get())
    assert 'map_name:=celisca_floor_1' in arguments
    assert not any(v.startswith(('spawn_x:=', 'spawn_y:=')) for v in arguments)


@pytest.fixture
def app():
    registry = get_registry(str(SRC / "robot_lab" / "robot_lab_registry" / "config"))

    def package_share(name):
        if name == "robot_lab_registry":
            return str(SRC / "robot_lab" / name)
        return str(SRC / name)

    available = {sim: (True, "") for sim in launcher.SIMULATOR_ORDER}
    with patch.object(launcher, "get_package_share_directory", side_effect=package_share), \
            patch.object(launcher, "get_registry", return_value=registry), \
            patch.object(launcher, "ensure_defaults"), \
            patch.object(launcher, "_available_simulators", return_value=available), \
            patch.object(launcher, "_allowed_simulators", return_value=available), \
            patch("robot_lab_gui.lab_tabs.create_tabs"):
        gui = launcher.SimulationLauncherGui()
        gui.update()
        try:
            yield gui
        finally:
            for job in gui.tk.call("after", "info"):
                # Cancel in Tcl without deleting the callback command twice:
                # each widget removes its own registered commands on destroy.
                gui.tk.call("after", "cancel", job)
            gui.destroy()


def widgets(parent):
    for child in parent.winfo_children():
        yield child
        yield from widgets(child)


def select(app, combo, value):
    combo.set(value)
    combo.event_generate("<<ComboboxSelected>>")
    app.update()


def displayed_command(app):
    text = app.command_preview.get("1.0", "end-1c")
    assert text == app.command_var.get()
    return shlex.split(text)


def test_command_is_filled_and_run_is_visible_on_open(app):
    command = displayed_command(app)
    assert command[:4] == [
        "ros2", "launch", "robot_lab_bringup", "simulated_robot.launch.py"]
    assert f"robot_model:={app.robot_var.get()}" in command
    assert f"mode:={app.mode_var.get()}" in command
    assert f"simulator:={app.simulator_var.get()}" in command
    assert "gui:=auto" in command
    assert app.start_button.instate(["!disabled"])
    assert app.copy_command_button.instate(["!disabled"])
    assert app.start_button.winfo_viewable()
    assert app.start_button.winfo_rooty() < app.winfo_rooty() + app.winfo_height()


def test_import_without_base_controller_does_not_enable_twist_inputs(app):
    profile = dict(app.robot_profiles['bumperbot'])
    profile.update(source_id='test_vendor', supported_modes=['display'], features=[])
    profile.pop('drive', None)
    app.robot_profiles['imported_display_fixture'] = profile
    app.robot_var.set('imported_display_fixture')
    app.mode_var.set('display')
    app._update_from_selection()
    assert all(button.instate(['disabled']) for button in app.drive_button_widgets.values())
    assert app.drive_input_checkbox.instate(['disabled'])
    with patch.object(app, '_publish_drive') as publish:
        app._start_drive(1., 0.)
        app.drive_input_enabled.set(True)
        app._toggle_drive_input()
        assert not app.drive_input_enabled.get()
        publish.assert_not_called()
    app.robot_var.set('bumperbot')
    app._update_from_selection()
    assert all(button.instate(['!disabled']) for button in app.drive_button_widgets.values())
    assert app.drive_input_checkbox.instate(['!disabled'])


def test_backend_qualification_gates_modes_and_preserves_real_algorithm_defaults(app):
    profile = dict(app.robot_profiles['bumperbot'])
    profile['supported_modes_by_simulator'] = {
        'mujoco': ['display'], 'pybullet': ['display', 'loc']}
    app.robot_profiles['bumperbot'] = profile
    app.robot_var.set('bumperbot'); app.map_var.set('nav_empty')
    app.simulator_var.set('mujoco'); app.mode_var.set('loc')
    app._update_from_selection(); app.update()
    assert app.mode_var.get() == 'display'
    assert 'qualified loc support in mujoco' in app._mode_reasons()['loc']
    app.simulator_var.set('pybullet'); app.mode_var.set('loc')
    app._update_from_selection(); app.update()
    assert app.mode_var.get() == 'loc'
    assert app.slot_vars['localizer'].get() == 'amcl'
    assert 'localization:=amcl' in displayed_command(app)


def test_hand_fixture_and_reset_require_the_selected_native_panda(app):
    from robot_lab_gui.arm_tab import ArmTab
    from robot_lab_gui.hand_tab import HandTab
    profile = dict(app.robot_profiles['bumperbot'])
    profile.update(supported_modes=['display'], supported_simulators=['mujoco'],
                   features=['joint_control'], source_id='test_vendor', arm_control='panda', hand_control='panda')
    profile.pop('drive', None)
    app.robot_profiles['menagerie_franka_emika_panda'] = profile
    app.arm_tab = ArmTab(app.control_notebook, app)
    app.hand_tab = HandTab(app.control_notebook, app)
    try:
        app.robot_var.set('menagerie_franka_emika_panda')
        app.simulator_var.set('mujoco'); app.map_var.set('nav_empty'); app.mode_var.set('display')
        app._update_from_selection(); app.update()
        assert app.hand_tab.selected()
        assert app.hand_tab.fixture_button.instate(['!disabled'])
        assert all(b.instate(['disabled']) for b in
                   (app.hand_tab.open_button, app.hand_tab.close_button, app.hand_tab.stop_button))
        app.hand_tab.fixture_button.invoke(); app.update()
        assert 'grasp_fixture:=true' in displayed_command(app)
        app._launch_running = True; app._update_reset_button()
        assert app.reset_robot_button.instate(['!disabled'])
        app._launch_running = False
        app.robot_var.set('bumperbot'); app._update_from_selection(); app.update()
        assert not app.hand_tab.selected()
        assert app.hand_tab.fixture_button.instate(['disabled'])
        assert 'grasp_fixture:=true' not in displayed_command(app)
        app.simulator_var.set('pybullet'); app.robot_var.set('menagerie_franka_emika_panda')
        # Direct selection counterexample; do not auto-correct it back to MuJoCo.
        assert not app.hand_tab.selected()
    finally:
        app._launch_running = False
        app.hand_tab.close(); app.arm_tab.close()


def test_qualified_panda_planning_autofill_and_fixture_fallback(app):
    from robot_lab_gui.arm_tab import ArmTab
    from robot_lab_gui.hand_tab import HandTab
    profile = dict(app.robot_profiles['bumperbot'])
    profile.update(supported_modes=['display'], supported_simulators=['mujoco'],
                   features=['joint_control'], source_id='test_vendor',
                   arm_control='panda', hand_control='panda', arm_planning='moveit')
    profile.pop('drive', None)
    app.robot_profiles['menagerie_franka_emika_panda'] = profile
    app.arm_tab = ArmTab(app.control_notebook, app)
    app.hand_tab = HandTab(app.control_notebook, app)
    try:
        app.robot_var.set('menagerie_franka_emika_panda')
        app.simulator_var.set('mujoco')
        app.mode_var.set('display')
        app._update_from_selection(); app.update()
        assert 'arm_control:=panda' in displayed_command(app)
        assert 'arm_planning:=moveit' in displayed_command(app)
        assert app.arm_tab.cartesian.plan_button.instate(['disabled'])  # No owned/fresh state.
        app.hand_tab.fixture_button.invoke(); app.update()
        assert 'grasp_fixture:=true' in displayed_command(app)
        assert 'arm_planning:=none' in displayed_command(app)
        app.robot_var.set('bumperbot'); app._update_from_selection(); app.update()
        assert not any(part.startswith('arm_planning:=') for part in displayed_command(app))
    finally:
        app.hand_tab.close(); app.arm_tab.close()


def test_stale_cartesian_callback_cannot_replace_a_new_plan_after_stop(app):
    from robot_lab_gui.arm_tab import ArmTab
    arm = ArmTab(app.control_notebook, app)
    try:
        controls = arm.cartesian
        old = controls.generation
        arm.stop()  # Invalidates the in-flight request before any result.
        controls.planning = True  # A later request now owns this generation.
        current = controls.generation
        future = Mock()
        controls.got_plan(future, old)
        controls.got_fk(future, old, [0, 0, .05], None)
        controls.got_ik(future, old, None)
        controls.got_current_pose(future, old, {})
        future.result.assert_not_called()
        assert controls.generation == current and controls.planning
        assert controls.points is None
    finally:
        arm.close()


def test_cartesian_axis_target_edits_invalidate_execution_and_reject_large_steps(app):
    from robot_lab_gui.arm_tab import ArmTab
    arm = ArmTab(app.control_notebook, app)
    try:
        cart = arm.cartesian
        cart.reference = (.4, .0, .5)
        cart.points = [(1., [0.]*7)]
        with patch.object(cart, 'ready', return_value=True), patch.object(cart, 'plan') as plan:
            cart.step_target(0, 1)
            assert cart.offset[0].get() == .01
            assert '0.410' in cart.target_display.get()
            assert cart.points is None and cart.execute_button.instate(['disabled'])
            cart.step.set(.5)
            cart.step_target(2, 1)
            assert cart.offset[2].get() == 0.
            assert '1–50 mm' in cart.status.get()
            plan.assert_not_called()  # Target selection never starts an arm trajectory.
        with patch.object(cart, 'ready', return_value=False):
            cart.step.set(.01)
            cart.step_target(0, 1)
            assert cart.offset[0].get() == .01
    finally:
        arm.close()


def test_live_monitor_uses_its_own_executor_and_disconnects_before_destroy(app):
    import threading
    import time
    import rclpy
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from robot_lab_gui.live_monitor import LiveMonitorTab
    initialized_here = not rclpy.ok()
    assert app._ensure_ros_publisher()
    monitor = LiveMonitorTab(app.notebook, app)
    gui_threads, monitor_threads = [], []
    main_thread = threading.get_ident()
    receive = monitor._on_clock
    def measured_clock(message):
        monitor_threads.append(threading.get_ident())
        receive(message)
    monitor._on_clock = measured_clock
    app.ros_node.create_subscription(String, '/robot_lab/test_tk_callback',
        lambda message: gui_threads.append(threading.get_ident()), 10)
    gui_pub = app.ros_node.create_publisher(String, '/robot_lab/test_tk_callback', 10)
    clock_pub = app.ros_node.create_publisher(Clock, '/clock', 10)
    try:
        for _ in range(2):
            monitor._connect()
            end = time.monotonic()+5
            while time.monotonic() < end and (len(gui_threads)<5 or len(monitor_threads)<5):
                gui_pub.publish(String(data='measured callback'))
                clock_pub.publish(Clock())
                app.update()
                for _ in range(8): rclpy.spin_once(app.ros_node, timeout_sec=0.)
                time.sleep(.02)
            assert gui_threads and monitor_threads
            assert set(gui_threads) == {main_thread}
            assert main_thread not in monitor_threads
            monitor._disconnect()
            assert not monitor._ros_thread.is_alive() and monitor._node is None
            assert not monitor._subscriptions
            gui_threads.clear(); monitor_threads.clear()
    finally:
        monitor.shutdown()
        app.ros_node.destroy_node(); app.ros_node = None
        if initialized_here: rclpy.shutdown()


def test_robot_reset_stops_drive_and_is_gated_by_task(app):
    app.robot_var.set('four_wheel_steer_car')
    app.mode_var.set('loc')
    app._update_reset_button()
    assert app.reset_robot_button.instate(['disabled'])
    app._launch_running = True
    app._update_reset_button()
    assert app.reset_robot_button.instate(['!disabled'])
    with patch.object(app, '_stop_drive') as stop, patch.object(launcher.threading, 'Thread') as thread:
        app.reset_robot_button.invoke()
        stop.assert_called_once_with()
        command, label = thread.call_args.kwargs['args']
        assert command[5:] == ['/robot_lab/reset', 'std_srvs/srv/Trigger', '{}']
        assert command[:2] == ['timeout', '30']
    for robot, mode in [('four_wheel_steer_car','nav'), ('px4_x500','flight'),
                        ('berkeley_humanoid_lite_sim','loc')]:
        app.robot_var.set(robot)
        app.mode_var.set(mode)
        app._update_reset_button()
        assert app.reset_robot_button.instate(['disabled'])
    app._launch_running = False


def test_drone_altitude_buttons_ramp_both_directions_and_stop(app):
    app.robot_var.set('px4_x500')
    app.mode_var.set('flight')
    app.simulator_var.set('gazebo')
    app._update_from_selection()
    with patch.object(app,'_publish_drive') as publish:
        up, down = app.drive_altitude_widgets[1.0], app.drive_altitude_widgets[-1.0]
        assert up.cget('text') == 'Altitude Up'
        assert down.cget('text') == 'Altitude Down'
        up.invoke()
        assert app.current_vertical>0
        first = app.current_vertical
        app._repeat_drive()
        assert app.current_vertical>first
        assert publish.call_args.args[3]>0
        up.invoke()
        for _ in range(5):
            app._repeat_drive()
        assert app.current_vertical == 0
        down.invoke()
        assert publish.call_args.args[3]<0
        app._stop_drive()
        assert app.current_vertical == 0
        assert not app.drive_altitude_buttons
        publish.assert_called_with(0.0,0.0,0.0)
        app.robot_var.set('bumperbot')
        app._update_from_selection()
        assert up.instate(['disabled']) and down.instate(['disabled'])
        app._start_altitude(1.0)
        assert not app.drive_altitude_buttons


def test_drone_seven_directions_share_drive_state_stop_and_gentle_yaw(app):
    app.robot_var.set('px4_x500'); app.mode_var.set('flight')
    app.simulator_var.set('gazebo'); app._update_from_selection()
    pad = app.drone_drive_widgets
    assert set(pad) == {'Forward', 'Reverse', 'Turn Left', 'Turn Right', 'Strafe L', 'Strafe R', 'Stop'}
    assert all(button.instate(['!disabled']) for button in pad.values())
    with patch.object(app, '_publish_drive') as publish, \
            patch.object(app.drive_joystick, 'poll', return_value=(0., 0.)):
        app.drive_input_checkbox.invoke()
        app._repeat_drive()
        publish.assert_not_called()
        pad['Turn Left'].invoke()
        app._repeat_drive()  # The neutral input poll already owns the scheduled tick.
        assert app.current_drive[1] == .05
        assert app.drive_button_widgets[(0., 1.)].instate(['pressed'])
        for _ in range(20):
            app._repeat_drive()
        assert app.current_drive[1] == .3
        app.drive_button_widgets[(0., 1.)].invoke()
        assert pad['Turn Left'].instate(['!pressed'])
        pad['Stop'].invoke()
        for label, value in [('Forward', 1), ('Reverse', -1)]:
            pad[label].invoke()
            assert app.current_drive[0]*value > 0
            pad['Stop'].invoke()
        pad['Turn Right'].invoke()
        assert app.current_drive[1] == -.05
        pad['Stop'].invoke()
        pad['Strafe L'].invoke()
        assert app.current_lateral > 0
        assert app.strafe_left_button.instate(['pressed'])
        app.strafe_left_button.invoke()
        assert pad['Strafe L'].instate(['!pressed'])
        pad['Strafe R'].invoke()
        for _ in range(3):
            app._repeat_drive()  # Reverse input brakes before changing sign.
        assert app.current_lateral < 0
        pad['Stop'].invoke()
        assert app.current_drive == (0., 0.) and app.current_lateral == 0.
        assert all(button.instate(['!pressed']) for button in pad.values())
        publish.assert_called_with(0., 0., 0.)
    app.robot_var.set('bumperbot'); app._update_from_selection()
    assert all(button.instate(['disabled']) for button in pad.values())


def test_3d_save_uses_flushed_backup_and_installed_cloud_exporter(app, tmp_path):
    source = tmp_path/'live.db'
    source.touch()
    with sqlite3.connect(str(source)+'.back') as database:
        database.execute('CREATE TABLE keyframes (id INTEGER)')
        database.execute('INSERT INTO keyframes VALUES (42)')
    target = tmp_path/'saved.db'
    with patch.object(launcher.subprocess,'run',return_value=CompletedProcess([],0,'response: Empty_Response()','')) as run, \
            patch.object(app,'_run_aux_command') as exporter:
        app._snapshot_3d_map(source,target)
        assert run.call_args.args[0][3] == '/rtabmap/backup'
        assert exporter.call_args.args[0] == ['ros2','run','robot_lab_bringup','export_3d_map.py',
                                              '--cloud-topic','/cloud_map','--output',str(target.with_suffix('.pcd'))]
    with sqlite3.connect(target) as saved:
        assert saved.execute('SELECT id FROM keyframes').fetchall() == [(42,)]


def test_console_flood_leaves_tk_events_and_stop_responsive(app):
    for i in range(1000):
        app.output_queue.put(('line', f'ROS warning {i}\n'))
    with patch.object(app, 'after') as after:
        app._poll_output()
    assert app.output_queue.qsize() >= 800
    after.assert_called_with(100, app._poll_output)
    responsive = []
    app.after_idle(lambda: responsive.append(True))
    app.update_idletasks()
    assert responsive == [True]
    app._append_output('older line\n' * 10050 + 'most recent line\n')
    assert int(app.output.index('end-1c').split('.')[0]) <= 10000
    assert 'most recent line' in app.output.get('end-3l', 'end')


def test_launch_exit_bypasses_console_backlog_and_cannot_stop_a_new_run(app):
    old_process, current_process = Mock(), Mock()
    app.process = current_process
    app._launch_running = True
    for _ in range(1000):
        app.output_queue.put(('line', 'noisy node\n'))
    app._lifecycle_queue.put((old_process, 0))
    with patch.object(app, 'after'), patch.object(app, '_stop_drive') as stop:
        app._poll_output()
        assert app._launch_running
        stop.assert_not_called()
        app._lifecycle_queue.put((current_process, 0))
        app._poll_output()
        assert not app._launch_running
        assert app.output_queue.qsize() >= 600
        assert app.status_var.get() == 'Idle'
        stop.assert_called_once()
    app.process = None


def test_stopping_launch_reaches_its_owned_auxiliary_process(app, tmp_path):
    import threading
    import time
    pid_file = tmp_path/'export.pid'
    command = [sys.executable, '-c',
               'import os,sys,time; open(sys.argv[1], "w").write(str(os.getpid())); time.sleep(60)',
               str(pid_file)]
    thread = threading.Thread(target=app._run_aux_command, args=(command, 'test export'))
    thread.start()
    try:
        deadline = time.monotonic()+5
        while not pid_file.exists() and time.monotonic()<deadline:
            time.sleep(.01)
        assert pid_file.exists()
        with app._aux_lock:
            process = next(iter(app._aux_processes))
        assert process.pid == int(pid_file.read_text())
        # This also applies after the launch leader has already exited.
        app._stop_launch()
        thread.join(timeout=5)
        assert not thread.is_alive()
        assert process.poll() is not None
        assert not app._aux_processes
    finally:
        app._stop_aux_commands()
        thread.join(timeout=5)


def test_drive_pad_and_wasd_use_incremental_speed_and_release_ramp(app):
    app.drive_override_var.set(True)
    with patch.object(app, "_publish_drive") as publish, \
            patch.object(app.drive_joystick, "poll", return_value=(0.0, 0.0)):
        app._start_drive(1.0, 0.0)
        assert app.current_drive == (0.025, 0.0)
        app._repeat_drive()
        assert app.current_drive == (0.05, 0.0)
        app._start_drive(1.0, 0.0)  # second click releases the latched button
        app._repeat_drive()
        assert app.current_drive == (0.025, 0.0)
        app._repeat_drive()
        assert app.current_drive == (0.0, 0.0)

        app.drive_input_enabled.set(True)
        app._toggle_drive_input()
        app._drive_key_press(Mock(keysym="w", widget=app))
        app._repeat_drive()
        assert app.current_drive == (0.025, 0.0)
        app._drive_key_press(Mock(keysym="a", widget=app))
        app._repeat_drive()
        assert app.current_drive == (0.05, 0.08)
        app._drive_key_release(Mock(keysym="w"))
        app._drive_key_release(Mock(keysym="a"))
        app._repeat_drive()
        assert app.current_drive == (0.025, 0.0)
        app._stop_drive()
        assert app.current_drive == (0.0, 0.0)
        assert publish.call_args.args == (0.0, 0.0, 0.0)
    app.drive_input_enabled.set(False)
    app._toggle_drive_input()


def test_space_stops_without_disabling_keyboard(app):
    app.drive_input_enabled.set(True)
    with patch.object(app, "_publish_drive") as publish, \
            patch.object(app.drive_joystick, "poll", return_value=(0.0, 0.0)):
        app._drive_key_press(Mock(keysym="w", widget=app))
        assert app.current_drive[0] > 0
        app._drive_key_press(Mock(keysym="space", widget=app))
        assert app.current_drive == (0.0, 0.0)
        assert app.drive_input_enabled.get()
        assert publish.call_args.args == (0.0, 0.0, 0.0)


def test_arming_keyboard_joystick_does_not_publish_until_driven(app):
    with patch.object(app, "_publish_drive") as publish, \
            patch.object(app.drive_joystick, "poll", return_value=(0.0, 0.0)):
        app.drive_input_enabled.set(True)
        app._toggle_drive_input()
        app._repeat_drive()
        publish.assert_not_called()

        app._drive_key_press(Mock(keysym="w", widget=app))
        app._repeat_drive()
        assert app.current_drive[0] > 0.0
        app._drive_key_release(Mock(keysym="w"))
        for _ in range(50):
            app._repeat_drive()
            if app.current_drive == (0.0, 0.0):
                break
        assert publish.call_args.args == (0.0, 0.0, 0.0)
        publish.reset_mock()
        app._repeat_drive()
        publish.assert_not_called()
        app.drive_input_enabled.set(False)
        app._toggle_drive_input()


def test_drive_button_stays_visually_pressed_until_second_click(app):
    button = app.drive_button_widgets[(1.0, 0.0)]
    with patch.object(app, "_publish_drive"):
        for _ in range(2):
            button.event_generate("<ButtonPress-1>", x=4, y=4)
            button.event_generate("<ButtonRelease-1>", x=4, y=4)
            app.update()
            assert button.instate(["pressed"]) == ((1.0, 0.0) in app.drive_buttons)


@pytest.mark.parametrize("label,value", [("GUI", "true"), ("Headless", "false"), ("Auto", "auto")])
def test_gui_radio_immediately_updates_command(app, label, value):
    for widget in widgets(app):
        if isinstance(widget, launcher.ttk.Radiobutton) and widget.cget("text") == label:
            widget.invoke()
            break
    else:
        pytest.fail(f"Missing {label} radio button")
    assert f"gui:={value}" in displayed_command(app)


def test_command_tracks_robot_map_mode_backend_and_planner(app):
    select(app, app.robot_combo, "labbot")
    assert "robot_model:=labbot" in displayed_command(app)
    select(app, app.map_combo, "simple_office")
    assert "map_name:=simple_office" in displayed_command(app)
    app.mode_buttons["loc"].invoke()
    assert "mode:=loc" in displayed_command(app)
    app.mode_buttons["nav"].invoke()
    # Environments are ported to every backend (world geometry is generated
    # for all four from the same source world), so switching the backend
    # keeps a runnable command and simply re-targets it.
    select(app, app.simulator_combo, "mujoco")
    assert "simulator:=mujoco" in displayed_command(app)
    select(app, app.simulator_combo, "gazebo")
    assert "simulator:=gazebo" in displayed_command(app)
    select(app, app.slot_combos["global_planner"], "navfn_planner")
    command = displayed_command(app)
    assert "global_planner_plugin:=nav2_navfn_planner/NavfnPlanner" in command
    # The selection is also forwarded by category so the launch applies it.
    assert "global_planning:=navfn_planner" in command


@pytest.mark.parametrize(
    "robot,planner,controller",
    [
        ("four_wheel_steer_car", "a_star_planner", "dwb_local_planner"),
        ("mecanum_car", "a_star_planner", "dwb_local_planner"),
    ],
)
def test_new_wheel_robots_autoselect_algorithms_and_fill_command(
        app, robot, planner, controller):
    select(app, app.map_combo, "celisca_floor_1")
    select(app, app.simulator_combo, "mujoco")
    app.mode_buttons["nav"].invoke()
    # A user-selected algorithm on the previous robot must not become the
    # default for a newly selected robot merely because it also accepts it.
    select(app, app.slot_combos["global_planner"], "navfn_planner")
    select(app, app.robot_combo, robot)
    assert app.slot_vars["global_planner"].get() == planner
    assert app.slot_vars["local_planner"].get() == controller
    command = displayed_command(app)
    assert f"robot_model:={robot}" in command
    assert "mode:=nav" in command
    assert "simulator:=mujoco" in command
    assert f"global_planning:={planner}" in command
    assert f"local_planning:={controller}" in command
    assert app.start_button.instate(["!disabled"])

    # An explicit change for the current robot remains selected when only
    # the map or simulator changes.
    select(app, app.slot_combos["global_planner"], "navfn_planner")
    select(app, app.map_combo, "celisca_floor_2")
    assert app.slot_vars["global_planner"].get() == "navfn_planner"
    assert "global_planning:=navfn_planner" in displayed_command(app)
    if robot == "four_wheel_steer_car":
        # A* is the mode default but a valid manual choice for this robot.
        select(app, app.slot_combos["global_planner"], "a_star_planner")
        assert app.slot_vars["global_planner"].get() == "a_star_planner"
        assert "global_planning:=a_star_planner" in displayed_command(app)


def test_go2_policy_opt_in_autofills_only_its_supported_launch(app):
    select(app, app.robot_combo, "unitree_go2")
    select(app, app.simulator_combo, "mujoco")
    app.mode_buttons["loc"].invoke()
    assert app.go2_policy_checkbox.instate(["!disabled"])
    app.go2_policy_checkbox.invoke()
    assert "go2_policy_path:=auto" in displayed_command(app)
    select(app, app.simulator_combo, "gazebo")
    assert app.go2_policy_checkbox.instate(["disabled"])
    assert "go2_policy_path:=auto" not in displayed_command(app)
    select(app, app.simulator_combo, "mujoco")
    assert "go2_policy_path:=auto" in displayed_command(app)


def test_bhl_policy_toggle_autofills_only_its_supported_launch(app):
    """The BHL walking policy is surfaced like the Go2 one: explicit in the
    command whenever the toggle is selectable, absent otherwise. It defaults
    to on so the integrated GUI-drive path is unchanged."""
    select(app, app.robot_combo, "berkeley_humanoid_lite_sim")
    select(app, app.map_combo, "nav_obstacle")
    select(app, app.simulator_combo, "mujoco")
    app.mode_buttons["loc"].invoke()
    assert app.bhl_policy_checkbox.instate(["!disabled"])
    assert app.bhl_policy_var.get() is True
    assert "bhl_enable_policy:=true" in displayed_command(app)
    app.bhl_policy_checkbox.invoke()
    assert "bhl_enable_policy:=false" in displayed_command(app)
    assert "bhl_enable_policy:=true" not in displayed_command(app)
    app.bhl_policy_checkbox.invoke()
    assert "bhl_enable_policy:=true" in displayed_command(app)
    # A different simulator never carries the flag: the policy node only
    # exists in the MuJoCo loc launch path.
    select(app, app.simulator_combo, "gazebo")
    assert app.bhl_policy_checkbox.instate(["disabled"])
    assert "bhl_enable_policy" not in displayed_command(app)
    select(app, app.simulator_combo, "mujoco")
    # Re-enter loc explicitly: switching to a backend that cannot localize
    # may have reset the mode, and the toggle must follow the mode either way.
    app.mode_buttons["loc"].invoke()
    assert "bhl_enable_policy:=true" in displayed_command(app)
    # Display mode has no locomotion to gate either.
    app.mode_buttons["display"].invoke()
    assert app.bhl_policy_checkbox.instate(["disabled"])
    assert "bhl_enable_policy" not in displayed_command(app)


@pytest.mark.parametrize("robot", ["berkeley_humanoid_lite_sim", "unitree_go2"])
def test_celisca_legged_command_keeps_map_and_robot_spawn_defaults(app, robot):
    """A GUI launch must not replace the calibrated spawn with registry zeroes."""
    select(app, app.robot_combo, robot)
    select(app, app.map_combo, "celisca_floor_1")
    select(app, app.simulator_combo, "mujoco")
    app.mode_buttons["loc"].invoke()
    command = displayed_command(app)
    assert "map_name:=celisca_floor_1" in command
    assert not any(token.startswith(f"spawn_{axis}:=")
                   for token in command for axis in ("x", "y", "z", "yaw"))


def test_room_vacuum_choice_changes_launch_file(app):
    app.vacuum_radio.invoke()
    assert displayed_command(app)[3] == "simulated_room_vacuum.launch.py"
    app.simulation_radio.invoke()
    assert displayed_command(app)[3] == "simulated_robot.launch.py"


def test_copy_and_run_use_preview_with_shell_quoting(app):
    command = ["ros2", "launch", "robot_lab_bringup", "simulated_robot.launch.py",
               "world_path:=/tmp/a map's world;test.world", "gui:=false"]
    manifest = {"ros2_command": command}
    process = Mock()
    process.poll.return_value = None
    with patch.object(launcher, "resolve_selection", return_value=(True, manifest)), \
            patch.object(launcher.subprocess, "Popen", return_value=process) as popen, \
            patch.object(launcher.threading, "Thread"), \
            patch.object(launcher, "subprocess_env", return_value={}):
        app._update_validation_and_command()
        shown = displayed_command(app)
        # The resolver command is carried through verbatim; the launcher
        # additionally appends the per-category algorithm selections.
        assert shown[:len(command)] == command
        assert "world_path:=/tmp/a map's world;test.world" in shown
        app.copy_command_button.invoke()
        assert shlex.split(app.clipboard_get()) == shown
        app.start_button.invoke()
        assert popen.call_args.args[0] == shown
        assert not popen.call_args.kwargs.get("shell", False)
        assert f"$ {app.command_var.get()}" in app.output.get("1.0", "end")
        # Ctrl+Enter cannot start another process while one is running.
        app._start_launch()
        popen.assert_called_once()


def test_invalid_selection_clears_command_and_blocks_run(app):
    app.map_var.set("missing_environment")
    app._update_validation_and_command()
    assert displayed_command(app) == []
    assert "Invalid:" in app.validation_var.get()
    assert app.start_button.instate(["disabled"])
    assert app.copy_command_button.instate(["disabled"])
    with patch.object(launcher.subprocess, "Popen") as popen, \
            patch.object(launcher.messagebox, "showerror") as showerror:
        app._start_launch()
        popen.assert_not_called()
        showerror.assert_called_once()
    select(app, app.map_combo, "simple_office")
    assert displayed_command(app)
    assert app.start_button.instate(["!disabled"])


def test_resolver_error_clears_previous_command(app):
    with patch.object(launcher, "resolve_selection", side_effect=RuntimeError("unavailable")):
        app._update_validation_and_command()
    assert displayed_command(app) == []
    assert "Resolver error: unavailable" == app.validation_var.get()
    assert app.start_button.instate(["disabled"])


def test_legacy_fallback_still_autofills_selected_options(app):
    app.composition_registry = None
    app.gui_var.set("false")
    app.vacuum_radio.invoke()
    command = displayed_command(app)
    assert command[3] == "simulated_room_vacuum.launch.py"
    assert "gui:=false" in command
    assert app.start_button.instate(["!disabled"])

def test_algorithm_arguments_follow_the_mode_without_leaking(app):
    """Switching modes rebuilds the slots; stale steps must not survive.

    Each mode runs its own set of categories. When a previous mode's steps
    were left behind they emitted a second, contradictory
    `<category>:=none` next to the real selection, so the launch received two
    values for the same category.
    """
    select(app, app.robot_combo, "bumperbot")
    select(app, app.map_combo, "celisca_floor_1")

    expected = {
        "display": {"perception"},
        "loc": {"localization", "state_estimation", "sensor_fusion"},
        "slam": {"localization", "state_estimation", "sensor_fusion",
                 "perception"},
        "nav": {"global_planning", "local_planning", "control",
                "localization", "state_estimation", "sensor_fusion"},
    }
    # Includes a return to an earlier mode, which is when leakage showed up.
    for mode in ("display", "loc", "slam", "nav", "display", "nav"):
        app.mode_buttons[mode].invoke()
        app.update()
        arguments = app._algorithm_arguments()
        categories = [argument.split(":=", 1)[0] for argument in arguments]
        assert len(categories) == len(set(categories)), (
            f"{mode} emitted duplicate categories: {arguments}")
        assert set(categories) == expected[mode], (
            f"{mode} emitted {sorted(categories)}")


def test_slot_selection_reaches_the_launch_command(app):
    """A changed slot must appear in the command that Run would execute."""
    select(app, app.robot_combo, "bumperbot")
    select(app, app.map_combo, "celisca_floor_1")
    app.mode_buttons["nav"].invoke()
    app.update()
    select(app, app.slot_combos["local_planner"], "mppi_controller")
    command = displayed_command(app)
    assert "local_planning:=mppi_controller" in command


def test_display_mode_runs_without_a_robot_or_without_a_map(app):
    """Display mode can show a world alone, or a robot alone."""
    app.mode_buttons["display"].invoke()
    app.update()

    select(app, app.map_combo, launcher.NONE_LABEL)
    command = displayed_command(app)
    assert "map_name:=none" in command
    assert f"robot_model:={app.robot_var.get()}" in command

    select(app, app.map_combo, "celisca_floor_1")
    select(app, app.robot_combo, launcher.NONE_LABEL)
    command = displayed_command(app)
    assert "robot_model:=none" in command
    assert "map_name:=celisca_floor_1" in command


def test_modes_needing_a_robot_are_disabled_without_one(app):
    """An unavailable mode is greyed out, not silently re-selected."""
    app.mode_buttons["display"].invoke()
    select(app, app.robot_combo, launcher.NONE_LABEL)
    app.update()
    for mode in ("loc", "slam", "nav", "3d_slam"):
        assert app.mode_buttons[mode].instate(["disabled"]), (
            f"{mode} should be unavailable with no robot selected")
    assert app.mode_var.get() == "display"


def test_labbot_rgbd_mode_builds_a_launch_command(app):
    select(app, app.robot_combo, 'labbot')
    select(app, app.map_combo, 'nav_obstacle')
    app.mode_buttons['3d_slam'].invoke()
    app.update()
    command = displayed_command(app)
    assert 'mode:=3d_slam' in command
    assert 'robot_model:=labbot' in command
    assert app.start_button.instate(['!disabled'])


@pytest.mark.parametrize('robot', ['berkeley_humanoid_lite', 'berkeley_humanoid_lite_sim'])
@pytest.mark.parametrize('simulator, can_localize', [
    ('pybullet', True),   # bridge casts the scan and holds the stance
    ('mujoco', True),
    ('gazebo', False),    # no LiDAR in the description
    ('isaac', False),     # Isaac's joint hold does not keep it standing
])
def test_humanoid_mode_buttons_follow_what_the_robot_can_do(app, robot, simulator,
                                                            can_localize):
    select(app, app.robot_combo, robot)
    select(app, app.map_combo, 'nav_obstacle')
    if simulator not in app.simulator_combo['values']:
        pytest.skip('%s not installed' % simulator)
    select(app, app.simulator_combo, simulator)
    if app.simulator_var.get() != simulator:
        pytest.skip('%s not selectable on this host' % simulator)
    assert app.mode_buttons['loc'].instate(['!disabled' if can_localize else 'disabled'])
    # MuJoCo localization can walk with the Drive pad. Mapping and
    # navigation stay unavailable until their sensor/goal paths qualify.
    for mode in ('slam', 'nav'):
        assert app.mode_buttons[mode].instate(['disabled'])
    if can_localize:
        app.mode_buttons['loc'].invoke()
        command = app._prepared_command
        assert 'mode:=loc' in command
        assert 'robot_model:=%s' % robot in command


def test_all_declared_occupancy_maps_pass_gui_validation(app):
    select(app, app.robot_combo, 'bumperbot')
    checked = []
    for name, profile in app.map_profiles.items():
        if profile.get('map', {}).get('has_2d_map'):
            assert app._map_has_2d_map(name), name
            checked.append(name)
    assert len(checked) >= 20


def test_mecanum_strafe_publishes_lateral_motion_only_for_mecanum(app):
    """R5.6: the Drive pad gains lateral input only on a mecanum base."""
    app.robot_var.set("mecanum_car")
    app._update_from_selection()
    assert app._mecanum_selectable()
    with patch.object(app, "_publish_drive") as publish, \
            patch.object(app.drive_joystick, "poll", return_value=(0.0, 0.0)):
        app._start_strafe(1.0)
        app._repeat_drive()
        assert app.current_drive == (0.0, 0.0)
        assert app.current_lateral > 0.0
        assert publish.call_args.args[2] == app.current_lateral
        app._start_strafe(1.0)          # second click releases the latch
        for _ in range(40):
            app._repeat_drive()
            if app.current_lateral == 0.0:
                break
        assert app.current_lateral == 0.0
        assert publish.call_args.args == (0.0, 0.0, 0.0)

    # A differential base never sends lateral motion and its strafe latches
    # are cleared when the robot changes.
    app.robot_var.set("bumperbot")
    app._update_from_selection()
    assert not app._mecanum_selectable()
    assert not app.drive_strafe_buttons
    assert app.strafe_left_button.instate(["disabled"])


def test_four_wheel_steer_pattern_is_appended_to_the_command(app):
    """R5.6: the pattern selector is gated to the four-wheel-steer base."""
    app.robot_var.set("four_wheel_steer_car")
    app._update_from_selection()
    assert app._four_wheel_steer_selectable()
    app.steering_mode_var.set("crab")
    app._update_validation_and_command()
    assert "steering_mode:=crab" in app.command_var.get()
    app.steering_mode_var.set("")
    app._update_validation_and_command()
    assert "steering_mode:=" not in app.command_var.get()

    app.robot_var.set("bumperbot")
    app._update_from_selection()
    assert not app._four_wheel_steer_selectable()
    app.steering_mode_var.set("pivot")
    app._update_validation_and_command()
    assert "steering_mode:=" not in app.command_var.get()


@pytest.mark.parametrize("pattern", ["crab", "in_phase"])
def test_four_wheel_parallel_steering_enables_strafe_and_saved_manifest(app, pattern):
    select(app, app.robot_combo, "four_wheel_steer_car")
    select(app, app.map_combo, "nav_empty")
    select(app, app.steering_mode_combo, pattern)
    assert app.strafe_left_button.instate(["!disabled"])
    assert app._lateral_drive_selectable()
    ok, manifest = launcher.resolve_selection(app.composition_registry,
                                              app._composition_selection())
    assert ok, manifest
    assert manifest["launch"]["arguments"]["steering_mode"] == pattern
    assert "steering_mode:=" + pattern in manifest["ros2_command"]
    assert manifest["resolved_from"]["steering_mode"] == pattern
    with patch.object(app, "_publish_drive") as publish:
        app._start_strafe(1.0)
        app._repeat_drive()
        assert publish.call_args.args[2] > 0.0
        app._stop_drive()
    select(app, app.steering_mode_combo, "ackermann")
    assert app.strafe_left_button.instate(["disabled"])
    assert not app.drive_strafe_buttons
    app._apply_manifest_to_controls(manifest)
    assert app.steering_mode_var.get() == pattern


def test_px4_flight_autofill_services_and_backend_correction(app):
    select(app, app.map_combo, 'celisca_floor_1')
    select(app, app.robot_combo, 'px4_x500')
    assert app.map_var.get() == 'celisca_floor_1'
    app.mode_var.set('flight')
    app.simulator_var.set('pybullet')
    app._update_from_selection()
    assert app.simulator_var.get() == 'gazebo'
    assert app.mode_var.get() == 'flight'
    assert app._flight_selectable()
    assert 'robot_model:=px4_x500' in app.command_var.get()
    assert 'mode:=flight' in app.command_var.get()
    assert 'map_name:=celisca_floor_1' in app.command_var.get()
    assert 'use_sim_time:=false' in app.command_var.get()
    assert 'spawn_z:=0.0' in app.command_var.get()
    assert all(button.instate(['!disabled']) for button in app.flight_buttons)
    assert app.strafe_left_button.instate(['!disabled'])
    with patch.object(app, '_publish_drive') as publish, patch.object(app, '_run_aux_command') as command:
        app.drive_input_enabled.set(True)
        app._toggle_drive_input()
        app._repeat_drive()
        command.assert_not_called()
        assert all(not any(call.args) for call in publish.call_args_list)
    select(app, app.robot_combo, 'bumperbot')
    assert not app._flight_selectable()
    assert all(button.instate(['disabled']) for button in app.flight_buttons)


def test_px4_preserves_every_installed_world_and_autofills_flight(app):
    app.robot_var.set('px4_x500')
    app.mode_var.set('flight')
    app.simulator_var.set('gazebo')
    for world in sorted(app.map_profiles):
        app.map_var.set(world)
        app._update_from_selection()
        assert app.map_var.get()==world
        # The resolver canonicalizes aliases such as outdoor_terrain to the
        # installed terrain_rough world, while keeping the operator selection.
        canonical=app.map_profiles[world]['gazebo'].get('world_name',world)
        assert 'map_name:='+canonical in app.command_var.get(), app.validation_var.get()
        assert 'mode:=flight' in app.command_var.get()
