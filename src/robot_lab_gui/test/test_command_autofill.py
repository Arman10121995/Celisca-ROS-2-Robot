"""Exercise command autofill through real Tk controls (run with xvfb-run)."""

import os
from pathlib import Path
import shlex
import sys
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
                gui.after_cancel(job)
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
    select(app, app.simulator_combo, "mujoco")
    # Labbot is not cataloged for MuJoCo; never leave a runnable Gazebo
    # command on screen when that unsupported backend is selected.
    assert displayed_command(app) == []
    assert "mujoco" in app.validation_var.get()
    select(app, app.simulator_combo, "gazebo")
    assert "simulator:=gazebo" in displayed_command(app)
    select(app, app.slot_combos["global_planner"], "navfn_planner")
    assert "global_planner_plugin:=nav2_navfn_planner/NavfnPlanner" in displayed_command(app)


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
        assert displayed_command(app) == command
        app.copy_command_button.invoke()
        assert shlex.split(app.clipboard_get()) == command
        app.start_button.invoke()
        assert popen.call_args.args[0] == command
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
