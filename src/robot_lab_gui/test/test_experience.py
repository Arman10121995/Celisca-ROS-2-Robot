"""Real navigation/search/controls interactions, without starting any robot."""
import os
import time
from unittest.mock import patch

import pytest
from test_command_autofill import app  # noqa: F401

pytestmark=[pytest.mark.integration,pytest.mark.skipif(not os.environ.get('DISPLAY'),reason='Requires Xvfb')]


def settle(app):
    deadline=time.monotonic()+.3
    while time.monotonic()<deadline:app.update();time.sleep(.01)


def test_search_routes_without_launching_arming_or_changing_configuration(app):
    from robot_lab_gui.experience import open_search
    before=app.command_var.get()
    with patch.object(app,'_start_launch') as launch,patch.object(app,'_stop_launch') as stop, \
            patch.object(app,'_schedule_drive') as drive,patch.object(app,'_ensure_ros_publisher') as connect:
        app.drive_input_enabled.set(True)
        open_search(app);settle(app);dialog=app.workspace_search
        dialog.entry.focus_force();settle(app)
        dialog.entry.event_generate('<KeyPress-w>');dialog.entry.event_generate('<KeyRelease-w>')
        dialog.entry.event_generate('<Control-Return>');settle(app)
        launch.assert_not_called();drive.assert_not_called();connect.assert_not_called()
        if dialog.winfo_exists():dialog.dismiss()
        open_search(app);settle(app);dialog=app.workspace_search
        dialog.query.set('drive');settle(app)
        assert dialog.rows and dialog.rows[0]['target']=='Drive'
        dialog.activate();settle(app)
        assert app.notebook.select()==str(app.launch_tab)
        assert app.control_notebook.select()==str(app.drive_page)
        open_search(app);settle(app);app.workspace_search.entry.focus_force()
        app.workspace_search.entry.event_generate('<Escape>');settle(app)
        stop.assert_not_called();launch.assert_not_called();drive.assert_not_called()
        assert app.command_var.get()==before and app.ros_node is None
        app.drive_input_enabled.set(False)


def test_control_selector_and_navigation_toggle_preserve_widgets(app):
    from tkinter import ttk
    command=app.command_var.get();page=app.drive_page
    app.control_page_var.set('Drone & limits');app.control_page_combo.event_generate('<<ComboboxSelected>>');settle(app)
    assert app.control_notebook.select()==str(app.drone_page)
    app.show_tab('Drive');settle(app)
    assert app.control_page_var.get()=='Drive & limits'
    app.toggle_navigation();settle(app);assert app.navigation_collapsed
    app.toggle_navigation();settle(app);assert not app.navigation_collapsed
    assert app.drive_page is page and app.command_var.get()==command and app.ros_node is None
    extra=ttk.Frame(app.control_notebook);app.control_notebook.add(extra,text='Marine missions')
    app.show_tab('Marine missions');settle(app)
    assert app.control_notebook.select()==str(extra)
    app.robot_notes_button.invoke();assert app.robot_notes_open
    app.robot_notes_button.invoke();assert not app.robot_notes_open
    assert app.command_var.get()==command


def test_home_actions_and_resizing_are_neutral(app):
    from robot_lab_gui.experience import HomeTab
    page=HomeTab(app.notebook,app);app.notebook.select(page)
    before=app.command_var.get()
    for size in ('1600x980','1024x768'):
        app.geometry(size);settle(app)
        assert page.canvas.winfo_viewable()
        assert page.canvas.xview()[1]==1.
    assert len(page.card_widgets)==6
    assert app.command_var.get()==before and app.ros_node is None


def test_header_session_indicator_reads_state_only():
    from types import SimpleNamespace
    from robot_lab_gui.workspace_ui import session_text

    class Var:
        def __init__(self, value): self.value = value
        def get(self): return self.value

    class Alive:
        def poll(self): return None

    app = SimpleNamespace(robot_var=Var('bumperbot'), map_var=Var('nav_empty'), simulator_var=Var('mujoco'),
                          status_var=Var('Idle'), process=None, _launch_running=False)
    assert session_text(app) == ('○  No session running', 'PillIdle.TLabel')
    app.status_var = Var('Launch blocked: invalid configuration')
    text, style = session_text(app)
    assert style == 'PillWarn.TLabel' and 'Launch blocked' in text
    app.process, app._session_started = Alive(), 100.0
    text, style = session_text(app, now=165.0)
    assert style == 'PillRunning.TLabel'
    assert text == '●  Running · bumperbot · nav_empty · mujoco · 01:05'
