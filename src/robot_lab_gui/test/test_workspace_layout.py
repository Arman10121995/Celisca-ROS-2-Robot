"""Real Tk selection, command and control ownership after workspace redesign."""
import os
import time
from unittest.mock import Mock, patch

import pytest

from test_command_autofill import app, select, displayed_command  # noqa: F401

pytestmark = [pytest.mark.integration,
              pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Requires Xvfb or a display')]


def settle(app):
    deadline = time.monotonic()+.25
    while time.monotonic() < deadline:
        app.update()
        time.sleep(.01)


def test_robot_filters_change_exact_selection_and_autofill_without_commands(app):
    with patch.object(app, '_publish_drive') as publish, patch.object(app, '_ensure_ros_publisher') as connect:
        select(app, app.robot_category_combo, 'Mobile robots')
        select(app, app.robot_subtype_combo, 'Four-wheel drive')
        families = app.robot_combo['values']
        assert 'four_wheel_steer_car' in families and 'mecanum_car' in families
        assert 'bumperbot' not in families and 'unitree_go2' not in families
        select(app, app.robot_combo, 'mecanum_car')
        assert 'robot_model:=mecanum_car' in displayed_command(app)
        assert 'Holonomic' in app.robot_tags_var.get()
        select(app, app.robot_category_combo, 'Legged robots')
        assert app.robot_subtype_var.get() == 'All types'
        select(app, app.robot_subtype_combo, 'Four legs')
        assert 'unitree_go2' in app.robot_combo['values']
        assert 'berkeley_humanoid_lite' not in app.robot_combo['values']
        publish.assert_not_called()
        connect.assert_not_called()


def test_control_pages_share_the_right_launch_column_and_old_routes_still_work(app):
    from robot_lab_gui.arm_tab import ArmTab
    from robot_lab_gui.hand_tab import HandTab
    app.arm_tab = ArmTab(app.control_notebook, app)
    app.hand_tab = HandTab(app.control_notebook, app)
    try:
        settle(app)
        assert app.setup_notebook.winfo_viewable()
        assert app.setup_notebook.winfo_rootx() < app.command_preview.winfo_rootx()
        assert app.drive_page.winfo_rootx() > app.command_preview.winfo_rootx()
        assert app.arm_tab.master == app.hand_tab.master == app.control_notebook
        for title, page in [('Drive', app.drive_page), ('Arm', app.arm_tab),
                            ('Hand', app.hand_tab), ('Drone', app.drone_page)]:
            app.show_tab(title); settle(app)
            assert app.notebook.select() == str(app.launch_tab)
            assert app.control_notebook.select() == str(page)
            from robot_lab_gui.workspace_ui import ScrollPanel
            panel = page if isinstance(page, ScrollPanel) else next(
                widget for widget in page.winfo_children() if isinstance(widget, ScrollPanel))
            assert panel.canvas.xview()[1] == 1., (title, panel.canvas.xview())
        assert app.drive_pad_host.master == app.drive_limits_host.master == app.drive_page.body
        assert app.drone_actions_host.master == app.drone_limits_host.master == app.drone_page.body
        assert not app.drive_input_enabled.get()
        assert app.ros_node is None
    finally:
        app.arm_tab.close(); app.hand_tab.close()


def test_narrow_window_keeps_run_and_copy_visible_and_limits_scrollable(app):
    app.geometry('1024x768'); settle(app)
    assert app.compact_setup.winfo_viewable()
    app.compact_setup.setup_button.invoke(); settle(app)
    assert app.robot_combo.winfo_viewable()
    app.compact_setup.command_button.invoke(); settle(app)
    for button in (app.start_button, app.copy_command_button):
        assert button.winfo_viewable()
        assert button.winfo_rootx()+button.winfo_width() <= app.winfo_rootx()+app.winfo_width()
    app.show_tab('Drive'); settle(app)
    canvas = app.drive_page.canvas
    assert canvas.winfo_viewable()
    assert canvas.xview()[1] == 1.
    assert canvas.bbox('all')[3] > canvas.winfo_height()
    canvas.yview_moveto(1.); settle(app)
    assert app.drive_input_checkbox.winfo_rooty()+app.drive_input_checkbox.winfo_height() \
        <= canvas.winfo_rooty()+canvas.winfo_height()
    app.drive_override_var.set(True); app._update_drive_limits_label()
    assert all('disabled' not in field.state() for field in app.drive_limit_widgets)
    app.drive_override_var.set(False); app._update_drive_limits_label()
    assert all('disabled' in field.state() for field in app.drive_limit_widgets)
    # Resize an already opened GUI, not just its initial dimensions. Setup
    # and command become columns again without replacing the live widgets.
    app.geometry('1600x980'); settle(app)
    assert not app.compact_setup.winfo_viewable()
    assert app.setup_notebook.winfo_viewable()
    assert app.setup_notebook.winfo_rootx() < app.command_preview.winfo_rootx()
    assert app.start_button.winfo_viewable()
    assert app.drive_page.winfo_rootx() > app.command_preview.winfo_rootx()


def test_header_stop_uses_existing_controllers_and_logs_preserve_command(app):
    before = app.command_var.get()
    app._stop_drive = Mock()
    app.arm_tab = Mock(); app.hand_tab = Mock()
    app._stop_all_motion()
    app._stop_drive.assert_called_once_with()
    app.arm_tab.stop.assert_called_once_with(); app.hand_tab.stop.assert_called_once_with()
    app._toggle_console(); settle(app)
    assert app.console_frame.winfo_viewable()
    app._toggle_console(); settle(app)
    assert not app.console_frame.winfo_viewable()
    assert app.command_var.get() == before


def test_registry_classification_filters_components_and_searches_structural_tags(app):
    from robot_lab_gui.lab_tabs import RegistryTab
    tab = RegistryTab(app.notebook, app)
    tab.category_var.set('Mobile robots'); tab._filter_changed()
    tab.subtype_var.set('Four-wheel drive'); tab.search_var.set('mecanum'); tab._refresh_tree()
    assert 'family:mecanum_car' in tab.rows
    assert 'family:unitree_go2' not in tab.rows
    row = next(name for name, data in tab.rows.items() if data.get('profile_id') == 'mecanum_car' and 'group' not in data)
    tab.tree.selection_set(row); tab._show_details()
    assert 'Holonomic' in tab.details.get('1.0', 'end')
    assert 'Four-wheel drive' in tab.tree.item(row, 'values')[1]
    tab.type_var.set('Environments'); tab._refresh_tree()
    assert 'disabled' in tab.category_combo.state()
    assert 'disabled' in tab.subtype_combo.state()
