"""Real route editor interactions with a recording publisher, no FCU/plant."""
import json
import os
import time
from unittest.mock import Mock

import pytest
from test_command_autofill import app  # noqa: F401

pytestmark = [pytest.mark.integration, pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Requires Xvfb')]


def test_formation_route_editor_is_neutral_and_plan_execute_are_distinct(app):
    from robot_lab_gui.aerial_swarm_tab import AerialSwarmTab
    tab = AerialSwarmTab(app.notebook, app)
    try:
        app.notebook.select(tab); app.update()
        command = app.command_var.get()
        assert tab.node is None and not tab.armed and not tab.route_open
        tab.toggle_route_editor(); app.update()
        assert tab.route_open and tab.route_frame.winfo_viewable()
        assert not tab.route_manual_widgets[0].winfo_viewable()
        tab.command_pub = Mock()
        tab.runtime = dict(session_id='session', operator_id=tab.operator, members=[
            dict(id='air1', ready=True, position=[0., -1.5, 2.]),
            dict(id='air2', ready=True, position=[0., 1.5, 2.])])
        tab.runtime_at = time.monotonic()
        tab.add_route_anchor()
        tab.command_pub.publish.assert_not_called()
        assert tab.route_editor.get('1.0', 'end').strip() == '0.000 0.000 2.000'
        tab.route_editor.insert('end', '2 0 2\n')
        tab.operation('plan_route')
        request = json.loads(tab.command_pub.publish.call_args.args[0].data)
        assert request['op'] == 'plan_route' and request['anchors'] == [[0., 0., 2.], [2., 0., 2.]]
        tab.operation('execute_route')
        assert json.loads(tab.command_pub.publish.call_args.args[0].data)['op'] == 'execute_route'
        tab.command_pub.reset_mock()
        tab.route_editor.delete('1.0', 'end'); tab.route_editor.insert('1.0', 'nan 0 2')
        tab.operation('plan_route')
        tab.command_pub.publish.assert_not_called()
        tab.toggle_route_editor(); app.update()
        assert not tab.route_open and tab.route_manual_widgets[0].winfo_viewable()
        assert app.command_var.get() == command and tab.node is None
    finally:
        tab.runtime = None
        tab.close(); tab.destroy()
