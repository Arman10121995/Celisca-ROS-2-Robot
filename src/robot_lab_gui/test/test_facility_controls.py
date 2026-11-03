"""Native facility controls generate explicit intents; construction stays neutral."""
import json
import os
from types import SimpleNamespace
import tkinter as tk
from tkinter import ttk
from unittest.mock import Mock

import pytest

pytestmark = [pytest.mark.integration, pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Requires Xvfb')]


def test_queue_controls_keep_member_round_trips_ordered_and_do_not_publish_on_open():
    from robot_lab_gui.fleet_infrastructure_controls import InfrastructureControls
    root = tk.Tk()
    try:
        notebook = ttk.Notebook(root)
        fleet = SimpleNamespace(_ids=lambda: ['alpha', 'beta'], operator_id='op', runtime=dict(operator_id='op', operator_fresh=True))
        panel = InfrastructureControls(notebook, fleet)
        assert panel.publisher is None and panel.runtime is None
        assert panel.recover_button.instate(['disabled'])
        panel.runtime = dict(session_id='session', operator_id='op', operator_fresh=True, robot_floors=dict(alpha='L0', beta='L0'))
        panel.send = Mock()
        panel.floor.set('L1')
        panel.queue_members(True)
        assert panel.send.call_args.args == ('transport_queue',)
        assert panel.send.call_args.kwargs['requests'] == [
            dict(robot='alpha', floor='L1',parking='auto'), dict(robot='beta', floor='L1',parking='auto'),
            dict(robot='alpha', floor='L0',parking='auto'), dict(robot='beta', floor='L0',parking='auto')]
        panel.send.reset_mock()
        panel.floor.set('L0')
        panel.queue_members()
        panel.send.assert_not_called()
        assert 'different destination' in panel.status.get()
    finally:
        root.destroy()


def test_long_facility_feedback_remains_scrollable_in_a_small_control_column():
    from robot_lab_gui.fleet_infrastructure_controls import InfrastructureControls
    root = tk.Tk()
    try:
        root.geometry('420x300')
        notebook = ttk.Notebook(root); notebook.pack(fill='both', expand=True)
        panel = InfrastructureControls(notebook, SimpleNamespace())
        panel.feedback.set('\n'.join('Observed member/map row '+str(i) for i in range(60)))
        root.update()
        assert panel.canvas.yview()[1] < 1.
        panel.canvas.yview_moveto(1.); root.update()
        assert panel.canvas.yview()[0] > 0.
        assert panel.publisher is None and panel.runtime is None
    finally:
        root.destroy()
