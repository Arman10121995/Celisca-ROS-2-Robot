"""Actual Tk target edits, exact launch ownership and explicit park/cancel requests."""
import json
import math
import os
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

pytestmark=[pytest.mark.integration,pytest.mark.skipif(not os.environ.get('DISPLAY'),reason='Real Tk needs Xvfb')]


def test_target_edits_and_estimated_pose_copy_do_not_start_or_connect_a_robot():
    import tkinter as tk
    from robot_lab_gui.car_parking_controls import CarParkingControls
    root=tk.Tk();root.withdraw()
    app=NS(robot_var=tk.StringVar(root,value='rear_steer_car'),mode_var=tk.StringVar(root,value='nav'),
        simulator_var=tk.StringVar(root,value='gazebo'),launch_kind_var=tk.StringVar(root,value='simulation'),
        _launch_running=True,_stop_drive=Mock(),process=NS(poll=lambda:None,
            args=['robot_model:=rear_steer_car','mode:=nav','simulator:=gazebo']))
    panel=CarParkingControls(root,app)
    try:
        assert panel.node is None and panel.owned()
        panel.state=dict(instance_id='this-run',robot_model='rear_steer_car',phase='idle',pose=[1.,2.,math.pi/2],navigation_ready=True)
        panel.received=time.monotonic();panel.use_pose()
        assert [v.get() for v in panel.target]==pytest.approx([1.,2.,90.])
        assert panel.node is None;app._stop_drive.assert_not_called()
        panel.node=NS(destroy_node=Mock());panel.publisher=Mock();panel.heartbeat=Mock()
        panel.target[0].set(3.);panel.target[2].set(-45.)
        panel.publisher.publish.assert_not_called()
        panel.start()
        command=json.loads(panel.publisher.publish.call_args.args[0].data)
        assert command['op']=='park' and command['instance_id']=='this-run'
        assert command['x']==3. and command['y']==2. and command['yaw']==pytest.approx(-math.pi/4)
        app._stop_drive.assert_called_once();assert panel.requested
        panel.cancel()
        assert json.loads(panel.publisher.publish.call_args.args[0].data)==dict(op='cancel',instance_id='this-run')
        count=panel.publisher.publish.call_count
        panel.timeout.set(math.nan);panel.start()
        assert panel.publisher.publish.call_count==count and 'Finite' in panel.status.get()
        panel.timeout.set(180.);panel.received=time.monotonic()-2;panel.start()
        assert panel.publisher.publish.call_count==count
        app.process.args=['robot_model:=ackermann_car','mode:=nav','simulator:=gazebo']
        assert not panel.owned()
    finally:panel.close();root.destroy()
