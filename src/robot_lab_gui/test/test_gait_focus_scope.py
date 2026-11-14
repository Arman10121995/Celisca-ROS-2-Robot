"""A hidden gait panel must not interrupt another robot's motion workflow."""
from types import SimpleNamespace

import pytest

from robot_lab_gui.gait_controls import GaitControls


@pytest.mark.parametrize('selected,closed,focused,stops',[
    (False,False,None,0),(True,False,None,1),
    (True,True,None,0),(True,False,object(),0)])
def test_focus_stop_is_owned_by_selected_gait(monkeypatch,selected,closed,focused,stops):
    calls=[]
    monkeypatch.setattr('robot_lab_gui.gait_controls.focused_widget',lambda _:focused)
    panel=SimpleNamespace(closed=closed,selected=lambda:selected,
        app=SimpleNamespace(_stop_drive=lambda **kw:calls.append(kw)))
    GaitControls.focus_check(panel)
    assert calls==[dict(keep_input_enabled=True)]*stops
