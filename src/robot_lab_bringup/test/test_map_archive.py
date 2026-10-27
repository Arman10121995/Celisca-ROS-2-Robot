"""Acquired-map persistence and immutable reload regressions."""
import hashlib
from pathlib import Path
import sqlite3
import sys
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.map_archive import save_occupancy,snapshot_database


def test_snapshot_contains_actual_nodes_and_leaves_saved_source_unchanged(tmp_path):
    source=tmp_path/'saved.db';target=tmp_path/'copy.db'
    with sqlite3.connect(source) as db:
        db.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY, pose BLOB)')
        db.executemany('INSERT INTO Node VALUES(?,?)',[(1,b'pose-one'),(2,b'pose-two')])
    before=hashlib.sha256(source.read_bytes()).hexdigest()
    observation=snapshot_database(source,target)
    assert observation['nodes']==2
    assert observation['sha256']==hashlib.sha256(target.read_bytes()).hexdigest()
    assert hashlib.sha256(source.read_bytes()).hexdigest()==before
    with sqlite3.connect(target) as db:assert db.execute('SELECT pose FROM Node WHERE id=2').fetchone()[0]==b'pose-two'
    with pytest.raises(ValueError,match='overwrite'):snapshot_database(source,source)


def test_empty_rtabmap_database_cannot_be_reported_saved(tmp_path):
    source=tmp_path/'empty.db';target=tmp_path/'copy.db'
    with sqlite3.connect(source) as db:db.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY)')
    with pytest.raises(ValueError,match='No acquired'):snapshot_database(source,target)
    assert not target.exists()


def test_saved_grid_keeps_real_origin_yaw_and_unknown_cells(tmp_path):
    q=SimpleNamespace(x=0.,y=0.,z=np.sin(.2),w=np.cos(.2))
    msg=SimpleNamespace(header=SimpleNamespace(frame_id='map'),data=[0,100,-1,100,0,-1],
        info=SimpleNamespace(width=3,height=2,resolution=.1,
            origin=SimpleNamespace(position=SimpleNamespace(x=2.,y=-3.,z=0.),orientation=q)))
    folder=tmp_path/'acquired'
    result=save_occupancy(msg,folder,initial_pose=[.1,.2,.3])
    params=yaml.safe_load((folder/'map.yaml').read_text())
    assert params['origin']==pytest.approx([2.,-3.,.4])
    image=np.array(Image.open(folder/'map.pgm'))
    assert image.tolist()==[[0,254,205],[254,0,205]]
    assert result['known_cells']==4 and result['occupied_cells']==2
    assert result['initial_pose_in_map']==[.1,.2,.3]
    msg.data=[-1]*6
    with pytest.raises(ValueError,match='No acquired'):save_occupancy(msg,tmp_path/'unknown')
