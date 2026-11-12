"""Physical-roller assembly conservation, independent of live mission evidence."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from robot_lab_utils.myagv_rollers import physical_rollers, translate_inertia, matrix


def assembly(root,name):
    links={n.get('name'):n for n in root.findall('link')}
    total=0.;moment=np.zeros(3);tensor=np.zeros((3,3))
    def visit(link_name,xyz,rotation):
        nonlocal total,moment,tensor
        inertial=links[link_name].find('inertial')
        if inertial is not None:
            mass=float(inertial.find('mass').get('value'))
            origin=inertial.find('origin')
            local=np.array(list(map(float,origin.get('xyz','0 0 0').split()))) if origin is not None else np.zeros(3)
            center=xyz+rotation.apply(local)
            frame=rotation*(Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))) if origin is not None else Rotation.identity())
            total+=mass;moment+=mass*center
            tensor+=frame.as_matrix()@matrix(inertial.find('inertia'))@frame.as_matrix().T+translate_inertia(mass,center)
        for joint in root.findall('joint'):
            if joint.find('parent').get('link')!=link_name:continue
            origin=joint.find('origin')
            child_xyz=xyz+rotation.apply(list(map(float,origin.get('xyz','0 0 0').split())))
            child_rotation=rotation*Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split())))
            visit(joint.find('child').get('link'),child_xyz,child_rotation)
    visit(name,np.zeros(3),Rotation.identity())
    return total,moment/total,tensor


def test_original_vendor_wheel_assembly_is_conserved():
    store=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'external_assets/installed/robots.yaml'
    if not store.is_file():pytest.skip('Source myAGV not provisioned')
    cfg=yaml.safe_load(store.read_text())['robots'].get('asset_elephant_myagv_pro')
    if not cfg:pytest.skip('Source myAGV not provisioned')
    root=ET.parse(cfg['xacro']).getroot()
    before={name:assembly(root,name+'_link') for name in ('left_front_wheel','left_rear_wheel','right_front_wheel','right_rear_wheel')}
    assets=Path(__file__).resolve().parents[2]/'robot_lab_robots/holonomic_wheels/third_party/fuji_mecanum'
    contract=physical_rollers(root,assets)
    assert sum(len(c['roller_joints']) for c in contract)==60
    assert len(root.findall('.//fdir1'))==0
    for name,values in before.items():
        actual=assembly(root,name+'_link')
        assert actual[0]==pytest.approx(values[0],abs=1e-12)
        assert actual[1]==pytest.approx(values[1],abs=1e-12)
        assert actual[2]==pytest.approx(values[2],abs=1e-12)
