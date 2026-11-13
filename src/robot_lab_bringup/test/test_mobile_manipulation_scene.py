"""Fixture geometry/occupancy consistency; separate from robot mission acceptance."""
import xml.etree.ElementTree as ET

from robot_lab_utils.mobile_manipulation_scene import BENCHES,OBJECT,sdf,mjcf,occupancy


def test_sdf_and_native_have_the_same_free_cube_and_static_benches():
    world=ET.fromstring(sdf()).find('world');native=ET.fromstring(mjcf()).find('worldbody')
    for name,position,size in [*BENCHES,OBJECT]:
        model=world.find(f"model[@name='{name}']")
        body=native.find(f"body[@name='{name}']")
        assert list(map(float,model.findtext('pose').split()))[:3]==position
        assert list(map(float,body.get('pos').split()))==position
        assert [float(x)*2 for x in body.find('geom').get('size').split()]==size
        assert (body.find('freejoint') is not None)==(name==OBJECT[0])
        assert model.findtext('static')==('false' if name==OBJECT[0] else 'true')
    assert float(world.find(f"model[@name='{OBJECT[0]}']/link/inertial/mass").text)==.035
    assert float(native.find(f"body[@name='{OBJECT[0]}']/geom").get('mass'))==.035


def test_static_occupancy_blocks_benches_but_leaves_the_navigation_lane_free():
    data=occupancy();header,pixels=data.split(b'255\n',1)
    assert len(pixels)==120*80
    def at(x,y):return pixels[(79-int((y+2)/.05))*120+int((x+2)/.05)]
    assert at(0.,0.)==254 and at(1.2,0.)==254
    assert at(0.,-.62)==0 and at(1.2,-.62)==0
