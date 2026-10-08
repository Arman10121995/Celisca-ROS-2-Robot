"""Repairs require a reviewed source and leave unrelated poses unchanged."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import trimesh

PATH = Path(__file__).resolve().parents[3] / 'scripts/external_world_repairs.py'
spec = importlib.util.spec_from_file_location('external_world_repairs', PATH)
repairs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(repairs)


def test_unknown_source_is_never_clamped(tmp_path):
    source = tmp_path/'world.sdf'
    source.write_text('<sdf><model name="StorageRack_1"><pose>1 2 -754989000 0 0 0</pose></model></sdf>')
    root = ET.parse(source).getroot()
    before = ET.tostring(root)
    assert repairs.repair_hospital_poses(source, root) == []
    assert ET.tostring(root) == before


def test_static_grouping_preserves_moving_fixtures_and_explicit_flags():
    root = ET.fromstring('''<world>
        <model name="building"><pose>2 3 0 0 0 .4</pose><model name="floor"><static>1</static><link name="slab"/></model></model>
        <model name="moving"><model name="cart"><static>false</static><link name="body"/></model></model>
        <model name="explicit"><static>false</static><model name="fixed"><static>true</static></model></model>
        <model name="articulated"><joint name="hinge"/><model name="fixed"><static>true</static></model></model>
        <model name="linked"><link name="body"/><model name="fixed"><static>true</static></model></model>
        </world>''')
    record = repairs.preserve_static_containers(root)
    assert [v['model'] for v in record] == ['building']
    assert root.find("model[@name='building']/pose").text == '2 3 0 0 0 .4'
    assert root.find("model[@name='building']/static").text == 'true'
    assert root.find("model[@name='explicit']/static").text == 'false'
    for name in ('moving','articulated','linked'):
        assert root.find("model[@name='%s']/static" % name) is None


def test_static_container_recursion_keeps_nesting_and_floor_pose():
    root = ET.fromstring('<model name="upper"><pose>0 0 3 0 0 0</pose>'
        '<model name="room"><model name="floor"><static>true</static><link name="slab"/></model></model></model>')
    assert [v['model'] for v in repairs.preserve_static_containers(root)] == ['room','upper']
    assert root.findtext('pose') == '0 0 3 0 0 0'
    assert root.find('model/model/link') is not None


def test_explicit_static_snapshot_records_dynamic_flags_without_changing_geometry_or_actors():
    root=ET.fromstring('<world><model name="cart"><static>false</static><pose>1 2 0 0 0 .3</pose>'
        '<link name="body"><collision name="c"><geometry><box><size>1 2 3</size></box></geometry></collision></link></model>'
        '<model name="building"><static>1</static></model><actor name="walker"><pose>3 4 0 0 0 0</pose></actor></world>')
    geometry=ET.tostring(root.find('model/link'));actor=ET.tostring(root.find('actor'))
    records=repairs.static_fixture_snapshot(root)
    assert records==[dict(model='cart',source_static='false',derived_static=True,
        reason='Static environment snapshot; dynamic fixtures are not implemented')]
    assert ET.tostring(root.find('model/link'))==geometry and ET.tostring(root.find('actor'))==actor
    assert root.findtext('model/pose')=='1 2 0 0 0 .3'
    assert root.find("model[@name='building']/static").text=='1'


def test_canonical_collada_keeps_unprefixed_names_for_native_gazebo(tmp_path):
    source = tmp_path/'source.dae'
    source.write_text('<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">'
        '<asset><up_axis>Y_UP</up_axis></asset><library_geometries/></COLLADA>')
    target,report = repairs.gazebo_collada_copy(source,tmp_path/'derived',lambda u,b:None)
    text = target.read_text()
    assert '<COLLADA xmlns=' in text and '<library_geometries' in text
    assert 'ns0:' not in text
    assert report['recipe'] == 'gazebo-node-frame-v2'
    assert 'Y_UP' in source.read_text()


def test_occupancy_cache_rejects_changed_geometry_and_missing_artifact(tmp_path):
    mesh = tmp_path/'floor.dae';mesh.write_text('original geometry')
    output = tmp_path/'map.yaml';output.write_text('image: map.pgm')
    image = output.with_suffix('.pgm');image.write_bytes(b'P5\n1 1\n255\n\xff')
    sha = lambda path: repairs.hashlib.sha256(path.read_bytes()).hexdigest()
    report = dict(source_sha256='world-sha',seed_xy=[1,2],output_yaml=str(output),
        projection_recipe='complete-static-height-slice-v3',mesh_sha256={str(mesh):sha(mesh)},
        artifact_sha256={'.yaml':sha(output),'.pgm':sha(image)})
    assert repairs.cached_occupancy_matches(report,'world-sha',[1,2])
    mesh.write_text('changed geometry at the same URI')
    assert not repairs.cached_occupancy_matches(report,'world-sha',[1,2])
    report['mesh_sha256'][str(mesh)] = sha(mesh)
    assert repairs.cached_occupancy_matches(report,'world-sha',[1,2])
    image.unlink()
    assert not repairs.cached_occupancy_matches(report,'world-sha',[1,2])


def test_reviewed_source_restores_only_z_and_keeps_both_floors(tmp_path, monkeypatch):
    source = tmp_path/'world.sdf';source.write_text('test reviewed snapshot')
    sha = repairs.hashlib.sha256(source.read_bytes()).hexdigest()
    monkeypatch.setitem(repairs.HOSPITAL_SOURCES, sha, 1)
    root = ET.Element('sdf')
    for prefix in ('', 'floor_1_'):
        for name in repairs.FIXTURES:
            model = ET.SubElement(root,'model',name=prefix+name)
            ET.SubElement(model,'pose').text='1 2 -754989000 .01 -.15 .8'
    untouched = ET.SubElement(root,'model',name='unreviewed_fixture')
    ET.SubElement(untouched,'pose').text='5 6 -9 0 0 1'
    before = source.read_bytes()
    records = repairs.repair_hospital_poses(source,root)
    assert len(records) == 6
    for record in records:
        values = record['derived_pose'].split()
        assert values[:2] == ['1','2'] and values[3:] == ['.01','-.15','.8']
        assert values[2] == ('3' if record['model'].startswith('floor_1_') else '0')
    assert untouched.findtext('pose') == '5 6 -9 0 0 1'
    assert source.read_bytes() == before


def test_changed_reviewed_defect_is_rejected(tmp_path,monkeypatch):
    source = tmp_path/'world.sdf';source.write_text('test reviewed snapshot')
    monkeypatch.setitem(repairs.HOSPITAL_SOURCES,repairs.hashlib.sha256(source.read_bytes()).hexdigest(),0)
    root = ET.fromstring('<sdf><model name="StorageRack_1"><pose>1 2 0 0 0 0</pose></model></sdf>')
    with pytest.raises(ValueError,match='defect changed'):
        repairs.repair_hospital_poses(source,root)


def test_collada_derivative_preserves_scene_vertices_metres_and_texture_closure(tmp_path):
    source=tmp_path/'floor.dae';texture=tmp_path/'tile.png';texture.write_bytes(b'texture')
    source.write_text('''<COLLADA><asset><unit meter=".01"/><up_axis>Y_UP</up_axis></asset>
        <library_images><image id="tile"><init_from>tile.png</init_from></image></library_images>
        <library_geometries><geometry id="mesh"/></library_geometries>
        <library_visual_scenes><visual_scene><node><rotate>0 1 0 90</rotate></node></visual_scene></library_visual_scenes></COLLADA>''')
    before=source.read_bytes()
    target,report=repairs.gazebo_collada_copy(source,tmp_path/'derived',lambda uri,base:base/uri)
    tree=ET.parse(target).getroot()
    assert tree.findtext('asset/up_axis')=='Z_UP'
    assert tree.find('asset/unit').get('meter')=='.01'
    assert tree.findtext('library_visual_scenes/visual_scene/node/rotate')=='0 1 0 90'
    assert tree.findtext('library_images/image/init_from')==texture.as_uri()
    assert report['unresolved_textures']==[] and str(texture) in report['texture_sha256']
    assert source.read_bytes()==before


def test_already_z_up_is_not_rewritten(tmp_path):
    source=tmp_path/'mesh.dae';source.write_text('<COLLADA><asset><up_axis>Z_UP</up_axis></asset></COLLADA>')
    assert repairs.gazebo_collada_copy(source,tmp_path/'derived',None)==(source,None)


def test_gazebo_accepted_invalid_comments_are_removed_only_in_derivative(tmp_path):
    source=tmp_path/'mesh.dae';source.write_text('<COLLADA><!-- author -- export -->'
        '<asset><up_axis>Z_UP</up_axis></asset><library_geometries/></COLLADA>')
    before=source.read_bytes()
    target,report=repairs.gazebo_collada_copy(source,tmp_path/'derived',None)
    assert report['invalid_xml_comments_removed']
    assert ET.parse(target).find('library_geometries') is not None
    assert source.read_bytes()==before


def test_assimp_literal_attribute_brackets_keep_the_original_node_name(tmp_path):
    source=tmp_path/'mesh.dae';source.write_text('<COLLADA><asset><up_axis>Z_UP</up_axis></asset>'
        '<library_visual_scenes><visual_scene><node name="<STL_BINARY>"/></visual_scene></library_visual_scenes></COLLADA>')
    before=source.read_bytes()
    target,report=repairs.gazebo_collada_copy(source,tmp_path/'derived',None)
    assert report['invalid_xml_attribute_brackets_escaped']
    assert not report['invalid_xml_comments_removed']
    assert ET.parse(target).find('.//node').get('name')=='<STL_BINARY>'
    assert source.read_bytes()==before


def test_floor_support_uses_real_triangles_instead_of_filling_a_gap():
    # Two separated strips in one authored mesh. Their combined bounds
    # enclose a void at x=0 which must remain unsupported.
    vertices=[[-3,-2,0],[-1,-2,0],[-1,2,0],[-3,2,0],
              [1,-2,0],[3,-2,0],[3,2,0],[1,2,0]]
    mesh=trimesh.Trimesh(vertices=vertices,faces=[[0,1,2],[0,2,3],[4,5,6],[4,6,7]],process=False)
    polygons=repairs.floor_mesh_polygons(mesh)
    assert len(polygons)==4
    assert all(p[:,0].max()<0 or p[:,0].min()>0 for p in polygons)


def test_upper_decks_furniture_and_vertical_walls_cannot_supply_ground_support():
    for extent,position in [([4,4,.1],[0,0,3]),([1,1,.1],[0,0,0]),([4,.1,4],[0,0,0])]:
        mesh=trimesh.creation.box(extents=extent);mesh.apply_translation(position)
        assert repairs.floor_mesh_polygons(mesh)==[]
