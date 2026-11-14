"""Per-model license, mesh and texture checks (R3.6)."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('robot_asset_audit',
    Path(__file__).resolve().parents[3] / 'scripts' / 'robot_asset_audit.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

MIT = 'Permission is hereby granted, free of charge, to any person obtaining a copy'


def test_nearest_license_walks_up_to_the_source_root(tmp_path):
    (tmp_path / 'LICENSE').write_text(MIT)
    model = tmp_path / 'robots/dog/urdf'
    model.mkdir(parents=True)
    (model / 'dog.urdf').write_text('<robot/>')
    assert module.nearest_license(model / 'dog.urdf', tmp_path) == ('MIT', str(tmp_path / 'LICENSE'))
    (tmp_path / 'robots/dog/LICENSE.md').write_text('Apache License\nVersion 2.0, January 2004')
    assert module.nearest_license(model / 'dog.urdf', tmp_path)[0] == 'Apache-2.0'
    assert module.nearest_license(model / 'dog.urdf', model) == (None, None)       # never above the stop


def test_package_xml_is_the_labelled_fallback(tmp_path):
    (tmp_path / 'package.xml').write_text('<package><name>p</name><license>BSD</license></package>')
    (tmp_path / 'urdf').mkdir()
    assert module.package_xml_license(tmp_path / 'urdf', tmp_path) == ('BSD', str(tmp_path / 'package.xml'))


def test_mesh_textures_follow_obj_materials_and_collada_images_only(tmp_path):
    (tmp_path / 'a.obj').write_text('mtllib a.mtl\nv 0 0 0\n')
    (tmp_path / 'a.mtl').write_text('newmtl m\nmap_Kd tex.png\n')
    found = dict(module.mesh_textures(tmp_path / 'a.obj'))
    assert found == {str(tmp_path / 'a.mtl'): True, str(tmp_path / 'tex.png'): False}
    (tmp_path / 'b.dae').write_text('<COLLADA><library_images><image id="img"><init_from>skin.png</init_from></image>'
                                    '</library_images><surface><init_from>img</init_from></surface></COLLADA>')
    (tmp_path / 'skin.png').write_bytes(b'')
    assert module.mesh_textures(tmp_path / 'b.dae') == [(str(tmp_path / 'skin.png'), True)]   # the id is not a file


def test_urdf_and_sdf_mesh_references_must_exist(tmp_path):
    (tmp_path / 'meshes').mkdir()
    (tmp_path / 'meshes/base.stl').write_bytes(b'')
    urdf = tmp_path / 'r.urdf'
    urdf.write_text('<robot name="r"><link name="a"><visual><geometry><mesh filename="meshes/base.stl"/></geometry>'
                    '</visual><collision><geometry><mesh filename="meshes/gone.stl"/></geometry></collision></link></robot>')
    result = module.check_urdf(urdf, [tmp_path])
    assert result['meshes'] == 2 and result['missing_meshes'] == ['meshes/gone.stl']
    models = tmp_path / 'models'
    (models / 'base/meshes').mkdir(parents=True)
    (models / 'base/meshes/body.dae').write_text('<COLLADA/>')
    (models / 'base/model.sdf').write_text('<sdf><model><link><visual><geometry><mesh><uri>model://base/meshes/body.dae'
                                           '</uri></mesh></geometry></visual></link></model></sdf>')
    (models / 'vehicle').mkdir()
    (models / 'vehicle/model.sdf').write_text('<sdf><model><include><uri>model://base</uri></include></model></sdf>')
    assert module.check_sdf(models / 'vehicle/model.sdf', models) == dict(format='sdf', meshes=1, missing_meshes=[])
