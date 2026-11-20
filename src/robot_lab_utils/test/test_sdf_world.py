

def test_relative_files_also_resolve_under_a_model_root(tmp_path):
    # AWS RoboMaker models name meshes as models/<model>/meshes/... relative to
    # the folder holding models/ (Gazebo resource-path search; R6.5 2026-11-20).
    from robot_lab_utils.sdf_world import uri_resolver
    mesh = tmp_path/'dataset'/'models'/'chair'/'meshes'/'chair.dae'
    mesh.parent.mkdir(parents=True)
    mesh.write_text('x')
    resolve = uri_resolver([str(tmp_path/'dataset'/'models')])
    model_dir = str(tmp_path/'dataset'/'models'/'chair')
    assert resolve('model://chair', '') == model_dir
    assert resolve('file://models/chair/meshes/chair.dae', model_dir) == str(mesh)
    assert resolve('file://models/missing.dae', model_dir) == ''
