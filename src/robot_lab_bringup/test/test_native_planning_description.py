"""Independent FK/contact-shape and bounded path checks; no mission claims."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robot_lab_utils'))
from robot_lab_utils.native_planning_description import fixed_arm_planning_model, compiled_arm_sha256  # noqa: E402
from robot_lab_utils.native_arm_trajectory import checked_arm_path, retime_arm_path  # noqa: E402


def urdf_fk(urdf, measured):
    """Independent URDF forward kinematics, including anchor dummy frames."""
    robot = ET.parse(urdf).getroot()
    remaining = list(robot.findall('joint'))
    frames = {'native_world': np.eye(4)}
    while remaining:
        previous = len(remaining)
        for joint in list(remaining):
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in frames:
                continue
            origin = joint.find('origin')
            transform, motion = np.eye(4), np.eye(4)
            transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            if joint.get('type') != 'fixed':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                q = measured[joint.get('name')]
                if joint.get('type') == 'prismatic':
                    motion[:3, 3] = axis*q
                else:
                    motion[:3, :3] = Rotation.from_rotvec(axis*q).as_matrix()
            frames[child] = frames[parent]@transform@motion
            remaining.remove(joint)
        assert len(remaining) < previous, 'URDF graph is disconnected or cyclic'
    return frames


def assert_native_fk(model, description, offsets=(0., .05, -.05)):
    import mujoco as mj
    data = mj.MjData(model)
    if model.nkey:
        mj.mj_resetDataKeyframe(model, data, 0)
    initial = data.qpos.copy()
    worst = 0.
    for offset in offsets:
        data.qpos[:] = initial+offset
        mj.mj_forward(model, data)
        names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j) or 'joint_'+str(j) for j in range(model.njnt)]
        measured = {name: float(data.qpos[int(model.jnt_qposadr[j])]) for j, name in enumerate(names)}
        frames = urdf_fk(description['urdf'], measured)
        for body in range(1, model.nbody):
            expected = np.eye(4)
            expected[:3, 3], expected[:3, :3] = data.xpos[body], data.xmat[body].reshape(3, 3)
            error = float(np.max(np.abs(frames['native_body_'+str(body)]-expected)))
            worst = max(worst, error)
            np.testing.assert_allclose(frames['native_body_'+str(body)], expected, atol=5e-9, rtol=0.)
        for site in range(model.nsite):
            expected = np.eye(4)
            expected[:3, 3], expected[:3, :3] = data.site_xpos[site], data.site_xmat[site].reshape(3, 3)
            np.testing.assert_allclose(frames['native_site_'+str(site)], expected, atol=5e-9, rtol=0.)
    return worst


@pytest.fixture
def native_arm(tmp_path):
    mj = pytest.importorskip('mujoco')
    pytest.importorskip('trimesh')
    source = tmp_path/'arm.xml'
    source.write_text('''<mujoco><compiler angle="radian"/><worldbody><body name="base" pos=".1 -.2 .3" euler=".1 .2 .3">
      <geom type="box" size=".02 .02 .02"/><body name="arm" pos=".05 .04 .03" euler=".3 -.2 .1">
        <joint name="a" axis="0 0 1" pos=".01 -.02 .03" ref=".2" range="-1 1"/>
        <joint name="b" axis="0 1 0" pos="-.03 .02 .01" ref="-.1" range="-1 1"/>
        <joint name="c" type="slide" axis="1 0 0" pos=".02 -.01 .04" ref=".02" range="-.2 .2"/>
        <geom type="capsule" size=".02 .1"/><site name="tool" pos=".03 .04 .2" euler=".2 .3 -.4"/>
      </body></body></worldbody><actuator>
      <position name="pa" joint="a" kp="40" kv="5" ctrlrange="-1 1"/>
      <position name="pb" joint="b" kp="40" kv="5" ctrlrange="-1 1"/>
      <position name="pc" joint="c" kp="40" kv="5" ctrlrange="-.2 .2"/>
    </actuator></mujoco>''')
    return mj, source


def test_nonzero_refs_anchors_multiple_joints_and_spawn_match_native_fk(native_arm, tmp_path):
    mj, source = native_arm
    model = fixed_arm_planning_model(source, tmp_path/'export', 'site:tool', spawn=(.4, -.2, .1, .7))
    spec = mj.MjSpec.from_file(str(source))
    for body in spec.worldbody.bodies:
        body.pos = Rotation.from_euler('z', .7).as_matrix()@body.pos+[.4, -.2, .1]
        result = np.zeros(4)
        mj.mju_mulQuat(result, np.array([np.cos(.35), 0, 0, np.sin(.35)]), body.quat)
        body.quat = result
    assert_native_fk(spec.compile(), model)


def test_capsule_exports_exact_cylinder_plus_end_spheres(native_arm, tmp_path):
    _, source = native_arm
    model = fixed_arm_planning_model(source, tmp_path/'export', 'site:tool')
    link = ET.parse(model['urdf']).find("link[@name='native_body_2']")
    pieces = link.findall('collision')
    assert len(pieces) == 3
    assert [piece.find('geometry')[0].tag for piece in pieces] == ['cylinder', 'sphere', 'sphere']
    centers = [np.fromstring(piece.find('origin').get('xyz'), sep=' ') for piece in pieces]
    np.testing.assert_allclose(centers, [[0, 0, 0], [0, 0, -.1], [0, 0, .1]])


def test_exact_pitch_singularity_preserves_authored_orientation(native_arm, tmp_path):
    mj, source = native_arm
    source.write_text(source.read_text().replace('euler=".1 .2 .3"', 'euler="0 1.5707963267948966 .25"'))
    exported = fixed_arm_planning_model(source, tmp_path/'export', 'site:tool')
    assert_native_fk(mj.MjModel.from_xml_path(str(source)), exported)


def test_retimed_hinge_slide_path_tracks_checked_edges_and_bounds(native_arm, tmp_path):
    _, source = native_arm
    model = fixed_arm_planning_model(source, tmp_path/'export', 'site:tool')
    actual = [.2, -.1, .02]
    via, end = [.3, -.05, .04], [.4, .1, .06]
    points = retime_arm_path(['c', 'b', 'a'], model, [list(reversed(q)) for q in (actual, via, end)], actual)
    plan = checked_arm_path(model['joint_names'], model, points, actual)
    np.testing.assert_allclose(plan.sample(points[0][0])[0], via)
    np.testing.assert_allclose(plan.sample((points[0][0]+points[1][0])/2)[0], (np.array(via)+end)/2)
    rates = np.asarray(model['max_velocity'])
    assert all(np.all(np.abs(plan.sample(t)[1]) <= rates+1e-9) for t in np.linspace(0, points[-1][0], 201))
    with pytest.raises(ValueError, match='bounds'):
        checked_arm_path(model['joint_names'], model, [(20., [1., 0., .02])], actual)
    with pytest.raises(ValueError, match='velocity/acceleration'):
        checked_arm_path(model['joint_names'], model, [(.05, end)], actual)


def test_compiled_fingerprint_changes_for_included_collision_or_gain(native_arm):
    mj, source = native_arm
    model = mj.MjModel.from_xml_path(str(source))
    before = compiled_arm_sha256(model, model.nbody, model.njnt, model.nu)
    model.geom_size[1, 0] += .001
    assert before != compiled_arm_sha256(model, model.nbody, model.njnt, model.nu)
    model.geom_size[1, 0] -= .001
    model.actuator_gainprm[0, 0] += 1.
    assert before != compiled_arm_sha256(model, model.nbody, model.njnt, model.nu)


def test_unexported_coupling_is_rejected(native_arm, tmp_path):
    _, source = native_arm
    source.write_text(source.read_text().replace('</mujoco>', '<equality><joint joint1="a" joint2="b"/></equality></mujoco>'))
    with pytest.raises(ValueError, match='equality'):
        fixed_arm_planning_model(source, tmp_path/'export', 'site:tool')
