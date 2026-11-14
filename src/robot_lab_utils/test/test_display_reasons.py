"""Display-only reasons derived from each source model (R3.6)."""
from pathlib import Path

import yaml

from robot_lab_utils.display_reasons import _mjcf_facts, _urdf_facts, reason

GENERATED = Path(__file__).resolve().parents[2] / 'robot_lab_robots/config/display_reasons.yaml'


def write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text)
    return path


def test_floating_legged_mjcf_needs_a_locomotion_policy(tmp_path):
    legs = ''.join('<body name="%s_thigh"><joint name="%s_hip" type="hinge"/><body name="%s_foot"><joint name="%s_knee" '
                   'type="hinge"/></body></body>' % (s, s, s, s) for s in ('fl', 'fr', 'rl', 'rr'))
    path = write(tmp_path, 'dog.xml', '<mujoco><worldbody><body name="trunk"><freejoint/>%s</body></worldbody>'
                 '<actuator><motor joint="fl_hip"/></actuator></mujoco>' % legs)
    code, text = reason(_mjcf_facts(path), {})
    assert code == 'needs_locomotion_policy' and 'fall' in text


def test_drone_and_torque_and_bare_urdf_reasons(tmp_path):
    drone = write(tmp_path, 'x2.xml', '<mujoco><worldbody><body name="x2"><freejoint/></body></worldbody><actuator>'
                  + '<motor site="s"/>' * 4 + '</actuator></mujoco>')
    assert reason(_mjcf_facts(drone), {})[0] == 'needs_flight_controller'
    arm = write(tmp_path, 'arm.xml', '<mujoco><worldbody><body name="link"><joint name="j" type="hinge"/></body></worldbody>'
                '<actuator><motor joint="j"/></actuator></mujoco>')
    assert reason(_mjcf_facts(arm), {})[0] == 'torque_actuators_only'
    urdf = write(tmp_path, 'arm.urdf', '<robot name="r"><link name="a"/><link name="b"/><joint name="j" type="revolute">'
                 '<parent link="a"/><child link="b"/></joint></robot>')
    code, text = reason(_urdf_facts(urdf), {})
    assert code == 'no_control_interface' and '1 movable joints' in text


def test_generated_file_gives_every_entry_a_reason():
    data = yaml.safe_load(GENERATED.read_text())
    assert data and all(row['reason'] and row['code'] for row in data.values())
    assert 'menagerie_ufactory_lite6' not in data            # unnamed actuators are controllable now


def test_native_locomotion_profiles_are_controllable_and_siblings_are_named(tmp_path):
    """R3.6: a vendor policy makes its own profile controllable; family members point to it."""
    from robot_lab_utils.asset_groups import AssetGroups
    from robot_lab_utils.display_reasons import display_only, generate
    legs = ''.join('<body name="%s_thigh"><joint name="%s_hip" type="hinge"/><body name="%s_foot"><joint name="%s_knee" '
                   'type="hinge"/></body></body>' % (s, s, s, s) for s in ('fl', 'fr', 'rl', 'rr'))
    path = write(tmp_path, 'dog.xml', '<mujoco><worldbody><body name="trunk"><freejoint/>%s</body></worldbody>'
                 '<actuator><motor joint="fl_hip"/></actuator></mujoco>' % legs)
    profiles = {'menagerie_dog': dict(native_mjcf=str(path), supported_modes=['display']),
                'policy_dog': dict(native_mjcf=str(path), supported_modes=['display'], locomotion_policy_config='x.yaml')}
    assert not display_only(profiles['policy_dog'])
    groups = AssetGroups('robots', profiles, dict(schema_version=1, robots=[dict(id='dog', name='Dog', members=[
        dict(id='menagerie_dog', label='Source model'), dict(id='policy_dog', label='Vendor policy model')])]))
    result = generate(profiles, groups)
    assert list(result) == ['menagerie_dog']
    assert result['menagerie_dog']['controllable_variants'] == ['policy_dog']
    assert 'Vendor policy model (policy_dog)' in result['menagerie_dog']['reason']
    assert generate(profiles)['menagerie_dog'].get('controllable_variants') is None
