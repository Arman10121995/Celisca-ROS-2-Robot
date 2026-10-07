"""Structural filtering never grants runtime capabilities to source assets."""
import copy
from pathlib import Path

import pytest

from robot_lab_utils.asset_groups import AssetGroups, load_group_definitions
from robot_lab_utils.robot_taxonomy import RobotTaxonomy, load_taxonomy


@pytest.fixture
def catalog():
    definitions = load_group_definitions(Path(__file__).resolve().parents[1]/'config/asset_groups.yaml')
    ids = {item['id'] for group in definitions['robots'] for item in group['members']}
    ids.update(['bumperbot', 'four_wheel_steer_car', 'mecanum_car', 'asset_yumi',
                'asset_pr2', 'asset_fetch', 'px4_x500', 'new_unreviewed_robot'])
    profiles = {name: {'supported_modes': ['display']} for name in ids}
    return RobotTaxonomy(AssetGroups('robots', profiles, definitions)), profiles


@pytest.mark.parametrize('model,category,subtype', [
    ('bumperbot', 'Mobile robots', 'Two-wheel drive'),
    ('four_wheel_steer_car', 'Mobile robots', 'Four-wheel drive'),
    ('mecanum_car', 'Mobile robots', 'Four-wheel drive'),
    ('asset_turtlebot3_burger', 'Mobile robots', 'Two-wheel drive'),
    ('unitree_go2', 'Legged robots', 'Four legs'),
    ('berkeley_humanoid_lite_sim', 'Legged robots', 'Two legs'),
    ('asset_r2_description_r2c6', 'Legged robots', 'Two legs'),
    ('menagerie_franka_emika_panda', 'Manipulators', 'Single arm'),
    ('asset_yumi', 'Manipulators', 'Dual arm'),
    ('asset_fetch', 'Mobile manipulators', 'Single arm on mobile base'),
    ('asset_pr2', 'Mobile manipulators', 'Dual arm on mobile base'),
    ('px4_x500', 'Drones', 'Quadrotor'),
])
def test_reviewed_families_have_specific_structural_labels(catalog, model, category, subtype):
    taxonomy, _profiles = catalog
    labels = taxonomy.classify(model)
    assert (labels['category'], labels['subtype']) == (category, subtype)
    assert taxonomy.matches(model, category, subtype)
    assert not taxonomy.matches(model, 'Unclassified', subtype)


def test_partial_robot_is_a_component_and_does_not_inherit_parent_locomotion(catalog):
    taxonomy, _profiles = catalog
    labels = taxonomy.classify('asset_r2_description_r2_left_forearm')
    assert labels['category'] == 'Hands and components'
    assert labels['subtype'] == 'Source subassembly'
    assert 'Legged robots' in labels['tags']
    assert not taxonomy.matches('asset_r2_description_r2_left_forearm', 'Legged robots')


def test_filtering_and_editing_tags_leave_exact_support_unchanged(catalog):
    taxonomy, profiles = catalog
    before = copy.deepcopy(profiles)
    assert 'Single arm' in taxonomy.subtypes('Manipulators')
    assert 'Four legs' not in taxonomy.subtypes('Manipulators')
    labels = taxonomy.classify('mecanum_car')
    assert 'Holonomic' in labels['tags']
    labels['tags'].append('Caller annotation')
    assert 'Caller annotation' not in taxonomy.classify('mecanum_car')['tags']
    assert taxonomy.classify('new_unreviewed_robot')['category'] == 'Unclassified'
    assert profiles == before
    assert profiles['menagerie_franka_emika_panda']['supported_modes'] == ['display']


def test_ambiguous_classification_is_rejected_instead_of_last_entry_winning(tmp_path):
    path = tmp_path/'taxonomy.yaml'
    path.write_text('schema_version: 1\nclasses:\n'
        '  - {category: Drones, subtype: Quadrotor, members: [robot]}\n'
        '  - {category: Mobile robots, subtype: Two-wheel drive, members: [robot]}\n')
    with pytest.raises(ValueError, match='Ambiguous robot classification'):
        load_taxonomy(path)
