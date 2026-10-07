"""Grouping retains legacy identity and never grants a fragment a controller."""
from pathlib import Path
import copy

import pytest

from robot_lab_utils.asset_groups import AssetGroups, load_group_definitions, audit_contained_components


def test_complete_family_keeps_source_variants_components_and_exact_support():
    profiles = {'full': {'supported_modes': ['display', 'nav']},
                'alternate': {'supported_modes': ['display']}, 'arm': {'supported_modes': ['display']}}
    original = copy.deepcopy(profiles)
    definitions = {'robots': [dict(id='family', preferred='full', members=[
        dict(id='full', registry_ids=['legacy']), dict(id='alternate', role='source_alternative'),
        dict(id='arm', role='component')])]}
    groups = AssetGroups('robots', profiles, definitions)
    assert groups.choices() == ['family']
    assert groups.members('family', selectable=True) == ['full', 'alternate']
    assert groups.preferred('family', ['alternate']) == 'alternate'
    view = groups.registry_view({name: {'id': name} for name in ['legacy', 'alternate', 'arm']})
    assert {member['id'] for member in view['family']['members']} == {'legacy', 'alternate', 'arm'}
    assert groups.registry_profiles['legacy'] == 'full'
    assert profiles == original
    assert profiles['alternate']['supported_modes'] == ['display']


@pytest.mark.parametrize('definitions', [
    {'robots': [dict(id='same', members=[]), dict(id='same', members=[])]},
    {'robots': [dict(id='a', members=[{'id': 'arm'}]), dict(id='b', members=[{'id': 'arm'}])]},
    {'robots': [dict(id='a', members=[{'id': 'arm', 'role': 'fully_qualified'}])]},
])
def test_ambiguous_group_metadata_fails_even_if_assets_are_not_installed(definitions):
    with pytest.raises(ValueError):
        AssetGroups('robots', {}, definitions)


def test_uninstalled_preference_and_inspection_only_library_are_not_launch_choices():
    groups = AssetGroups('robots', {'alternative': {}, 'foot': {}, 'new_robot': {}}, {'robots': [
        dict(id='family', preferred='missing', members=[{'id': 'missing'}, {'id': 'alternative'}]),
        dict(id='parts', complete=False, members=[{'id': 'foot', 'role': 'component'}])]})
    assert groups.choices() == ['family', 'new_robot']
    assert groups.preferred('family') == 'alternative'
    assert groups.members('parts') == ['foot']
    assert groups.members('parts', selectable=True) == []


def test_component_containment_checks_internal_topology_not_only_link_names(tmp_path):
    full = tmp_path/'full.urdf'; part = tmp_path/'part.urdf'
    full.write_text('<robot><link name="base"/><link name="arm"/><link name="finger"/>'
        '<joint type="fixed"><parent link="base"/><child link="arm"/></joint>'
        '<joint type="revolute"><parent link="arm"/><child link="finger"/></joint></robot>')
    part.write_text('<robot><link name="world"/><link name="arm"/><link name="finger"/>'
        '<joint type="fixed"><parent link="world"/><child link="arm"/></joint>'
        '<joint type="revolute"><parent link="arm"/><child link="finger"/></joint></robot>')
    profiles = {'full': {'xacro': str(full)}, 'part': {'xacro': str(part)}}
    groups = AssetGroups('robots', profiles, {'robots': [dict(id='assembly', members=[
        {'id': 'full'}, {'id': 'part', 'role': 'component', 'contained_in': 'full'}])]})
    measured = audit_contained_components(groups)[0]
    assert measured['contained'] and measured['links'] == 2 and measured['joints'] == 1
    part.write_text(part.read_text().replace('type="revolute"', 'type="prismatic"'))
    assert audit_contained_components(groups)[0]['contained'] is False


def test_checked_in_groups_cover_reviewed_fragments_and_preserve_celisca_variants():
    definitions = load_group_definitions(Path(__file__).resolve().parents[1]/'config/asset_groups.yaml')
    robots = {item['id']: {} for group in definitions['robots'] for item in group['members']}
    groups = AssetGroups('robots', robots, definitions)
    assert 'nasa_r2' in groups.choices()
    assert groups.family('asset_r2_description_r2_left_gripper') == 'nasa_r2'
    assert 'asset_r2_description_r2_left_gripper' not in groups.members('nasa_r2', selectable=True)
    maps = {item['id']: {} for group in definitions['maps'] for item in group['members']}
    worlds = AssetGroups('maps', maps, definitions)
    assert worlds.members('celisca_floor_2', selectable=True) == ['celisca_floor_2', 'celisca_floor_2_furniture', 'celisca_f2_actor']


def test_dataset_static_and_dynamic_sources_share_the_installed_room_identity():
    definitions = load_group_definitions(Path(__file__).resolve().parents[1]/'config/asset_groups.yaml')
    # Actual importer IDs differ from the upstream directory spelling.
    profiles = {name: {} for room in (2, 3, 4) for name in
                (f'dataset_room{room}', f'dataset_room{room}_world_dynamic')}
    groups = AssetGroups('maps', profiles, definitions)
    assert groups.choices() == ['dataset_room2', 'dataset_room3', 'dataset_room4']
    for room in (2, 3, 4):
        family = f'dataset_room{room}'
        assert groups.members(family, selectable=True) == [family, family+'_world_dynamic']
