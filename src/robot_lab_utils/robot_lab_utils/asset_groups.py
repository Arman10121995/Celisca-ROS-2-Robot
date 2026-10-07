"""Complete-model catalog views without changing executable asset IDs.

Grouping is presentation metadata, never a transfer of controller support.
Unlisted new imports remain independent complete entries until reviewed.
"""
from pathlib import Path

import yaml


INSPECTION_ROLES = {'component', 'simplified', 'reference'}
ROLES = INSPECTION_ROLES | {'variant', 'source_alternative'}


def load_group_definitions(path=None):
    if path is None:
        # Source checkouts and installed ROS packages share the same catalog.
        for parent in Path(__file__).resolve().parents:
            candidate = parent / 'src/robot_lab_bringup/config/asset_groups.yaml'
            if candidate.is_file():
                path = candidate
                break
        else:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('robot_lab_bringup')) / 'config/asset_groups.yaml'
    data = yaml.safe_load(Path(path).read_text()) or {}
    if data.get('schema_version') != 1:
        raise ValueError('Unsupported asset group schema')
    return data


class AssetGroups:
    """One parent per family; variants/components retain their exact profiles."""

    def __init__(self, kind, profiles, definitions=None):
        if kind not in ('robots', 'maps'):
            raise ValueError('Unknown asset group kind: ' + kind)
        self.kind, self.profiles = kind, profiles
        self.groups, self.parents, self.registry_profiles = {}, {}, {}
        definitions = load_group_definitions() if definitions is None else definitions
        declared_groups, declared_members = set(), set()
        for definition in definitions.get(kind, []):
            group_id = definition['id']
            if group_id in declared_groups:
                raise ValueError('Duplicate asset family: ' + group_id)
            declared_groups.add(group_id)
            members = []
            for raw in definition.get('members', []):
                member = dict(raw, role=raw.get('role', 'variant'))
                member_id = member['id']
                if member_id in declared_members:
                    raise ValueError('Asset belongs to multiple families: ' + member_id)
                declared_members.add(member_id)
                if member['role'] not in ROLES:
                    raise ValueError('Unknown member role: ' + member['role'])
                if member_id in profiles:
                    members.append(member)
                    self.parents[member_id] = group_id
                for registry_id in [member_id, *member.get('registry_ids', [])]:
                    previous = self.registry_profiles.get(registry_id)
                    if previous is not None and previous != member_id:
                        raise ValueError('Ambiguous registry asset: ' + registry_id)
                    self.registry_profiles[registry_id] = member_id
            if members:
                self.groups[group_id] = dict(definition, members=members)
        for profile_id, profile in profiles.items():
            if profile_id in self.parents:
                continue
            if profile_id in self.groups:
                raise ValueError('Family ID collides with a different executable profile: ' + profile_id)
            self.groups[profile_id] = dict(id=profile_id, name=profile.get('name', profile_id),
                members=[dict(id=profile_id, label=profile.get('name', profile_id), role='variant')])
            self.parents[profile_id] = profile_id
            self.registry_profiles[profile_id] = profile_id

    def family(self, profile_id):
        return self.parents.get(profile_id, profile_id)

    def members(self, family, selectable=False, allowed=None):
        group = self.groups.get(family, {})
        return [member['id'] for member in group.get('members', [])
                if (not selectable or group.get('complete', True)
                    and member['role'] not in INSPECTION_ROLES)
                and (allowed is None or member['id'] in allowed)]

    def choices(self, allowed=None):
        return sorted(group_id for group_id in self.groups
                      if self.members(group_id, selectable=True, allowed=allowed))

    def preferred(self, family, allowed=None):
        members = self.members(family, selectable=True, allowed=allowed)
        preferred = self.groups.get(family, {}).get('preferred')
        return preferred if preferred in members else (members[0] if members else None)

    def member(self, profile_id):
        group = self.groups.get(self.family(profile_id), {})
        return next((member for member in group.get('members', []) if member['id'] == profile_id), {})

    def registry_view(self, entities):
        """Return nested parents, retaining every entity and exact legacy ID.

        Registry-only entries have no installed preview/Launch target. Core
        registry IDs such as go2 resolve to the existing unitree_go2 profile.
        """
        parents = {}
        for entity_id, entity in entities.items():
            profile_id = self.registry_profiles.get(entity_id, entity_id)
            # Core registry IDs not explicitly grouped still use their launch
            # mapping, supplied by the caller when constructing the view.
            family = self.family(profile_id)
            group = self.groups.get(family, {})
            if family not in parents:
                parents[family] = dict(id=family, name=group.get('name', entity.get('name', family)),
                    complete=group.get('complete', True), notes=group.get('notes', ''), members=[])
            member = self.member(profile_id)
            parents[family]['members'].append(dict(id=entity_id, profile_id=profile_id,
                label=member.get('label', entity.get('name', entity_id)),
                role=member.get('role', 'variant'), entity=entity))
        represented = {member['profile_id'] for group in parents.values() for member in group['members']}
        for profile_id, profile in self.profiles.items():
            if profile_id in represented:
                continue
            family = self.family(profile_id)
            group = self.groups[family]
            if family not in parents:
                parents[family] = dict(id=family, name=group.get('name', family),
                    complete=group.get('complete', True), notes=group.get('notes', ''), members=[])
            member = self.member(profile_id)
            entity = dict(id=profile_id, name=member.get('label', profile_id),
                status='Installed profile variant', features=profile.get('features', []),
                supported_modes=profile.get('supported_modes', ['display']),
                supported_modes_by_simulator=profile.get('supported_modes_by_simulator'),
                xacro=profile.get('xacro'), notes=profile.get('notes', ''),
                provenance='Launch profile view; original canonical registry entity remains unchanged')
            parents[family]['members'].append(dict(id=profile_id, profile_id=profile_id,
                label=member.get('label', profile_id), role=member.get('role', 'variant'), entity=entity))
        return parents


def audit_contained_components(groups):
    """Measure authored subassembly containment; never weld guessed frames.

    Full models already contain these links/joints. Duplicate attachments
    would create a second limb, so the reviewed full source is the assembly.
    """
    import xml.etree.ElementTree as ET
    reports = []
    for group in groups.groups.values():
        for member in group['members']:
            parent_id = member.get('contained_in')
            if not parent_id or parent_id not in groups.profiles:
                continue
            component = ET.parse(groups.profiles[member['id']]['xacro']).getroot()
            parent = ET.parse(groups.profiles[parent_id]['xacro']).getroot()
            # Standalone exports add virtual world attachment frames. The
            # complete source supplies the real limb mounting joint instead.
            virtual_names = {'world', 'r2/world_ref', 'r2/robot_world'}
            anchors = {item.get('name') for item in component.findall('link')
                       if item.get('name') in virtual_names and not item.findall('visual')
                       and not item.findall('collision')}
            links = {item.get('name') for item in component.findall('link')}-anchors
            parent_links = {item.get('name') for item in parent.findall('link')}
            # Shared topology must match, too: names alone are insufficient.
            def joints(root, omit=()):
                return {(j.find('parent').get('link'), j.find('child').get('link'), j.get('type'))
                        for j in root.findall('joint') if j.find('parent').get('link') not in omit
                        and j.find('child').get('link') not in omit}
            internal_joints = joints(component, anchors)
            missing_links, missing_joints = links-parent_links, internal_joints-joints(parent)
            report = dict(component=member['id'], parent=parent_id, links=len(links),
                joints=len(internal_joints), virtual_attachment_frames=sorted(anchors),
                missing_links=sorted(missing_links), missing_joints=sorted(missing_joints),
                contained=not missing_links and not missing_joints,
                scope='Authored internal link/joint topology containment; virtual standalone mount excluded. Not controller or physical equivalence.')
            reports.append(report)
    return reports
