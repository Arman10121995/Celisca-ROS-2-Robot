"""Reviewed structural labels, separate from algorithm/controller support."""
from pathlib import Path

import yaml

ALL_CATEGORIES = 'All categories'
ALL_TYPES = 'All types'
CATEGORIES = ('Mobile robots', 'Legged robots', 'Manipulators', 'Mobile manipulators',
              'Hands and components', 'Drones', 'Reference models', 'Unclassified')


def load_taxonomy(path=None):
    if path is None:
        # Use the same source/installed catalog resolution as asset families.
        for parent in Path(__file__).resolve().parents:
            candidate = parent/'src/robot_lab_bringup/config/robot_taxonomy.yaml'
            if candidate.is_file():
                path = candidate
                break
        else:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('robot_lab_bringup'))/'config/robot_taxonomy.yaml'
    data = yaml.safe_load(Path(path).read_text()) or {}
    if data.get('schema_version') != 1:
        raise ValueError('Unsupported robot taxonomy schema')
    labels = {}
    for item in data.get('classes', []):
        if item['category'] not in CATEGORIES:
            raise ValueError('Unknown robot category: '+item['category'])
        for name in item['members']:
            if name in labels:
                raise ValueError('Ambiguous robot classification: '+name)
            labels[name] = dict(category=item['category'], subtype=item['subtype'],
                                tags=list(item.get('tags', [])))
    return labels


class RobotTaxonomy:
    def __init__(self, groups, definitions=None):
        self.groups = groups
        self.definitions = load_taxonomy() if definitions is None else definitions

    def classify(self, profile_id):
        family = self.groups.family(profile_id)
        result = dict(self.definitions.get(profile_id, self.definitions.get(family,
                      dict(category='Unclassified', subtype='Unreviewed model', tags=[]))))
        result['tags'] = list(result['tags'])
        role = self.groups.member(profile_id).get('role')
        if role in ('component', 'simplified') and result['category'] != 'Hands and components':
            result = dict(category='Hands and components', subtype='Source subassembly',
                          tags=['Component', result['category'], result['subtype']])
        return result

    def matches(self, profile_id, category=ALL_CATEGORIES, subtype=ALL_TYPES):
        result = self.classify(profile_id)
        return ((category == ALL_CATEGORIES or result['category'] == category)
                and (subtype == ALL_TYPES or result['subtype'] == subtype))

    def subtypes(self, category=ALL_CATEGORIES):
        return sorted({self.classify(name)['subtype'] for name in self.groups.profiles
                       if category == ALL_CATEGORIES or self.classify(name)['category'] == category})
