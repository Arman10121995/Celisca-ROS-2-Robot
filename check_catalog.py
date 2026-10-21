import sys
sys.path.insert(0, '/home/molar1/bumperbot_ws/src/robot_lab_utils')
from catalog_external_assets import CATALOG

for k, v in CATALOG.items():
    entries = v.get('entries', [])
    revision = v.get('revision', '?')
    print(f"{k}: {len(entries)} entries, revision={revision}")