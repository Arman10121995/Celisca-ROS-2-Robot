import yaml

with open('/home/molar1/bumperbot_ws/docs/status/platform-status.yaml') as f:
    data = yaml.safe_load(f)

phases = data.get('legacy_phases', {})
print(f"legacy_phases type: {type(phases)}")
print(f"legacy_phases keys: {list(phases.keys())[:50]}")

# Find R3.6 anywhere in phases
def find_key(obj, target, path=""):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == target:
                print(f"Found '{target}' at path: {path}.{k}")
                print(f"  Value: {yaml.dump(v, default_flow_style=False)[:500]}")
            find_key(v, target, f"{path}.{k}")
    elif isinstance(obj, list):
        for i, item in enumerate(obj):
            find_key(item, target, f"{path}[{i}]")

find_key(phases, 'R3.6', 'legacy_phases')