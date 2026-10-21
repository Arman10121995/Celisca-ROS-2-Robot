import yaml

with open('/home/molar1/bumperbot_ws/docs/status/platform-status.yaml') as f:
    data = yaml.safe_load(f)

phases = data.get('legacy_phases', {})

# Look for R3.6 across all phases
for phase_name, phase_data in phases.items():
    if isinstance(phase_data, dict):
        if 'R3.6' in phase_data:
            print(f"Found R3.6 directly in phase '{phase_name}'")
            print(yaml.dump(phase_data['R3.6'], default_flow_style=False)[:1000])
        # Check nested tasks
        if 'tasks' in phase_data and isinstance(phase_data['tasks'], dict):
            if 'R3.6' in phase_data['tasks']:
                print(f"Found R3.6 in tasks of phase '{phase_name}'")
                print(yaml.dump(phase_data['tasks']['R3.6'], default_flow_style=False)[:1000])

# Also check for any key containing R3
for phase_name, phase_data in phases.items():
    if isinstance(phase_data, dict):
        for key in phase_data.keys():
            if 'R3' in str(key):
                print(f"Phase '{phase_name}' has key: {key}")