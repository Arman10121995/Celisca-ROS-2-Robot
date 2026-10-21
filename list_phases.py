import yaml

with open('/home/molar1/bumperbot_ws/docs/status/platform-status.yaml') as f:
    data = yaml.safe_load(f)

phases = data.get('legacy_phases', {})

for phase_name, phase_data in phases.items():
    if isinstance(phase_data, dict):
        state = phase_data.get('state', '?')
        owner = phase_data.get('owner', '?')
        title = phase_data.get('title', '?')
        if len(title) > 60:
            title = title[:60] + '...'
        print(f"{phase_name}: state={state} owner={owner} title={title}")