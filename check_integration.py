import json

with open('/workspace/molar/robot_lab_runtime/external_assets/installed/integration-report.json') as f:
    data = json.load(f)

# Show full details for entries needing repair
for e in data['entries']:
    if 'repair' in e.get('status', ''):
        print(f"=== {e['source_id']} / {e['entry']} ===")
        print(f"  Status: {e['status']}")
        print(f"  Reason: {e.get('reason', 'N/A')}")
        print()