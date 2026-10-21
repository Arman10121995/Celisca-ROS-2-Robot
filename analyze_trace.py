import json
with open('/workspace/molar/robot_lab_runtime/gazebo_gui_test.json/trace.json') as f:
    d = json.load(f)
body = d['body']
print(f'Total body entries: {len(body)}')
print('First 10 entries:')
for b in body[:10]:
    print(f'  t={b["sim"]:.2f} x={b["x"]:.4f} y={b["y"]:.4f} yaw={b["yaw"]:.4f} tilt={b["tilt"]:.4f}')
print('Last 10 entries:')
for b in body[-10:]:
    print(f'  t={b["sim"]:.2f} x={b["x"]:.4f} y={b["y"]:.4f} yaw={b["yaw"]:.4f} tilt={b["tilt"]:.4f}')