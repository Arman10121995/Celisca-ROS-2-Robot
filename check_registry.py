#!/usr/bin/env python3
from robot_lab_registry.registry import Registry

r = Registry()
r.load()
all_robots = r.robots.get_all()
print("Total robots:", len(all_robots))
# Show first 20 keys
for i, key in enumerate(sorted(all_robots.keys())):
    if i >= 20:
        break
    robot = all_robots[key]
    print(f"  {key}: source_id={robot.get('source_id')}, name={robot.get('name')}")