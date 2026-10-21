#!/usr/bin/env python3
import yaml
import sys

with open('/workspace/molar/robot_lab_runtime/external_assets/installed/robots.yaml') as f:
    data = yaml.safe_load(f)

robots = data.get('robots', {})
for name, profile in robots.items():
    sid = profile.get('source_id', '')
    if sid in ('husky_vendor', 'turtlebot3_vendor', 'turtlebot4_vendor'):
        print(f"SOURCE: {sid}", flush=True)
        print(f"  NAME: {profile.get('name')}", flush=True)
        print(f"  XACRO: {profile.get('xacro')}", flush=True)
        print(f"  Native MJCF: {profile.get('native_mjcf')}", flush=True)
        print(f"  Supported simulators: {profile.get('supported_simulators')}", flush=True)
        print(f"  Supported modes: {profile.get('supported_modes')}", flush=True)
        print(f"  Default mode: {profile.get('default_mode')}", flush=True)
        print("---", flush=True)