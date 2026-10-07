"""Prepare source geometry for the GUI's embedded native 3D viewport.

This bounded child only reads assets and writes SSD preview buffers. It does
not start RViz, a simulator, a ROS node or a command publisher.
"""
import argparse
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from robot_lab_utils.asset_preview import prepare_scene, prepare_render_geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', required=True, type=Path)
    args = parser.parse_args()
    request = json.loads(args.request.read_text())
    if request.get('schema_version') != 1:
        raise ValueError('Unsupported preview request')
    output = Path(request['output'])
    if Path('/workspace') not in output.resolve().parents:
        raise ValueError('Preview buffers must be on the workspace SSD')
    started = time.monotonic()
    scene = prepare_scene(request['kind'], request['profile'], output, get_package_share_directory)
    geometry = prepare_render_geometry(scene, output/'geometry.npz')
    scene.update(asset_id=request['asset_id'], render_geometry=geometry,
                 preparation_wall_s=time.monotonic()-started)
    temporary = output/'scene.json.pending'
    temporary.write_text(json.dumps(scene, indent=2)+'\n')
    temporary.replace(output/'scene.json')
    print('Prepared embedded 3D preview:', request['asset_id'], len(scene['shapes']), 'geometries', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
