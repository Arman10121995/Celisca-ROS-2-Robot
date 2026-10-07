"""Collect the actual full TurtleBot3 mode batch; never synthesize success.

This runs only after all 48 named producers finish. It preserves each report,
recomputes numeric acceptance and source fingerprints, then creates normal
profile certificates. Raw logs/traces/saved maps stay on the mounted SSD.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

import yaml

repo = Path.cwd()
runtime = Path(__file__).parent
sys.path.insert(0, str(repo/'src/robot_lab_utils'))
from robot_lab_utils.asset_support import apply_recorded_modes


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprint(path):
    return dict(path=str(path.relative_to(repo)), sha256=sha(path))


def artifact(path):
    return dict(path=str(path), sha256=sha(path), bytes=path.stat().st_size)


def source_contract(path, source):
    key = str(path.relative_to(repo))
    value = sha(path)
    assert source['source_sha256'].get(key) == value, 'Source differs from measured stage: '+key
    return dict(path=key, sha256=value)


def committed_world_contract(path, source):
    # The producer captured map paths and HEAD, but its broad source sweep
    # did not include robot_lab_maps. Check the unchanged committed bytes and
    # actual installed asset now; keep this distinction explicit in evidence.
    key = str(path.relative_to(repo))
    baseline = subprocess.check_output(['git', 'show', source['git_head']+':'+key])
    assert baseline == path.read_bytes(), 'World/map differs from recorded baseline: '+key
    relative = path.relative_to(repo/'src/robot_lab_maps')
    installed = repo/'install/robot_lab_maps/share/robot_lab_maps'/relative
    assert installed.read_bytes() == baseline, 'Installed world/map differs: '+str(installed)
    return dict(path=key, sha256=sha(path))


spec = importlib.util.spec_from_file_location('support_matrix', repo/'scripts/r9_3_support_matrix.py')
matrix = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = matrix
spec.loader.exec_module(matrix)
profiles = yaml.safe_load((runtime/'qualification/external_assets/installed/robots.yaml').read_text())['robots']
variants = ('burger', 'waffle', 'waffle_pi')
backends = ('gazebo', 'mujoco', 'pybullet', 'isaac')
mode_worlds = (('loc', 'nav_empty'), ('slam', 'nav_empty'),
               ('nav', 'nav_empty'), ('nav', 'nav_obstacle'))
expected = {f'asset_turtlebot3_{variant}-{backend}-{mode}-{world}'
            for variant in variants for backend in backends for mode, world in mode_worlds}
exits = {}
for name in ('burger-pybullet-modes-batch.log', 'remaining-modes-batch.log', 'waffle-pi-gazebo-isaac-modes-batch.log', 'waffle-pi-nav-inflation065-batch.log'):
    for line in (runtime/name).read_text().splitlines():
        found = re.search(r' END (\S+) (-?\d+) (True|False)$', line)
        if found:
            exits[found[1]] = dict(returncode=int(found[2]), passed=found[3] == 'True')
assert set(exits) == expected, 'The complete measured producer batch has not finished'
assert all(row['returncode'] == 0 and row['passed'] is True for row in exits.values())

destination = repo/'docs/status/evidence/turtlebot3-modes-2026-10-07'
destination.mkdir(parents=True, exist_ok=True)
support_path = repo/'docs/status/asset-runtime-support.yaml'
support = yaml.safe_load(support_path.read_text())
index_path = repo/'docs/status/runtime-evidence-index.yaml'
index = yaml.safe_load(index_path.read_text())
index['records'] = [record for record in index['records']
                    if not str(record['report']).startswith(str(destination.relative_to(repo)))]
manifest = dict(raw_root=str(runtime/'modes-final/workflows'),
    baseline='18ecdd24172fd6adfa0d5e2c55809dd2d882d4ae plus each exact pre-trial source/installed stage',
    scope='48 actual named GUI mode producers; this collector does not start or simulate a mission',
    records=[], generation_files=[], world_files=[])
contracts_by_backend = {
    'gazebo': ['src/robot_lab_description/launch/gazebo.launch.py',
               'src/robot_lab_utils/robot_lab_utils/gazebo_physics_world.py'],
    'mujoco': ['src/robot_lab_mujoco/python/robot_lab_mujoco/mujoco_spawner.py',
               'src/robot_lab_mujoco/launch/mujoco_simulator.launch.py'],
    'pybullet': ['src/robot_lab_pybullet/python/robot_lab_pybullet/pybullet_spawner.py',
                'src/robot_lab_pybullet/launch/pybullet_simulator.launch.py'],
    'isaac': ['src/robot_lab_isaac/python/robot_lab_isaac/isaac_spawner.py',
              'src/robot_lab_isaac/python/robot_lab_isaac/isaac_runtime.py',
              'src/robot_lab_isaac/launch/isaac_simulator.launch.py'],
}
common_paths = ['src/robot_lab_utils/robot_lab_utils/robot_spawn.py',
    'src/robot_lab_utils/robot_lab_utils/drive_kinematics.py',
    'src/robot_lab_utils/robot_lab_utils/sensor_config.py',
    'src/robot_lab_utils/robot_lab_utils/sim_frames.py',
    'src/robot_lab_gui/robot_lab_gui/drive_control.py',
    'src/robot_lab_bringup/launch/simulated_robot.launch.py',
    'src/robot_lab_localization/launch/global_localization.launch.py',
    'src/robot_lab_localization/launch/local_localization.launch.py',
    'src/robot_lab_localization/config/amcl.yaml',
    'src/robot_lab_localization/config/ekf.yaml']
mode_paths = {'loc': [], 'slam': ['src/robot_lab_mapping/launch/slam.launch.py',
    'src/robot_lab_mapping/config/slam_toolbox.yaml',
    'src/robot_lab_mapping/robot_lab_mapping/slam_supervisor.py'],
    'nav': ['src/robot_lab_navigation/launch/navigation.launch.py']}
# These generation/collector bytes are recorded now. Runtime source files,
# profiles and the executed URDF are separately captured before every trial.
# Do not describe archive-time fingerprints as pre-trial snapshots.
for path in ('scripts/provision_extension_assets.py', 'scripts/extension_mobile_control.py'):
    manifest['generation_files'].append(dict(fingerprint(repo/path),
        captured_at='collection; executed derivative checksum captured before each trial'))

for variant in variants:
    robot = 'asset_turtlebot3_'+variant
    profile = profiles[robot]
    model = Path(profile['xacro'])
    controller = model.parent/'drive-controllers.yaml'
    first = json.loads((runtime/'modes-final/workflows'/f'{robot}-pybullet-loc-nav_empty'/'source-manifest.json').read_text())
    nav_config = repo/f'src/robot_lab_navigation/config/robots/{robot}.yaml'
    certificate = dict(source_revision='90a68bd2e3c61c12966779da89d8eeaec82730e9',
        drive=profile['drive'], sensor_config=profile['sensor_config'],
        spawn_by_map=profile['spawn_by_map'], executed_urdf_sha256=sha(model),
        generated_model_files=[dict(path=controller.name, sha256=sha(controller))],
        configuration_files=[source_contract(repo/path, first) for path in common_paths],
        configuration_files_by_simulator={}, configuration_files_by_mode={}, screens=[])
    world_contracts = []
    for world in ('nav_empty', 'nav_obstacle'):
        base = repo/'src/robot_lab_maps/maps'/world
        config = base/'maps/map.yaml'
        image = config.parent/yaml.safe_load(config.read_text())['image']
        for path in (base/'worlds'/(world+'.world'), config, image):
            world_contracts.append(committed_world_contract(path, first))
    certificate['configuration_files'].extend(world_contracts)
    if not manifest['world_files']:
        manifest['world_files'] = [dict(item,
            provenance='Verified against recorded HEAD and installed asset at collection; not a pre-trial source-sweep entry')
            for item in world_contracts]
    for backend in backends:
        certificate['configuration_files_by_simulator'][backend] = []
        certificate['configuration_files_by_mode'][backend] = {}
        for mode in ('loc', 'slam', 'nav'):
            proofs = []
            for world in (('nav_empty', 'nav_obstacle') if mode == 'nav' else ('nav_empty',)):
                key = f'{robot}-{backend}-{mode}-{world}'
                raw = runtime/'modes-final/workflows'/key
                report = json.loads((raw/'report.json').read_text())
                source = json.loads((raw/'source-manifest.json').read_text())
                assert report['passed'] is True and report['launch_returncode'] == report['check_returncode'] == 0
                assert (report['robot'], report['backend'], report['mode'], report['map']) == (robot, backend, mode, world)
                assert source['producer_sha256'] == sha(raw/'producer.py')
                assert source['executed_urdf_sha256'] == sha(raw/'executed.urdf') == sha(model)
                for name in ('drive', 'sensor_config', 'spawn_by_map'):
                    assert source['profile'][name] == profile[name], name+' differs from measured profile'
                for item in certificate['configuration_files']:
                    if item in world_contracts:
                        committed_world_contract(repo/item['path'], source)
                    else:
                        assert source['source_sha256'].get(item['path']) == item['sha256']
                certificate['configuration_files_by_simulator'][backend] = [
                    source_contract(repo/path, source) for path in contracts_by_backend[backend]]
                certificate['configuration_files_by_mode'][backend][mode] = [
                    source_contract(repo/path, source) for path in mode_paths[mode]]
                if mode == 'nav':
                    certificate['configuration_files_by_mode'][backend][mode].append(source_contract(nav_config, source))
                measured_name = 'navigation.json' if mode == 'nav' else 'workflow.json'
                measured = json.loads((raw/measured_name).read_text())
                assert report['workflow'] == measured
                kind = 'navigation_screen' if mode == 'nav' else 'workflow_screen'
                assert matrix.measured_result(measured, kind), key
                if mode == 'slam':
                    assert report.get('map_exporter_exited') is True
                    assert (raw/'saved-map.yaml').is_file() and (raw/'saved-map.pgm').stat().st_size > 100
                folder = destination/key
                folder.mkdir(exist_ok=True)
                for name in ('report.json', 'source-manifest.json', 'producer.py', measured_name):
                    shutil.copy2(raw/name, folder/name)
                proofs.append(fingerprint(folder/'report.json'))
                manifest['records'].append(dict(robot=robot, backend=backend, mode=mode, map=world,
                    producer_returncode=exits[key]['returncode'], raw_directory=str(raw),
                    artifacts=[artifact(path) for path in sorted(raw.iterdir()) if path.is_file()]))
                index['records'].append(dict(robot_id=robot, environment_id=world, simulator=backend,
                    task_type=dict(loc='localization', slam='slam', nav='navigation')[mode],
                    date='2026-10-07', revision=source['git_head']+' plus exact pre-trial source/installed stage',
                    report=str((folder/measured_name).relative_to(repo)), sha256=sha(folder/measured_name),
                    source_hashes=str((folder/'source-manifest.json').relative_to(repo)),
                    source_hashes_sha256=sha(folder/'source-manifest.json'), report_kind=kind,
                    scope='Actual Tk selection/autofill/Run/Stop in a private qualification catalog; same '
                          'executed model/drive/sensors/runtime settings as the normal installation. '+
                          ('RViz-style topic goal, independent body terminal error and swept static clearance.'
                           if mode == 'nav' else 'Physical Drive, both turns, publisher loss, reset/resume and actual sensor/estimate feedback.'),
                    limitations=['Named nav_empty/nav_obstacle screens; other maps/routes need measurements',
                        'Lab source LDS at 5 Hz; no RGB-D or OpenCR/vendor firmware qualification',
                        'Non-Gazebo estimation uses ideal engine odometry; no independent noisy hardware localization claim',
                        'Static swept footprint clearance is not measured contact-force telemetry',
                        'Headless RViz can log shader errors; owned plant/launch exits and physical acceptance are measured separately']))
            certificate['screens'].append(dict(backend=backend, mode=mode, reports=proofs))
    validated = apply_recorded_modes(profile, certificate['source_revision'], certificate, repo)
    assert all(modes == ['display', 'loc', 'slam', 'nav']
               for modes in validated['supported_modes_by_simulator'].values())
    support['robots'][robot] = certificate
    print(robot, validated['supported_modes_by_simulator'])

manifest['gui_contract_scope'] = 'Each producer captured the exact GUI source/installed hashes and generated command. Catalog/embedded-view changes are presentation-only; normal GUI selection/autofill and physical repeats are recorded separately. Runtime guards cover actual model/control/estimation/algorithm settings.'
assert len(manifest['records']) == 48
for name in ('burger-pybullet-modes-batch.log', 'remaining-modes-batch.log', 'waffle-pi-gazebo-isaac-modes-batch.log', 'waffle-pi-nav-inflation065-batch.log', 'run_workflows.py', 'modes-final-batch.json'):
    shutil.copy2(runtime/name, destination/name)
shutil.copy2(Path(__file__), destination/'collect_modes.py')
(destination/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
support_path.write_text(yaml.safe_dump(support, sort_keys=False))
index_path.write_text(yaml.safe_dump(index, sort_keys=False))
print('Archived 48 actual mode trials; normal certificates ready for provisioning.')
