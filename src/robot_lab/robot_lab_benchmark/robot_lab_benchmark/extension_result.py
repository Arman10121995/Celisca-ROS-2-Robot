"""Read actual acquired artifacts and verify extension result identities."""
import hashlib
import json
from pathlib import Path
import sqlite3

from .extension_contract import RESULT_SCHEMA, CONTRACTS, endpoint_ok


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inspect_map(artifact):
    if not isinstance(artifact,dict): raise ValueError('Missing actual map archive')
    if artifact.get('database'):
        database=Path(artifact['database']).resolve()
        if not str(database).startswith('/workspace/') or not database.is_file():
            raise ValueError('Actual SSD map database missing')
        with sqlite3.connect(database.as_uri()+'?mode=ro',uri=True) as connection:
            if connection.execute('PRAGMA quick_check').fetchone()[0]!='ok':raise ValueError('Invalid acquired SQLite map')
            nodes=connection.execute('SELECT COUNT(*) FROM Node WHERE id>0').fetchone()[0]
        if nodes<1:raise ValueError('No acquired RTAB-Map nodes')
        return dict(kind='database',nodes=nodes,sha256=digest(database))
    import yaml
    import numpy as np
    from PIL import Image
    path=Path(artifact['map_yaml']).resolve()
    if not str(path).startswith('/workspace/'):raise ValueError('Actual SSD occupancy map required')
    metadata=yaml.safe_load(path.read_text());image=(path.parent/metadata['image']).resolve()
    pixels=np.asarray(Image.open(image).convert('L'))
    known=int(np.count_nonzero((pixels==0)|(pixels==254)))
    if known<1:raise ValueError('No acquired occupancy cells')
    return dict(kind='occupancy',known_cells=known,yaml_sha256=digest(path),image_sha256=digest(image))


def read(directory, run_id, domain, specification, invalidated):
    directory=Path(directory);path=directory/'extension-mission-result.json'
    if not path.is_file():return None,'No measured extension result was produced.'
    try:
        result=json.loads(path.read_text())
        if (result.get('schema')!=RESULT_SCHEMA or result.get('run_id')!=run_id
                or result.get('ros_domain_id')!=domain or result.get('workflow')!=specification['workflow']
                or result.get('task')!=specification['task'] or not result.get('session_id')
                or result.get('workload_sha256')!=digest(directory/'workload.json')
                or result.get('trace_sha256')!=digest(directory/'extension-trace.jsonl')
                or result.get('producer_sha256')!=digest(Path(__file__).with_name('extension_workload.py'))):
            raise ValueError('Extension result lacks source/trace/workload/session identity')
        if invalidated:raise ValueError('Reset invalidated this measured workload')
        if type(result.get('success')) is not bool:raise ValueError('Measured outcome is missing')
        contract=result.get('goal_contract')
        allowed=[CONTRACTS[specification['workflow']]]
        if specification['workflow']=='marine':allowed.append(CONTRACTS['auv'])
        if contract not in allowed:raise ValueError('Physical goal contract changed')
        rows=result.get('measurements');counts=result.get('truth_samples',{})
        if not isinstance(rows,list):raise ValueError('Measured endpoints missing')
        if result['success']:
            if result.get('control_released') is not True or not counts or min(counts.values())<3:
                raise ValueError('Native control release and independent observations required')
            if specification['task']=='mapping':
                if len(rows)!=1 or rows[0]['distance_m']<specification['distance_m']:
                    raise ValueError('Acquisition lacks actual physical travel')
                if inspect_map(rows[0]['map_artifact'])!=rows[0]['observation']:
                    raise ValueError('Acquired map changed since measurement')
            else:
                if specification['task']=='navigation' and len(rows)!=len(specification['goals']):
                    raise ValueError('Every requested goal requires a measurement')
                if specification['task']=='formation' and (len(rows)!=len(counts) or len(rows)<2
                        or {r['member_id'] for r in rows}!=set(counts)):
                    raise ValueError('Every actual aerial member requires a measurement')
                for i,row in enumerate(rows):
                    if specification['task']=='navigation':
                        if row['requested']!=specification['goals'][i] or row.get('native_terminal') is not True:
                            raise ValueError('Goal or native terminal identity differs')
                        if specification['workflow']=='gait' and row.get('nav2_status')!=4:
                            raise ValueError('Actual Nav2 success terminal required')
                    if (row.get('settled_sim_s',0)<contract['settle_sim_s']
                            or not endpoint_ok(row.get('physical_pose'),row['requested'],row.get('linear_m_s'),row.get('angular_rad_s'),contract)):
                        raise ValueError('Physical endpoint fails the declared contract')
        return result,None
    except (ValueError,KeyError,TypeError,OSError,sqlite3.Error) as exc:return None,str(exc)
