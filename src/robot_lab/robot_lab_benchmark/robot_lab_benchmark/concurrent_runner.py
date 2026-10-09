"""Real concurrent launch/capture jobs with isolated domains and owned cleanup.

Process completion and telemetry are recorded separately from mission success.
This implementation replaces neither historical artifacts nor mission gates.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid


@dataclass
class RunSpec:
    command: list
    duration_s: float = 30.
    startup_timeout_s: float = 300.
    max_rss_mb: float = 4096.
    max_cpu_percent: float = 800.
    max_log_mb: float = 64.
    task_command: list = field(default_factory=list)
    selection: dict = field(default_factory=dict)
    seed: int = 42
    record_bag: bool = False
    readiness_topic: str = '/clock'

    def validate(self):
        for name in ('command', 'task_command'):
            command = getattr(self, name)
            if not isinstance(command, list) or any(not isinstance(part, str) or '\x00' in part for part in command):
                raise ValueError(name+' must be a list of literal executable arguments')
        if not self.command:
            raise ValueError('A real launch command is required')
        if self.readiness_topic not in ('/clock', '/px4/odometry'):
            raise ValueError('Readiness uses actual /clock or PX4 FCU /px4/odometry messages')
        for name in ('duration_s', 'startup_timeout_s', 'max_rss_mb', 'max_cpu_percent', 'max_log_mb'):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name+' must be positive and finite')
            setattr(self, name, value)
        if self.duration_s > 3600 or self.startup_timeout_s > 600:
            raise ValueError('Use bounded jobs: duration <=3600 s and startup <=600 s')


def write_json(path, value):
    temporary = path.with_suffix('.new')
    temporary.write_text(json.dumps(value, indent=2, default=str)+'\n')
    temporary.replace(path)


def source_provenance():
    here = Path(__file__).resolve()
    record = dict(module_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (here, here.with_name('telemetry_recorder.py'))}, scope='Coordinator and recorder source; workload source remains separate')
    repository = next((parent for parent in here.parents if (parent/'.git').exists()), None)
    if repository:
        try:
            def git(*arguments):
                return subprocess.check_output(['git', *arguments], cwd=repository, timeout=10)
            record.update(repository=str(repository), base_revision=git('rev-parse', 'HEAD').decode().strip(),
                working_tree_status=git('status', '--porcelain').decode(),
                tracked_diff_sha256=hashlib.sha256(git('diff', '--binary', 'HEAD')).hexdigest())
        except (OSError, subprocess.SubprocessError) as exc:
            record['error'] = str(exc)
    return record


class ConcurrentRunner:
    def __init__(self, output, max_concurrent=2, notify=None):
        self.output = Path(output).resolve()
        if not str(self.output).startswith('/workspace/'):
            raise ValueError('Experiment artifacts must stay on the workspace SSD')
        self.output.mkdir(parents=True, exist_ok=True)
        if self.output.stat().st_dev == Path('/').stat().st_dev:
            raise ValueError('Refusing experiment artifacts on internal storage')
        if not 1 <= max_concurrent <= 4:
            raise ValueError('Choose one to four concurrent jobs')
        self.parallelism, self.notify = max_concurrent, notify or (lambda _: None)
        runtime = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
        self.leases = runtime/'experiment_domains'
        self.leases.mkdir(parents=True, exist_ok=True)
        if self.leases.stat().st_dev == Path('/').stat().st_dev:
            raise ValueError('Experiment leases must stay on the workspace SSD')
        self.provenance = source_provenance()
        self.jobs, self.lock = {}, threading.Lock()
        self.cancel_all = threading.Event()

    def cancel(self, job_id=None):
        if job_id is None:
            self.cancel_all.set()
        with self.lock:
            for key, job in self.jobs.items():
                if job_id is None or key == job_id:
                    job['cancel'].set()

    def reset(self, job_id):
        with self.lock:
            job = self.jobs.get(job_id)
            if not job or not job.get('ready') or job['cancel'].is_set():
                raise ValueError('Reset requires a ready owned run')
            environment = job['environment'].copy()
            directory = job['directory']
        process = subprocess.Popen(['ros2', 'service', 'call', '/robot_lab/reset',
            'std_srvs/srv/Trigger', '{}'], env=environment, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, start_new_session=True)
        try:
            stdout, stderr = process.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            stdout, stderr = process.communicate()
            stderr += '\nReset response timed out.'
        response = dict(returncode=process.returncode, stdout=stdout, stderr=stderr,
                        scope='Real domain-scoped service response; caller must inspect success acknowledgement')
        write_json(directory/('reset-'+str(time.time_ns())+'.json'), response)
        return response

    def domain_lease(self):
        for domain in range(20, 120):
            handle = (self.leases/('domain-'+str(domain)+'.lock')).open('a')
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return domain, handle
            except BlockingIOError:
                handle.close()
        raise RuntimeError('No free experiment-domain lease')

    def run(self, specifications):
        for specification in specifications:
            specification.validate()
        if self.parallelism > 1 and sum('robot_model:=px4_x500' in spec.command for spec in specifications) > 1:
            raise ValueError('PX4 instance 0 shares MAVLink port 14580; queue multiple X500 jobs with Parallel=1')
        with ThreadPoolExecutor(max_workers=self.parallelism) as pool:
            futures = [pool.submit(self.execute, specification) for specification in specifications]
            return [future.result() for future in futures]

    def execute(self, specification):
        import psutil
        identifier = uuid.uuid4().hex
        directory = self.output/identifier
        directory.mkdir()
        cancel = threading.Event()
        processes, handles, observed = [], [], {}
        try:
            domain, lease = self.domain_lease()
        except Exception as exc:
            result = dict(id=identifier, state='failed', error=str(exc), output=str(directory),
                mission_result=None, command=specification.command, source=self.provenance)
            write_json(directory/'run.json', result)
            self.notify(dict(result))
            return result
        partition = 'robot_lab_'+identifier
        environment = dict(os.environ, ROS_DOMAIN_ID=str(domain), IGN_PARTITION=partition,
            GZ_PARTITION=partition, ROS_LOG_DIR=str(directory/'ros_logs'),
            ROBOT_LAB_EXPERIMENT_ID=identifier, ROBOT_LAB_EXPERIMENT_SEED=str(specification.seed),
            ROBOT_LAB_EXPERIMENT_OUTPUT=str(directory))
        result = dict(id=identifier, state='starting', ros_domain_id=domain, partition=partition,
            command=specification.command, task_command=specification.task_command,
            selection=specification.selection, seed_requested=specification.seed,
            readiness_topic=specification.readiness_topic,
            source=self.provenance, requested_budgets=dict(duration_s=specification.duration_s,
                startup_timeout_s=specification.startup_timeout_s, max_rss_mb=specification.max_rss_mb,
                max_cpu_percent=specification.max_cpu_percent, max_artifact_mb=specification.max_log_mb),
            seed_scope='Recorded and exposed to workload; simulator seed application needs its explicit contract',
            mission_result=None, output=str(directory), qualification='implementation; validation pending')
        job = dict(cancel=cancel, environment=environment, directory=directory, ready=False)
        with self.lock:
            self.jobs[identifier] = job
        resources = (directory/'resources.jsonl').open('w', buffering=1)

        def launch(command, label):
            log = (directory/(label+'.log')).open('w')
            handles.append(log)
            process = subprocess.Popen(command, env=environment, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            try:
                tracked = psutil.Process(process.pid)
                tracked.cpu_percent(None)
                observed[(tracked.pid, tracked.create_time())] = tracked
            except psutil.NoSuchProcess:
                pass
            return process

        started = time.monotonic()
        ready_at = None
        task = None
        try:
            write_json(directory/'run.json', result)
            if self.cancel_all.is_set():
                result['state'] = 'canceled'
                return result
            plant = launch(specification.command, 'launch')
            recorder = launch([sys.executable, '-m', 'robot_lab_benchmark.telemetry_recorder',
                '--output', str(directory)], 'recorder')
            if specification.record_bag:
                launch(['ros2', 'bag', 'record', '-o', str(directory/'bag'),
                        '/clock', '/odom/ground_truth', '/odom', '/scan', '/joint_states',
                        '/px4/odometry', '/px4/odometry_truth'], 'bag')
            overload = 0
            while True:
                now = time.monotonic()
                cpu, rss = 0., 0
                for root in processes:
                    try:
                        parent = psutil.Process(root.pid)
                        for process in [parent, *parent.children(recursive=True)]:
                            identity = (process.pid, process.create_time())
                            if identity not in observed:
                                observed[identity] = process
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
                alive = []
                for identity, process in list(observed.items()):
                    try:
                        if process.create_time() != identity[1] or not process.is_running():
                            continue
                        cpu += process.cpu_percent(None)
                        rss += process.memory_info().rss
                        alive.append(process.pid)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
                resources.write(json.dumps(dict(monotonic=now, cpu_percent=cpu,
                    rss_mb=rss/1024**2, pids=alive, gpu_usage=None))+'\n')
                summary_path = directory/'telemetry-summary.json'
                summary = json.loads(summary_path.read_text()) if summary_path.is_file() else {}
                if ready_at is None and summary.get('topic_counts', {}).get(specification.readiness_topic, 0) >= 2:
                    ready_at = now
                    job['ready'] = True
                    result['state'] = 'capturing'
                    if specification.task_command:
                        task = launch(specification.task_command, 'task')
                    write_json(directory/'run.json', result)
                    self.notify(dict(result))
                if cancel.is_set() or self.cancel_all.is_set():
                    result['state'] = 'canceled'
                    break
                if plant.poll() is not None or recorder.poll() is not None:
                    result['state'] = 'process_exited'
                    break
                # Bound telemetry, bag and ROS logs as well as process stdout.
                log_bytes = sum(path.stat().st_size for path in directory.rglob('*') if path.is_file())
                overloaded = rss/1024**2 > specification.max_rss_mb or cpu > specification.max_cpu_percent
                overload = overload+1 if overloaded else 0
                if overload >= 3 or log_bytes > specification.max_log_mb*1024**2:
                    result['state'] = 'resource_budget_exceeded'
                    break
                if ready_at is None and now-started > specification.startup_timeout_s:
                    result['state'] = 'startup_timeout'
                    break
                if ready_at is not None and now-ready_at >= specification.duration_s:
                    result['state'] = 'capture_completed'
                    break
                if task is not None and task.poll() is not None:
                    result['state'] = 'task_process_completed'
                    result['task_returncode'] = task.returncode
                    break
                if ready_at is not None and now-summary.get('last_topic_monotonic', {}).get(specification.readiness_topic, now) > 10:
                    result['state'] = 'readiness_stream_stalled'
                    break
                time.sleep(.25)
        except Exception as exc:
            result['state'], result['error'] = 'failed', str(exc)
        finally:
            def surviving():
                alive = []
                for (pid, birth), process in observed.items():
                    try:
                        if (process.create_time() == birth and process.is_running()
                                and process.status() != psutil.STATUS_ZOMBIE):
                            alive.append(pid)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
                return alive

            # Signal only tracked process groups, retaining PID birth times to
            # avoid signaling a reused PID or the GUI/coordinator's own group.
            for sig, grace in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
                groups = {p.pid for p in processes if p.poll() is None}
                for (pid, birth), process in observed.items():
                    try:
                        if process.create_time() == birth and process.is_running():
                            groups.add(os.getpgid(pid))
                    except (psutil.NoSuchProcess, psutil.AccessDenied, ProcessLookupError):
                        pass
                for group in groups - {os.getpgrp(), os.getpid()}:
                    try:
                        os.killpg(group, sig)
                    except OSError as exc:
                        result.setdefault('cleanup_errors', []).append(str(exc))
                deadline = time.monotonic()+grace
                while time.monotonic() < deadline and (any(p.poll() is None for p in processes) or surviving()):
                    time.sleep(.1)
            result['process_returncodes'] = [process.poll() for process in processes]
            result['wall_duration_s'] = time.monotonic()-started
            try:
                result['telemetry'] = json.loads(summary_path.read_text()) if 'summary_path' in locals() and summary_path.is_file() else None
            except (OSError, ValueError) as exc:
                result['telemetry'], result['telemetry_error'] = None, str(exc)
            result['observed_surviving_pids'] = surviving()
            result['cleanup_complete'] = not result['observed_surviving_pids'] and all(p.poll() is not None for p in processes)
            result['mission_result'] = None  # Workload result contracts are explicit, never inferred from exit zero.
            write_json(directory/'run.json', result)
            resources.close()
            for handle in handles:
                handle.close()
            lease.close()
            with self.lock:
                self.jobs.pop(identifier, None)
            self.notify(dict(result))
        return result


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('specifications', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--parallel', type=int, default=2)
    options = parser.parse_args()
    specifications = [RunSpec(**entry) for entry in json.loads(options.specifications.read_text())]
    runner = ConcurrentRunner(options.output, options.parallel, lambda result: print(json.dumps(result), flush=True))
    def stop(_signal, _frame):
        runner.cancel()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    results = runner.run(specifications)
    raise SystemExit(0 if all(r['state'] == 'capture_completed' or
        (r['state'] == 'task_process_completed' and r.get('task_returncode') == 0)
        for r in results) else 1)


if __name__ == '__main__':
    main()
