#!/usr/bin/env python3
"""Train, export and evaluate legged-robot policies (robot_lab_adapter.legged_training).

  legged-train train --robot unitree_go2 --task walk --envs 512 --iterations 1500 --out RUN
  legged-train export --checkpoint RUN/checkpoints/latest.pt --out RUN/policy
  legged-train evaluate --policy-dir RUN/policy --task walk

``train --randomize --terrain rough --obs-noise 1`` adds legged_gym-style
domain randomisation, a rough heightfield and observation noise.

``train`` writes into RUN: ``run.json`` (settings), ``progress.jsonl`` (one
line per iteration), ``checkpoints/`` and, when it finishes, is interrupted
(SIGINT/SIGTERM) or finds a ``STOP`` file, the deployable policy directory
``policy/``.  Run it with ``legged_policy:=RUN/policy`` on the same robot.
``evaluate`` drives the exported policy through the deployment code
(legged_policy.LeggedPolicy) in the training simulator and reports the
achieved velocities per command.
"""
import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


def _revision():
    try:
        here = Path(__file__).resolve().parent
        return subprocess.run(['git', '-C', str(here), 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:  # noqa: BLE001
        return ''


def export_policy(checkpoint, out_dir):
    """Deployable policy directory from a training checkpoint."""
    import torch
    import yaml
    from .legged_policy import PolicyContract
    from .legged_training import policy_config, template_directory
    from .ppo import ActorCritic, export_onnx
    data = torch.load(checkpoint, map_location='cpu', weights_only=False)
    run = data['run']
    template = template_directory(run['robot'])
    contract = PolicyContract.load(template)
    model = ActorCritic(run['obs_dim'], run['act_dim'], tuple(run['hidden']))
    model.load_state_dict(data['model'])
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    source = json.loads((Path(template) / 'source.json').read_text())
    robot_key = source['robot']
    variant = 'lab_' + run['task'] + '_' + run['name']
    shutil.copyfile(Path(template) / 'base.yaml', out / 'base.yaml')
    (out / 'config.yaml').write_text(
        '# Trained by robot_lab_adapter.legged_train in the contract of %s.\n' % Path(template).name
        + yaml.safe_dump(policy_config(contract, robot_key, variant), sort_keys=False))
    export_onnx(model, str(out / 'policy.onnx'))
    (out / 'source.json').write_text(json.dumps(dict(
        robot=robot_key, variant=variant, profile=run['robot'],
        trained_by='robot_lab_adapter.legged_train', template_policy=Path(template).name,
        task=run['task'], iteration=int(data.get('iteration', 0)),
        environment_steps=int(data.get('environment_steps', 0)),
        source_revision=run.get('revision', ''), checkpoint=str(Path(checkpoint).resolve()),
        exported=time.strftime('%Y-%m-%dT%H:%M:%S'),
        qualification='trained in the MuJoCo training environment; runtime evidence pending'),
        indent=1) + '\n')
    # The exported directory must load as an ordinary policy.
    PolicyContract.load(out)
    return out


def train(args):
    import torch
    from .legged_training import LeggedTrainingEnv, Randomization
    from .ppo import PPO
    torch.set_num_threads(args.torch_threads or max(1, (os.cpu_count() or 2) - 2))
    torch.manual_seed(args.seed)
    out = Path(args.out)
    (out / 'checkpoints').mkdir(parents=True, exist_ok=True)
    name = out.name.replace(' ', '_')
    env = LeggedTrainingEnv(args.robot, args.task, args.envs, seed=args.seed, threads=args.threads,
                            episode_s=args.episode_s,
                            randomization=Randomization() if args.randomize else None,
                            terrain=args.terrain, roughness=args.roughness, obs_noise=args.obs_noise)
    hidden = (512, 256, 128)
    agent = PPO(env.obs_dim, env.act_dim, args.envs, args.steps, device=args.device,
                learning_rate=args.learning_rate, hidden=hidden, entropy=args.entropy)
    run = dict(robot=args.robot, task=args.task, envs=args.envs, steps=args.steps,
               iterations=args.iterations, learning_rate=args.learning_rate, seed=args.seed,
               episode_s=args.episode_s, randomize=bool(args.randomize), terrain=args.terrain,
               roughness=args.roughness, obs_noise=args.obs_noise, obs_dim=env.obs_dim, act_dim=env.act_dim, hidden=list(hidden),
               name=name, revision=_revision(), template=env.template, policy_period_s=env.dt,
               stand_height_m=env.stand_height, feet=env.num_feet, started=time.strftime('%Y-%m-%dT%H:%M:%S'),
               pid=os.getpid())
    start_iteration, steps_done = 0, 0
    if args.resume:
        data = agent.load(args.resume)
        start_iteration = int(data.get('iteration', 0))
        steps_done = int(data.get('environment_steps', 0))
        run['resumed_from'] = str(args.resume)
    (out / 'run.json').write_text(json.dumps(run, indent=1) + '\n')
    stop = {'requested': False}

    def request_stop(*_):
        stop['requested'] = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    progress = open(out / 'progress.jsonl', 'a', buffering=1)
    obs = env.reset_all()
    returns, lengths = [], []
    iteration = start_iteration
    checkpoint = out / 'checkpoints' / 'latest.pt'
    started = time.monotonic()
    for iteration in range(start_iteration + 1, start_iteration + args.iterations + 1):
        tick = time.monotonic()
        sums = {}
        for _ in range(args.steps):
            actions = agent.act(obs)
            obs, reward, done, info = env.step(actions)
            agent.record(reward, done, info['timeout'] & ~info['fallen'])
            returns.extend(info['finished_returns'].tolist())
            lengths.extend((info['finished_lengths'] * env.dt).tolist())
            for key, value in info['terms'].items():
                sums[key] = sums.get(key, 0.0) + value / args.steps
        collected = time.monotonic()
        stats = agent.update(obs)
        steps_done += args.envs * args.steps
        returns, lengths = returns[-200:], lengths[-200:]
        line = dict(iteration=iteration, environment_steps=steps_done,
                    steps_per_s=round(args.envs * args.steps / max(1e-6, collected - tick)),
                    iteration_s=round(time.monotonic() - tick, 3), wall_s=round(time.monotonic() - started, 1),
                    mean_return=float(np.mean(returns)) if returns else None,
                    mean_episode_s=float(np.mean(lengths)) if lengths else None,
                    terms={k: round(v, 6) for k, v in sums.items()}, **{k: round(v, 6) for k, v in stats.items()})
        progress.write(json.dumps(line) + '\n')
        if iteration % args.checkpoint_every == 0 or stop['requested'] or (out / 'STOP').exists():
            agent.save(checkpoint, run=run, iteration=iteration, environment_steps=steps_done)
            shutil.copyfile(checkpoint, out / 'checkpoints' / ('model_%05d.pt' % iteration))
        if stop['requested'] or (out / 'STOP').exists():
            break
    agent.save(checkpoint, run=run, iteration=iteration, environment_steps=steps_done)
    policy = export_policy(checkpoint, out / 'policy')
    progress.write(json.dumps(dict(event='finished', iteration=iteration, environment_steps=steps_done,
                                   policy=str(policy), stopped=bool(stop['requested'] or (out / 'STOP').exists()))) + '\n')
    progress.close()
    print(json.dumps(dict(policy=str(policy), iteration=iteration, environment_steps=steps_done)))
    return 0


def evaluate(args):
    """Run the exported policy through the deployment code in the training simulator."""
    from .legged_policy import LeggedPolicy, LeggedState, PolicyContract
    from .legged_training import LeggedTrainingEnv
    directory = Path(args.policy_dir)
    contract = PolicyContract.load(directory)
    robot = contract.source['profile']
    policy = LeggedPolicy(contract, model_path=str(directory / 'policy.onnx'))
    env = LeggedTrainingEnv(robot, args.task, 1, seed=args.seed, threads=1, episode_s=1e9,
                            terrain=args.terrain, roughness=args.roughness)
    commands = [tuple(float(v) for v in item.split(',')) for item in args.commands.split(';') if item.strip()]
    phases = []
    for command in commands:
        env._reset(np.array([0]))
        policy.reset()
        env.commands[0] = command
        steps = int(round(args.seconds / env.dt))
        settle = int(round(1.0 / env.dt))
        measured, fallen = [], False
        for step in range(steps):
            qpos, qvel = env._split(env.state)
            quat = qpos[0, 3:7]
            state = LeggedState(dict(zip(env.names, qpos[0, env.qpos_index])),
                                dict(zip(env.names, qvel[0, env.qvel_index])),
                                quat, qvel[0, 3:6])
            policy.step(state, command)
            _, _, done, info = env.step(policy.actions[None, :])
            if done[0]:
                fallen = bool(info['fallen'][0])
                break
            if step >= settle:
                from .legged_training import _quat_rotate_inverse
                qpos, qvel = env._split(env.state)
                linear = _quat_rotate_inverse(qpos[:, 3:7], qvel[:, 0:3])[0]
                measured.append((linear[0], linear[1], qvel[0, 5]))
        mean = np.mean(measured, axis=0) if measured else [math.nan] * 3
        phases.append(dict(command=list(command), measured=[round(float(v), 3) for v in mean],
                           seconds=round(len(measured) * env.dt, 2), fallen=fallen))
    report = dict(policy=str(directory), robot=robot, task=args.task, phases=phases, terrain=args.terrain,
                  simulator='mujoco (training environment)', deployment_code='legged_policy.LeggedPolicy')
    print(json.dumps(report, indent=1))
    if args.out:
        Path(args.out).write_text(json.dumps(report, indent=1) + '\n')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    t = sub.add_parser('train')
    t.add_argument('--robot', required=True)
    t.add_argument('--task', default='walk')
    t.add_argument('--envs', type=int, default=512)
    t.add_argument('--iterations', type=int, default=1500)
    t.add_argument('--steps', type=int, default=24, help='policy steps per environment per iteration')
    t.add_argument('--learning-rate', type=float, default=1e-3)
    t.add_argument('--entropy', type=float, default=0.01)
    t.add_argument('--seed', type=int, default=1)
    t.add_argument('--episode-s', type=float, default=20.0)
    t.add_argument('--threads', type=int, default=0, help='MuJoCo rollout threads (0: CPUs - 2)')
    t.add_argument('--torch-threads', type=int, default=0, help='torch threads for the update (0: CPUs - 2)')
    t.add_argument('--device', default='cpu')
    t.add_argument('--randomize', action='store_true',
                   help='per-episode friction, added base mass and motor strength')
    t.add_argument('--terrain', choices=('flat', 'rough'), default='flat')
    t.add_argument('--roughness', type=float, default=0.03, help='rough terrain height range (+- m)')
    t.add_argument('--obs-noise', type=float, default=0.0, help='observation noise level (1: legged_gym)')
    t.add_argument('--checkpoint-every', type=int, default=50)
    t.add_argument('--resume', default='')
    t.add_argument('--out', required=True)
    e = sub.add_parser('export')
    e.add_argument('--checkpoint', required=True)
    e.add_argument('--out', required=True)
    v = sub.add_parser('evaluate')
    v.add_argument('--policy-dir', required=True)
    v.add_argument('--task', default='walk')
    v.add_argument('--commands', default='0.5,0,0;0,0,0.8;0,0,0')
    v.add_argument('--seconds', type=float, default=6.0)
    v.add_argument('--seed', type=int, default=3)
    v.add_argument('--terrain', choices=('flat', 'rough'), default='flat')
    v.add_argument('--roughness', type=float, default=0.03)
    v.add_argument('--out', default='')
    args = parser.parse_args(argv)
    if args.command == 'train':
        if args.threads <= 0:
            args.threads = None
        return train(args)
    if args.command == 'export':
        print(export_policy(args.checkpoint, args.out))
        return 0
    return evaluate(args)


if __name__ == '__main__':
    sys.exit(main())
