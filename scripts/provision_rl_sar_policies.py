#!/usr/bin/env python3
"""Export pinned rl_sar locomotion policies to ONNX for the legged policy controller.

rl_sar (Apache-2.0) ships TorchScript checkpoints plus base.yaml/config.yaml
contracts.  This copies the contract files unchanged, exports each actor
to ONNX (input = the flattened observation the contract describes), checks
that ONNX and TorchScript agree on random inputs, and records provenance in
source.json.  Output: src/robot_lab_adapter/policies/rl_sar/<robot>_<variant>/.

  python3 scripts/provision_rl_sar_policies.py --rl-sar <checkout>
"""
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np

PINNED_REVISION = '376d42c9b128f963ab08579762d5a216a976ce39'
# (robot, variant, lab profile) - the robot models in this repository that
# are the same Unitree descriptions the policies were trained on.
POLICIES = [
    ('a1', 'legged_gym', 'unitree_a1'),
    ('b2', 'robot_lab', 'unitree_b2'),
    ('go2w', 'robot_lab', 'unitree_go2w'),
    ('go2', 'himloco', 'unitree_go2'),
    # 29-DoF humanoid locomotion (robomimic); same observation terms as the
    # quadrupeds, no history.  Joint names match g1_29dof.urdf.
    ('g1', 'robomimic/locomotion', 'unitree_g1_29dof'),
]


def directory_name(robot, variant):
    return robot + '_' + variant.replace('/', '_')


def recurrent_policy(module):
    """An eager copy of an rsl_rl recurrent actor with explicit LSTM state.

    Some checkpoints (g1 robomimic) keep the LSTM state in module buffers
    (``hidden_state``/``cell_state``) and update them inside ``forward``, so
    a plain ONNX export freezes the state.  This rebuilds the same network
    - EmpiricalNormalization ``(x - mean) / (std + eps)``, a one-layer LSTM
    and the Linear/ELU actor - with the state as inputs/outputs.  Returns
    None for a stateless module.
    """
    import torch
    buffers = dict(module.named_buffers())
    if 'hidden_state' not in buffers or 'cell_state' not in buffers:
        return None
    rnn_params = dict(module.rnn.named_parameters())
    hidden = buffers['hidden_state'].shape[-1]
    lstm = torch.nn.LSTM(rnn_params['weight_ih_l0'].shape[1], hidden, 1)
    for name, value in rnn_params.items():
        getattr(lstm, name).data.copy_(value)
    layers = []
    for child in module.actor.children():
        kind = child.original_name
        if kind == 'Linear':
            weight = dict(child.named_parameters())
            layer = torch.nn.Linear(weight['weight'].shape[1], weight['weight'].shape[0])
            layer.weight.data.copy_(weight['weight'])
            layer.bias.data.copy_(weight['bias'])
            layers.append(layer)
        elif kind == 'ELU':
            layers.append(torch.nn.ELU(float(getattr(child, 'alpha', 1.0))))
        else:
            raise SystemExit('unsupported actor layer ' + kind)
    mean = buffers['normalizer._mean'].clone()
    std = buffers['normalizer._std'].clone()
    eps = float(module.normalizer.eps)

    class Recurrent(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.lstm = lstm
            self.actor = torch.nn.Sequential(*layers)
            self.register_buffer('mean', mean)
            self.register_buffer('std', std)

        def forward(self, obs, h_in, c_in):
            x = (obs - self.mean) / (self.std + eps)
            y, (h_out, c_out) = self.lstm(x.unsqueeze(0), (h_in, c_in))
            return self.actor(y.squeeze(0)), h_out, c_out

    return Recurrent().eval(), tuple(buffers['hidden_state'].shape)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rl-sar', required=True, type=Path)
    parser.add_argument('--only', nargs='*', default=None,
                        help='export only these <robot>_<variant> directories '
                             '(re-exporting leaves the others byte-identical)')
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[1]/'src/robot_lab_adapter/policies/rl_sar')
    args = parser.parse_args()
    import onnxruntime
    import torch
    import yaml

    revision = subprocess.check_output(['git', '-C', str(args.rl_sar), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != PINNED_REVISION:
        raise SystemExit('rl_sar checkout is %s, expected pinned %s' % (revision, PINNED_REVISION))
    args.output.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.rl_sar/'LICENSE', args.output/'LICENSE')
    report = []
    for robot, variant, profile in POLICIES:
        if args.only is not None and directory_name(robot, variant) not in args.only:
            continue
        source_dir = args.rl_sar/'policy'/robot/variant
        base = yaml.safe_load((args.rl_sar/'policy'/robot/'base.yaml').read_text())
        config = yaml.safe_load((source_dir/'config.yaml').read_text())
        section = config[robot+'/'+variant]
        history = section.get('observations_history') or []
        frame = int(section['num_observations'])
        width = frame*len(history) if history else frame
        model_path = source_dir/section['model_name']
        module = torch.jit.load(str(model_path), map_location='cpu').eval()
        destination = args.output/directory_name(robot, variant)
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.rl_sar/'policy'/robot/'base.yaml', destination/'base.yaml')
        shutil.copy2(source_dir/'config.yaml', destination/'config.yaml')
        onnx_path = destination/'policy.onnx'
        example = torch.zeros(1, width)
        recurrent = recurrent_policy(module)
        if recurrent is None:
            torch.onnx.export(module, example, str(onnx_path), input_names=['obs'], output_names=['actions'],
                              opset_version=17, dynamo=False)
        else:
            eager, state_shape = recurrent
            zeros = torch.zeros(state_shape)
            torch.onnx.export(eager, (example, zeros, zeros), str(onnx_path),
                              input_names=['obs', 'h_in', 'c_in'], output_names=['actions', 'h_out', 'c_out'],
                              opset_version=17, dynamo=False)
            module.hidden_state.zero_()
            module.cell_state.zero_()
        session = onnxruntime.InferenceSession(str(onnx_path), providers=['CPUExecutionProvider'])
        rng = np.random.default_rng(0)
        worst = 0.0
        state = None if recurrent is None else [np.zeros(recurrent[1], np.float32)] * 2
        # Sequential calls: a recurrent policy must carry its state the same way.
        for _ in range(16):
            sample = rng.normal(0, 1, size=(1, width)).astype(np.float32)
            expected = module(torch.from_numpy(sample)).detach().numpy()
            if state is None:
                actual = session.run(None, {'obs': sample})[0]
            else:
                actual, h, c = session.run(None, {'obs': sample, 'h_in': state[0], 'c_in': state[1]})
                state = [h, c]
            worst = max(worst, float(np.max(np.abs(expected-actual))))
        if worst > 1e-4:
            raise SystemExit('%s/%s: ONNX differs from TorchScript by %g' % (robot, variant, worst))
        source = dict(robot=robot, variant=variant, profile=profile, upstream='https://github.com/fan-ziqi/rl_sar',
                      revision=revision, license='Apache-2.0', torchscript=str(model_path.relative_to(args.rl_sar)),
                      torchscript_sha256=sha256(model_path), onnx_sha256=sha256(onnx_path),
                      input_width=width, max_onnx_torch_difference=worst,
                      recurrent_state=None if recurrent is None else list(recurrent[1]))
        (destination/'source.json').write_text(json.dumps(source, indent=2)+'\n')
        report.append(source)
        print('%s/%s -> %s (input %d, max diff %.2e)' % (robot, variant, destination, width, worst))
    (args.output/'SOURCE.md').write_text(
        '# rl_sar locomotion policies\n\n'
        'Exported by `scripts/provision_rl_sar_policies.py` from '
        '[rl_sar](https://github.com/fan-ziqi/rl_sar) revision `%s` (Apache-2.0, LICENSE '
        'copied here). Each directory keeps the upstream `base.yaml`/`config.yaml` contract '
        'unchanged; `policy.onnx` is the TorchScript actor exported with opset 17 and checked '
        'against it on random inputs (`source.json`).\n' % revision)


if __name__ == '__main__':
    main()
