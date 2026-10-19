"""Proximal policy optimisation for the legged trainer (torch, CPU or GPU).

A Gaussian MLP actor and an MLP critic (ELU), generalized advantage
estimation, clipped surrogate and value losses, an entropy bonus and the
adaptive learning rate of rsl_rl (KL target).  Observations are normalised
with running statistics, which ``export_onnx`` folds into the first layer,
so the exported network maps raw contract observations to raw actions.

``export_onnx`` writes the ONNX protobuf directly (Gemm and Elu nodes), so
no ``onnx`` package is needed; onnxruntime runs the result.
"""
from __future__ import annotations

import struct

import numpy as np
import torch
from torch import nn


class RunningNorm(nn.Module):
    """Running mean/variance of observations (frozen after training)."""

    def __init__(self, size, epsilon=1e-2):
        super().__init__()
        self.register_buffer('mean', torch.zeros(size))
        self.register_buffer('var', torch.ones(size))
        self.register_buffer('count', torch.tensor(epsilon))

    def update(self, batch):
        mean, var, count = batch.mean(0), batch.var(0, unbiased=False), batch.shape[0]
        delta, total = mean - self.mean, self.count + count
        self.mean += delta * count / total
        self.var = (self.var * self.count + var * count + delta ** 2 * self.count * count / total) / total
        self.count = total

    def forward(self, x):
        return (x - self.mean) / torch.sqrt(self.var + 1e-8)


def mlp(sizes):
    layers = []
    for i in range(len(sizes) - 1):
        layers.append(nn.Linear(sizes[i], sizes[i + 1]))
        if i < len(sizes) - 2:
            layers.append(nn.ELU())
    return nn.Sequential(*layers)


class ActorCritic(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden=(512, 256, 128), init_std=1.0):
        super().__init__()
        self.norm = RunningNorm(obs_dim)
        self.actor = mlp([obs_dim, *hidden, act_dim])
        self.critic = mlp([obs_dim, *hidden, 1])
        self.log_std = nn.Parameter(torch.full((act_dim,), float(np.log(init_std))))

    def distribution(self, obs):
        mean = self.actor(self.norm(obs))
        return torch.distributions.Normal(mean, self.log_std.exp().expand_as(mean))

    def value(self, obs):
        return self.critic(self.norm(obs)).squeeze(-1)

    @torch.no_grad()
    def act(self, obs, deterministic=False):
        dist = self.distribution(obs)
        action = dist.mean if deterministic else dist.sample()
        return action, dist.log_prob(action).sum(-1), self.value(obs)


class PPO:
    def __init__(self, obs_dim, act_dim, num_envs, steps, device='cpu', learning_rate=1e-3,
                 gamma=0.99, lam=0.95, clip=0.2, epochs=5, minibatches=4, entropy=0.01,
                 value_coef=1.0, max_grad_norm=1.0, desired_kl=0.01, hidden=(512, 256, 128)):
        self.device = torch.device(device)
        self.model = ActorCritic(obs_dim, act_dim, hidden).to(self.device)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
        self.lr = learning_rate
        self.gamma, self.lam, self.clip = gamma, lam, clip
        self.epochs, self.minibatches = epochs, minibatches
        self.entropy, self.value_coef = entropy, value_coef
        self.max_grad_norm, self.desired_kl = max_grad_norm, desired_kl
        shape = (steps, num_envs)
        self.obs = torch.zeros(*shape, obs_dim, device=self.device)
        self.actions = torch.zeros(*shape, act_dim, device=self.device)
        self.log_probs = torch.zeros(shape, device=self.device)
        self.values = torch.zeros(shape, device=self.device)
        self.rewards = torch.zeros(shape, device=self.device)
        self.dones = torch.zeros(shape, device=self.device)
        self.means = torch.zeros(*shape, act_dim, device=self.device)
        self.stds = torch.zeros(*shape, act_dim, device=self.device)
        self.step = 0

    def act(self, obs):
        obs = torch.as_tensor(obs, device=self.device)
        with torch.no_grad():
            dist = self.model.distribution(obs)
            action = dist.sample()
            self.obs[self.step] = obs
            self.actions[self.step] = action
            self.log_probs[self.step] = dist.log_prob(action).sum(-1)
            self.values[self.step] = self.model.value(obs)
            self.means[self.step] = dist.mean
            self.stds[self.step] = dist.stddev
        return action.cpu().numpy()

    def record(self, reward, done, timeout):
        reward = torch.as_tensor(reward, device=self.device, dtype=torch.float32)
        done = torch.as_tensor(done, device=self.device, dtype=torch.float32)
        # Bootstrap time-outs with the state's value (as rsl_rl): the episode
        # was cut, not ended.
        reward = reward + self.gamma * self.values[self.step] * torch.as_tensor(
            timeout, device=self.device, dtype=torch.float32)
        self.rewards[self.step] = reward
        self.dones[self.step] = done
        self.step += 1

    def update(self, last_obs):
        with torch.no_grad():
            last_value = self.model.value(torch.as_tensor(last_obs, device=self.device))
        advantages = torch.zeros_like(self.rewards)
        gae = torch.zeros_like(last_value)
        for t in reversed(range(self.step)):
            next_value = last_value if t == self.step - 1 else self.values[t + 1]
            not_done = 1.0 - self.dones[t]
            delta = self.rewards[t] + self.gamma * next_value * not_done - self.values[t]
            gae = delta + self.gamma * self.lam * not_done * gae
            advantages[t] = gae
        returns = advantages + self.values
        flat = lambda x: x[:self.step].reshape(-1, *x.shape[2:])  # noqa: E731
        obs, actions, old_log_probs = flat(self.obs), flat(self.actions), flat(self.log_probs)
        old_values, returns, advantages = flat(self.values), flat(returns), flat(advantages)
        old_means, old_stds = flat(self.means), flat(self.stds)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        size = obs.shape[0]
        batch = size // self.minibatches
        stats = dict(value_loss=0.0, surrogate_loss=0.0, entropy=0.0, kl=0.0)
        updates = 0
        for _ in range(self.epochs):
            order = torch.randperm(size, device=self.device)
            for start in range(0, size - batch + 1, batch):
                index = order[start:start + batch]
                dist = self.model.distribution(obs[index])
                log_prob = dist.log_prob(actions[index]).sum(-1)
                with torch.no_grad():
                    kl = torch.sum(torch.log(dist.stddev / old_stds[index] + 1e-5)
                                   + (old_stds[index] ** 2 + (old_means[index] - dist.mean) ** 2)
                                   / (2.0 * dist.stddev ** 2) - 0.5, dim=-1).mean()
                if self.desired_kl:
                    if kl > 2.0 * self.desired_kl:
                        self.lr = max(1e-5, self.lr / 1.5)
                    elif 0.0 < kl < self.desired_kl / 2.0:
                        self.lr = min(1e-2, self.lr * 1.5)
                    for group in self.optimizer.param_groups:
                        group['lr'] = self.lr
                ratio = torch.exp(log_prob - old_log_probs[index])
                surrogate = -torch.min(ratio * advantages[index],
                                       torch.clamp(ratio, 1 - self.clip, 1 + self.clip) * advantages[index]).mean()
                value = self.model.value(obs[index])
                clipped = old_values[index] + (value - old_values[index]).clamp(-self.clip, self.clip)
                value_loss = torch.max((value - returns[index]) ** 2, (clipped - returns[index]) ** 2).mean()
                entropy = dist.entropy().sum(-1).mean()
                loss = surrogate + self.value_coef * value_loss - self.entropy * entropy
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(), self.max_grad_norm)
                self.optimizer.step()
                stats['value_loss'] += float(value_loss.detach())
                stats['surrogate_loss'] += float(surrogate.detach())
                stats['entropy'] += float(entropy.detach())
                stats['kl'] += float(kl)
                updates += 1
        # The normaliser moves only between iterations, so the log
        # probabilities stored during a rollout stay those of its policy.
        with torch.no_grad():
            self.model.norm.update(obs)
        self.step = 0
        stats = {k: v / max(1, updates) for k, v in stats.items()}
        stats.update(learning_rate=self.lr, action_std=float(self.model.log_std.detach().exp().mean()))
        return stats

    def save(self, path, **extra):
        torch.save(dict(model=self.model.state_dict(), optimizer=self.optimizer.state_dict(),
                        lr=self.lr, **extra), path)

    def load(self, path):
        data = torch.load(path, map_location=self.device, weights_only=False)
        self.model.load_state_dict(data['model'])
        if 'optimizer' in data:
            self.optimizer.load_state_dict(data['optimizer'])
        self.lr = float(data.get('lr', self.lr))
        return data


# --- ONNX export without the onnx package ------------------------------------
def _varint(value):
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def _field(number, wire, payload):
    key = _varint((number << 3) | wire)
    if wire == 0:
        return key + _varint(payload)
    if wire == 5:
        return key + payload
    return key + _varint(len(payload)) + payload


def _string(number, text):
    return _field(number, 2, text.encode())


def _tensor(name, array):
    array = np.ascontiguousarray(array, dtype='<f4')
    body = b''.join(_field(1, 0, int(d)) for d in array.shape)
    body += _field(2, 0, 1)                       # FLOAT
    body += _string(8, name) + _field(9, 2, array.tobytes())
    return body


def _value_info(name, width):
    dims = _field(1, 2, _string(2, 'batch')) + _field(1, 2, _field(1, 0, width))
    tensor_type = _field(1, 0, 1) + _field(2, 2, dims)
    return _string(1, name) + _field(2, 2, _field(1, 2, tensor_type))


def _node(op, inputs, outputs, name, attributes=b''):
    body = b''.join(_string(1, i) for i in inputs) + b''.join(_string(2, o) for o in outputs)
    return body + _string(3, name) + _string(4, op) + attributes


def export_onnx(model, path, input_name='obs', output_name='actions'):
    """Deterministic actor (normalisation folded in) as an ONNX model."""
    linears = [layer for layer in model.actor if isinstance(layer, nn.Linear)]
    with torch.no_grad():
        std = torch.sqrt(model.norm.var + 1e-8)
        mean = model.norm.mean
        weights = [layer.weight.detach().clone() for layer in linears]
        biases = [layer.bias.detach().clone() for layer in linears]
        # W ((x - m) / s) + b  ==  (W / s) x + (b - W (m / s))
        biases[0] = biases[0] - weights[0] @ (mean / std)
        weights[0] = weights[0] / std
    nodes, initializers, current = b'', b'', input_name
    for index, (weight, bias) in enumerate(zip(weights, biases)):
        out = output_name if index == len(weights) - 1 else 'h%d' % index
        initializers += _field(5, 2, _tensor('W%d' % index, weight.cpu().numpy()))
        initializers += _field(5, 2, _tensor('b%d' % index, bias.cpu().numpy()))
        trans_b = _string(1, 'transB') + _field(3, 0, 1) + _field(20, 0, 2)   # INT
        gemm_out = out if index == len(weights) - 1 else 'g%d' % index
        nodes += _field(1, 2, _node('Gemm', [current, 'W%d' % index, 'b%d' % index], [gemm_out],
                                    'gemm%d' % index, _field(5, 2, trans_b)))
        if index < len(weights) - 1:
            alpha = _string(1, 'alpha') + _field(2, 5, struct.pack('<f', 1.0)) + _field(20, 0, 1)  # FLOAT
            nodes += _field(1, 2, _node('Elu', [gemm_out], [out], 'elu%d' % index, _field(5, 2, alpha)))
        current = out
    graph = nodes + _string(2, 'robot_lab_legged_policy') + initializers
    graph += _field(11, 2, _value_info(input_name, weights[0].shape[1]))
    graph += _field(12, 2, _value_info(output_name, weights[-1].shape[0]))
    opset = _string(1, '') + _field(2, 0, 13)
    model_bytes = _field(1, 0, 8) + _string(2, 'robot_lab_adapter.ppo') + _field(7, 2, graph) + _field(8, 2, opset)
    with open(path, 'wb') as handle:
        handle.write(model_bytes)
    return path
