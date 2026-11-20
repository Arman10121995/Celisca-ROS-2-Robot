import sys, numpy as np, torch
sys.path.insert(0, '/workspace/molar/ros_ws/bumperbot_ws/src/robot_lab_adapter')
from robot_lab_adapter.legged_training import LeggedTrainingEnv
from robot_lab_adapter.ppo import PPO, read_onnx_mlp
from robot_lab_adapter.bhl_training import input_permutation
onnx = '/workspace/molar/ros_ws/bumperbot_ws/src/robot_lab_robots/_upstream/Berkeley-Humanoid-Lite/checkpoints/policy_humanoid.onnx'
env = LeggedTrainingEnv('berkeley_humanoid_lite_sim', task='humanoid_walk', num_envs=3, seed=1)
layers = read_onnx_mlp(onnx)
agent = PPO(env.obs_dim, env.act_dim, 3, 24, hidden=tuple(int(w.shape[0]) for w, _ in layers[:-1]), normalize=False)
agent.init_actor(layers, input_permutation(env.act_dim), std=0.1)
cmds = [float(v) for v in sys.argv[1].split(',')]
obs = env.reset_all()
env.task = __import__('dataclasses').replace(env.task, resample_s=1e9)
def yaw(e):
    q = e.data_qpos()[:, 3:7] if hasattr(e, 'data_qpos') else None
    return q
import math
out = []
for step in range(int(12 / env.dt)):
    env.commands[:] = cmds
    obs = env.observations()
    with torch.no_grad():
        action = agent.model.actor(agent.model.norm(torch.as_tensor(obs, dtype=torch.float32))).numpy()
    obs, reward, done, info = env.step(np.asarray(action))
    if step % int(2 / env.dt) == 0 or step == int(12 / env.dt) - 1:
        qpos = env._split(env.state)[0][:, :7]
        if qpos is not None:
            q = qpos[:, 3:7]; y = np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]), 1-2*(q[:,2]**2+q[:,3]**2))
            out.append((round(step*env.dt,1), np.round(qpos[:, :2], 2).tolist(), np.round(y, 2).tolist(), done.tolist()))
for o in out: print(o)
