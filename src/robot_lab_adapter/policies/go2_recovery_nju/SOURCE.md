# NJU-RLC Go2 recovery policy

This experimental ONNX actor was exported from the MIT-licensed recovery
checkpoint in [NJU-RLC/quadrupedal-agility](https://github.com/NJU-RLC/quadrupedal-agility),
revision `15d16ea99bc23e5d401b0b0e3b88a8edcc28f0ed`:
`go2_deploy/models/recover_policy/1_model/model.pt`.
The original checkpoint SHA-256 is
`8cc647f956cfce5659c6f76555643d158987c87c9071308cda9f354aae81e326`.
The upstream MIT license is included as `LICENSE`. The ONNX SHA-256 is
`11a40a6c8d4dde82aeb1d17c3ec29b4ddbe97a0a69a120700a83e02b1d781a51`.

The exported graph combines the upstream 57→4 estimator, 10×57 history encoder,
and 12-action actor with the recovery command fixed to `[0, 0, 0, 0, 0, 0, 1]`.
Its inputs are `proprio` `[1,57]` and `history` `[1,570]`; its output is
`joint_action` `[1,12]`. Other checkpoint objects (critic, discriminators,
optimizers, and normalizers) are excluded. The `weights_only=True` load used an
inert `Normalizer` placeholder for the unused metadata and explicitly
allowlisted only the NumPy data types in that checkpoint. PyTorch and ONNX
Runtime output differed by at most `3.875e-7` on a zero-observation sample.
Across 50 finite random input pairs in `[-2, 2]` with NumPy seed 52, the
maximum absolute difference was `1.336e-5`.
Export used PyTorch 2.12.1, ONNX 1.17.0, opset 17, and ONNX Runtime 1.23.2.

This model is supplied for **opt-in simulation experiments**. The upstream
deployment uses a 50 Hz policy, 40/1 joint PD, a 0.25 action scale, a 0.3
hip-action scale, and 10 proprioceptive history frames. Robot Lab implements
that observation and actuation contract with measured MuJoCo/ROS data. No
get-up or terrain qualification follows from the upstream checkpoint alone.
