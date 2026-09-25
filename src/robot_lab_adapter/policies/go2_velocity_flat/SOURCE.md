# Go2 flat-ground velocity policy

Model source: [diasAiMaster/unitree-go2-velocity-flat](https://huggingface.co/diasAiMaster/unitree-go2-velocity-flat),
revision `9a723b7ab0784cd86abb942836aa1f208ddde891` (accessed 2026-09-25).
The model card declares the BSD 3-Clause license. The ONNX graph, its
external data and the matching `params/deploy.yaml` are redistributed
unmodified; this adapter is separate project code.

SHA-256:

- `policy.onnx`: `fbb8b61b12f2cd44dfc889f33566a8be2438124efee88b60c700b9605a94187a`
- `policy.onnx.data`: `40474dd235917877d657085f3f0a2d485d10e947e3599cad5328337e32222a74`
- `deploy.yaml`: `a86582e599411ebaa011cb30f1711497a50a21230decf9c91bd02cb43d1e5300`

The policy is opt-in through `go2_policy_path:=auto`. It is trained on the
Unitree MuJoCo model; this workspace imports a URDF into its own MuJoCo
plant, so live forward/turn/reverse/stop and terrain evidence is required
before making it the default or enabling navigation.
The optional path requires Python `onnxruntime` in the same environment as
ROS 2; it is intentionally not a dependency of the default stance path.
