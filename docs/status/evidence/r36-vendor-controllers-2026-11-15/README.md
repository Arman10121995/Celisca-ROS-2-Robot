# Vendor controllers and per-model asset checks (R3.6, 2026-11-15)

Claude. Implemented, experimental. MuJoCo, headless.

## Vendor locomotion policies

Four legged profiles now run their vendor's own trained policy on their
vendor's own MuJoCo model.

| Profile | Vendor source | Policy | Controller |
|---|---|---|---|
| `policy_unitree_g1`, `policy_unitree_h1`, `policy_unitree_h1_2` | [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) `276801e` (BSD-3-Clause) | `deploy/pre_train/<robot>/motion.pt` | `robot_lab_mujoco.unitree_policy` (was already provisioned) |
| `policy_booster_t1` (new) | [booster_gym](https://github.com/BoosterRobotics/booster_gym) `da396a0` (Apache-2.0) | `deploy/models/T1.pt` | `robot_lab_mujoco.booster_policy` (new) |

**Booster T1 contract.** These come from the vendor's MuJoCo sim-to-sim script (`play_mujoco.py`):

- the 47-value observation;
- the substring-matched gains from `envs/T1.yaml`;
- the PD law, with torques clipped to the actuator `ctrlrange`.

These come from the deploy policy that ships with `T1.pt`:

- the 1 Hz gait clock;
- command smoothing.

**Lab changes, all recorded in the contract:**

- the vendor file's ground plane and lights are removed, because the selected world owns them;
- `meshdir` points at the vendor meshes;
- a command watchdog and a latched fall stop are added.

Every resource carries a sha256, and the controller refuses a changed file.

**Display reasons.** The three Unitree profiles were already wired, but
`display_reasons.py` still listed them as needing a policy. It did not count
`locomotion_policy_config` as a control path. Display-only profiles: 77 → 73.

### Walking screen

The screen is `scripts/screen_native_counterpart.py`: commands go through the
operator input path (`/key_vel`), and position is the true root pose. Each
phase is 6 s of simulated time; startup, neutral and publisher loss are
3 s. All four stayed upright with no fault through the whole screen.

| Profile | Forward 0.12 m/s (0.72 m) | Reverse (−0.72 m) | Turn 0.25 rad/s (1.5 rad) | Neutral drift |
|---|---:|---:|---:|---:|
| Booster T1 | 0.70 m | −0.57 m | 1.34 rad | 0.00 m |
| Unitree G1 | 0.73 m | −0.56 m | 1.35 rad | 0.11 m |
| Unitree H1 | 0.15 m | −1.07 m | 1.16 rad | 0.20 m |
| Unitree H1_2 | 0.64 m (0.42 m sideways) | −0.56 m | 1.39 rad | 0.13 m |

H1 tracks forward commands poorly. All three Unitree policies keep stepping
at zero command. Raw summaries: `screen_<profile>.json`.

## Controllable siblings named in display reasons

When a family has a controllable member, a display-only profile's reason now
names it. The sibling is a different source model, and its controller is
never borrowed. This applies to 12 profiles. For example:

- `menagerie_booster_t1` names `policy_booster_t1`;
- `menagerie_unitree_go2` names `unitree_go2`;
- `asset_ur5_ur5_gripper` names `menagerie_universal_robots_ur5e`.

## Per-model license, mesh, texture and skin audit

`scripts/robot_asset_audit.py` checks all 163 profiles
([asset_audit.md](asset_audit.md), [asset_audit.json](asset_audit.json)):

- **License:** the license file nearest each model, identified from its text.
  The `package.xml` `<license>` tag is a labelled fallback.
- **MJCF:** must compile. That proves every mesh, texture, height field and skin it names exists.
- **URDF and SDF:** every mesh reference must resolve to a file.
- **Textures inside meshes:** OBJ material files and COLLADA image files must exist.

Result: 114 ok, 49 need attention.

- **48 robot_assets models: license unresolved.** That repository ships no
  license file and its URDFs carry no license tag. They stay local until each
  model's terms are recorded. This matches the R9.1 source scan.
- **`unitree_b2_mujoco`:** its OBJ meshes name 10 `.mtl` material files the
  vendor copy never included. The meshes still render with their URDF
  material colours.
- Menagerie OBJ files that name missing `.mtl` files are recorded as unused.
  MuJoCo draws MJCF materials, not OBJ ones.

## Not done

- **Remaining legged models: 47 display-only.** No published vendor policy
  was found for the other Menagerie models (Spot, ANYmal, Cassie, Apollo,
  Fourier N1, PNDbotics Adam Lite, OP3, Barkour, ToddlerBot). ToddlerBot's
  checkpoints are on Google Drive, outside its repository. The same holds for
  the NASA R2 and Valkyrie descriptions. These need training (R12).
- **URDFs without a control interface: 19.** No ros2_control or transmission
  description ships with them, so a controller would invent gains.
