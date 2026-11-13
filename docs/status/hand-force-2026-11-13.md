# Native hand force feedback — November 13, 2026

Codex owns this R5.8 continuation; Claude's aerial/import edits are preserved.
The existing Hand page now offers explicit source-bound force regulation after
an established two-sided grasp. It commands bounded position-target changes
and leaves source PID/transmissions/actuator force limits unchanged. The
[operator guide](../tutorials/native-hand-controls.md#regulate-an-established-grasp)
describes requested force, effort fraction, timing and interruption behavior.

[40 focused/regression/tutorial checks](evidence/hand-force-2026-11-13/checks-final.log)
pass. robot_lab_utils, robot_lab_mujoco and robot_lab_gui build. An installed
[Xvfb widget/protocol smoke check](evidence/hand-force-2026-11-13/gui.log)
confirms that editing sends no action, explicit activation builds the
source/contract-bound request and non-finite input is rejected. These are
software/control checks, not a normal GUI physical mission.

The pinned Robotiq source component fixture uses its original model and the
existing documented free cube/pedestal producer. It recorded about 12 N per
opposing side before the request and about **5.247 N per side** at the end of a
**5 N** request. There are 50 force-phase samples, 39 reporting holding,
covering about 1.52 simulated seconds between the first/last holding samples.
Source gain and actuator-force arrays were checked unchanged. Explicit Stop
interrupted regulation. [Measured summary, controller hashes and raw trace
hash/path](evidence/hand-force-2026-11-13/component-summary.json).

The object remained supported; there was no lift/transport. This establishes
only a native source-component observation. It does not certify regulation on
DEX-EE/v4/UMI, a normal GUI/map mission, different objects, a hard contact-force
limit or hardware behavior. Source-specific calibration and those missions
remain ROADMAP R5.8 acceptance/implementation work.

Retained failures drove the functional repairs:

- [First component trial](evidence/hand-force-2026-11-13/component-negative-1.json)
  correctly refused an approximately 12 N entry grasp as above twice the 5 N
  request. Bounded unloading now handles a pre-existing over-force grasp;
  further rising overload still stops it. A 50 N entry bound is declared.
- [Second component trial](evidence/hand-force-2026-11-13/component-negative-2.json)
  stopped on existing finger/pedestal contact before changing targets. The
  controller now records that established contact baseline and refuses new or
  worsening external contact. It does not exempt all pedestal contacts.

Raw physical traces and the exact producer remain on SSD:
`/workspace/molar/robot_lab_runtime/hand-force-20261113/`. No fabricated success
flag or selected catalog count is used as mission evidence.

The preceding object-task CI run 43484855884 failed only the required tutorial
Run heading check (1,214 passed, 40 skipped, one failed). The guide heading is
repaired in `37dbc463`; the unchanged tutorial checks pass locally here. The
published revision's complete CI status is separate from these scoped checks.
