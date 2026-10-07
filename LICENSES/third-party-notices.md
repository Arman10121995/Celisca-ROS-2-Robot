# Third-Party Notices

Documentation reviewed October 7, 2026 against runtime checkpoint `091d388`.
The installed robot/world extensions retain upstream sources and model-specific
notices on SSD. See the [pinned source snapshot](../docs/status/asset-sources-2026-10-05.yaml),
[extension guide](../docs/ASSET_EXTENSION_GUIDE.md) and
[current status](../docs/status/CURRENT_STATUS.md). Remaining per-model terms,
redistribution review and clean-host reproduction stay R9.1; installation or
successful control does not establish blanket license compatibility.

The top-level MIT license does not relicense third-party code, policies or
assets. Attribution is restored after the 2026-10-02 audit found that a generated
inventory had removed the robot asset notices. Dependency discovery is not a
complete license verification; the October 1 report records 152 pending entries.

## Robot and collision assets

| Asset | Source / local license | Status |
|---|---|---|
| Unitree descriptions in Awesome-URDFs | `src/robot_lab_robots/_upstream/Awesome-URDFs/LICENSE` | BSD 3-Clause; original Unitree Robotics notice retained |
| Berkeley-Humanoid-Lite software | [upstream](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite), `src/robot_lab_robots/_upstream/Berkeley-Humanoid-Lite/LICENCE` | MIT; Berkeley Humanoid Lite Project Developers notice retained |
| Berkeley-Humanoid-Lite-Assets | `src/robot_lab_robots/_upstream/Berkeley-Humanoid-Lite-Assets/LICENCE` | Attribution-ShareAlike 4.0 International; asset terms retained |
| legacy_humanoid_import | `src/robot_lab_robots/_upstream/legacy_humanoid_import/LICENSE` | Original provenance/license unresolved; this descriptive file does not establish redistribution permission |
| FUJI mecanum roller/hub meshes | [DaiGuard/fuji_mecanum](https://github.com/DaiGuard/fuji_mecanum), revision `646431a5e107448e0e2bdeeba4575db9aa3665ff` | MIT; copyright 2022 DaiGuard, license and unmodified meshes in `src/robot_lab_robots/holonomic_wheels/third_party/fuji_mecanum/` |
| PX4 FCU and upstream X500 simulation assets | [PX4-Autopilot v1.16.2](https://github.com/PX4/PX4-Autopilot/tree/v1.16.2), revision `54f0455ffcd755534539a7cf33a09a20bf71d29d` | BSD 3-Clause; copyright 2012–2023 PX4 Development Team. The SSD checkout retains its LICENSE and submodule notices; the runtime reads upstream X500 assets from that checkout. |
| pymavlink 2.4.50 | [ArduPilot/pymavlink](https://github.com/ArduPilot/pymavlink) | Installed dependency metadata specifies LGPLv3; installed in the SSD venv. Dependency terms are separate from Robot Lab adapter code. |

Additional checkpoints and assets have model-specific provenance in their own
policy directories and evidence reports. Preserve those notices; this table
is not a claim that every external dependency has been audited.

## Software dependency inventory

The existing R9.1 discovery report is an inventory draft, not proof that all
versions, licenses, source URLs or redistribution terms are verified. Its
substring-based license inference and synthetic clean-host tests must be
replaced with actual upstream license/pin checks and executed reproduction
logs. No blanket license compatibility, commercial-use or clean-host claim is
made here. See `docs/status/audit-2026-10-02.md` and R9.1 in the current ledger.
