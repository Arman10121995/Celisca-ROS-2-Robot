# Launch control column and robot categories, October 7

The rebuilt normal GUI keeps Drive, Arm, Hand and Drone controls/limits in a
separate **right-hand column**. Setup and command/algorithm review sit beside
it. At 1024 pixels, Set up / Command & algorithms switch the left content;
the control column stays visible. Returning to a wide window restores setup
and command side by side while retaining their existing widgets and state.

Reviewed structural categories/types distinguish two/four wheels, two/four
legs, single/dual arms, mobile manipulators, hands and drones. Registry shares
these tags, has independent filters and an adjustable native 3D/details pane.
The sidebar, Logs and Stop motion retain existing commands/controllers.

[Actual normal workspace producer](producer.py), [selection report](report.json)
and screenshots cover nine views, exact autofilled commands, neutral input,
all four control pages, tags and 1600/1024 sizes. These are interface checks;
they do not qualify a flight or robot mission. Pre-trial screenshot fingerprints
and [software verification](verification-manifest.json) retain their stages.

Final GUI regression: **94 passed**. The preceding stage passed **936 fast**
(one explicit skip/four integration deselections) and **146 integration** checks;
five final real-Tk layout checks additionally cover compact setup access,
wide-window restoration, limits and original control routes. Changed packages
were rebuilt. The unchanged 20-test physics stage is recorded separately in
[continuation evidence](../continuation-2026-10-07/).

The source-matched normal
[Panda physical regression](../panda-cartesian-controls-column-2026-10-07/README.md)
measures both Cartesian targets and all original invalidation/interruption/
reset/cleanup checks. [Final normal Burger/PyBullet Drive](live-drive-final/report-measured.json)
passes the unchanged neutral enable, W/S/A/D, Stop, publisher loss, source LDS,
wheel/joint/TF and tilt criteria. Preview emits zero movement commands and its
measured x drift is 0.00019 m. The producer and plant exit zero. Actual trace
hashes and derived measured envelopes are retained in its manifest.

Retained negatives include the initial Tk scroll bind-tag naming error, old
test fixtures attaching Arm/Hand to the former notebook, missing tutorial Run
headings, narrow-window clipping and inactive setup after a Notebook/pane
transition. The final layout uses Grid for both setup/session views. An actual
Drive repetition exited at the neutral-phase minimum-sample check; its preview
still sent zero movement commands and its plant exited cleanly. Its failure
remains in `negative/`, separate from measured successful repeats.

See the [operator guide](../../../tutorials/gui-workspace.md) and
[remaining checklist](../../CHECKLIST.md). Unsupported models/modes retain
their actual controller/sensor/backend gates. Physical joystick, other
manipulation backends, unqualified imported worlds and full roadmap acceptance
remain open.
