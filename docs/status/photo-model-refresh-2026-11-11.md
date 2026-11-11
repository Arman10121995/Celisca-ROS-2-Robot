# Photo-based MOLAR assets — 11 November 2026

R11.2 is complete within its asset-refresh scope. Source/installed assets are
updated from the usual molar1 HRI checkout. Windows and both Jetson HRI clients
also received the reviewed models, with private settings preserved.

Carrier/HRI/charger: 100/163/26 named meshes. Three 340 mm cassettes share a
lifting yoke and guide shoes between the towers; the central bed is black.
The base cross-bar and unsupported rear I/O/rating blocks are removed. ASTI
marks, matched upright projectors, side wings and mount details follow photos.
The charger is wall-mounted, with both V-plan plates inside its cabinet outline,
a low cover/shoe, ASTI label and filter. Its internal blue unit is an unverified
manual-illustrated reference, shown separately in the CAD service view.

Detailed F1/F2 (building floors 2/3) retain original floor geometry/transforms
and spawns, with 148/122 added bathroom, wayfinding and charging parts. Full
material-preserving content-addressed visuals, collision derivatives and
occupancy projections were regenerated natively on SSD. Exact previous
predecessors were verified and preserved; see the JSON backup path.

The two asset packages build. 97 installed/source, frame, telescope coupling/FK,
backend contract and MOLAR control tests pass. The installed PyBullet test
measures minimum 0.440 m, maximum 1.100 m and 0.280 m tip travel. Dynamics and
fine mechanical dimensions remain estimates. No new Gazebo/Isaac mission or
physical robot acceptance is claimed. No FMS/AIC/PLC commands were issued.

Nine profile/registry/navigation/localization/GUI files preserve their prior
hashes, retaining the navigation modes and station-transfer work from be016bd0.
Use `scripts/provision_molar_hri.py --geometry-only` for geometry refreshes;
its default path still replaces profile registrations. Preserve all unrelated
legged-policy/training changes currently in this checkout; none were edited
or committed by this asset task.

See [receipt](photo-model-refresh-2026-11-11.json),
[HRI review](/workspace/molar_voice/docs/PHOTO_MODEL_REVIEW_2026-11-11.md) and
[HRI aggregate receipt](/workspace/molar_voice/docs/PHOTO_MODEL_RECEIPT_2026-11-11.json).
Broad R11/platform qualification remains partial in the roadmap.
