# CELISCA F1 detailed / building floor 2

Separate original-metre-scale local floor, Z-up. F1 denotes building
floor 2 (2.OG). No historical CELISCA asset or transform is changed.
Reconstruct with `python3 scripts/provision_celisca_detailed.py` on the SSD.

The normal SDF references a source-bound full visual DAE on the SSD. It bakes
all enhanced GLB node transforms with (x,-z,y), no scale/shift/elevation change;
full original furniture and new bathroom visuals remain. The immutable full
GLB and extracted textures are in `/workspace/molar/robot_lab_runtime/external_assets/celisca_detailed/d697a399fb54150ad4a8e56e5e8ba6347c43b7d815270e12d4689b79635f3066`. PBR-to-Phong shading is approximate.
`adapters/` contains checksum-identical source adapters and their explicitly
lighter visual LOD. Their standalone world is a source artifact, not the native
Robot Lab entry; use `worlds/celisca_floor_1_detailed.world` through the normal map selector.

Native collision references the original structural triangles and conservative
per-object furniture/fixture boxes. No full-building convex hull or added floor
slab is used. MuJoCo's normal staging supplies rigid triangle-flex contacts;
PyBullet/Isaac/Gazebo use their existing static triangle-world pathways.
Decorative tile/grout adds no collision. Doors/lifts are static.

`maps/map.yaml` is an offline conservative collision-supported height-band
projection (0.06–2.3 m) at 0.05 m/cell. Only the source-supported spawn component
is free; disconnected/unsupported cells are unknown. `projection.json` records
actual floor and independent horizontal rays at the chosen spawn. This is not
a Fortress plugin export or a Nav2/runtime qualification. Runtime contacts,
visuals, source-map alignment, routes and each backend require actual evidence.

Fixtures/dimensions are approximate research illustrations, not a physical
survey or a safety-compliant building model. Original/derivative rights are
unresolved; no legal/redistribution claim is made. See `PROVENANCE.json` for
all source hashes, coordinate policy, limitations and derivative checks.
