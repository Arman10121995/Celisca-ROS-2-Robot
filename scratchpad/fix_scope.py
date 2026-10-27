import sys
p = "docs/status/platform-status.yaml"
lines = open(p).read().splitlines(keepends=True)
i = 2186
new = "        current_scope: 'Catalog restoration completed: the seven missing historical robot registrations (asset_turtlebot3_burger, asset_turtlebot3_waffle, asset_turtlebot3_waffle_pi, asset_turtlebot4_standard, asset_turtlebot4_lite, asset_husky, menagerie_franka_emika_panda) are restored via source-scoped --kind robots --source <source_id> --no-download repairs on cached SSD sources. All seven now resolve to existing xacro paths in installed/robots.yaml and registry_robots.yaml. Preservation merge in provision() verified by test_asset_provision_preservation.py. Missing controller qualification remains visible; no runtime mission qualification claimed.'\n"
lines[i:i+2] = [new]
open(p, "w").writelines(lines)
print("done")
