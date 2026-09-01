#!/usr/bin/env python3
"""
Export helper for 3D SLAM (RTAB-Map).
Usage examples:
  python3 export_3d_map.py --db /path/to/map.db --output-dir /tmp/export --pcd
  python3 export_3d_map.py --cloud-topic /rtabmap/cloud_map --output /tmp/map.pcd
"""

import argparse
import os
import struct
import sys
import time
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
except Exception:
    rclpy = None
    PointCloud2 = None
    point_cloud2 = None


def save_pcd_from_msg(msg, output_path):
    """Save a PointCloud2 message to PCD (ascii XYZ or XYZRGB)."""
    points = []
    has_rgb = False

    for p in point_cloud2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
        x, y, z = p[0], p[1], p[2]
        if len(p) > 3 and p[3] is not None:
            has_rgb = True
            rgb_packed = int(p[3])
            r = (rgb_packed >> 16) & 0xFF
            g = (rgb_packed >> 8) & 0xFF
            b = rgb_packed & 0xFF
            points.append((x, y, z, r, g, b))
        else:
            points.append((x, y, z))

    with open(output_path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        if has_rgb:
            f.write("FIELDS x y z rgb\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F U\n")
            f.write("COUNT 1 1 1 1\n")
        else:
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for pt in points:
            if has_rgb:
                f.write(f"{pt[0]} {pt[1]} {pt[2]} {pt[3]*65536 + pt[4]*256 + pt[5]}\n")
            else:
                f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")
    print(f"[export] Saved PCD: {output_path}")


class OneShotCloudSaver(Node):
    def __init__(self, topic, output_path, timeout=15.0):
        super().__init__("export_cloud_saver")
        self.output_path = output_path
        self.received = False
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.callback, 10
        )
        self.timer = self.create_timer(timeout, self.timeout_cb)
        self.start_time = time.time()

    def callback(self, msg):
        if self.received:
            return
        self.received = True
        save_pcd_from_msg(msg, self.output_path)
        rclpy.shutdown()

    def timeout_cb(self):
        if not self.received:
            print(f"[export] Timeout: no point cloud received on {self.subscription.topic_name}")
        rclpy.shutdown()


def export_pcd_live(topic="/rtabmap/cloud_map", output_path="map_cloud.pcd", timeout=20.0):
    if rclpy is None:
        print("rclpy not available, cannot export live cloud.")
        return False
    rclpy.init()
    node = OneShotCloudSaver(topic, output_path, timeout)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Spin error: {e}")
    return Path(output_path).exists()


def export_pcd_from_db(db_path, output_path):
    """Best effort: use rtabmap-export if available, else fallback."""
    db_path = str(db_path)
    output_path = str(output_path)

    # Try rtabmap-export (part of rtabmap)
    candidates = [
        ["rtabmap-export", "--cloud", "--output", output_path, db_path],
        ["rtabmap", "--export", "--cloud", "--output", output_path, db_path],
    ]

    for cmd in candidates:
        try:
            import subprocess
            print(f"[export] Trying: {' '.join(cmd)}")
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            if res.returncode == 0 and Path(output_path).exists():
                print(f"[export] Success via rtabmap tool: {output_path}")
                return True
        except Exception as e:
            print(f"[export] Tool failed: {e}")

    print("[export] Could not auto-export PCD from .db. "
          "Install rtabmap or open the .db with rtabmap-databaseViewer to export manually.")
    return False


def try_export_octomap(cloud_pcd, output_bt):
    """Try to generate OctoMap if octomap_server or octomap_tools available."""
    try:
        import subprocess
        # Common: use octomap_server or pcl to octomap, but simplest is to warn + use known tool
        print("[export] Attempting OctoMap generation (requires octomap_server / octomap_tools).")
        # Placeholder: many setups do:
        # ros2 run octomap_server octomap_saver -f map.bt
        # But needs the node running.
        cmd = ["ros2", "run", "octomap_server", "octomap_saver", "-f", str(output_bt)]
        print(f"[export] Running: {' '.join(cmd)} (may require octomap_server2 running)")
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if Path(output_bt).exists():
            print(f"[export] OctoMap saved: {output_bt}")
            return True
    except Exception as e:
        print(f"[export] OctoMap export not available or failed: {e}")
    print("[export] Tip: ros2 run octomap_server octomap_saver -f map.bt while 3D SLAM is running.")
    return False


def create_simple_world_from_pcd(pcd_path, output_world, map_name="exported_3d"):
    """Create a minimal Gazebo .world that references the PCD (user must convert to mesh manually or use as pointcloud)."""
    pcd_path = Path(pcd_path).resolve()
    content = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="{map_name}">
    <physics type="ode">
      <max_step_size>0.01</max_step_size>
    </physics>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.3 -1.0</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 
      NOTE: This is a basic placeholder world.
      To use the 3D map as static environment:
      1. Convert the .pcd to a mesh (e.g. using CloudCompare, MeshLab or open3d poisson reconstruction -> STL/DAE)
      2. Place the mesh in meshes/ and update the visual/collision below.
    -->

    <model name="exported_3d_map">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <!-- Placeholder: user should replace with actual mesh -->
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- You can load the PCD directly in RViz or Gazebo via pointcloud plugin for visualization. -->
  </world>
</sdf>
"""
    Path(output_world).parent.mkdir(parents=True, exist_ok=True)
    with open(output_world, "w") as f:
        f.write(content)
    print(f"[export] Created basic world file: {output_world}")
    print("         Convert the .pcd to mesh (CloudCompare/MeshLab) for full static use.")


def generate_mesh_from_pcd(pcd_path, output_mesh, use_poisson=True):
    """Try to generate a mesh from PCD using open3d Poisson reconstruction if available.
    Falls back to just copying pcd if open3d not installed.
    """
    pcd_path = Path(pcd_path)
    output_mesh = Path(output_mesh)
    if not pcd_path.exists():
        print(f"[export] PCD not found: {pcd_path}")
        return False

    try:
        import open3d as o3d
        print("[export] open3d found, attempting Poisson reconstruction for mesh...")
        pcd = o3d.io.read_point_cloud(str(pcd_path))
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(100)

        if use_poisson:
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
            # Remove low density vertices
            vertices_to_remove = densities < np.quantile(densities, 0.1)
            mesh.remove_vertices_by_mask(vertices_to_remove)
        else:
            mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                pcd, o3d.utility.DoubleVector([0.01, 0.02])
            )

        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(output_mesh), mesh)
        print(f"[export] Generated mesh: {output_mesh}")
        return True
    except ImportError:
        print("[export] open3d not installed. Install with: pip install open3d")
        print("[export] Mesh generation skipped. PCD is still available for manual conversion.")
        return False
    except Exception as e:
        print(f"[export] Mesh generation failed: {e}")
        return False


def create_mesh_world(pcd_or_mesh_path, output_world, map_name="exported_3d", mesh_path=None):
    """Create a Gazebo .world that can use a mesh if provided, else placeholder."""
    pcd_path = Path(pcd_or_mesh_path).resolve()
    mesh_ref = ""
    if mesh_path and Path(mesh_path).exists():
        mesh_rel = Path(mesh_path).name
        mesh_ref = f"""
        <visual name="mesh_visual">
          <geometry>
            <mesh>
              <uri>meshes/{mesh_rel}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="mesh_collision">
          <geometry>
            <mesh>
              <uri>meshes/{mesh_rel}</uri>
            </mesh>
          </geometry>
        </collision>"""

    content = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="{map_name}">
    <physics type="ode">
      <max_step_size>0.01</max_step_size>
    </physics>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.3 -1.0</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="exported_3d_map">
      <static>true</static>
      <link name="link">
        {mesh_ref if mesh_ref else '''
        <visual name="visual">
          <geometry>
            <box><size>0.1 0.1 0.1</size></box>
          </geometry>
        </visual>'''}
      </link>
    </model>

    <!-- PCD can be visualized separately in RViz using PointCloud2 plugin on /map_cloud or similar -->
  </world>
</sdf>
"""
    Path(output_world).parent.mkdir(parents=True, exist_ok=True)
    with open(output_world, "w") as f:
        f.write(content)
    print(f"[export] Created world file: {output_world}")
    if mesh_ref:
        print("         World references the generated mesh.")


def main():
    parser = argparse.ArgumentParser(description="Export RTAB-Map 3D data")
    parser.add_argument("--db", help="Path to RTAB-Map .db file")
    parser.add_argument("--cloud-topic", default="/rtabmap/cloud_map", help="Live cloud topic")
    parser.add_argument("--output-dir", default=".", help="Directory to write exports")
    parser.add_argument("--pcd", action="store_true", help="Export PCD")
    parser.add_argument("--octomap", action="store_true", help="Try to export OctoMap")
    parser.add_argument("--world", action="store_true", help="Create .world (with mesh if available)")
    parser.add_argument("--mesh", action="store_true", help="Generate mesh from PCD using open3d if available")
    parser.add_argument("--map-name", default="exported_map", help="Name for world")
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    pcd_file = out_dir / f"{args.map_name}.pcd"
    mesh_file = out_dir / f"{args.map_name}.ply"
    db_file = Path(args.db) if args.db else None

    success = False

    if db_file and db_file.exists():
        print(f"[export] Using database: {db_file}")
        if args.pcd:
            success = export_pcd_from_db(db_file, pcd_file) or success
    else:
        if args.pcd:
            print("[export] No DB provided or not found. Trying live topic...")
            success = export_pcd_live(args.cloud_topic, str(pcd_file)) or success

    if not pcd_file.exists() and args.pcd:
        print("[export] Warning: PCD not generated. You can save /rtabmap/cloud_map manually from RViz or use rtabmap tools.")

    if args.mesh and pcd_file.exists():
        if generate_mesh_from_pcd(pcd_file, mesh_file):
            # update pcd ref for world if needed
            pass

    if args.octomap and pcd_file.exists():
        bt_file = out_dir / f"{args.map_name}.bt"
        try_export_octomap(pcd_file, bt_file)

    if args.world:
        world_file = out_dir / f"{args.map_name}.world"
        used_mesh = str(mesh_file) if mesh_file.exists() else None
        create_mesh_world(pcd_file if pcd_file.exists() else "map.pcd", world_file, args.map_name, used_mesh)

    print("[export] Done.")


if __name__ == "__main__":
    main()
