"""SDF heightfields must convert to the terrain Gazebo actually builds.

The layout asserted here was measured against the Gazebo on this host
(gz-sim 8 / Harmonic, ODE) by dropping 0.2 m spheres onto asymmetric rasters
and reading the settled poses back from ``/world/<name>/dynamic_pose/info``;
see ``docs/status/evidence/r6-7-heightfield-probe-2026-10-05/``.  Three of
these conventions are easy to get subtly wrong and none of them are stated in
the SDF spec:

* elevation is normalised by the raster's own maximum, not by 255;
* raster row 0 is maximum +y;
* the grid spans ``(n-1)/2 * size/n``, not ``size/2``.

Every non-Gazebo backend used to drop heightmaps entirely, so this also pins
that the MJCF generator emits a real hfield instead of skipping it.
"""
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, os.path.join(_SRC, "robot_lab_utils"))
sys.path.insert(0, os.path.join(_SRC, "robot_lab_maps", "tools"))

from robot_lab_utils import heightfield, sdf_world  # noqa: E402
import gen_mjcf_worlds  # noqa: E402


def _raster(root, array, name="terrain.png"):
    path = root / name
    Image.fromarray(np.asarray(array, dtype=np.uint8), "L").save(path)
    return str(path)


def _shape(path, size=(20.0, 20.0, 4.0), **kwargs):
    shape = {"type": "heightmap", "heightmap": path, "size": list(size),
             "position": [0.0, 0.0, 0.0], "orientation": [1.0, 0.0, 0.0, 0.0]}
    shape.update(kwargs)
    return shape


class HeightfieldConversionTests(unittest.TestCase):

    def test_elevation_is_normalised_by_the_raster_maximum(self):
        """A uniform 100 raster fills size_z; /255 would only reach 1.57 m."""
        with tempfile.TemporaryDirectory() as directory:
            path = _raster(Path(directory), np.full((33, 33), 100))
            vertices, _ = heightfield.heightfield_grid(_shape(path))
            self.assertAlmostEqual(4.0, float(vertices[:, 2].min()), places=6)
            self.assertAlmostEqual(4.0, float(vertices[:, 2].max()), places=6)

    def test_partial_levels_scale_by_the_raster_maximum(self):
        """Measured: bands of 50/150 gave 1.333 m and 4.0 m for size_z = 4."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            array = np.full((33, 33), 50)
            array[:16] = 150
            vertices, _ = heightfield.heightfield_grid(_shape(_raster(root, array)))
            self.assertAlmostEqual(50.0 / 150.0 * 4.0,
                                   float(vertices[:, 2].min()), places=5)
            self.assertAlmostEqual(4.0, float(vertices[:, 2].max()), places=6)

    def test_raster_row_zero_is_maximum_positive_y(self):
        """Measured: top half 0 / bottom half 255 put y=+8 at 0 m, y=-8 at 4 m."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            array = np.zeros((33, 33))
            array[16:] = 255
            vertices, _ = heightfield.heightfield_grid(_shape(_raster(root, array)))
            high_y = vertices[np.abs(vertices[:, 1] - 8.0).argmin()]
            low_y = vertices[np.abs(vertices[:, 1] + 8.0).argmin()]
            self.assertAlmostEqual(0.0, float(high_y[2]), places=6)
            self.assertAlmostEqual(4.0, float(low_y[2]), places=6)

    def test_grid_span_matches_the_measured_heightfield_aabb(self):
        """Gazebo AABB measured +-9.69697 / +-9.84615 / +-9.41177 for 33/65/17."""
        for samples, expected in ((33, 9.69697), (65, 9.84615), (17, 9.41177)):
            with self.subTest(samples=samples):
                with tempfile.TemporaryDirectory() as directory:
                    path = _raster(Path(directory), np.full((samples, samples), 255))
                    vertices, _ = heightfield.heightfield_grid(_shape(path))
                    self.assertAlmostEqual(expected,
                                           float(np.abs(vertices[:, 0]).max()),
                                           places=4)

    def test_the_collision_pose_places_the_terrain(self):
        """Local vertices are pose-free; callers apply the shape's pose.

        Gazebo places the terrain with the collision pose, so the generator
        and the spawners attach the same pos/quat to the hfield/mesh.  The
        converter itself stays in shape-local metres, which is what keeps one
        grid usable by every backend.
        """
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = _raster(root, np.full((33, 33), 255))
            shape = _shape(path, position=[1.0, 2.0, 0.5])
            vertices, _ = heightfield.heightfield_grid(shape)
            # Local elevation is measured from the pose origin and normalised
            # so the raster maximum reaches size_z; a uniform raster is a flat
            # plateau at 4.0 m.  The 0.5 m pose offset is applied by the backend.
            self.assertAlmostEqual(4.0, float(vertices[:, 2].min()), places=6)
            self.assertAlmostEqual(4.0, float(vertices[:, 2].max()), places=6)
            centre_x = 0.5 * (vertices[:, 0].min() + vertices[:, 0].max())
            centre_y = 0.5 * (vertices[:, 1].min() + vertices[:, 1].max())
            self.assertAlmostEqual(0.0, float(centre_x), places=6)
            self.assertAlmostEqual(0.0, float(centre_y), places=6)
            # The pose is carried on the record for the backend to apply.
            self.assertEqual([1.0, 2.0, 0.5], shape["position"])

    def test_support_height_reports_the_surface_under_a_point(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            array = np.zeros((33, 33))
            array[:16] = 255
            path = _raster(root, array)
            shape = _shape(path, position=[1.0, 2.0, 0.5])
            self.assertAlmostEqual(4.5, heightfield.support_height(shape, 1.0, 6.0),
                                   places=3)
            self.assertAlmostEqual(0.5, heightfield.support_height(shape, 1.0, -2.0),
                                   places=3)

    def test_faces_form_a_connected_surface(self):
        with tempfile.TemporaryDirectory() as directory:
            path = _raster(Path(directory), np.full((33, 33), 255))
            vertices, faces = heightfield.heightfield_grid(_shape(path))
            self.assertEqual(33 * 33, len(vertices))
            self.assertEqual(2 * 32 * 32, len(faces))
            self.assertEqual(faces.max(), len(vertices) - 1)

    def test_decimation_reduces_resolution_and_keeps_the_peak(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            array = np.zeros((64, 64))
            array[:32] = 255
            shape = _shape(_raster(root, array))
            full, _ = heightfield.heightfield_grid(shape)
            coarse, coarse_faces = heightfield.heightfield_grid(shape, samples=2)
            self.assertEqual(32 * 32, len(coarse))
            self.assertEqual(2 * 31 * 31, len(coarse_faces))
            self.assertGreater(len(full), len(coarse))
            self.assertAlmostEqual(float(full[:, 2].max()),
                                   float(coarse[:, 2].max()), places=6)

    def test_pgm_is_read_without_pillow(self):
        """Binary PGM parsing keeps the conversion usable on a bare host."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "terrain.pgm"
            path.write_bytes(b"P5\n2 3\n255\n" + bytes([0, 0, 100, 100, 200, 250]))
            raster = heightfield.load_heightmap(str(path))
            self.assertEqual((3, 2), raster.shape)
            np.testing.assert_allclose(
                [[0.0, 0.0], [100.0, 100.0], [200.0, 250.0]], raster)

    def test_all_zero_raster_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = _raster(Path(directory), np.zeros((8, 8)))
            with self.assertRaises(ValueError):
                heightfield.heightfield_grid(_shape(path))

    def test_obj_output_is_a_triangle_soup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            vertices, faces = heightfield.heightfield_grid(
                _shape(_raster(root, np.full((9, 9), 255))))
            target = root / "terrain.obj"
            heightfield.write_obj(vertices, faces, str(target))
            lines = target.read_text().splitlines()
            self.assertEqual(81, sum(1 for line in lines if line.startswith("v ")))
            self.assertEqual(len(faces),
                             sum(1 for line in lines if line.startswith("f ")))


class SdfReaderHeightmapTests(unittest.TestCase):

    WORLD = ('<sdf version="1.7"><world name="t"><model name="terrain">'
             '<static>true</static><pose>1 2 0.5 0 0 0</pose>'
             '<link name="link"><collision name="c"><geometry><heightmap>'
             '<uri>%s</uri><size>%s</size></heightmap></geometry></collision>'
             '</link></model></world></sdf>')

    def _shapes(self, root, uri="terrain.png", size="20 20 4"):
        world = root / "hm.world"
        world.write_text(self.WORLD % (uri, size))
        # The real resolver returns '' for a URI that does not exist on disk,
        # which is what the unresolved-heightmap report depends on.
        resolve = sdf_world.uri_resolver()
        return sdf_world.extract_static_shapes(str(world), resolve)

    def test_heightmap_is_returned_as_a_shape_not_skipped(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _raster(root, np.full((17, 17), 255))
            shapes, skipped = self._shapes(root)
            self.assertEqual(1, len(shapes))
            self.assertEqual("heightmap", shapes[0]["type"])
            self.assertEqual([20.0, 20.0, 4.0], shapes[0]["size"])
            self.assertEqual([1.0, 2.0, 0.5], shapes[0]["position"])
            self.assertEqual([], [n for n in skipped if "unsupported" in n])

    def test_unresolved_heightmap_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shapes, skipped = self._shapes(root, uri="missing.png")
            self.assertEqual([], shapes)
            self.assertTrue(any("unresolved heightmap" in n for n in skipped))

    def test_non_positive_size_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _raster(root, np.full((17, 17), 255))
            shapes, skipped = self._shapes(root, size="20 20 0")
            self.assertEqual([], shapes)
            self.assertTrue(any("non-positive" in n for n in skipped))


class MjcfHeightmapTests(unittest.TestCase):

    WORLD = ('<sdf version="1.7"><world name="hm"><model name="terrain">'
             '<static>true</static>'
             '<link name="link"><collision name="c"><geometry><heightmap>'
             '<uri>terrain.png</uri><size>20 20 4</size></heightmap>'
             '</geometry></collision></link></model></world></sdf>')

    def test_heightmap_becomes_a_real_hfield_geom(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _raster(root, np.full((33, 33), 255))
            world = root / "hm.world"
            world.write_text(self.WORLD)
            mjcf, skipped = gen_mjcf_worlds.convert_world(
                str(world), "hm", asset_dir=str(root))
            model = ET.fromstring(mjcf)
            hfields = model.findall("./asset/hfield")
            self.assertEqual(1, len(hfields), skipped)
            self.assertTrue(os.path.isfile(hfields[0].get("file")))
            geoms = [geom for geom in model.find("worldbody").findall("geom")
                     if geom.get("type") == "hfield"]
            self.assertEqual(1, len(geoms))
            # MuJoCo hfield size is (radius_x, radius_y, elev_x, elev_y).
            self.assertEqual(["10", "10", "4", "4"],
                             hfields[0].get("size").split())

    def test_generated_hfield_png_is_not_downscaled(self):
        """MuJoCo normalises by the raster max, so values must survive intact."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            array = np.full((33, 33), 100)
            array[:16] = 150
            emitted = gen_mjcf_worlds._write_hfield_png(
                _shape(_raster(root, array)), str(root))
            np.testing.assert_array_equal(
                array.astype(np.uint8),
                np.asarray(Image.open(emitted).convert("L")))


if __name__ == "__main__":
    unittest.main()
