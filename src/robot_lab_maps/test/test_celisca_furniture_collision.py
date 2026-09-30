"""Keep the furnished worlds drivable without dropping furniture collision."""
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET


MAPS = Path(__file__).resolve().parents[1] / "maps"
TRIANGLE = struct.Struct("<12fH")


class CeliscaFurnitureCollisionTests(unittest.TestCase):
    def test_furniture_collision_does_not_duplicate_the_physics_floor(self):
        for floor in (1, 2):
            stem = f"celisca_floor_{floor}_furniture"
            root = ET.parse(MAPS / stem / "worlds" / f"{stem}.world").getroot()
            building = root.find(f".//model[@name='{stem}']")
            floor_model = root.find(".//model[@name='physics_floor']")
            self.assertIsNotNone(floor_model.find("./link/collision/geometry/box"))
            visual = building.findtext("./link/visual/geometry/mesh/uri")
            collision = building.findtext("./link/collision/geometry/mesh/uri")
            self.assertTrue(visual.endswith(f"/{stem}.stl"))
            self.assertTrue(collision.endswith(f"/{stem}_collision.stl"))
            scale = float(building.findtext(
                "./link/collision/geometry/mesh/scale").split()[0])
            mesh = MAPS / stem / "meshes" / f"{stem}_collision.stl"
            data = mesh.read_bytes()
            count = struct.unpack_from("<I", data, 80)[0]
            self.assertTrue(1_000 < count < 40_000)
            self.assertEqual(84 + TRIANGLE.size * count, len(data))
            for values in struct.iter_unpack(TRIANGLE.format, data[84:]):
                z = (values[5], values[8], values[11])
                self.assertGreater(max(z) * scale, 0.08)


if __name__ == "__main__":
    unittest.main()
