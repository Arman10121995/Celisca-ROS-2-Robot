"""Named gripper pad normal forces from actual PhysX collider contacts."""
import math
import xml.etree.ElementTree as ET


def pad_colliders(stage, robot, urdf_path, named_pads, load_stl):
    """Match the imported collider to the source pad bounds in the jaw frame.

    The importer may rename collision meshes to mesh_0, etc. The F100 pads
    are distinct STL colliders with no URDF rotation and bounds different
    from the surrounding hulls. Ambiguous or missing matches fail closed.
    """
    from pxr import Usd, UsdGeom, UsdPhysics
    import numpy as np

    source = ET.parse(urdf_path).getroot()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'proxy', 'render', 'guide'])
    links = list(Usd.PrimRange(robot, Usd.TraverseInstanceProxies()))
    result = {}
    for side, name in named_pads.items():
        matches = [(link, collision) for link in source.findall('link') for collision in link.findall('collision')
                   if collision.get('name') == name]
        if len(matches) != 1:
            raise ValueError('Expected one source collider for grip pad '+name)
        link, collision = matches[0]
        mesh = collision.find('geometry/mesh')
        origin = collision.find('origin')
        origin = origin.attrib if origin is not None else {}
        if mesh is None or any(float(v) for v in origin.get('rpy', '0 0 0').split()):
            raise ValueError('Expected a link-aligned STL grip pad: '+name)
        vertices, _ = load_stl(mesh.get('filename').removeprefix('file://'))
        vertices = (np.asarray(vertices, dtype=float) * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                    + np.fromstring(origin.get('xyz', '0 0 0'), sep=' '))
        bounds = [vertices.min(axis=0), vertices.max(axis=0)]
        jaws = [prim for prim in links if prim.GetName() == link.get('name')
                and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(jaws) != 1:
            raise ValueError('Expected one imported rigid jaw for '+name)
        jaw = jaws[0]
        candidates = []
        for prim in Usd.PrimRange(jaw, Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            bound = cache.ComputeRelativeBound(prim, jaw).ComputeAlignedRange()
            if np.allclose([bound.GetMin(), bound.GetMax()], bounds, rtol=0., atol=.0002):
                candidates.append(str(prim.GetPath()))
        if len(candidates) != 1:
            raise ValueError('Could not identify the imported grip pad for '+name)
        result[candidates[0]] = side
    if set(result.values()) != {'left', 'right'}:
        raise ValueError('Expected separate PhysX contacts for left and right pads')
    return result


class GripContacts:
    def __init__(self, stage, pads, objects, dt):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools, PhysxSchema, UsdPhysics

        encode = PhysicsSchemaTools.sdfPathToInt
        self.pads = {encode(path): side for path, side in pads.items()}
        self.jaws = {}
        for path, side in pads.items():
            prim = stage.GetPrimAtPath(path)
            while prim and not prim.IsPseudoRoot():
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.jaws[encode(prim.GetPath())] = side
                    break
                prim = prim.GetParent()
        self.objects = {encode(path): name for name, path in objects.items()}
        self.dt = dt
        self.begin_step()
        for path in objects.values():
            reporter = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path))
            reporter.CreateThresholdAttr().Set(0.)
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contacts)

    def begin_step(self):
        self.forces = {name: {'left': 0., 'right': 0.} for name in self.objects.values()}
        self.other_jaw_forces = {name: {'left': 0., 'right': 0.} for name in self.objects.values()}

    def _on_contacts(self, headers, data):
        for header in headers:
            obj = self.objects.get(header.actor1)
            collider, actor = header.collider0, header.actor0
            if obj is None:
                obj = self.objects.get(header.actor0)
                collider, actor = header.collider1, header.actor1
            if obj is None:
                continue
            side = self.pads.get(collider)
            forces = self.forces
            if side is None:
                side = self.jaws.get(actor)
                forces = self.other_jaw_forces
            if side is None:
                continue
            start = header.contact_data_offset
            for index in range(start, start + header.num_contact_data):
                contact = data[index]
                impulse = abs(sum(a * b for a, b in zip(contact.impulse, contact.normal)))
                if math.isfinite(impulse):
                    forces[obj][side] += impulse / self.dt

    def measure(self, time):
        return {'sim_time_s': time, 'normal_force_n': self.forces,
                'non_pad_jaw_normal_force_n': self.other_jaw_forces}
