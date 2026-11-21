"""Measure free-object contact on two named URDF pads in PyBullet."""

import xml.etree.ElementTree as ET
from urllib.parse import unquote

import numpy as np


# Bullet identifies a contact's link, but not the compound collision child.
# Allow for the convex contact margin when comparing the point on the robot
# with the source pad's bounds.
PAD_POINT_MARGIN_M = 0.0015


class GripContacts:
    def __init__(self, bullet, body, links, urdf, named_pads):
        from scipy.spatial.transform import Rotation

        if set(named_pads) != {'left', 'right'}:
            raise ValueError('Grip contact sensing requires named left and right pads')
        self.bullet = bullet
        self.body = body
        self.pads = {}
        root = ET.fromstring(urdf)
        for side, name in named_pads.items():
            matches = [(link, collider) for link in root.findall('link') for collider in link.findall('collision')
                       if collider.get('name') == name]
            if len(matches) != 1:
                raise ValueError(f'Expected one named collision for {side} grip pad {name}')
            link, collider = matches[0]
            link_index = links[link.get('name')]
            if link_index < 0 or link_index in self.pads:
                raise ValueError('Grip pads must be on distinct articulated links')
            geometry = collider.find('geometry')
            box, mesh = geometry.find('box'), geometry.find('mesh')
            if box is not None:
                bounds = np.asarray(box.get('size').split(), dtype=float)
                to_bounds = np.eye(4)
            elif mesh is not None:
                import trimesh

                filename = mesh.get('filename')
                if filename.startswith('file://'):
                    filename = unquote(filename.removeprefix('file://'))
                shape = trimesh.load(filename, force='mesh')
                shape.apply_scale(np.asarray(mesh.get('scale', '1 1 1').split(), dtype=float))
                to_bounds, bounds = trimesh.bounds.oriented_bounds(shape)
            else:
                raise ValueError(f'Grip pad {name} requires a box or mesh collision')
            origin = collider.find('origin')
            origin_fields = origin.attrib if origin is not None else {}
            link_from_geom = np.eye(4)
            link_from_geom[:3, :3] = Rotation.from_euler(
                'xyz', np.asarray(origin_fields.get('rpy', '0 0 0').split(), dtype=float)).as_matrix()
            link_from_geom[:3, 3] = np.asarray(origin_fields.get('xyz', '0 0 0').split(), dtype=float)
            self.pads[link_index] = (side, to_bounds @ np.linalg.inv(link_from_geom), bounds * 0.5)

    def measure(self, bodies, sim_time):
        forces = {name: dict.fromkeys((pad[0] for pad in self.pads.values()), 0.) for name in bodies}
        objects = {uid: name for name, uid in bodies.items()}
        if objects:
            local_frames = {}
            for contact in self.bullet.getContactPoints(bodyA=self.body):
                obj, link = objects.get(contact[2]), contact[3]
                if obj is None or link not in self.pads or contact[9] <= 0.:
                    continue
                if link not in local_frames:
                    state = self.bullet.getLinkState(self.body, link, computeForwardKinematics=True)
                    local_frames[link] = self.bullet.invertTransform(state[4], state[5])
                point, _ = self.bullet.multiplyTransforms(*local_frames[link], contact[5], (0., 0., 0., 1.))
                side, to_bounds, half = self.pads[link]
                local = to_bounds[:3, :3] @ point + to_bounds[:3, 3]
                if np.all(np.abs(local) <= half + PAD_POINT_MARGIN_M):
                    forces[obj][side] += float(contact[9])
        return {'sim_time_s': float(sim_time), 'normal_force_n': forces}
