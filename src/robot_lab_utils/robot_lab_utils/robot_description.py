"""Expand a description without losing the base directory of relative assets."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from .mesh_assets import resolve_mesh_source


def resolve_relative_assets(urdf, model_path):
    root = ET.fromstring(urdf)
    # Gazebo's extension readers use literal attribute names such as
    # ignition:expressed_in. ElementTree's ns0 rewrite preserves XML meaning
    # but loses that native SDK contract, making friction rotate with a wheel.
    namespaces={name[1:].split('}',1)[0] for node in root.iter() for name in node.attrib if name.startswith('{')}
    for uri in namespaces:
        if uri in ('http://ignitionrobotics.org/schema','https://ignitionrobotics.org/schema'):
            ET.register_namespace('ignition',uri)
        elif uri in ('http://gazebosim.org/schema','https://gazebosim.org/schema'):
            ET.register_namespace('gz',uri)
    for element in list(root.iter('mesh')) + list(root.iter('texture')):
        uri = element.get('filename', '')
        if not uri or '://' in uri:
            continue
        resolved = resolve_mesh_source(uri, {}, str(Path(model_path).parent))
        if not resolved or not Path(resolved).is_file():
            raise ValueError(f"Description asset does not exist: {uri} (model {model_path})")
        element.set('filename', Path(resolved).resolve().as_uri())
    return ET.tostring(root, encoding='unicode')


def load_description(model_path, *xacro_arguments):
    urdf = subprocess.run(['xacro', str(model_path), *xacro_arguments],
                          capture_output=True, text=True, check=True, timeout=60).stdout
    return resolve_relative_assets(urdf, model_path)
