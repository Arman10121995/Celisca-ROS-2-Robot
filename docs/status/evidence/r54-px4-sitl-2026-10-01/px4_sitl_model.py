#!/usr/bin/env python3
"""Flatten a PX4 Gazebo model SDF and insert the vehicle into a running world.

PX4's own gz_bridge asks the Gazebo server to insert the vehicle, and on this
host that request never produces an entity (``gz model --list`` shows only
``ground_plane``).  ``gz sdf -p`` is not a workaround either: it cannot resolve
``<include><uri>model://NAME</uri></include>`` without a find-callback, so the
"resolved" model it prints has the plugins but **no links** - a vehicle with no
body, no motors and no sensors.

This tool resolves the include graph itself and posts one complete entity to
the world's ``create`` service, under the name PX4 expects (``x500_0``).  With
the vehicle present the FCU's estimator converges (GPS fix type 3, 10 sats) and
it arms; without it PX4 reports "Preflight Fail: height estimate not stable"
and refuses to arm (measured 2026-10-01).

    python3 scripts/px4_sitl_model.py --models-dir ~/.px4/Tools/simulation/gz/models \\
        --model x500 --world default --post

Print the flattened SDF with ``--print``; nothing is sent without ``--post``.
"""
import argparse
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

# Tags that live inside <model> and merge straight into the parent when an
# <include> is resolved (URDF/SDF include semantics with merge='true').
_MERGED = ("link", "joint", "visual", "collision", "sensor", "plugin", "model",
           "light", "physics", "static", "pose")


def _model_file(models_dir, name):
    return os.path.join(models_dir, name, "model.sdf")


def flatten_model(models_dir, name, seen=None):
    """Return the <model> element for *name* with every include resolved."""
    seen = seen or set()
    path = _model_file(models_dir, name)
    if not os.path.isfile(path):
        raise FileNotFoundError("no model.sdf for %r (%s)" % (name, path))
    if name in seen:
        raise ValueError("include cycle at %r" % name)
    seen = seen | {name}
    root = ET.parse(path).getroot()
    model = root.find("model")
    if model is None:
        raise ValueError("%s has no <model>" % path)
    # Children collected in order, includes expanded in place.
    children = []
    for child in list(model):
        if child.tag == "include":
            uri = (child.findtext("uri") or "").strip()
            if not uri.startswith("model://"):
                raise ValueError("unsupported include uri %r in %s" % (uri, path))
            merged = flatten_model(models_dir, uri[len("model://"):], seen)
            for merged_child in list(merged):
                if merged_child.tag in _MERGED:
                    children.append(merged_child)
                elif merged_child.tag == "name":
                    continue
                else:
                    children.append(merged_child)
            continue
        children.append(child)
    model[:] = children
    model.set("name", model.get("name") or name)
    return model


def entity_sdf(models_dir, name, entity_name=None, motor_topic=None):
    """A single <sdf> document holding the flattened model.

    *motor_topic* rewrites every rotor plugin's ``commandSubTopic``.  PX4's
    gz_bridge publishes rotor speeds on ``/<model>/command/motor_speed`` while
    an entity inserted through the create service subscribes on the model-
    relative ``/model/<model>/command/motor_speed``; the two never meet, the
    vehicle is armed but never lifts off.  Pointing the plugins at the FCU's
    topic removes the mismatch (and any republishing process).
    """
    model = flatten_model(models_dir, name)
    if entity_name:
        model.set("name", entity_name)
    if motor_topic:
        for plugin in model.iter("plugin"):
            sub = plugin.find("commandSubTopic")
            if sub is not None:
                sub.text = motor_topic
    root = ET.Element("sdf", {"version": "1.9"})
    root.append(model)
    return ET.tostring(root, encoding="unicode")


def post_entity(world, sdf_text, timeout=30):
    """Send the entity to <world>/create; returns the CLI output."""
    command = [
        "gz", "service", "-s", "/world/%s/create" % world,
        "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
        "--timeout", str(timeout * 1000), "--req", sdf_text,
    ]
    result = subprocess.run(command, capture_output=True, text=True,
                            timeout=timeout + 20)
    return result.returncode, (result.stdout + result.stderr).strip()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--models-dir", required=True,
                        help="PX4 Tools/simulation/gz/models directory")
    parser.add_argument("--model", default="x500",
                        help="model directory name (e.g. x500, x500_base)")
    parser.add_argument("--entity-name", default=None,
                        help="entity name to create; defaults to the model's own")
    parser.add_argument("--world", default="default")
    parser.add_argument("--motor-topic", default=None,
                        help="absolute rotor command topic the FCU publishes "
                             "on, e.g. /x500_0/command/motor_speed")
    parser.add_argument("--print", dest="print_only", action="store_true",
                        help="print the flattened SDF and exit")
    parser.add_argument("--post", action="store_true",
                        help="insert the model into the running world")
    args = parser.parse_args(argv)

    sdf_text = entity_sdf(args.models_dir, args.model, args.entity_name,
                         args.motor_topic)
    links = sdf_text.count("<link")
    if links == 0:
        print("refusing to send a model with no links", file=sys.stderr)
        return 2
    if args.print_only or not args.post:
        print(sdf_text)
        if not args.post:
            print("(%d link(s); pass --post to insert it)" % links,
                  file=sys.stderr)
        return 0
    code, output = post_entity(args.world, sdf_text)
    print("create service rc=%d %s" % (code, output[-400:]))
    return code


if __name__ == "__main__":
    sys.exit(main())
