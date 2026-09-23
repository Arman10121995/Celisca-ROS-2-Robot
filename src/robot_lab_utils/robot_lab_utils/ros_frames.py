"""Static frames the simulator bridges publish for synthesized sensors.

PyBullet, MuJoCo and Isaac cast a 2D scan for every robot; a robot with no
laser link in its description is scanned from 0.12 m above its root link.
Those scans were stamped ``laser_link``, a frame that robot's TF tree does
not have, so AMCL dropped every scan ("earlier than all the data in the
transform cache") and localization never started.  Publishing the frame
the scan is actually taken from fixes that.
"""
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# Must match the fallback scan origin of the bridges.
FALLBACK_SCAN_HEIGHT = 0.12


def publish_fallback_scan_frame(node, parent, child, height=FALLBACK_SCAN_HEIGHT):
    """Latch ``parent`` -> ``child`` at *height* on /tf_static; returns the broadcaster."""
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.z = float(height)
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    return broadcaster
