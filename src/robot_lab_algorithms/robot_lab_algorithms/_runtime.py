"""Shared runtime helpers for the algorithm node entry points.

Every algorithm console script spins its node the same way, and every one of
them is stopped the same way: the launch system (and the GUI's Stop button)
sends SIGINT/SIGTERM.  rclpy turns that into ``ExternalShutdownException``,
which is a normal stop, not a failure - so it must not reach the console as a
traceback, or an ordinary shutdown looks like a crashed algorithm.
"""

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover - optional dependency
    rclpy = None

    class ExternalShutdownException(Exception):
        """Placeholder so the except clauses below stay valid."""

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:  # pragma: no cover - private binding may move
    class RCLError(Exception):
        """Placeholder so the except clauses below stay valid."""


def spin_node(node, timeout_sec=0.1):
    """Spin *node* until shutdown and tear it down without noise.

    Returns 0 for a normal stop (including SIGINT/SIGTERM) so a stopped
    algorithm is never reported as a failed one.
    """
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=timeout_sec)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        # A publish/timer that raced the shutdown of the context. Only a
        # genuine shutdown can invalidate the context here, so this is the
        # same normal stop as the exceptions above.
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


def run(node_factory, name, args=None):
    """Initialise rclpy, build the node and spin it; 1 when rclpy is absent."""
    if rclpy is None:
        print('%s: rclpy unavailable (dry mode)' % name)
        return 1
    rclpy.init(args=args)
    try:
        node = node_factory()
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    return spin_node(node)
