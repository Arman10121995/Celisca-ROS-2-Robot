"""Stop the Gazebo processes a launch leaves behind when it exits.

``ros2 launch`` stops ``ign gazebo`` by signalling the Ruby wrapper it
started.  When the server is slow to shut down (gz_ros2_control often is),
launch escalates to SIGTERM on the wrapper and exits, and the real server
process lives on as an orphan: it keeps simulating the old world, keeps its
controller_manager on the ROS graph, and a later run in the same partition
or domain sees that stale world and those stale controllers.

Every bringup run gets its own Gazebo transport partition, and every process
it starts inherits it, so the leftovers are exactly the processes whose
environment carries that partition.  This watcher is started detached from
the launch, waits for the launch process to exit, gives its children a grace
period, then terminates whatever still carries the partition.

Usage: python3 -m robot_lab_utils.partition_reaper <launch pid> <partition>
"""
import os
import signal
import sys
import time


def processes_in_partition(partition, exclude=()):
    """PIDs whose environment has IGN_PARTITION or GZ_PARTITION = *partition*."""
    wanted = {b"IGN_PARTITION=" + partition.encode(), b"GZ_PARTITION=" + partition.encode()}
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) in exclude:
            continue
        try:
            with open("/proc/%s/environ" % entry, "rb") as handle:
                variables = set(handle.read().split(b"\0"))
        except OSError:
            continue
        if variables & wanted:
            found.append(int(entry))
    return found


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def reap(partition, grace=5.0, exclude=()):
    """SIGTERM, then SIGKILL, every process left in *partition*; returns them."""
    exclude = set(exclude) | {os.getpid()}
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline and processes_in_partition(partition, exclude):
        time.sleep(0.5)
    leftovers = processes_in_partition(partition, exclude)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in leftovers:
            try:
                os.kill(pid, sig)
            except OSError:
                pass
        stop = time.monotonic() + 3.0
        while time.monotonic() < stop and any(_alive(pid) for pid in leftovers):
            time.sleep(0.2)
        leftovers = [pid for pid in leftovers if _alive(pid)]
        if not leftovers:
            break
    return leftovers


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    launch_pid, partition = int(argv[0]), argv[1]
    if not partition:
        return 2
    while _alive(launch_pid):
        time.sleep(1.0)
    reap(partition)
    return 0


if __name__ == "__main__":
    sys.exit(main())
