"""Locating the Isaac Sim runtime.

Isaac Sim cannot run in the ROS 2 process: rclpy on Humble is Python 3.10
only, while isaacsim needs 3.12.  The backend therefore spawns the runtime
child under Isaac's own interpreter, and everything hinges on finding that
interpreter.

Requiring the user to export ``ISAAC_PYTHON`` by hand meant a perfectly good
local installation went unused and the GUI reported Isaac as unavailable, so
the search below looks in the standard installation locations as well.  An
explicit ``ISAAC_PYTHON`` always wins, so a host can still pin a specific
build.
"""

import glob
import os

#: Directories searched for an Isaac Sim installation, most specific first.
#: ``*`` expands, so several versioned installs can coexist.
SEARCH_GLOBS = (
    "/workspace/*/installs/IsaacSim-*/_build/linux-*/release",
    "~/isaacsim",
    "~/.local/share/ov/pkg/isaac*-*",
    "/isaac-sim",
    "/opt/isaacsim",
    "/opt/nvidia/isaac-sim*",
)

#: Interpreter entry points inside an installation, in order of preference.
INTERPRETER_NAMES = ("python.sh", "python3", "python")


def _candidate_roots(env):
    roots = []
    configured = env.get("ISAAC_SIM_PATH", "")
    if configured:
        roots.append(configured)
    for pattern in SEARCH_GLOBS:
        roots.extend(sorted(glob.glob(os.path.expanduser(pattern)),
                            reverse=True))
    return roots


def find_isaac_python(env=None):
    """Return the Isaac Sim interpreter path, or '' when none is installed.

    Order: ``ISAAC_PYTHON`` (explicit override) -> ``ISAAC_SIM_PATH`` ->
    the standard installation locations.
    """
    env = os.environ if env is None else env

    explicit = env.get("ISAAC_PYTHON", "")
    if explicit:
        # An explicit setting is honoured even when it does not exist, so a
        # typo surfaces as a clear error instead of being silently replaced.
        return explicit

    for root in _candidate_roots(env):
        for name in INTERPRETER_NAMES:
            candidate = os.path.join(root, name)
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
    return ""


def isaac_status(env=None):
    """``(available, detail)`` for the Isaac Sim runtime on this host."""
    env = os.environ if env is None else env
    interpreter = find_isaac_python(env)
    if not interpreter:
        return False, ("Isaac Sim runtime not found - install it, or set "
                       "ISAAC_PYTHON to its python.sh interpreter")
    if not os.path.isfile(interpreter):
        return False, ("ISAAC_PYTHON points at %s, which does not exist"
                       % interpreter)
    if not os.access(interpreter, os.X_OK):
        return False, "%s is not executable" % interpreter
    return True, interpreter


#: Libraries bundled with Isaac Sim that must win over the host's copies.
#: Isaac's torch (cu128) links its cuSPARSE against nvJitLink 12.8, but on a
#: JetPack host another Kit extension loads the system CUDA 12.6
#: libnvJitLink.so.12 first; torch then binds to that older soname, the
#: import fails with "undefined symbol: __nvJitLinkCreate_12_8", and Isaac's
#: deprecation manager shuts the app down during extension startup.  The
#: bundled library exports every symbol the system one does (12_0..12_8), so
#: preloading it is safe for the other CUDA consumers.
BUNDLED_PRELOADS = (
    os.path.join("extsDeprecated", "omni.isaac.ml_archive", "pip_prebundle",
                 "nvidia", "nvjitlink", "lib", "libnvJitLink.so.12"),
)

#: Host libraries the runtime has always preloaded (static TLS on aarch64).
HOST_PRELOADS = ("/lib/aarch64-linux-gnu/libgomp.so.1",)


def isaac_preload_libraries(interpreter, existing=""):
    """LD_PRELOAD value for running the Isaac runtime with *interpreter*.

    Bundled libraries found next to the interpreter come first, then the
    host preloads, then whatever the caller already had, without duplicates.
    """
    root = os.path.dirname(os.path.abspath(interpreter)) if interpreter else ""
    entries = []
    for relative in BUNDLED_PRELOADS:
        candidate = os.path.join(root, relative) if root else ""
        if candidate and os.path.isfile(candidate):
            entries.append(candidate)
    for library in HOST_PRELOADS:
        if os.path.isfile(library):
            entries.append(library)
    for library in (existing or "").replace(":", " ").split():
        entries.append(library)
    ordered = []
    for library in entries:
        if library not in ordered:
            ordered.append(library)
    return " ".join(ordered)
