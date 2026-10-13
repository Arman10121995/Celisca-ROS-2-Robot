"""The Isaac runtime only runs inside Isaac Sim (pxr, omni), so its drive
authoring functions are not exercised by the unit tests.  A static check
catches names that would only fail at runtime: a damping rename left two
other drive functions referring to an undefined name, and every Isaac
wheeled robot failed to spawn (2026-10-13)."""
import os

import pytest

pyflakes_api = pytest.importorskip('pyflakes.api')
from pyflakes import messages, reporter  # noqa: E402

RUNTIME = os.path.join(os.path.dirname(__file__), '..', '..', 'robot_lab_isaac', 'python',
                       'robot_lab_isaac', 'isaac_runtime.py')


class _Collect(reporter.Reporter):
    def __init__(self):
        super().__init__(None, None)
        self.found = []

    def flake(self, message):
        self.found.append(message)

    def syntaxError(self, filename, msg, lineno, offset, text):
        self.found.append('%s:%s: %s' % (filename, lineno, msg))

    def unexpectedError(self, filename, msg):
        self.found.append('%s: %s' % (filename, msg))


def test_isaac_runtime_has_no_undefined_names():
    collected = _Collect()
    pyflakes_api.checkPath(RUNTIME, reporter=collected)
    undefined = [str(m) for m in collected.found
                 if isinstance(m, str) or isinstance(m, messages.UndefinedName)]
    assert undefined == []
