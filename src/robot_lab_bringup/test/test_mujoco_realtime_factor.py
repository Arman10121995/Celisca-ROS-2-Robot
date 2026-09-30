"""The MuJoCo tick loop holds simulated time to the wall clock.

Physics, the sensor publishes and the viewer sync all share one per-tick
budget.  When they overran it the loop simply started fewer ticks per second,
so simulated time lagged the wall clock and the whole run played back slower
than real time - measured RTF 0.52 at 8 ms of work per tick, which is the
"everything runs very slow in MuJoCo, as if slowed by half" report.

The catch-up arithmetic is read straight out of the spawner source with
``ast`` so these stay plain unit checks: no mujoco import, no ROS graph, no
subprocess, and the code under test is the shipped code rather than a copy.
"""

import ast
import threading
import time
import types
from pathlib import Path

import pytest

SPAWNER = (Path(__file__).resolve().parents[2] / "robot_lab_mujoco" /
           "python" / "robot_lab_mujoco" / "mujoco_spawner.py")
# SPAWNER is <pkg>/python/robot_lab_mujoco/mujoco_spawner.py, so the package
# root - which is where launch/ lives - is three levels up.
LAUNCH = SPAWNER.parents[2] / "launch" / "mujoco_simulator.launch.py"

TICK = 1.0 / 240.0     # the default physics_rate


def _catch_up_method():
    """The real _catch_up, extracted from the spawner without importing it.

    The method is rebuilt inside a throwaway class so its own module globals
    (``time``) resolve, rather than being exec'd at module level where the
    spawner's imports are absent.
    """
    tree = ast.parse(SPAWNER.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == "_catch_up":
                    wrapper = ast.ClassDef(
                        name="_Extracted", bases=[], keywords=[],
                        body=[item], decorator_list=[])
                    module = ast.Module(body=[wrapper], type_ignores=[])
                    ast.fix_missing_locations(module)
                    namespace = {"time": time}
                    exec(compile(module, str(SPAWNER), "exec"), namespace)
                    return namespace["_Extracted"]._catch_up
    raise AssertionError("_catch_up is missing from %s" % SPAWNER)


CATCH_UP = _catch_up_method()


def _spawner(max_catch_up_ticks, dt=TICK):
    """A stand-in holding only what _catch_up touches, plus a step counter."""
    spawner = types.SimpleNamespace()
    spawner._dt = dt
    spawner._max_catch_up_ticks = max_catch_up_ticks
    spawner._time_debt = 0.0
    spawner._physics_lock = threading.RLock()
    spawner._viewer = None
    spawner.steps = 0
    spawner._step_physics = lambda: setattr(spawner, "steps",
                                            spawner.steps + 1)
    spawner._catch_up = types.MethodType(CATCH_UP, spawner)
    return spawner


def _spin(seconds):
    """Burn *seconds* without sleeping, so the test does not depend on the
    scheduler's sleep granularity."""
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        pass
    return start


def test_catch_up_replays_the_ticks_the_wall_clock_paid_for():
    spawner = _spawner(max_catch_up_ticks=8)
    # 8.4 ms of work is two whole ticks, one of which already ran.
    spawner._catch_up(_spin(2.5 * TICK))
    assert spawner.steps == 1, "a whole owed tick was not replayed"

    # A tick that finished inside its budget owes nothing.
    spawner = _spawner(max_catch_up_ticks=8)
    spawner._catch_up(_spin(0.2 * TICK))
    assert spawner.steps == 0


def test_catch_up_is_bounded_so_a_stall_cannot_queue_a_physics_burst():
    """Without a cap, a blocking render would queue physics that takes longer
    to run than the stall did, and the run would fall further behind."""
    spawner = _spawner(max_catch_up_ticks=4)
    spawner._time_debt = 10.0
    spawner._catch_up(time.monotonic())
    assert spawner.steps == 4, "catch-up ran past its cap"

    # 0 disables catch-up and restores the old one-tick-per-iteration behaviour.
    spawner = _spawner(max_catch_up_ticks=0)
    spawner._time_debt = 100.0 * TICK
    spawner._catch_up(time.monotonic())
    assert spawner.steps == 0


def test_catch_up_banks_the_sub_tick_remainder_instead_of_leaking_it():
    """The loop can only ever run whole ticks, so a dropped remainder would
    accumulate into permanent slow drift even with catch-up enabled."""
    # Whatever the call spent, the banked debt stays under one tick: a whole
    # tick's worth is always either replayed or still owed, never discarded.
    for extra in (0.0, 0.25, 0.5, 0.75, 0.99):
        spawner = _spawner(max_catch_up_ticks=8)
        spawner._time_debt = extra * TICK
        spawner._catch_up(time.monotonic())
        assert 0.0 <= spawner._time_debt < TICK, \
            "remainder %r of a tick was not banked" % extra

    # Several short overruns that each owe nothing must together eventually
    # pay for a whole tick, rather than rounding away to nothing forever.
    # Elapsed time enters the way it does in the loop - as the gap since the
    # tick started - so each call accounts for its own share honestly.
    spawner = _spawner(max_catch_up_ticks=8)
    clock = time.monotonic()
    for _ in range(20):
        clock -= 0.6 * TICK          # pretend this tick started 0.6 TICK ago
        spawner._catch_up(clock)
    assert spawner.steps > 0, "banked fractions never added up to a tick"


def test_catch_up_budget_is_a_parameter_and_resets_with_the_clock():
    text = SPAWNER.read_text()
    assert 'declare_parameter("max_catch_up_ticks"' in text
    assert "_max_catch_up_ticks" in text
    # A clock reset restarts simulated time at zero, so banked wall time
    # belongs to a discarded timeline and must not be repaid as physics.
    assert text.count("self._time_debt = 0.0") == 2
    assert 'DeclareLaunchArgument(\n            "max_catch_up_ticks"' in \
        LAUNCH.read_text()
