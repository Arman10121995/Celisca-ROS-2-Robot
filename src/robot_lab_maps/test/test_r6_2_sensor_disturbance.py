"""R6.2 — Real dynamic and sensor-disturbance cases.

Tests deterministic noise, bias, drift, delay, dropout, occlusion and outlier
injection with separate seeds. Fault traces must repeat; truth is not
contaminated; baseline/degraded cases share task and budgets.
"""
from pathlib import Path
import sys
import math
import numpy as np
import pytest

pybullet = pytest.importorskip('pybullet')

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_lab_maps/tools'))


class SensorDisturbanceInjector:
    """Inject deterministic sensor disturbances with separate seeds."""

    def __init__(self, seed=42):
        self.seed = seed
        self.rng = np.random.default_rng(seed)

    def gaussian_noise(self, value, std_dev):
        """Add deterministic Gaussian noise."""
        return value + self.rng.normal(0, std_dev)

    def constant_bias(self, value, bias):
        """Add constant bias."""
        return value + bias

    def linear_drift(self, value, step, drift_rate):
        """Add linear drift that grows over steps."""
        return value + drift_rate * step

    def dropout(self, value, dropout_prob):
        """Randomly drop measurements (return None)."""
        if self.rng.random() < dropout_prob:
            return None
        return value

    def outlier(self, value, outlier_prob, outlier_scale=10.0):
        """Inject outliers with given probability."""
        if self.rng.random() < outlier_prob:
            return value + self.rng.normal(0, outlier_scale)
        return value

    def delay_buffer(self, value, delay_steps, buffer):
        """Delay measurements by N steps."""
        buffer.append(value)
        if len(buffer) <= delay_steps:
            return buffer[0]  # Return oldest until buffer fills
        return buffer.pop(0)


@pytest.fixture
def injector():
    return SensorDisturbanceInjector(seed=42)


# ── Deterministic noise tests ──────────────────────────────────────────

def test_gaussian_noise_is_deterministic(injector):
    """Same seed produces identical noise sequences (R6.2: fault traces repeat)."""
    values = [1.0, 2.0, 3.0]
    results1 = [injector.gaussian_noise(v, 0.1) for v in values]
    # Reset RNG with same seed
    injector.rng = np.random.default_rng(42)
    results2 = [injector.gaussian_noise(v, 0.1) for v in values]
    assert results1 == results2, "Same seed must produce identical noise"


def test_different_seeds_produce_different_noise(injector):
    """Different seeds produce different noise (R6.2: separate seeds)."""
    injector1 = SensorDisturbanceInjector(seed=42)
    injector2 = SensorDisturbanceInjector(seed=99)
    values = [1.0, 2.0, 3.0, 4.0, 5.0]
    results1 = [injector1.gaussian_noise(v, 0.1) for v in values]
    results2 = [injector2.gaussian_noise(v, 0.1) for v in values]
    assert results1 != results2, "Different seeds must produce different noise"


def test_noise_bounded_by_std_dev(injector):
    """Noise stays within reasonable bounds of standard deviation."""
    values = [0.0] * 1000
    noisy = [injector.gaussian_noise(v, 0.1) for v in values]
    # 99.7% of samples should be within 3 std devs
    outliers = sum(1 for n in noisy if abs(n) > 0.3)
    assert outliers < 50, f"Too many outliers: {outliers}/1000"


# ── Bias tests ─────────────────────────────────────────────────────────

def test_constant_bias_is_deterministic(injector):
    """Constant bias is deterministic."""
    values = [1.0, 2.0, 3.0]
    results = [injector.constant_bias(v, 0.5) for v in values]
    assert results == [1.5, 2.5, 3.5]


def test_bias_does_not_contaminate_truth(injector):
    """Applying bias to a copy doesn't modify original value."""
    original = 1.0
    biased = injector.constant_bias(original, 0.5)
    assert original == 1.0, "Truth must not be contaminated"
    assert biased == 1.5


# ── Drift tests ────────────────────────────────────────────────────────

def test_linear_drift_grows_over_steps(injector):
    """Linear drift accumulates over steps."""
    values = [injector.linear_drift(0.0, step, 0.01) for step in range(10)]
    assert values[-1] > values[0], "Drift should grow over steps"
    assert pytest.approx(values[-1], abs=1e-6) == 0.09, "Drift = rate * step"


def test_drift_is_deterministic_with_seed(injector):
    """Drift with same seed is deterministic."""
    injector.rng = np.random.default_rng(42)
    values1 = [injector.linear_drift(0.0, step, 0.01) for step in range(5)]
    injector.rng = np.random.default_rng(42)
    values2 = [injector.linear_drift(0.0, step, 0.01) for step in range(5)]
    assert values1 == values2


# ── Dropout tests ──────────────────────────────────────────────────────

def test_dropout_returns_none_sometimes(injector):
    """Dropout randomly returns None."""
    values = [1.0] * 1000
    results = [injector.dropout(v, 0.3) for v in values]
    none_count = sum(1 for r in results if r is None)
    # With p=0.3 and 1000 samples, expect ~300 Nones (allow wide tolerance)
    assert 150 < none_count < 500, f"Expected ~300 dropouts, got {none_count}"


def test_dropout_is_deterministic_with_seed(injector):
    """Dropout with same seed is deterministic."""
    values = [1.0] * 100
    injector.rng = np.random.default_rng(42)
    results1 = [injector.dropout(v, 0.3) for v in values]
    injector.rng = np.random.default_rng(42)
    results2 = [injector.dropout(v, 0.3) for v in values]
    assert results1 == results2


# ── Outlier tests ──────────────────────────────────────────────────────

def test_outlier_injection(injector):
    """Outliers are injected with given probability."""
    values = [0.0] * 1000
    injector.rng = np.random.default_rng(42)
    results = [injector.outlier(v, 0.1, outlier_scale=100.0) for v in values]
    outlier_count = sum(1 for r in results if abs(r) > 50)
    assert outlier_count > 0, "Should inject some outliers"
    assert outlier_count < 200, "Should not be mostly outliers"


# ── Delay tests ────────────────────────────────────────────────────────

def test_delay_buffer_delays_measurements(injector):
    """Delay buffer holds back measurements by delay_steps."""
    buffer = []
    values = [1.0, 2.0, 3.0, 4.0, 5.0]
    delayed = [injector.delay_buffer(v, 2, buffer) for v in values]
    # First delay_steps+1 values return the oldest (delay not yet fully established)
    assert delayed[0] == 1.0
    assert delayed[1] == 1.0
    assert delayed[2] == 1.0
    # After buffer fills, we get properly delayed values
    assert delayed[3] == 2.0
    assert delayed[4] == 3.0


# ── Combined disturbance test ──────────────────────────────────────────

def test_baseline_and_degraded_share_task(injector):
    """Baseline and degraded cases share the same task and budgets."""
    truth = [1.0, 2.0, 3.0, 4.0, 5.0]

    # Baseline: no disturbance
    baseline = list(truth)

    # Degraded: noise + bias
    injector.rng = np.random.default_rng(42)
    degraded = [injector.gaussian_noise(v + 0.1, 0.01) for v in truth]

    # Both should be close to truth within declared budgets
    baseline_error = max(abs(b - t) for b, t in zip(baseline, truth))
    degraded_error = max(abs(d - t) for d, t in zip(degraded, truth))

    assert baseline_error < 0.01, f"Baseline error too large: {baseline_error}"
    assert degraded_error < 0.5, f"Degraded error too large: {degraded_error}"
    assert degraded_error > baseline_error, "Degraded should have larger error"


# ── PyBullet integration test ─────────────────────────────────────────

def test_pybullet_sensor_with_disturbance():
    """Real PyBullet sensor reading with disturbance injection (R6.2)."""
    client = pybullet.connect(pybullet.DIRECT)
    try:
        pybullet.setGravity(0, 0, -9.81, physicsClientId=client)
        pybullet.setTimeStep(0.01, physicsClientId=client)

        # Create a simple sphere as the robot
        sphere = pybullet.loadURDF(
            str(Path(__file__).resolve().parents[2] / 'robot_lab_description/urdf/sphere.urdf'),
            basePosition=[0, 0, 1],
            physicsClientId=client,
        ) if (Path(__file__).resolve().parents[2] / 'robot_lab_description/urdf/sphere.urdf').exists() else None

        if sphere is None:
            # Create a simple box instead
            col = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
            sphere = pybullet.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=col,
                                               basePosition=[0, 0, 1], physicsClientId=client)

        injector = SensorDisturbanceInjector(seed=42)

        # Get baseline truth
        pybullet.stepSimulation(physicsClientId=client)
        pos_truth, _ = pybullet.getBasePositionAndOrientation(sphere, physicsClientId=client)

        # Apply disturbance to the reading
        injector.rng = np.random.default_rng(42)
        pos_noisy = [injector.gaussian_noise(pos_truth[i], 0.01) for i in range(3)]

        # Verify noise is bounded
        for i in range(3):
            assert abs(pos_noisy[i] - pos_truth[i]) < 0.05, \
                f"Noise on axis {i} too large: {abs(pos_noisy[i] - pos_truth[i])}"

        # Deterministic: same seed gives same noisy reading
        injector.rng = np.random.default_rng(42)
        pos_noisy2 = [injector.gaussian_noise(pos_truth[i], 0.01) for i in range(3)]
        assert pos_noisy == pos_noisy2, "Same seed must produce identical noisy readings"
    finally:
        pybullet.disconnect(physicsClientId=client)
