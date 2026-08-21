import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from safeformation import Config, Controller, communication_barrier, obstacle_barrier, metrics, run
import numpy as np


def test_barrier_zero_near_desired():
    s, _ = communication_barrier(np.array([1.0, 1.0]), np.zeros(2), np.array([1.0, 1.0]), np.zeros(2), 5.0)
    assert np.linalg.norm(s) < 1e-3


def test_nominal_run_is_finite():
    cfg = Config(horizon=0.2, dt=0.02)
    result = run("full", cfg)
    assert metrics(result, cfg)["finite"]


def test_controller_variants_are_explicit():
    cfg = Config()
    assert Controller("full", cfg).variant == "engineering_stabilized"
    assert Controller("paper_exact", cfg).variant == "formula_only_unvalidated"


def test_delay_changes_the_controlled_state_path_but_remains_finite():
    nominal = Config(horizon=0.4, dt=0.02)
    delayed = Config(horizon=0.4, dt=0.02, delay_steps=5)
    nominal_result = run("full", nominal)
    delayed_result = run("full", delayed)
    assert metrics(delayed_result, delayed)["finite"]
    assert not np.allclose(nominal_result["u"], delayed_result["u"])


if __name__ == "__main__":
    test_barrier_zero_near_desired()
    test_nominal_run_is_finite()
    test_controller_variants_are_explicit()
    test_delay_changes_the_controlled_state_path_but_remains_finite()
    print("core tests passed")
