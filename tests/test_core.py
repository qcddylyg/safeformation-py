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
    result = run("engineering_stabilized", cfg)
    assert metrics(result, cfg)["finite"]


def test_controller_variants_are_explicit():
    cfg = Config()
    assert Controller("full", cfg).variant == "engineering_stabilized"
    assert Controller("ordinary_adp", cfg).variant == "ordinary_adp_no_barrier_actor_critic"
    assert Controller("barrier_adp", cfg).variant == "heuristic_barrier_adp_actor_critic"
    assert Controller("paper_exact", cfg).variant == "heuristic_barrier_adp_actor_critic"


def test_adp_branch_records_online_diagnostics():
    cfg = Config(horizon=0.2, dt=0.02)
    result = run("barrier_adp", cfg)
    summary = metrics(result, cfg)
    assert not summary["rnn_error_available"]
    assert summary["adp_diagnostics_available"]
    assert summary["adp_weight_peak"] > 0.0
    assert result["adp_td"].shape[0] == len(result["t"])


def test_barrier_adp_delay_safety_layer_avoids_obstacle():
    cfg = Config(horizon=3.0, dt=0.01, delay_steps=10)
    result = run("barrier_adp", cfg)
    summary = metrics(result, cfg)
    assert summary["obstacle_violation_samples"] == 0
    assert summary["min_obstacle_distance"] >= cfg.obstacle_radius


def test_delay_changes_the_controlled_state_path_but_remains_finite():
    nominal = Config(horizon=0.4, dt=0.02)
    delayed = Config(horizon=0.4, dt=0.02, delay_steps=5)
    nominal_result = run("engineering_stabilized", nominal)
    delayed_result = run("engineering_stabilized", delayed)
    assert metrics(delayed_result, delayed)["finite"]
    assert not np.allclose(nominal_result["u"], delayed_result["u"])


if __name__ == "__main__":
    test_barrier_zero_near_desired()
    test_nominal_run_is_finite()
    test_controller_variants_are_explicit()
    test_adp_branch_records_online_diagnostics()
    test_delay_changes_the_controlled_state_path_but_remains_finite()
    print("core tests passed")
