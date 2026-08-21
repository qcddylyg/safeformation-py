# SafeFormation-Py

SafeFormation-Py is a reproducible numerical study of a four-agent,
dual-barrier formation-control problem. It migrates the supplied MATLAB
scenario to a small Python/RK4 toolchain, adds single-variable stress tests,
and records the evidence needed to explain both success and failure.

Repository: `safeformation-py`  
Author: Wendy  
Status: reproducible numerical MVP

It is an **engineering reproduction and extension**, not a claim of a new
control theorem, an exact line-by-line paper reproduction, or a real-robot
deployment.

## What is implemented

- Four 2-D followers, one virtual leader, the supplied directed topology and
  nominal initial conditions.
- Nonlinear follower dynamics, sinusoidal disturbance, input saturation,
  static or moving obstacle, and delayed neighbour/leader state use.
- Low-gain PD, heuristic barrier-PD, adaptive no-barrier ablation, a
  formula-only policy branch, and an engineering-stabilized full controller.
- Deterministic RK4 integration, per-run JSON manifests, and a CSV summary
  containing tracking, safety, connectivity, control-effort and completion
  metrics.

## Controller names and evidence boundary

| CLI controller | Manifest variant | Meaning |
|---|---|---|
| `pd` | `low_gain_pd_baseline` | MATLAB-style low-gain PD baseline. |
| `barrier_pd` | `heuristic_barrier_pd` | Heuristic barrier-PD, not a CBF-QP. |
| `rnn_adp` | `adaptive_residual_surrogate` | Engineering adaptive ablation. |
| `full` | `engineering_stabilized` | PD plus bounded adaptive residual. |
| `paper_exact` | `formula_only_unvalidated` | Formula-shaped branch; not yet equivalence-validated. |

The supplied MATLAB Case 4 explicitly adds a PD term and clips the output, so
it is represented by `full`, not by `paper_exact`. See
[`docs/paper_code_audit.md`](docs/paper_code_audit.md) before comparing results.

## Quick start

Python and NumPy are the only runtime requirements. Run commands from this
directory:

```powershell
python -m pip install -r requirements.txt
python tests\test_core.py

python scripts\run_matrix.py --scenario nominal --steps 5000 --dt 0.01
python scripts\run_matrix.py --scenario dynamic_obstacle --steps 5000 --dt 0.01 --controllers barrier_pd full
python scripts\run_matrix.py --scenario delay --delay-ms 100 --steps 5000 --dt 0.01 --controllers barrier_pd full
python scripts\run_matrix.py --scenario mass --mass-scale 1.2 --steps 5000 --dt 0.01 --controllers barrier_pd full
```

The runner writes one JSON manifest per controller and a scenario CSV summary
to `results/`. Each manifest records configuration, controller variant, seed,
metrics, and the interpretation boundary. Re-running a scenario overwrites its
same-named result files; copy an evidence set to an experiment-specific folder
before changing parameters.

## Reproduce The Checked Results

The checked-in result summaries were produced with the commands below. The
`--run-label` value prevents a later run from overwriting the evidence files.

```powershell
python scripts\run_matrix.py --scenario nominal --steps 5000 --dt 0.01 --controllers barrier_pd full --run-label safety_tuned
python scripts\run_matrix.py --scenario dynamic_obstacle --steps 5000 --dt 0.01 --controllers barrier_pd full --run-label moving
python scripts\run_matrix.py --scenario delay --delay-ms 50 --steps 5000 --dt 0.01 --controllers barrier_pd full --run-label 50ms
python scripts\run_matrix.py --scenario delay --delay-ms 100 --steps 5000 --dt 0.01 --controllers barrier_pd full --run-label 100ms
python scripts\run_matrix.py --scenario mass --mass-scale 1.2 --steps 5000 --dt 0.01 --controllers barrier_pd full --run-label mass120
```

The 100 ms delay run is intentionally retained as a failure case. Do not
interpret `success=false` as a crash: it means the safety radius was violated.

## Experiment protocol

1. Run `nominal` for the same horizon and step size for every controller.
2. Hold controller, seed, initial state, horizon and integration step fixed.
3. Change one variable only: obstacle motion, delay, mass scale, or disturbance
   scale.
4. Compare at least `barrier_pd` and `full`; keep failures rather than reducing
   their horizon.
5. Report `final_formation_rmse`, safety/connectivity violations, input
   peak/RMS, saturation ratio, and `success` together. A lower tracking error
   alone does not establish safety.

`dynamic_obstacle` and `delay` are empirical stress tests. The original
static-obstacle/no-delay proof does not automatically apply to them.

## Repository layout

```text
safeformation.py              simulation model, controllers, RK4 and metrics
scripts/run_matrix.py         reproducible scenario runner
tests/test_core.py            smoke, barrier and delay regression checks
docs/paper_code_audit.md      paper/MATLAB/Python boundary and mapping
results/                      generated JSON manifests and CSV summaries
```

The checked result figure is
[`results/figures/full_nominal_trajectory.svg`](results/figures/full_nominal_trajectory.svg).
The corresponding evidence-bounded numerical summary is
[`results/final_experiment_report.md`](results/final_experiment_report.md).

## Known limitations

- The adaptive module is an engineering surrogate; it does not reproduce the
  MATLAB continuous observer and critic/action weight updates exactly.
- `paper_exact` is deliberately not presented as a validated result until
  matched with a frozen formula-only reference.
- The current layer is a numerical simulation. MuJoCo visualization and any
  physical robot claims remain outside this version.
- A successful finite rollout is empirical evidence only, never a proof that a
  constraint holds for all initial states, delays, or obstacle trajectories.
