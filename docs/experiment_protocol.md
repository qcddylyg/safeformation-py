# Experiment Protocol

## Objective

Measure how the numerical controller variants behave under one controlled
change at a time. The protocol supports descriptive engineering comparisons;
it does not make a new stability or safety claim.

## Frozen nominal configuration

- Four followers and one leader; initial state and directed topology in
  `safeformation.py`.
- RK4 integration with `dt=0.01 s`, horizon `50 s`, input bound `20`.
- Static obstacle centered at `[2.5, 0]`, safety radius `1.2 m` and activation
  radius `2.5 m`.
- Communication success uses the physical active-link limit `8.0 m`.
- Fixed seed `42`; document every different seed in the saved manifest.

## Required comparison runs

Run the same horizon for all listed controllers. Do not compare a shorter
failed run with a completed 50-second run.

| Set | Independent variable | Conditions | Minimum controllers |
|---|---|---|---|
| Nominal | none | static / zero delay / nominal mass | `pd`, `barrier_pd`, `rnn_adp`, `full` |
| Dynamic obstacle | obstacle trajectory | static, dynamic | `barrier_pd`, `full` |
| Delay | delayed state age | 0, 50, 100 ms | `barrier_pd`, `full` |
| Mass mismatch | `mass_scale` | 1.0, 1.2 | `barrier_pd`, `full` |
| Disturbance | `disturbance_scale` | 1.0, 2.0 | `barrier_pd`, `full` |

`paper_exact` has a reserved output label and must not be included in a main
result table until it has been independently formula-validated.

## Commands

```powershell
python scripts\run_matrix.py --scenario nominal --steps 5000 --dt 0.01
python scripts\run_matrix.py --scenario dynamic_obstacle --steps 5000 --dt 0.01 --controllers barrier_pd full
python scripts\run_matrix.py --scenario delay --delay-ms 50 --steps 5000 --dt 0.01 --controllers barrier_pd full
python scripts\run_matrix.py --scenario delay --delay-ms 100 --steps 5000 --dt 0.01 --controllers barrier_pd full
python scripts\run_matrix.py --scenario mass --mass-scale 1.2 --steps 5000 --dt 0.01 --controllers barrier_pd full
python scripts\run_matrix.py --scenario disturbance --disturbance-scale 2.0 --steps 5000 --dt 0.01 --controllers barrier_pd full
```

## Required result fields

- `final_formation_rmse` and `max_formation_error`
- `min_obstacle_distance`, `obstacle_violation_steps`
- `max_active_link_distance`, `communication_violation_steps`
- `input_peak`, `input_rms`, and `saturation_ratio`
- `rnn_error_available` (currently `false`: no paper-equivalent RNN observer
  metric is claimed in this MVP)
- `finite`, `success`, and `first_failure_time`

## Interpretation rules

1. Call a run failed when it becomes non-finite, violates the physical active
   communication limit, or enters the obstacle safety radius.
2. Keep failed JSON/CSV output. Do not delete it, shorten the horizon, or only
   plot the successful prefix.
3. The moving-obstacle and delay tests sit outside the paper's static,
   no-delay theorem. State them as stress-test observations only.
4. Do not call `barrier_pd` a CBF-QP. Do not call `full` the pure paper policy.
5. Three or fewer repeat runs support descriptive variation, not broad
   statistical or sim-to-real conclusions.
