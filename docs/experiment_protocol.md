# Experiment Protocol

## Objective

Measure how the numerical controller variants behave under one controlled
change at a time. The main comparison is between traditional baselines and
actor-critic ADP branches with/without heuristic barrier state; the protocol supports descriptive
engineering comparisons and does not make a new stability or safety claim.

## Frozen nominal configuration

- Four followers and one leader; initial state and directed topology in
  `safeformation.py`.
- RK4 integration with `dt=0.01 s`, horizon `50 s`, input bound `20`.
- Static obstacle centered at `[2.5, 0]`, safety radius `1.2 m` and activation
  radius `2.5 m`.
- Communication success uses the physical active-link limit `8.0 m`.
- `barrier_adp` uses `safety_margin=0.10 m`, `max_speed=2.0 m/s`, and local
  obstacle-state sensing; delayed information applies to neighbours and the
  leader. Its conservative control radius is `r + margin + max_speed * delay`.
- Fixed seed `42`; document every different seed in the saved manifest.

## Required comparison runs

Run the same horizon for all listed controllers. Do not compare a shorter
failed run with a completed 50-second run.

| Set | Independent variable | Conditions | Minimum controllers |
|---|---|---|---|
| Nominal | none | static / zero delay / nominal mass | `low_gain_pd`, `heuristic_barrier_pd`, `ordinary_adp`, `barrier_adp` |
| Dynamic obstacle | obstacle trajectory | static, dynamic | `heuristic_barrier_pd`, `barrier_adp` |
| Delay | delayed state age | 0, 50, 100 ms | `heuristic_barrier_pd`, `barrier_adp` |
| Mass mismatch | `mass_scale` | 1.0, 1.2 | `heuristic_barrier_pd`, `barrier_adp` |
| Disturbance | `disturbance_scale` | 1.0, 2.0 | `heuristic_barrier_pd`, `barrier_adp` |

`engineering_stabilized` is an optional appendix controller. It must not be
used as a proxy for the main heuristic barrier-ADP comparison.

For the safety claim in this project, a controller passes a scenario only when
the full retained trajectory has no obstacle or communication violation. A
larger margin is useful, but it does not replace this pass/fail criterion.

## Commands

```powershell
python scripts\run_matrix.py --scenario nominal --steps 5000 --dt 0.01
python scripts\run_matrix.py --scenario dynamic_obstacle --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp
python scripts\run_matrix.py --scenario delay --delay-ms 50 --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp
python scripts\run_matrix.py --scenario delay --delay-ms 100 --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp
python scripts\run_matrix.py --scenario mass --mass-scale 1.2 --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp
python scripts\run_matrix.py --scenario disturbance --disturbance-scale 2.0 --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp
```

## Required result fields

- `final_formation_rmse` and `max_formation_error`
- `min_obstacle_distance`, `obstacle_violation_steps`
- `max_active_link_distance`, `communication_violation_steps`
- `input_peak`, `input_rms`, and `saturation_ratio`
- `adp_diagnostics_available`, `adp_td_rms`, and `adp_weight_peak` for ADP branches
- `finite`, `success`, and `first_failure_time`

## Interpretation rules

1. Call a run failed when it becomes non-finite, violates the physical active
   communication limit, or enters the obstacle safety radius.
2. Keep failed JSON/CSV output. Do not delete it, shorten the horizon, or only
   plot the successful prefix.
3. The moving-obstacle and delay tests sit outside the paper's static,
   no-delay theorem. State them as stress-test observations only.
4. Do not call `heuristic_barrier_pd` a CBF-QP. Do not call `barrier_adp` a
   paper RNN system-identification reproduction. The conservative safety layer
   is a bounded heuristic, not a delayed-system theorem.
5. A lower tracking error alone does not establish that ADP is superior;
   report safety and control effort together and retain failures.
6. Three or fewer repeat runs support descriptive variation, not broad
   statistical or sim-to-real conclusions.
