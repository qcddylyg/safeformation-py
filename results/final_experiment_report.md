# SafeFormation-Py Numerical Experiment Report

## Scope

These are deterministic Python/RK4 simulations of the supplied four-agent
scenario. The main comparison is a 2x2 design: PD versus ADP, with and without
the heuristic barrier mechanism. The former `full` controller is retained as
`engineering_stabilized` for an appendix only.

All runs use `dt=0.01 s`, horizon `50 s`, seed `42`, input bound `20`, and the
same initial state. The ADP branches expose online TD and actor-weight
diagnostics, but remain numerical actor-critic surrogates rather than a
line-by-line reproduction of the paper's RNN system-identification method.

## Nominal comparison

| Controller | Final formation RMSE (m) | Min obstacle distance (m) | Input RMS | Obstacle violations | Result |
|---|---:|---:|---:|---:|---|
| Low-gain PD | 0.98043 | 0.1765 | 1.0496 | 255 | failed |
| Heuristic barrier-PD | 0.02084 | 1.2314 | 1.0568 | 0 | success |
| Ordinary ADP (no barrier) | 0.06297 | 0.2364 | 0.9358 | 897 | failed |
| Heuristic barrier-ADP | 0.10222 | 1.4628 | 1.1919 | 0 | success |

The comparison isolates two contributions. Adding the heuristic barrier changes
ordinary ADP from an unsafe rollout to a successful nominal rollout with a
larger obstacle margin. ADP alone reduces control RMS relative to low-gain PD,
but does not guarantee obstacle safety. In this surrogate implementation,
barrier-PD has lower terminal tracking error than barrier-ADP, so no blanket
claim of ADP superiority is made.

## Stress tests

| Condition | Controller | Final RMSE | Min obstacle distance | Input RMS | Obstacle violations | Result |
|---|---|---:|---:|---:|---:|---|
| Moving obstacle | barrier-PD | 0.02084 | 1.2630 | 1.0274 | 0 | success |
| Moving obstacle | barrier-ADP | 0.09554 | 1.5028 | 1.1547 | 0 | success |
| 50 ms delay | barrier-PD | 0.02084 | 1.2067 | 1.2505 | 0 | success |
| 50 ms delay | barrier-ADP | 0.15392 | 1.3362 | 1.5494 | 0 | success |
| 100 ms delay | barrier-PD | 0.02084 | 1.0454 | 1.5515 | 121 | failed |
| 100 ms delay | barrier-ADP | 0.15046 | 0.7361 | 2.4766 | 300 | failed |
| +20% mass | barrier-PD | 0.02075 | 1.2304 | 1.0566 | 0 | success |
| +20% mass | barrier-ADP | 0.10221 | 1.4629 | 1.1918 | 0 | success |
| 2x disturbance | barrier-PD | 0.03404 | 1.2443 | 1.0688 | 0 | success |
| 2x disturbance | barrier-ADP | 0.10581 | 1.4562 | 1.2048 | 0 | success |

The 100 ms delay case is retained as a failure boundary. Moving-obstacle and
delay runs are empirical stress tests outside the static, no-delay theorem.

## Interpretation

- Low-gain PD and ordinary ADP show why tracking alone is not a safety metric.
- Barrier-PD versus low-gain PD isolates the benefit of heuristic safety
  feedback for a traditional controller.
- Barrier-ADP versus ordinary ADP isolates the benefit of adding that same
  barrier mechanism to the ADP controller.
- Barrier-ADP versus barrier-PD compares adaptive optimization with fixed-gain
  safety control; it does not prove universal optimality or stability.

## Evidence files

- `summary_nominal_four_way_final.csv`
- `summary_dynamic_obstacle_final.csv`
- `summary_delay_final_50ms.csv`
- `summary_delay_final_100ms.csv`
- `summary_mass_final.csv`
- `summary_disturbance_final.csv`
