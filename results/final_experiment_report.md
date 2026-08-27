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

For delay experiments, neighbour and leader states are delayed while each
agent's obstacle position is treated as a local measurement. Barrier-ADP uses
`safety_margin=0.10 m`, `max_speed=2.0 m/s`, an inflated control radius, and a
bounded radial safety layer.

## Nominal comparison

| Controller | Final formation RMSE (m) | Min obstacle distance (m) | Input RMS | Obstacle violations | Result |
|---|---:|---:|---:|---:|---|
| Low-gain PD | 0.98043 | 0.1765 | 1.0496 | 255 | failed |
| Heuristic barrier-PD | 0.02084 | 1.2314 | 1.0568 | 0 | success |
| Ordinary ADP (no barrier) | 0.06297 | 0.2364 | 0.9358 | 897 | failed |
| Heuristic barrier-ADP | 0.13313 | 1.5218 | 1.2087 | 0 | success |

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
| Moving obstacle | barrier-ADP | 0.12243 | 1.5710 | 1.1696 | 0 | success |
| 50 ms delay | barrier-PD | 0.02084 | 1.2067 | 1.2505 | 0 | success |
| 50 ms delay | barrier-ADP | 0.23526 | 1.4627 | 1.5617 | 0 | success |
| 100 ms delay | barrier-PD | 0.02084 | 1.0454 | 1.5515 | 121 | failed |
| 100 ms delay | barrier-ADP | 0.41923 | 1.2611 | 2.5209 | 0 | success |
| +20% mass | barrier-PD | 0.02075 | 1.2304 | 1.0566 | 0 | success |
| +20% mass | barrier-ADP | 0.13306 | 1.5218 | 1.2085 | 0 | success |
| 2x disturbance | barrier-PD | 0.03404 | 1.2443 | 1.0688 | 0 | success |
| 2x disturbance | barrier-ADP | 0.13763 | 1.5151 | 1.2210 | 0 | success |

The 100 ms delay case is retained as a stress test. Under the local obstacle
sensing and conservative safety-layer assumptions, barrier-ADP completes the
full 100 ms run without obstacle or communication violations. This is an
empirical result for this configuration, not a delayed-system theorem.

## Interpretation

- Low-gain PD and ordinary ADP show why tracking alone is not a safety metric.
- Barrier-PD versus low-gain PD isolates the benefit of heuristic safety
  feedback for a traditional controller.
- Barrier-ADP versus ordinary ADP isolates the benefit of adding that same
  barrier mechanism to the ADP controller.
- Barrier-ADP versus barrier-PD compares adaptive optimization with fixed-gain
  safety control; it does not prove universal optimality or stability.
- Barrier-ADP trades tracking accuracy and input effort for a larger safety
  margin in these runs.

## Evidence files

- `summary_nominal_safe_barrier_adp.csv`
- `summary_dynamic_obstacle_safe.csv`
- `summary_delay_safe50.csv`
- `summary_delay_safe100.csv`
- `summary_mass_safe.csv`
- `summary_disturbance_safe.csv`
