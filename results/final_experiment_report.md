# SafeFormation-Py Numerical Experiment Report

## Scope

These are deterministic Python/RK4 engineering simulations of the supplied
four-agent MATLAB scenario. `barrier_pd` is a heuristic barrier-PD controller;
`full` is a PD-stabilized engineering variant with a bounded adaptive residual.
Neither result validates a CBF-QP, the paper's pure equation-(99) policy, a
dynamic-obstacle theorem, or a physical robot deployment.

All runs use `dt=0.01 s`, horizon `50 s`, seed `42`, saturation bound `20`,
and the same initial state. Values are one deterministic rollout per condition.
The source CSV files are linked below.

## Main results

| Condition | Controller | Final formation RMSE (m) | Min obstacle distance (m) | Max link distance (m) | Input RMS | Safety result |
|---|---|---:|---:|---:|---:|---|
| Nominal | barrier-PD | 0.02084 | 1.2314 | 4.1232 | 1.0568 | success |
| Nominal | full engineering variant | 0.02075 | 1.2305 | 4.1232 | 1.0374 | success |
| Moving obstacle | barrier-PD | 0.02084 | 1.2630 | 4.1234 | 1.0274 | success |
| Moving obstacle | full engineering variant | 0.02075 | 1.2764 | 4.1234 | 1.0126 | success |
| 50 ms delay | barrier-PD | 0.02084 | 1.2067 | 4.1250 | 1.2505 | success |
| 50 ms delay | full engineering variant | 0.02075 | 1.2052 | 4.1254 | 1.2206 | success |
| +20% mass | barrier-PD | 0.02075 | 1.2304 | 4.1232 | 1.0566 | success |
| +20% mass | full engineering variant | 0.02066 | 1.2304 | 4.1232 | 1.0371 | success |

The physical communication limit is `8.0 m` and the obstacle safety radius is
`1.2 m`. Every successful row completed the full horizon with zero logged
communication-violation samples and zero obstacle-violation samples.

## Failure case: 100 ms delayed information

At 100 ms delay, both controllers remained finite and retained low terminal
formation error, but entered the obstacle safety radius. Barrier-PD had a
minimum distance of `1.0454 m` and 121 obstacle-violation samples; the full
engineering variant had `1.0904 m` and 84 samples. The appropriate conclusion
is not that the system is safe at 100 ms, but that this particular static
barrier treatment is insufficient under that delay. This is retained as a
reproducible failure case rather than excluded from the experiment.

## Interpretation

- Under the tested nominal, moving-obstacle, 50 ms-delay and +20% mass
  conditions, both engineering controllers completed safely.
- The full engineering variant has slightly lower terminal error and RMS input
  in these deterministic runs, but the differences are too small and the
  sample count too limited for a broad performance claim.
- The 100 ms delay failure establishes an observed engineering boundary for
  this configuration; it does not identify a universal delay threshold.
- `rnn_error_available=false` is intentional. The current adaptive residual is
  not yet a paper-equivalent RNN observer, so reporting a fabricated RNN error
  would be misleading.

## Evidence files

- `summary_nominal_safety_tuned.csv`
- `summary_dynamic_obstacle_moving.csv`
- `summary_delay_50ms.csv`
- `summary_mass_mass120.csv`
- `summary_delay_100ms.csv` (retained failure case)

The earlier `summary_nominal_baseline.csv` records a pre-correction diagnostic
run where direct high-gain barrier errors caused communication divergence. It
is intentionally excluded from the main table and preserved only for the
implementation history.
