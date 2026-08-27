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

## 项目描述

**基于启发式屏障约束与 ADP 的多智能体安全编队仿真**

**技术/方法：**Python/MATLAB、NumPy、RK4 数值积分、非线性多智能体动力学、PD 控制、Actor-Critic ADP、启发式屏障约束、通信延迟与输入饱和。

- 建立 Leader-Follower 多智能体编队模型，统一处理编队跟踪、通信距离、障碍物安全距离、外部扰动、质量变化和控制输入边界。
- 采用“有无屏障 × PD/ADP”的四组主对比：低增益 PD、启发式屏障-PD、无屏障普通 ADP、启发式屏障-ADP；另保留 `engineering_stabilized` 作为附录工程分支。
- 在名义、动态障碍物、50/100 ms 延迟、质量增加 20% 和扰动增强等工况下，比较编队误差、安全距离、通信违例、控制能量和饱和率。当前配置下，屏障-ADP 借助保守安全半径、延迟补偿和本地障碍物测量，在所测场景中保持无障碍物越界；该结论是数值实验观察，不是普适安全定理。

## What is implemented

- Four 2-D followers, one virtual leader, the supplied directed topology and
  nominal initial conditions.
- Nonlinear follower dynamics, sinusoidal disturbance, input saturation,
  static or moving obstacle, and delayed neighbour/leader state use.
- Low-gain PD, heuristic barrier-PD, ordinary actor-critic ADP, heuristic
  barrier-ADP, plus a separately labelled engineering-stabilized branch.
- Barrier-ADP uses an explicit bounded safety layer: conservative obstacle
  radius inflation for delayed information, local obstacle-state sensing, and
  inward-action removal near the safety boundary.
- Deterministic RK4 integration, per-run JSON manifests, and a CSV summary
  containing tracking, safety, connectivity, control-effort and completion
  metrics.

## Controller names and evidence boundary

| CLI controller | Manifest variant | Meaning |
|---|---|---|
| `low_gain_pd` (`pd`) | `low_gain_pd_baseline` | Low-gain formation PD baseline. |
| `heuristic_barrier_pd` (`barrier_pd`) | `heuristic_barrier_pd` | Barrier-PD heuristic, not a CBF-QP. |
| `ordinary_adp` (legacy `rnn_adp`) | `ordinary_adp_no_barrier_actor_critic` | Actor-critic ADP without barrier state. The `rnn_adp` name is retained only for compatibility. |
| `barrier_adp` (legacy `paper_rnn_adp`, `paper_exact`) | `heuristic_barrier_adp_actor_critic` | Actor-critic ADP with heuristic barrier state. The legacy names do not indicate an RNN implementation. |
| `engineering_stabilized` (`full`) | `engineering_stabilized` | PD plus bounded residual; appendix only. |

The supplied MATLAB Case 4 explicitly adds a PD term and clips the output, so
it is represented by `engineering_stabilized`, not by the barrier-ADP branch. See
[`docs/paper_code_audit.md`](docs/paper_code_audit.md) before comparing results.

For delayed-information tests, neighbour and leader states are delayed but the
obstacle position of each agent is treated as a local measurement. The
barrier-ADP safety layer inflates the control radius by `safety_margin` plus
`max_speed * delay_age`, and removes the action component pointing toward the
obstacle near that conservative radius. This is a bounded heuristic safety
layer, not a CBF-QP or a formal delayed-system guarantee.

## Quick start

Python and NumPy are the only runtime requirements. Run commands from this
directory:

```powershell
python -m pip install -r requirements.txt
python tests\test_core.py

python scripts\run_matrix.py --scenario nominal --steps 5000 --dt 0.01
python scripts\run_matrix.py --scenario dynamic_obstacle --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp
python scripts\run_matrix.py --scenario delay --delay-ms 100 --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp
python scripts\run_matrix.py --scenario mass --mass-scale 1.2 --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp
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
python scripts\run_matrix.py --scenario nominal --steps 5000 --dt 0.01 --controllers low_gain_pd heuristic_barrier_pd ordinary_adp barrier_adp --run-label main_nominal
python scripts\run_matrix.py --scenario dynamic_obstacle --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp --run-label moving
python scripts\run_matrix.py --scenario delay --delay-ms 50 --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp --run-label 50ms
python scripts\run_matrix.py --scenario delay --delay-ms 100 --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp --run-label 100ms
python scripts\run_matrix.py --scenario mass --mass-scale 1.2 --steps 5000 --dt 0.01 --controllers heuristic_barrier_pd barrier_adp --run-label mass120
python scripts\run_matrix.py --scenario nominal --steps 5000 --dt 0.01 --controllers engineering_stabilized --run-label engineering_appendix
```

The 100 ms delay run is intentionally retained as a stress test. In the
checked results, barrier-PD violates the physical obstacle radius while
barrier-ADP completes the run without obstacle or communication violations
under the local-obstacle-sensing assumption. Do not interpret `success=false`
as a crash: it means that a physical constraint was violated.

## Experiment protocol

1. Run `nominal` for the same horizon and step size for every controller.
2. Hold controller, seed, initial state, horizon and integration step fixed.
3. Change one variable only: obstacle motion, delay, mass scale, or disturbance
   scale.
4. Main tables compare `low_gain_pd`, `heuristic_barrier_pd`,
   `ordinary_adp`, and `barrier_adp`; keep engineering_stabilized in a
   separate appendix table and keep all failures.
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

- The actor-critic module is a reproducible educational ADP surrogate; it
  exposes online TD and weight diagnostics but does not reproduce the MATLAB
  continuous observer and every paper update equation exactly.
- `barrier_adp` is a heuristic barrier-plus-ADP experiment branch, not a CBF-QP
  and not a line-by-line paper RNN-ADP reproduction.
- The current layer is a numerical simulation. MuJoCo visualization and any
  physical robot claims remain outside this version.
- A successful finite rollout is empirical evidence only, never a proof that a
  constraint holds for all initial states, delays, or obstacle trajectories.
