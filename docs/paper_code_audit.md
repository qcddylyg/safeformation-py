# Paper-Code Audit

## Scope and conclusion

This project is an engineering reproduction of the supplied MATLAB simulation,
not a claim of a line-by-line theorem implementation. The MATLAB source is the
oracle for initial conditions, topology, dynamics, barrier settings, controller
labels, and evaluation definitions. Its Case 4 is a **PD-stabilized engineering
variant**, because it explicitly uses `u_total = u_pd + 0.1 * u_adp`.

The Python controller name `engineering_stabilized` therefore means the
PD-stabilized engineering branch. The project now also exposes ordinary
actor-critic ADP and heuristic barrier-ADP experiment branches, but they
remain numerical surrogates and are not treated as line-by-line paper
reproductions.

## Mapping

| MATLAB source | Meaning | Python location | Status |
|---|---|---|---|
| parameter block | four followers, 2-D state, leader, `h`, `A_adj`, `B_leader` | `Config`, `H`, `A_ADJ`, `B_LEADER` | implemented |
| `p_init`, `v_init`, `p0_0`, `v0_0` | nominal initial state | `initial_state` | implemented |
| `dp0`, `dv0` | leader oscillator | `derivatives` | implemented |
| follower `dv` | nonlinear velocity term, input, sinusoidal disturbance | `derivatives` | implemented |
| `compute_augmented_error` | communication, obstacle, and velocity errors | `errors` and barrier helpers | engineering approximation |
| Case 1 | low-gain saturated formation PD | `low_gain_pd` | implemented |
| Case 2 | PD with double-barrier error | `heuristic_barrier_pd` | heuristic implementation |
| Case 3 | ordinary ADP without barriers | `ordinary_adp` | actor-critic surrogate |
| Proposed experiment | heuristic barrier-ADP | `barrier_adp` | actor-critic surrogate |
| Engineering appendix | `u_pd + 0.1 * u_adp` with saturation | `engineering_stabilized` | engineering-stabilized branch |
| post-processing figures | tracking, obstacle, link, control and weight diagnostics | metrics, CSV, plots | partial; weight diagnostics excluded |

## Important discrepancies

1. The MATLAB main file labels Case 4 as the full method but adds an explicit
   PD stabilizer. This must never be presented as the pure paper policy.
2. Case 3 uses a shorter 20-second integration horizon while the other cases
   use 50 seconds. Cross-method comparisons need an explicitly shared horizon.
3. MATLAB integrates observer and actor/critic weights as continuous states;
   the current Python MVP does not reproduce those exact updates. Its adaptive
   components are engineering surrogates, so ADP findings are empirical
   only.
4. MATLAB declares `D_threshold = D_comm / sqrt(m)` for component-wise barrier
   construction, but its plots also show that threshold for Euclidean link
   distance. The Python evaluator uses the physical communication limit
   `D_comm` for success and records the distinction in its outputs.
5. The static-obstacle derivation does not establish safety under a moving
   obstacle or delayed communication. Those scenarios are stress tests, not
   extensions of the theorem.

## Validation gate

Before calling a controller a paper reproduction, run both implementations
from identical initial states and compare trajectory samples, final formation
error, minimum obstacle distance, maximum active-link distance, maximum input,
and observer error at a fixed horizon. Do not tune gains only on the Python
side to hide a mismatch.

## Detailed evidence and required decisions

### Paper-controlled version versus MATLAB Case 4

The paper states on page 11 (equation (99)) that the proposed input is exactly
`u_i = -beta_sat * tanh(Wa_i^T * psi_a_i)`. It further states that there is no
PD term, blending coefficient, safety filter, or post-processing saturation.
The MATLAB implementation conflicts with this in both its post-processing path
(`main_simulation_v8_adaptation.txt`, lines 293--300) and its ODE right-hand
side (lines 468--481): it computes a PD action from `s_c + s_o` and `kappa`,
adds `0.1 * u_adp`, then clips the sum at `0.95 * beta_sat`.

**Decision:** retain this controller as `engineering_stabilized`. The separate
`barrier_adp` branch is an explicitly labelled actor-critic experiment branch;
results and manifests must include `controller_variant` and must not call it a
formal RNN system-identification reproduction.

### Baseline definitions are not yet paper-equivalent

- The paper describes Case 1 as a tuned saturated PD policy, while the MATLAB
  comments deliberately call its `Kp=1`, `Kv=0.1` setting low gain (lines
  266--271 and 443--447).
- The paper describes Case 2 as a second-order CBF safety/connectivity filter
  acting on that same PD policy and using exact plant information. MATLAB
  instead changes the PD gains to `6/0.5` and adds `-0.3*s_all` (lines 274--281
  and 450--456). It contains neither a CBF-QP nor explicit second-order CBF
  constraints.
- The paper describes Case 3 as a no-barrier RNN-ADP method using raw formation
  errors. MATLAB drives its actor with `[0; 0; kappa]` but updates Critic,
  u-Action, and w-Action weights with the barrier-state `chi=[s_c;s_o;kappa]`
  (lines 284--290, 459--465, and 545--603). Thus its learning objective and
  deployed actor state are not a single self-consistent no-barrier system.

**Decision:** the Python project now uses a self-consistent raw-error
`ordinary_adp` and a barrier-state `barrier_adp` experiment branch. Both remain
numerical actor-critic surrogates; neither is a CBF-QP or a formal line-by-line
reproduction of the MATLAB RNN observer.

### Comparison protocol differs across cases

Paper page 11 requires every case to use 50 seconds with `ode45`,
`RelTol=1e-6`, `AbsTol=1e-8`, and `MaxStep=0.005`; it explicitly says no run is
terminated early. MATLAB changes Case 3 to 20 seconds, `RelTol=1e-4`,
`AbsTol=1e-6`, and `MaxStep=0.02` (lines 159--166). Its Fig.14/Fig.15 then load
all `simulation_case*.mat` files and compare their endpoint metrics
(`plot_figures_v8_adaptation.txt`, lines 537--664). Those comparisons are not
equal-horizon comparisons.

**Decision:** Python evaluation uses one declared horizon and integrator
configuration for every method. A numerical failure yields `success=false`,
`failure_time`, and retained trajectory/log data; it must not be converted to a
shorter nominal experiment.

### Barrier implementation needs explicit domain reporting

The communication barrier closely follows paper equation (31), but MATLAB adds
`eps=1e-4`; if a numerator or denominator becomes non-positive, the relevant
barrier coordinate remains zero rather than emitting an invalid-domain result
(`main_simulation_v8_adaptation.txt`, lines 670--689). The obstacle barrier is
also an engineering radial-vector construction (`B(delta) * direction`, lines
692--714), rather than a demonstrated line-by-line implementation of the
paper's component-wise notation in equations (36), (40), and (41).

**Decision:** barrier APIs must return both values and a domain/violation flag.
`distance <= r`, communication-bound violations, non-finite values, and near-
singular denominators must be logged with time, agent, and link; they cannot be
silently transformed into finite values.

### Initialisation and metric gaps

The paper says the first six actor features are initialized as `tanh(0.20*chi)`
with weights yielding small-error gains `K_s=18`, `K_v=4`. MATLAB instead uses
random hidden weights and random `Wa` under fixed `rng(42)` (lines 99--106 and
146--152), without constructing the stated features/weights. The plotting
script reports final instantaneous formation error, minimum obstacle distance,
and peak input, but not the paper's final-5-second mean error, maximum active
link distance, tail RNN errors, nonfinite-state count, or full-horizon status.

**Decision:** the unified evaluator emits tail formation error, minimum obstacle
distance, maximum physical active-link distance, input peak/RMS/saturation
ratio, ADP TD/weight diagnostics, completion status, and first violation time.
The ADP branches remain explicitly labelled surrogates until a MATLAB-to-Python
trajectory match is completed.

## Frozen paper-exact configuration

Use a dedicated immutable configuration for the first numerical comparison:

| Field | Value |
|---|---|
| agents / coordinates / horizon | `N=4`, `m=2`, `T=50 s` |
| topology | leader -> 1,2; 2 -> 3; 1 -> 4 |
| obstacle | center `[2.5, 0]`, `r=1.2`, `mu=2.5`, `A_f=5`, `c=1` |
| communication | `D=8`; component construction threshold `8/sqrt(2)` |
| RNN | `L=10I`, `E=30I`, identifier rate `0.5` |
| ADP | `nu=.15`, `gamma=2`, `Q=diag(100,100,100,100,300,300)`, `R=I`, `T=I`, `beta=20`, `Nc=21`, `Na=8`, rates `.5/.05/.05` |
| solver | paper's `ode45` tolerances above, or a documented Python solver with matched maximum step and convergence check |

Dynamic obstacles, communication delay, and parameter perturbation are outside
the static-obstacle/no-delay proof. They are useful engineering stress tests,
but must be marked as empirical extensions rather than theorem validation.
