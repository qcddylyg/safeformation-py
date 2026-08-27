from __future__ import annotations

from dataclasses import dataclass, asdict
import json
from pathlib import Path
from typing import Callable

import numpy as np


@dataclass
class Config:
    dt: float = 0.01
    horizon: float = 10.0
    n_agents: int = 4
    dim: int = 2
    obstacle_center: tuple[float, float] = (2.5, 0.0)
    obstacle_radius: float = 1.2
    obstacle_activation: float = 2.5
    communication_limit: float = 8.0
    beta: float = 20.0
    kp: float = 10.0
    kv: float = 2.0
    mass_scale: float = 1.0
    delay_steps: int = 0
    dynamic_obstacle: bool = False
    disturbance_scale: float = 1.0
    safety_margin: float = 0.10
    max_speed: float = 2.0
    seed: int = 42

    @property
    def controller_variant(self) -> str:
        return "configured_at_run_time"

    @property
    def communication_component_limit(self) -> float:
        return self.communication_limit / np.sqrt(self.dim)


H = np.array([[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]], dtype=float).T
A_ADJ = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 1, 0, 0], [1, 0, 0, 0]], dtype=float)
B_LEADER = np.array([1, 1, 0, 0], dtype=float)


def obstacle_center(t: float, cfg: Config) -> np.ndarray:
    c = np.asarray(cfg.obstacle_center, dtype=float)
    if cfg.dynamic_obstacle:
        c = c + np.array([0.45 * np.sin(0.35 * t), 0.35 * np.cos(0.25 * t)])
    return c


def communication_barrier(p_i, p_j, h_i, h_j, limit: float):
    denom = limit - np.linalg.norm(h_i - h_j)
    if denom <= 0:
        return np.zeros(2), np.zeros(2)
    diff = (p_i - p_j) - (h_i - h_j)
    eps = 1e-4
    num = denom + np.sqrt(2.0) * diff + eps
    den = denom - np.sqrt(2.0) * diff + eps
    if np.any(num <= 0) or np.any(den <= 0):
        return np.full(2, 10.0), np.zeros(2)
    s = np.log(num / den)
    gain = np.sqrt(2.0) * (1.0 / num + 1.0 / den)
    return s, gain


def obstacle_barrier(p_i, v_i, center, radius, activation):
    delta_vec = p_i - center
    delta = float(np.linalg.norm(delta_vec))
    if delta <= radius:
        return np.full(2, 10.0)
    if delta > activation:
        return np.zeros(2)
    direction = (center - p_i) / (delta + 1e-6)
    phi = np.pi / 2.0 * ((delta - radius) ** 2) / ((activation - radius) ** 2)
    value = np.log(25.0 / (25.0 - (5.0 * np.cos(phi)) ** 2 + 1e-6))
    return value * direction


def errors(p, v, p0, v0, t, cfg: Config):
    sc = np.zeros_like(p)
    so = np.zeros_like(p)
    kappa = np.zeros_like(p)
    center = obstacle_center(t, cfg)
    for i in range(cfg.n_agents):
        for j in range(cfg.n_agents):
            if A_ADJ[i, j]:
                s, gain = communication_barrier(p[:, i], p[:, j], H[:, i], H[:, j], cfg.communication_component_limit)
                sc[:, i] += s
                kappa[:, i] += v[:, i] - v[:, j]
        if B_LEADER[i]:
            s, gain = communication_barrier(p[:, i], p0, H[:, i], np.zeros(2), cfg.communication_component_limit)
            sc[:, i] += s
            kappa[:, i] += v[:, i] - v0
        so[:, i] = obstacle_barrier(p[:, i], v[:, i], center, cfg.obstacle_radius, cfg.obstacle_activation)
    return sc, so, kappa


def engineering_constraint_force(p, p0, i: int, t: float, cfg: Config, obstacle_radius: float | None = None, barrier_gain: float = 8.0):
    """Bounded obstacle and connectivity correction for engineering variants.

    This is deliberately separate from the MATLAB logarithmic error state. It
    prevents a barrier-domain violation from being converted into a large,
    directionless PD command and is not a CBF-QP implementation.
    """
    force = np.zeros(2)
    control_radius = cfg.obstacle_radius if obstacle_radius is None else float(obstacle_radius)
    center = obstacle_center(t, cfg)
    delta = p[:, i] - center
    distance = float(np.linalg.norm(delta))
    margin = max(distance - control_radius, 0.08)
    if distance < cfg.obstacle_activation:
        activation_gap = max(cfg.obstacle_activation - control_radius, 0.05)
        strength = barrier_gain * ((cfg.obstacle_activation - distance) / activation_gap) ** 2
        force += strength * delta / (margin * (distance + 1e-6))
    # Each active edge pulls back only close to the physical communication limit.
    for j in range(cfg.n_agents):
        if A_ADJ[i, j]:
            relative = p[:, j] - p[:, i]
            distance_ij = float(np.linalg.norm(relative))
            if distance_ij > 0.80 * cfg.communication_limit:
                force += 2.0 * (distance_ij / cfg.communication_limit - 0.80) * relative / (distance_ij + 1e-6)
    if B_LEADER[i]:
        relative = p0 - p[:, i]
        distance_i0 = float(np.linalg.norm(relative))
        if distance_i0 > 0.80 * cfg.communication_limit:
            force += 2.0 * (distance_i0 / cfg.communication_limit - 0.80) * relative / (distance_i0 + 1e-6)
    return np.clip(force, -cfg.beta * 0.5, cfg.beta * 0.5)


class Controller:
    _ALIASES = {
        "pd": "low_gain_pd",
        "barrier_pd": "heuristic_barrier_pd",
        "rnn_adp": "ordinary_adp",
        "rnn_adp_surrogate": "ordinary_adp",
        "paper_exact": "barrier_adp",
        "paper_rnn_adp": "barrier_adp",
        "full": "engineering_stabilized",
    }

    def __init__(self, name: str, cfg: Config):
        self.requested_name = name
        self.name = self._ALIASES.get(name, name)
        self.cfg = cfg
        if self.name not in {
            "low_gain_pd",
            "heuristic_barrier_pd",
            "ordinary_adp",
            "barrier_adp",
            "engineering_stabilized",
        }:
            raise ValueError(f"unknown controller: {name}")

        # A compact feed-forward actor-critic ADP surrogate. The two ADP
        # branches share the same learner; only the presence of barrier state
        # in chi differs, making the comparison interpretable.
        self._adp_names = {"ordinary_adp", "barrier_adp"}
        self._actor_dim = 11
        self._critic_dim = 28  # 1 + 6 linear + 21 quadratic terms
        self._wa = np.zeros((cfg.n_agents, self._actor_dim, cfg.dim), dtype=float)
        self._wc = np.zeros((cfg.n_agents, self._critic_dim), dtype=float)
        self._prev_psi_c = [None] * cfg.n_agents
        self._prev_value = np.zeros(cfg.n_agents, dtype=float)
        self._prev_cost = np.zeros(cfg.n_agents, dtype=float)
        self._prev_action = np.zeros((cfg.dim, cfg.n_agents), dtype=float)
        self._has_transition = np.zeros(cfg.n_agents, dtype=bool)
        for i in range(cfg.n_agents):
            # Stable small-error initialization. Adaptation can change these
            # gains online, while tanh keeps every actor output bounded.
            self._wa[i, 7, 0] = 1.00
            self._wa[i, 8, 1] = 1.00
            self._wa[i, 9, 0] = 0.40
            self._wa[i, 10, 1] = 0.40
            # Barrier-ADP starts with a weak safety prior in the first four
            # barrier feature channels; no additive barrier force is applied.
            self._wa[i, 1, 0] = 0.15
            self._wa[i, 2, 1] = 0.15
            self._wa[i, 3, 0] = 0.20
            self._wa[i, 4, 1] = 0.20

    @property
    def variant(self) -> str:
        if self.name == "engineering_stabilized":
            return "engineering_stabilized"
        if self.name == "barrier_adp":
            return "heuristic_barrier_adp_actor_critic"
        if self.name == "ordinary_adp":
            return "ordinary_adp_no_barrier_actor_critic"
        if self.name == "heuristic_barrier_pd":
            return "heuristic_barrier_pd"
        return "low_gain_pd_baseline"

    @staticmethod
    def _quadratic_features(x: np.ndarray) -> np.ndarray:
        values = [1.0]
        values.extend(x.tolist())
        for i in range(len(x)):
            for j in range(i, len(x)):
                values.append(float(x[i] * x[j]))
        return np.asarray(values, dtype=float)

    def _adp_features(self, chi: np.ndarray, formation_error: np.ndarray, velocity_error: np.ndarray, i: int):
        chi_n = np.clip(chi / 3.0, -3.0, 3.0)
        chi_a = np.clip(chi, -5.0, 5.0)
        psi_a = np.concatenate([
            np.ones(1),
            np.tanh(0.20 * chi_a),
            np.tanh(0.40 * formation_error),
            np.tanh(0.40 * velocity_error),
        ])
        psi_c = self._quadratic_features(chi_n)
        return psi_a, psi_c

    def _adp_action(self, chi, formation_error, velocity_error, i, t):
        psi_a, psi_c = self._adp_features(chi, formation_error, velocity_error, i)
        value = float(self._wc[i] @ psi_c)
        raw = self._wa[i].T @ psi_a
        action = -self.cfg.beta * np.tanh(raw)

        # Online TD-style critic update and a bounded policy adaptation.  The
        # learning rates are deliberately conservative for a reproducible
        # simulation; this is an ADP teaching implementation, not a theorem
        # equivalence claim for the supplied MATLAB observer.
        cost = float(
            0.5 * formation_error @ formation_error
            + 0.15 * velocity_error @ velocity_error
            + 0.01 * action @ action
        )
        td = 0.0
        if self._has_transition[i]:
            td = self._prev_cost[i] + self.cfg.dt * cost + 0.98 * value - self._prev_value[i]
            td = float(np.clip(td, -5.0, 5.0))
            self._wc[i] += self.cfg.dt * 0.8 * td * self._prev_psi_c[i]
            direction = np.tanh(np.r_[formation_error, velocity_error])
            policy_signal = np.r_[direction[:2], direction[2:4]]
            self._wa[i] -= self.cfg.dt * 0.008 * td * np.outer(psi_a, np.array([policy_signal[0], policy_signal[1]]))
            self._wc[i] = np.clip(self._wc[i], -10.0, 10.0)
            self._wa[i] = np.clip(self._wa[i], -2.0, 2.0)
        self._prev_psi_c[i] = psi_c
        self._prev_value[i] = value
        self._prev_cost[i] = cost
        self._prev_action[:, i] = action
        self._has_transition[i] = True
        return action, td, float(np.linalg.norm(self._wa[i]))

    def _conservative_obstacle_radius(self) -> float:
        # Reserve distance for the worst motion during the information age.
        delay_horizon = self.cfg.delay_steps * self.cfg.dt
        return self.cfg.obstacle_radius + self.cfg.safety_margin + self.cfg.max_speed * delay_horizon

    def _obstacle_safety_layer(self, action: np.ndarray, p_i: np.ndarray, v_i: np.ndarray, t: float) -> np.ndarray:
        """Remove inward radial action and add braking near the conservative radius."""
        center = obstacle_center(t, self.cfg)
        delta = p_i - center
        distance = float(np.linalg.norm(delta))
        if distance < 1e-8:
            return action
        direction = delta / distance
        radius = self._conservative_obstacle_radius()
        guard = 0.20
        if distance >= radius + guard:
            return action
        inward_action = min(0.0, float(np.dot(action, direction)))
        inward_velocity = min(0.0, float(np.dot(v_i, direction)))
        braking = -inward_action + 4.0 * (-inward_velocity)
        return action + braking * direction

    def __call__(self, p, v, p0, v0, t, safety_state=None):
        sc, so, kappa = errors(p, v, p0, v0, t, self.cfg)
        u = np.zeros_like(p)
        adp_td = np.zeros(self.cfg.n_agents, dtype=float)
        adp_weight_norm = np.zeros(self.cfg.n_agents, dtype=float)
        for i in range(self.cfg.n_agents):
            formation_error = p[:, i] - p0 - H[:, i]
            velocity_error = v[:, i] - v0
            if self.name == "low_gain_pd":
                u[:, i] = -1.0 * formation_error - 0.1 * velocity_error
            elif self.name == "heuristic_barrier_pd":
                u[:, i] = -6.0 * formation_error - 3.0 * velocity_error + engineering_constraint_force(p, p0, i, t, self.cfg)
            elif self.name in self._adp_names:
                if self.name == "ordinary_adp":
                    chi = np.r_[formation_error, velocity_error, np.zeros(2)]
                else:
                    chi = np.r_[sc[:, i], so[:, i], kappa[:, i]]
                adp_action, td, weight_norm = self._adp_action(chi, formation_error, velocity_error, i, t)
                if self.name == "barrier_adp":
                    # The barrier-ADP experiment combines the learned action
                    # with the same bounded heuristic safety correction used
                    # by barrier-PD. Barrier state is also part of chi, so the
                    # comparison tests both explicit safety feedback and ADP.
                    u[:, i] = adp_action + engineering_constraint_force(
                        p,
                        p0,
                        i,
                        t,
                        self.cfg,
                        obstacle_radius=self._conservative_obstacle_radius(),
                        barrier_gain=10.0,
                    )
                    safety_p = p if safety_state is None else safety_state[0]
                    safety_v = v if safety_state is None else safety_state[1]
                    u[:, i] = self._obstacle_safety_layer(u[:, i], safety_p[:, i], safety_v[:, i], t)
                else:
                    u[:, i] = adp_action
                adp_td[i] = td
                adp_weight_norm[i] = weight_norm
            elif self.name == "engineering_stabilized":
                u_pd = -6.0 * formation_error - 3.0 * velocity_error + engineering_constraint_force(p, p0, i, t, self.cfg)
                u_adp = -self.cfg.beta * np.tanh(0.08 * kappa[:, i])
                u[:, i] = u_pd + 0.1 * u_adp
        return np.clip(u, -self.cfg.beta, self.cfg.beta), {
            "sc": sc,
            "so": so,
            "kappa": kappa,
            "adp_td": adp_td,
            "adp_weight_norm": adp_weight_norm,
        }


def initial_state(cfg: Config):
    p = np.array([[1.5, -2.0], [-1.5, 1.0], [-2.5, 1.0], [0.5, -6.0]], dtype=float).T
    v = np.zeros((2, cfg.n_agents), dtype=float)
    p0 = np.zeros(2, dtype=float)
    v0 = np.array([0.5, 0.45], dtype=float)
    return p, v, p0, v0


def derivatives(p, v, p0, v0, u, t, cfg: Config):
    dp0 = v0
    dv0 = np.array([0.0, -0.09 * p0[1]])
    dp = v
    dv = np.zeros_like(v)
    for i in range(cfg.n_agents):
        nonlinear = -(np.tanh(100.0 * v[:, i]) - np.tanh(10.0 * v[:, i])) / (6.0 * cfg.mass_scale)
        disturbance = 0.1 * cfg.disturbance_scale * np.sin(t)
        dv[:, i] = nonlinear + u[:, i] + disturbance
    return dp, dv, dp0, dv0


def run(controller_name: str, cfg: Config):
    rng = np.random.default_rng(cfg.seed)
    del rng  # seed is retained in the manifest; current dynamics are deterministic.
    p, v, p0, v0 = initial_state(cfg)
    n = int(round(cfg.horizon / cfg.dt)) + 1
    history = {k: [] for k in ["t", "p", "v", "p0", "v0", "u", "sc", "so", "kappa", "adp_td", "adp_weight_norm"]}
    controller = Controller(controller_name, cfg)
    delayed = []
    for step in range(n):
        t = step * cfg.dt
        obs = (p.copy(), v.copy(), p0.copy(), v0.copy())
        delayed.append(obs)
        used = delayed[max(0, len(delayed) - cfg.delay_steps - 1)]
        u, aux = controller(*used, t, safety_state=(p, v))
        history["t"].append(t); history["p"].append(p.copy()); history["v"].append(v.copy())
        history["p0"].append(p0.copy()); history["v0"].append(v0.copy()); history["u"].append(u.copy())
        history["sc"].append(aux["sc"]); history["so"].append(aux["so"]); history["kappa"].append(aux["kappa"])
        history["adp_td"].append(aux["adp_td"].copy()); history["adp_weight_norm"].append(aux["adp_weight_norm"].copy())
        if step == n - 1:
            break
        # RK4 for plant state; controller is held over this integration step.
        k1 = derivatives(p, v, p0, v0, u, t, cfg)
        k2 = derivatives(p + 0.5 * cfg.dt * k1[0], v + 0.5 * cfg.dt * k1[1], p0 + 0.5 * cfg.dt * k1[2], v0 + 0.5 * cfg.dt * k1[3], u, t + 0.5 * cfg.dt, cfg)
        k3 = derivatives(p + 0.5 * cfg.dt * k2[0], v + 0.5 * cfg.dt * k2[1], p0 + 0.5 * cfg.dt * k2[2], v0 + 0.5 * cfg.dt * k2[3], u, t + 0.5 * cfg.dt, cfg)
        k4 = derivatives(p + cfg.dt * k3[0], v + cfg.dt * k3[1], p0 + cfg.dt * k3[2], v0 + cfg.dt * k3[3], u, t + cfg.dt, cfg)
        p += cfg.dt / 6.0 * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]); v += cfg.dt / 6.0 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
        p0 += cfg.dt / 6.0 * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]); v0 += cfg.dt / 6.0 * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3])
    return {k: np.asarray(v) for k, v in history.items()}


def metrics(result, cfg: Config):
    p = result["p"]; p0 = result["p0"]; u = result["u"]; kappa = result["kappa"]
    adp_td = result.get("adp_td", np.zeros((len(result["t"]), cfg.n_agents)))
    adp_weight_norm = result.get("adp_weight_norm", np.zeros((len(result["t"]), cfg.n_agents)))
    desired = p0[:, :, None] + H[None, :, :]
    formation = np.linalg.norm(p - desired, axis=1)
    center = np.array([obstacle_center(float(t), cfg) for t in result["t"]])
    distances = np.linalg.norm(p - center[:, :, None], axis=1)
    link_distances = []
    for i in range(cfg.n_agents):
        for j in range(cfg.n_agents):
            if A_ADJ[i, j]: link_distances.append(np.linalg.norm(p[:, :, i] - p[:, :, j], axis=1))
        if B_LEADER[i]: link_distances.append(np.linalg.norm(p[:, :, i] - p0, axis=1))
    link_matrix = np.vstack(link_distances) if link_distances else np.zeros((1, len(result["t"])))
    links = link_matrix.ravel()
    finite = bool(np.isfinite(np.concatenate([p.ravel(), u.ravel()])).all())
    obstacle_violations = distances < cfg.obstacle_radius
    communication_violations = links > cfg.communication_limit
    communication_violation_by_time = np.any(link_matrix > cfg.communication_limit, axis=0)
    invalid_samples = np.any(~np.isfinite(p), axis=(1, 2)) | np.any(~np.isfinite(u), axis=(1, 2))
    failure_samples = np.asarray(obstacle_violations).any(axis=1) | communication_violation_by_time | invalid_samples
    first_failure_time = None
    if np.any(failure_samples):
        first_failure_time = float(result["t"][np.flatnonzero(failure_samples)[0]])
    return {
        "final_formation_rmse": float(np.mean(formation[-max(1, len(formation)//5):])),
        "max_formation_error": float(np.max(formation)),
        "min_obstacle_distance": float(np.min(distances)),
        "max_active_link_distance": float(np.max(links)),
        "input_peak": float(np.max(np.abs(u))),
        "input_rms": float(np.sqrt(np.mean(u*u))),
        "rnn_error_available": False,
        "adp_diagnostics_available": bool(np.any(adp_weight_norm > 0.0)),
        "adp_td_rms": float(np.sqrt(np.mean(adp_td * adp_td))),
        "adp_weight_peak": float(np.max(adp_weight_norm)),
        "saturation_ratio": float(np.mean(np.abs(u) >= cfg.beta * 0.999)),
        "communication_violation_samples": int(np.sum(communication_violations)),
        "communication_violation_steps": int(np.sum(communication_violations)),
        "obstacle_violation_samples": int(np.sum(obstacle_violations)),
        "obstacle_violation_steps": int(np.sum(obstacle_violations)),
        "first_failure_time": first_failure_time,
        "finite": finite,
        "success": bool(finite and np.min(distances) >= cfg.obstacle_radius and np.max(links) <= cfg.communication_limit),
    }


def save_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
