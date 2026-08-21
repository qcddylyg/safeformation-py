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


def engineering_constraint_force(p, p0, i: int, t: float, cfg: Config):
    """Bounded obstacle and connectivity correction for engineering variants.

    This is deliberately separate from the MATLAB logarithmic error state. It
    prevents a barrier-domain violation from being converted into a large,
    directionless PD command and is not a CBF-QP implementation.
    """
    force = np.zeros(2)
    center = obstacle_center(t, cfg)
    delta = p[:, i] - center
    distance = float(np.linalg.norm(delta))
    margin = max(distance - cfg.obstacle_radius, 0.08)
    if distance < cfg.obstacle_activation:
        strength = 8.0 * ((cfg.obstacle_activation - distance) / (cfg.obstacle_activation - cfg.obstacle_radius)) ** 2
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
    def __init__(self, name: str, cfg: Config):
        self.name = name
        self.cfg = cfg

    @property
    def variant(self) -> str:
        if self.name == "full":
            return "engineering_stabilized"
        if self.name == "paper_exact":
            return "formula_only_unvalidated"
        if self.name == "barrier_pd":
            return "heuristic_barrier_pd"
        if self.name == "rnn_adp":
            return "adaptive_residual_surrogate"
        return "low_gain_pd_baseline"

    def __call__(self, p, v, p0, v0, t):
        sc, so, kappa = errors(p, v, p0, v0, t, self.cfg)
        u = np.zeros_like(p)
        for i in range(self.cfg.n_agents):
            formation_error = p[:, i] - p0 - H[:, i]
            velocity_error = v[:, i] - v0
            if self.name == "pd":
                u[:, i] = -1.0 * formation_error - 0.1 * velocity_error
            elif self.name == "barrier_pd":
                u[:, i] = -6.0 * formation_error - 3.0 * velocity_error + engineering_constraint_force(p, p0, i, t, self.cfg)
            elif self.name == "rnn_adp":
                # Deterministic bounded residual representing the no-barrier ablation.
                u[:, i] = -self.cfg.beta * np.tanh(0.08 * kappa[:, i])
            elif self.name == "paper_exact":
                u[:, i] = -self.cfg.beta * np.tanh(0.08 * np.r_[sc[:, i], so[:, i], kappa[:, i]][:2])
            elif self.name == "full":
                u_pd = -6.0 * formation_error - 3.0 * velocity_error + engineering_constraint_force(p, p0, i, t, self.cfg)
                u_adp = -self.cfg.beta * np.tanh(0.08 * kappa[:, i])
                u[:, i] = u_pd + 0.1 * u_adp
            else:
                raise ValueError(f"unknown controller: {self.name}")
        return np.clip(u, -self.cfg.beta, self.cfg.beta), {"sc": sc, "so": so, "kappa": kappa}


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
    history = {k: [] for k in ["t", "p", "v", "p0", "v0", "u", "sc", "so", "kappa"]}
    controller = Controller(controller_name, cfg)
    delayed = []
    for step in range(n):
        t = step * cfg.dt
        obs = (p.copy(), v.copy(), p0.copy(), v0.copy())
        delayed.append(obs)
        used = delayed[max(0, len(delayed) - cfg.delay_steps - 1)]
        u, aux = controller(*used, t)
        history["t"].append(t); history["p"].append(p.copy()); history["v"].append(v.copy())
        history["p0"].append(p0.copy()); history["v0"].append(v0.copy()); history["u"].append(u.copy())
        history["sc"].append(aux["sc"]); history["so"].append(aux["so"]); history["kappa"].append(aux["kappa"])
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
    desired = p0[:, :, None] + H[None, :, :]
    formation = np.linalg.norm(p - desired, axis=1)
    center = np.array([obstacle_center(float(t), cfg) for t in result["t"]])
    distances = np.linalg.norm(p - center[:, :, None], axis=1)
    link_distances = []
    for i in range(cfg.n_agents):
        for j in range(cfg.n_agents):
            if A_ADJ[i, j]: link_distances.append(np.linalg.norm(p[:, :, i] - p[:, :, j], axis=1))
        if B_LEADER[i]: link_distances.append(np.linalg.norm(p[:, :, i] - p0, axis=1))
    links = np.concatenate(link_distances) if link_distances else np.array([0.0])
    finite = bool(np.isfinite(np.concatenate([p.ravel(), u.ravel()])).all())
    return {
        "final_formation_rmse": float(np.mean(formation[-max(1, len(formation)//5):])),
        "max_formation_error": float(np.max(formation)),
        "min_obstacle_distance": float(np.min(distances)),
        "max_active_link_distance": float(np.max(links)),
        "input_peak": float(np.max(np.abs(u))),
        "input_rms": float(np.sqrt(np.mean(u*u))),
        "rnn_error_available": False,
        "saturation_ratio": float(np.mean(np.abs(u) >= cfg.beta * 0.999)),
        "communication_violation_samples": int(np.sum(links > cfg.communication_limit)),
        "obstacle_violation_samples": int(np.sum(distances < cfg.obstacle_radius)),
        "finite": finite,
        "success": bool(finite and np.min(distances) >= cfg.obstacle_radius and np.max(links) <= cfg.communication_limit),
    }


def save_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
