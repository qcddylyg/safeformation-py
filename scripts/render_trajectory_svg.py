"""Render a dependency-free trajectory figure from one numerical rollout."""

from __future__ import annotations

import argparse
import html
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from safeformation import Config, H, obstacle_center, run


COLORS = ("#0f766e", "#d97706", "#7c3aed", "#dc2626")


def scale(points: np.ndarray, x_min: float, y_min: float, extent: float, width: int, height: int) -> np.ndarray:
    x = 70 + (points[:, 0] - x_min) / extent * (width - 110)
    y = height - 55 - (points[:, 1] - y_min) / extent * (height - 105)
    return np.column_stack((x, y))


def polyline(points: np.ndarray) -> str:
    return " ".join(f"{x:.1f},{y:.1f}" for x, y in points)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--controller",
        choices=["low_gain_pd", "heuristic_barrier_pd", "ordinary_adp", "barrier_adp", "engineering_stabilized", "pd", "barrier_pd", "rnn_adp", "paper_exact", "paper_rnn_adp", "full"],
        default="barrier_adp",
    )
    parser.add_argument("--scenario", choices=["nominal", "dynamic_obstacle", "delay", "mass", "disturbance"], default="nominal")
    parser.add_argument("--delay-ms", type=float, default=0.0)
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "figures" / "full_nominal_trajectory.svg")
    args = parser.parse_args()
    cfg = Config(horizon=50.0, dt=0.01)
    if args.scenario == "dynamic_obstacle":
        cfg.dynamic_obstacle = True
    if args.scenario == "delay":
        cfg.delay_steps = round(args.delay_ms / 1000.0 / cfg.dt)
    result = run(args.controller, cfg)
    leader = result["p0"]
    agents = result["p"]
    all_points = np.vstack([leader, agents.transpose(0, 2, 1).reshape(-1, 2)])
    x_min, y_min = all_points.min(axis=0) - 1.5
    x_max, y_max = all_points.max(axis=0) + 1.5
    extent = max(x_max - x_min, y_max - y_min)
    width, height = 1080, 650
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#1f2937}.axis{stroke:#64748b;stroke-width:1}.track{fill:none;stroke-width:2}</style>',
        f'<text x="70" y="30" font-size="22" font-weight="700">{html.escape(args.controller)} - {html.escape(args.scenario)} trajectory</text>',
        f'<text x="70" y="52" font-size="13">50 s numerical rollout; obstacle radius {cfg.obstacle_radius:.1f} m, communication limit {cfg.communication_limit:.1f} m</text>',
    ]
    lines.append(f'<line class="axis" x1="70" y1="{height - 55}" x2="{width - 40}" y2="{height - 55}"/>')
    lines.append(f'<line class="axis" x1="70" y1="65" x2="70" y2="{height - 55}"/>')
    c = obstacle_center(0.0, cfg)[None, :]
    center_px = scale(c, x_min, y_min, extent, width, height)[0]
    radius_px = cfg.obstacle_radius / extent * (width - 110)
    activation_px = cfg.obstacle_activation / extent * (width - 110)
    lines.append(f'<circle cx="{center_px[0]:.1f}" cy="{center_px[1]:.1f}" r="{activation_px:.1f}" fill="none" stroke="#f59e0b" stroke-dasharray="5 4"/>')
    lines.append(f'<circle cx="{center_px[0]:.1f}" cy="{center_px[1]:.1f}" r="{radius_px:.1f}" fill="#fee2e2" stroke="#dc2626" stroke-width="2"/>')
    leader_px = scale(leader, x_min, y_min, extent, width, height)
    lines.append(f'<polyline class="track" points="{polyline(leader_px)}" stroke="#0f172a"/>')
    for i, color in enumerate(COLORS):
        points = scale(agents[:, :, i], x_min, y_min, extent, width, height)
        lines.append(f'<polyline class="track" points="{polyline(points)}" stroke="{color}"/>')
        lines.append(f'<circle cx="{points[-1, 0]:.1f}" cy="{points[-1, 1]:.1f}" r="4" fill="{color}"/>')
        lines.append(f'<text x="{800 + (i % 2) * 120}" y="{95 + (i // 2) * 24}" font-size="13" fill="{color}">Agent {i + 1}</text>')
    lines.append('<text x="800" y="143" font-size="13">Leader (black)</text>')
    lines.append('</svg>')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
