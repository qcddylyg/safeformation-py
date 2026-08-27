from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from safeformation import Config, Controller, metrics, run, save_json


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=["nominal", "dynamic_obstacle", "delay", "mass", "disturbance"], default="nominal")
    ap.add_argument("--steps", type=int, default=None)
    ap.add_argument("--dt", type=float, default=0.01)
    ap.add_argument("--delay-ms", type=float, default=100.0)
    ap.add_argument("--mass-scale", type=float, default=1.2)
    ap.add_argument("--disturbance-scale", type=float, default=2.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument(
        "--controllers",
        nargs="+",
        default=["low_gain_pd", "heuristic_barrier_pd", "ordinary_adp", "barrier_adp"],
        help="Main comparison: low-gain PD, heuristic barrier-PD, ordinary ADP, and heuristic barrier-ADP. Use engineering_stabilized explicitly for an appendix run.",
    )
    ap.add_argument("--run-label", default="", help="Optional stable label, e.g. delay_100ms; prevents result overwrites.")
    args = ap.parse_args()
    horizon = args.steps * args.dt if args.steps else 10.0
    cfg = Config(dt=args.dt, horizon=horizon, seed=args.seed)
    if args.scenario == "dynamic_obstacle": cfg.dynamic_obstacle = True
    if args.scenario == "delay": cfg.delay_steps = max(0, round(args.delay_ms / 1000.0 / cfg.dt))
    if args.scenario == "mass": cfg.mass_scale = args.mass_scale
    if args.scenario == "disturbance": cfg.disturbance_scale = args.disturbance_scale
    rows = []
    out = ROOT / "results"
    label = "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in args.run_label)
    stem = f"{args.scenario}_{label}" if label else args.scenario
    for name in args.controllers:
        controller = Controller(name, cfg)
        result = run(name, cfg)
        row = {
            "scenario": args.scenario,
            "controller": controller.name,
            "requested_controller": name,
            "controller_variant": controller.variant,
            "seed": cfg.seed,
            "delay_ms": cfg.delay_steps * cfg.dt * 1000.0,
            **metrics(result, cfg),
        }
        rows.append(row)
        save_json(out / f"{stem}_{name}.json", {
            "config": cfg.__dict__,
            "controller_variant": row["controller_variant"],
            "metrics": row,
            "interpretation": "Numerical engineering stress-test result; not a proof or real-robot result.",
        })
    keys = list(rows[0])
    out.mkdir(exist_ok=True)
    summary_path = out / f"summary_{stem}.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys); writer.writeheader(); writer.writerows(rows)
    print(f"wrote {summary_path}")


if __name__ == "__main__":
    main()
