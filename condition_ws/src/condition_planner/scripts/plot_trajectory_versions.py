#!/usr/bin/env python3
"""Compare trajectory_v1 and trajectory_v2 for condition-branch evaluation."""

import csv
import math
import os
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.rcParams["font.sans-serif"] = ["WenQuanYi Micro Hei", "Noto Sans CJK JP"]
matplotlib.rcParams["axes.unicode_minus"] = False

SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = SCRIPT_DIR.parent / "config"
V1_CSV = CONFIG_DIR / "trajectory_v1.csv"
V2_CSV = CONFIG_DIR / "trajectory_v2.csv"
OUT_PNG = CONFIG_DIR / "trajectory_versions_compare.png"

MAX_PHI = 0.3491
PHI_THRESHOLD = 0.5 * MAX_PHI
V_HALF = 0.125


def load_csv(path: Path):
    rows = []
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def nearest_path_distance(path_a, path_b):
    dist = []
    for p in path_a:
        d = min(math.hypot(p["x"] - q["x"], p["y"] - q["y"]) for q in path_b)
        dist.append(d)
    return max(dist), sum(dist) / len(dist)


def summarize(rows):
    moving = [r for r in rows if abs(r["v"]) > 1e-4]
    high_phi = [r for r in rows if abs(r["phi"]) > PHI_THRESHOLD]
    low_phi_moving = [r for r in moving if abs(r["phi"]) <= PHI_THRESHOLD]

    summary = {
        "n": len(rows),
        "tf": rows[-1]["t"],
        "s_end": rows[-1]["s"],
        "moving_tf": moving[-1]["t"] if moving else 0.0,
        "moving_n": len(moving),
        "v_max": max((r["v"] for r in moving), default=0.0),
        "v_mean_moving": sum((r["v"] for r in moving), 0.0) / len(moving) if moving else 0.0,
        "phi_max": max((abs(r["phi"]) for r in rows), default=0.0),
        "high_phi_n": len(high_phi),
        "high_phi_v_max": max((r["v"] for r in high_phi), default=0.0),
        "high_phi_v_mean": sum((r["v"] for r in high_phi), 0.0) / len(high_phi) if high_phi else 0.0,
        "low_phi_v_max": max((r["v"] for r in low_phi_moving), default=0.0),
        "low_phi_v_mean": sum((r["v"] for r in low_phi_moving), 0.0) / len(low_phi_moving) if low_phi_moving else 0.0,
    }
    return summary


def main():
    if not V1_CSV.exists() or not V2_CSV.exists():
        raise FileNotFoundError("trajectory_v1.csv or trajectory_v2.csv is missing")

    v1 = load_csv(V1_CSV)
    v2 = load_csv(V2_CSV)

    s1 = summarize(v1)
    s2 = summarize(v2)
    path_max, path_mean = nearest_path_distance(v2, v1)

    fig, axes = plt.subplots(2, 2, figsize=(16, 11))

    ax = axes[0, 0]
    ax.set_title("XY Path")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.plot([r["x"] for r in v1], [r["y"] for r in v1], label="V1 LIOM", lw=1.8)
    ax.plot([r["x"] for r in v2], [r["y"] for r in v2], label="V2 ECC", lw=1.8)
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.set_title("Velocity-Time")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("v (m/s)")
    ax.plot([r["t"] for r in v1], [r["v"] for r in v1], label="V1 LIOM", lw=1.8)
    ax.plot([r["t"] for r in v2], [r["v"] for r in v2], label="V2 ECC", lw=1.8)
    ax.axhline(V_HALF, color="red", linestyle="--", linewidth=1.0, label="vhalf = 0.125")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.set_title("Velocity-Station")
    ax.set_xlabel("s (m)")
    ax.set_ylabel("v (m/s)")
    ax.plot([r["s"] for r in v1], [r["v"] for r in v1], label="V1 LIOM", lw=1.8)
    ax.plot([r["s"] for r in v2], [r["v"] for r in v2], label="V2 ECC", lw=1.8)
    ax.axhline(V_HALF, color="red", linestyle="--", linewidth=1.0, label="vhalf = 0.125")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.set_title("|phi|-Time")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("|phi| (rad)")
    ax.plot([r["t"] for r in v1], [abs(r["phi"]) for r in v1], label="V1 LIOM", lw=1.8)
    ax.plot([r["t"] for r in v2], [abs(r["phi"]) for r in v2], label="V2 ECC", lw=1.8)
    ax.axhline(PHI_THRESHOLD, color="red", linestyle="--", linewidth=1.0,
               label="0.5 * phimax")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=160, bbox_inches="tight")
    plt.close()

    print(f"Saved: {OUT_PNG}")
    print()
    print("V1 summary")
    print(f"  points={s1['n']}, moving_points={s1['moving_n']}")
    print(f"  tf={s1['tf']:.3f}s, moving_tf={s1['moving_tf']:.3f}s, s_end={s1['s_end']:.3f}m")
    print(f"  v_max={s1['v_max']:.6f}, moving_v_mean={s1['v_mean_moving']:.6f}")
    print(f"  high_phi_pts={s1['high_phi_n']}, high_phi_v_max={s1['high_phi_v_max']:.6f}, high_phi_v_mean={s1['high_phi_v_mean']:.6f}")
    print()
    print("V2 summary")
    print(f"  points={s2['n']}, moving_points={s2['moving_n']}")
    print(f"  tf={s2['tf']:.3f}s, moving_tf={s2['moving_tf']:.3f}s, s_end={s2['s_end']:.3f}m")
    print(f"  v_max={s2['v_max']:.6f}, moving_v_mean={s2['v_mean_moving']:.6f}")
    print(f"  high_phi_pts={s2['high_phi_n']}, high_phi_v_max={s2['high_phi_v_max']:.6f}, high_phi_v_mean={s2['high_phi_v_mean']:.6f}")
    print(f"  low_phi_v_max={s2['low_phi_v_max']:.6f}, low_phi_v_mean={s2['low_phi_v_mean']:.6f}")
    print()
    print("Path difference")
    print(f"  V2 to V1 nearest-path max_dist={path_max:.6f}m, mean_dist={path_mean:.6f}m")


if __name__ == "__main__":
    main()
