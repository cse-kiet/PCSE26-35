#!/usr/bin/env python3
"""
visualise_traces.py
Reads fcd_traces.csv and produces:
  1. trajectory_plot.png  — full path of every vehicle over 100 s
  2. heatmap.png          — spatial density of vehicle presence
  3. speed_profile.png    — speed over time per vehicle

Usage:
    python3 visualise_traces.py
"""

import os
import csv
import collections
import matplotlib
matplotlib.use("Agg")          # headless — no display needed
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH   = os.path.join(SCRIPT_DIR, "fcd_traces.csv")
OUT_DIR    = SCRIPT_DIR


def load_csv(path):
    vehicles = collections.defaultdict(lambda: {"x": [], "y": [], "t": [], "speed": []})
    with open(path) as f:
        for row in csv.DictReader(f):
            vid = int(row["vehicle_id"])
            vehicles[vid]["x"].append(float(row["x"]))
            vehicles[vid]["y"].append(float(row["y"]))
            vehicles[vid]["t"].append(float(row["time"]))
            vehicles[vid]["speed"].append(float(row["speed"]))
    return vehicles


def plot_trajectories(vehicles):
    fig, ax = plt.subplots(figsize=(8, 8))

    cmap   = matplotlib.colormaps.get_cmap("tab20").resampled(len(vehicles))
    colors = [cmap(i) for i in range(len(vehicles))]

    for i, (vid, d) in enumerate(sorted(vehicles.items())):
        ax.plot(d["x"], d["y"], lw=1.2, alpha=0.75, color=colors[i], label=f"V{vid}")
        # Mark start and end
        ax.plot(d["x"][0],  d["y"][0],  "o", color=colors[i], ms=5)
        ax.plot(d["x"][-1], d["y"][-1], "s", color=colors[i], ms=5)

    # Draw intersection roads
    ax.axhline(0, color="gray", lw=0.8, ls="--", alpha=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.4)
    ax.set_xlim(-320, 320)
    ax.set_ylim(-320, 320)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"Vehicle Trajectories — {len(vehicles)} vehicles, 100 s")
    ax.set_aspect("equal")
    ax.legend(fontsize=6, ncol=4, loc="upper right")
    ax.grid(True, alpha=0.3)

    out = os.path.join(OUT_DIR, "trajectory_plot.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


def plot_heatmap(vehicles):
    all_x, all_y = [], []
    for d in vehicles.values():
        all_x.extend(d["x"])
        all_y.extend(d["y"])

    fig, ax = plt.subplots(figsize=(7, 7))
    h = ax.hist2d(all_x, all_y, bins=60, range=[[-310,310],[-310,310]],
                  cmap="hot_r", density=True)
    fig.colorbar(h[3], ax=ax, label="Relative density")
    ax.axhline(0, color="cyan", lw=0.8, ls="--", alpha=0.6)
    ax.axvline(0, color="cyan", lw=0.8, ls="--", alpha=0.6)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("Spatial Density Heatmap — Vehicle Presence")
    ax.set_aspect("equal")

    out = os.path.join(OUT_DIR, "heatmap.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


def plot_speed_profiles(vehicles):
    fig, ax = plt.subplots(figsize=(10, 5))
    cmap = matplotlib.colormaps.get_cmap("tab20").resampled(len(vehicles))

    for i, (vid, d) in enumerate(sorted(vehicles.items())):
        ax.plot(d["t"], d["speed"], lw=0.9, alpha=0.7,
                color=cmap(i), label=f"V{vid}")

    ax.set_xlabel("Time (s)"); ax.set_ylabel("Speed (m/s)")
    ax.set_title("Speed Profiles — All Vehicles")
    ax.legend(fontsize=6, ncol=4, loc="upper right")
    ax.grid(True, alpha=0.3)

    out = os.path.join(OUT_DIR, "speed_profile.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


def main():
    print(f"Loading {CSV_PATH} ...")
    vehicles = load_csv(CSV_PATH)
    print(f"  {len(vehicles)} vehicles found\n")

    print("Generating plots...")
    plot_trajectories(vehicles)
    plot_heatmap(vehicles)
    plot_speed_profiles(vehicles)
    print("\nDone. Open the PNG files in src/sumo/ to view.")


if __name__ == "__main__":
    main()