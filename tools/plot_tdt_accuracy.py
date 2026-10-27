#!/usr/bin/env python3
"""Reproduce T-DT navigation accuracy figures from recorded simulation results."""

from pathlib import Path
import argparse
import math
import re
from datetime import datetime, timezone

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA = ROOT / "docs" / "tdt_accuracy"
COLORS = {"Point-LIO": "#2563eb", "Adaptive-LIO": "#059669", "ZLIO": "#dc2626"}


def read_runs(folder):
    runs = pd.read_csv(folder / "navigation_runs.csv")
    required = ["frontend", "trusted_samples", "displacement_m", "median_xy_m",
                "p95_xy_m", "median_yaw_rad", "p95_yaw_rad", "final_state"]
    if runs[required].isna().any().any():
        raise ValueError("Navigation runs contain missing required values")
    if not (runs.p95_xy_m >= runs.median_xy_m).all():
        raise ValueError("XY 95th percentile is below its median")
    if not (runs.p95_yaw_rad >= runs.median_yaw_rad).all():
        raise ValueError("Yaw 95th percentile is below its median")
    for _, row in runs.iterrows():
        if row.frontend == "Point-LIO":
            continue  # Historical run has only its documented aggregate, not a copied verifier log.
        log = (folder / row.source).read_text(encoding="utf-8").splitlines()[0]
        for key in ("trusted_samples", "median_xy_m", "p95_xy_m",
                    "median_yaw_rad", "p95_yaw_rad", "trust_fraction"):
            log_key = "samples" if key == "trusted_samples" else key
            match = re.search(rf"(?:^| ){log_key}=([0-9.]+)", log)
            if not match or not math.isclose(float(match.group(1)), float(row[key]), abs_tol=1e-9):
                raise ValueError(f"{row.frontend}: {key} differs from {row.source}")
    return runs


def overview(runs):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2), gridspec_kw={"width_ratios": [1.3, 1.0, 0.9]})
    y = np.arange(len(runs))[::-1]
    labels = list(runs.frontend)
    colors = [COLORS[name] for name in labels]
    for i, row in runs.iterrows():
        c = colors[i]
        median_cm = row.median_xy_m * 100
        p95_cm = row.p95_xy_m * 100
        axes[0].plot([median_cm, p95_cm], [y[i], y[i]], color=c, lw=4, solid_capstyle="round")
        axes[0].scatter([median_cm], [y[i]], color=c, marker="o", s=70, zorder=3)
        axes[0].scatter([p95_cm], [y[i]], facecolor="white", edgecolor=c, marker="o", s=70, lw=2, zorder=3)
        axes[0].text(p95_cm * 1.12, y[i], f"{p95_cm:g}", va="center", fontsize=10)
    axes[0].set_xscale("log")
    axes[0].set_xlim(0.25, 360)
    axes[0].set_yticks(y, labels)
    axes[0].set_xlabel("XY error (cm, logarithmic)")
    axes[0].set_title("Position: median ● to 95th percentile ○")
    axes[0].grid(axis="x", which="both", alpha=.25)

    for i, row in runs.iterrows():
        med = math.degrees(row.median_yaw_rad)
        p95 = math.degrees(row.p95_yaw_rad)
        axes[1].plot([med, p95], [y[i], y[i]], color=colors[i], lw=4, solid_capstyle="round")
        axes[1].scatter([med], [y[i]], color=colors[i], s=70, zorder=3)
        axes[1].scatter([p95], [y[i]], facecolor="white", edgecolor=colors[i], s=70, lw=2, zorder=3)
        axes[1].text(p95 + .13, y[i], f"{p95:.2f}°", va="center", fontsize=10)
    axes[1].set_yticks(y, [""] * len(y))
    axes[1].set_xlim(0, 5.7)
    axes[1].set_xlabel("Absolute yaw error (degrees)")
    axes[1].set_title("Heading: median ● to 95th percentile ○")
    axes[1].grid(axis="x", alpha=.25)

    for i, row in runs.iterrows():
        if pd.notna(row.trust_fraction):
            axes[2].barh(y[i], 100 * row.trust_fraction, height=.5, color=colors[i])
            axes[2].text(100 * row.trust_fraction + 1.5, y[i],
                         f"{100 * row.trust_fraction:.1f}%", va="center", fontsize=10)
        else:
            axes[2].text(2, y[i], "not recorded", va="center", color="0.4")
    axes[2].set_yticks(y, [""] * len(y))
    axes[2].set_xlim(0, 116)
    axes[2].set_xlabel("Trusted sample fraction (%)")
    axes[2].set_title("Localization availability")
    axes[2].grid(axis="x", alpha=.25)
    fig.suptitle("T-DT simulated navigation: recorded acceptance runs", fontsize=16, fontweight="bold")
    fig.text(.02, .025,
             "One run per frontend; path displacement: Point-LIO 0.736 m, Adaptive-LIO 0.749 m, ZLIO 1.393 m. "
             "Error percentiles use trusted, time-matched samples only.\n"
             "Point-LIO trust fraction was not logged; ZLIO ended LOST, so its error dots exclude untrusted intervals.",
             fontsize=8, color="0.35")
    fig.tight_layout(rect=[0, .10, 1, .92])
    return fig


def diagnostic(folder):
    frame = pd.read_csv(folder / "zlio_diagnostic.csv")
    t = frame.elapsed.to_numpy()
    truth = frame[["truth_x", "truth_y"]].to_numpy()
    truth_t = frame.truth_t.to_numpy()
    errors = {}
    aligned = {}
    for name in ("front", "backend"):
        xy = frame[[f"{name}_x", f"{name}_y"]].to_numpy()
        stamp_gap = np.abs(truth_t - frame[f"{name}_t"].to_numpy())
        valid = np.isfinite(truth).all(axis=1) & np.isfinite(xy).all(axis=1) & (stamp_gap <= .05)
        baseline = valid & (t >= 8) & (t <= 12)
        if baseline.sum() < 20:
            raise ValueError(f"Too few stationary baseline samples for {name}")
        offset = np.median(truth[baseline] - xy[baseline], axis=0)
        aligned[name] = xy + offset
        errors[name] = np.where(valid, np.linalg.norm(aligned[name] - truth, axis=1), np.nan)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.7), gridspec_kw={"width_ratios": [1.25, 1]})
    state = frame.state.fillna("").str.extract(r"status=([^;]+)")[0].fillna("").to_numpy()
    lost_times = t[(state == "LOST") & (t >= 8)]
    first_lost = lost_times[0] if len(lost_times) else 36
    states = {"NORMAL": "#dcfce7", "OBS_DEGRADED": "#fef3c7", "SUSPECT": "#fed7aa", "LOST": "#fecaca"}
    interval_start = 0
    for i in range(1, len(state) + 1):
        if i == len(state) or state[i] != state[interval_start]:
            if state[interval_start] in states and t[i - 1] >= 8 and t[interval_start] <= 36:
                axes[0].axvspan(max(t[interval_start], 8), min(t[i - 1] + .1, 36),
                                color=states[state[interval_start]], alpha=.65, lw=0)
            interval_start = i
    use = (t >= 8) & (t < first_lost)
    axes[0].plot(t[use], errors["front"][use], label="ZLIO frontend", color="#2563eb", lw=1.8)
    axes[0].plot(t[use], errors["backend"][use], label="T-DT backend", color="#dc2626", lw=1.8)
    axes[0].axhline(.5, color="0.4", ls="--", lw=1)
    axes[0].set_yscale("log")
    axes[0].set_ylim(.003, 50)
    axes[0].set_xlim(8, 36)
    axes[0].set_xlabel("Elapsed time (s)")
    axes[0].set_ylabel("Baseline-aligned XY displacement error (m)")
    axes[0].set_title("Error grows before the backend becomes LOST")
    axes[0].legend(loc="upper left")
    axes[0].grid(which="both", alpha=.2)
    axes[0].text(35.6, 34, "LOST", ha="right", color="#991b1b", fontsize=9)

    segment = (t >= 12) & (t <= 24)
    axes[1].plot(truth[segment, 0], truth[segment, 1], color="black", lw=2.5, label="Gazebo truth")
    for name, color, label in (("front", "#2563eb", "ZLIO frontend"),
                               ("backend", "#dc2626", "T-DT backend")):
        xy = aligned[name]
        axes[1].plot(xy[segment, 0], xy[segment, 1], color=color, lw=1.8, label=label)
    axes[1].set_aspect("equal", adjustable="datalim")
    axes[1].set_xlabel("Map X (m; frontend aligned at start)")
    axes[1].set_ylabel("Map Y (m)")
    axes[1].set_title("Same diagnostic capture, 12–24 s")
    axes[1].legend(loc="best")
    axes[1].grid(alpha=.2)
    fig.suptitle("ZLIO navigation failure: separate 10 Hz diagnostic capture", fontsize=16, fontweight="bold")
    fig.text(.02, .025,
             "Latest-message snapshots; rows kept only when each pose stamp is within 50 ms of truth. "
             "XY origins aligned by median of stationary 8–12 s.\n"
             "These are drift curves, not absolute-pose accuracy. "
             "State background: green NORMAL, yellow degraded, orange SUSPECT, red LOST.",
             fontsize=8, color="0.35")
    fig.tight_layout(rect=[0, .10, 1, .92])
    return fig, frame, errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    args = parser.parse_args()
    folder = args.data_dir
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False})
    runs = read_runs(folder)
    first = overview(runs)
    second, frame, errors = diagnostic(folder)
    first.savefig(folder / "navigation_accuracy.png", dpi=180)
    second.savefig(folder / "zlio_failure_diagnostic.png", dpi=180)
    created = datetime(2026, 10, 27, tzinfo=timezone.utc)
    with PdfPages(folder / "tdt_accuracy_charts.pdf",
                  metadata={"CreationDate": created, "ModDate": created}) as pdf:
        pdf.savefig(first)
        pdf.savefig(second)
    for target in ("front", "backend"):
        t = frame.elapsed.to_numpy()
        err = errors[target]
        valid = (t >= 8) & (t <= 24) & np.isfinite(err)
        for threshold in (.5, 1.0):
            first_crossing = t[valid & (err > threshold)]
            print(f"{target} first >{threshold:g} m: "
                  f"{first_crossing[0]:.2f} s" if len(first_crossing) else "none")
    print(f"Wrote figures to {folder}")


if __name__ == "__main__":
    main()
