#!/usr/bin/env python3
"""Plot a recorded run: where the robot was told to go against where it went.

    python3 visualize_trajectory.py /shared/student_work/run.csv

Reads the CSV written by trajectory_analyzer.py and writes a PNG beside it.
Uses a non-interactive backend so it works in a container with no display.
"""

import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_run(path):
    """Read the analyzer's CSV into columns."""
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"{path} contains no samples")
    return {
        "time": [float(r["time_s"]) for r in rows],
        "ref_x": [float(r["ref_x"]) for r in rows],
        "ref_y": [float(r["ref_y"]) for r in rows],
        "actual_x": [float(r["actual_x"]) for r in rows],
        "actual_y": [float(r["actual_y"]) for r in rows],
        "error": [float(r["error_m"]) for r in rows],
    }


def plot_run(run, output_path):
    """Draw the path comparison and the error trace, and save a PNG."""
    fig, (path_ax, error_ax) = plt.subplots(1, 2, figsize=(13, 5.5))

    path_ax.plot(run["ref_x"], run["ref_y"], "--", linewidth=2, label="reference")
    path_ax.plot(run["actual_x"], run["actual_y"], linewidth=2, label="actual")
    path_ax.plot(run["actual_x"][0], run["actual_y"][0], "o",
                 markersize=9, label="start")
    path_ax.set_xlabel("x (m)")
    path_ax.set_ylabel("y (m)")
    path_ax.set_title("Path")
    path_ax.axis("equal")
    path_ax.grid(alpha=0.3)
    path_ax.legend()

    error_ax.plot(run["time"], run["error"], linewidth=1.6)
    error_ax.set_xlabel("time (s)")
    error_ax.set_ylabel("position error (m)")
    error_ax.set_title("Tracking error")
    error_ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=130)
    plt.close(fig)
    return output_path


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    csv_path = sys.argv[1]
    png_path = csv_path.rsplit(".", 1)[0] + ".png"
    plot_run(load_run(csv_path), png_path)
    print(f"wrote {png_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
