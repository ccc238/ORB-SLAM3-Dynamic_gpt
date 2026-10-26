#!/usr/bin/env python3
"""Create per-track B4-A count/state curves from b4_state_summary_*.csv."""
import argparse
import csv
from collections import defaultdict
from pathlib import Path


STATE_COLOR = {"P": "#f0ad4e", "S": "#2ca02c", "D": "#d62728"}


def number(row, name, default=0):
    try:
        return int(row.get(name, default))
    except (ValueError, TypeError):
        return default


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="b4_state_summary_*.csv")
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for B4 plots: " + str(exc))

    output = args.output_dir or args.csv.parent / "b4_state_plots"
    output.mkdir(parents=True, exist_ok=False)
    by_track = defaultdict(list)
    with args.csv.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row.get("reason_code") == "TRACK_TTL_EXPIRED":
                continue
            track = number(row, "b4_track_id", -1)
            if track >= 0:
                by_track[track].append(row)
    for track, rows in by_track.items():
        rows.sort(key=lambda row: number(row, "frame_id"))
        frames = [number(row, "frame_id") for row in rows]
        fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
        axes[0].plot(frames, [number(row, "n_d") for row in rows], label="N_D", color="#d62728")
        axes[0].plot(frames, [number(row, "n_s") for row in rows], label="N_S", color="#2ca02c")
        axes[0].plot(frames, [number(row, "n_r") for row in rows], label="N_R", color="#777777")
        axes[0].set_ylabel("consecutive evidence count")
        axes[0].legend(loc="upper right")
        axes[0].grid(alpha=0.25)
        states = [row.get("state", "P") for row in rows]
        axes[1].scatter(frames, [0] * len(rows), c=[STATE_COLOR.get(s, "#777777") for s in states], s=22)
        axes[1].set_yticks([])
        axes[1].set_xlabel("frame_id")
        axes[1].set_title("B4 state: P=provisional, S=static, D=dynamic")
        axes[1].grid(axis="x", alpha=0.25)
        fig.suptitle("B4-A temporal track {}".format(track))
        fig.tight_layout()
        fig.savefig(output / ("b4_track_{:04d}.png".format(track)), dpi=150)
        plt.close(fig)
    print("wrote {} track plots to {}".format(len(by_track), output))


if __name__ == "__main__":
    main()
