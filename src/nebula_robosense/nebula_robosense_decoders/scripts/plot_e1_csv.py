#!/usr/bin/env python3

import csv
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")


def load_points(csv_path: Path):
    xs = []
    ys = []
    zs = []
    intensities = []
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row["x"])
            y = float(row["y"])
            z = float(row["z"])
            intensity = float(row["intensity"])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            xs.append(x)
            ys.append(y)
            zs.append(z)
            intensities.append(intensity)
    return xs, ys, zs, intensities


def save_scatter(x, y, colors, x_label: str, y_label: str, title: str, output_path: Path):
    from matplotlib import pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    scatter = ax.scatter(x, y, c=colors, s=3, cmap="viridis", linewidths=0, alpha=0.9)
    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    fig.colorbar(scatter, ax=ax, label="intensity")
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def main() -> int:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <input_csv>", file=sys.stderr)
        return 2

    csv_path = Path(sys.argv[1])
    if not csv_path.is_file():
        print(f"CSV not found: {csv_path}", file=sys.stderr)
        return 1

    xs, ys, zs, intensities = load_points(csv_path)
    if not xs:
        print(f"No valid points in {csv_path}", file=sys.stderr)
        return 1

    xy_path = csv_path.with_name(csv_path.stem + "_xy.png")
    xz_path = csv_path.with_name(csv_path.stem + "_xz.png")

    save_scatter(xs, ys, intensities, "x [m]", "y [m]", "Robosense E1 XY Projection", xy_path)
    save_scatter(xs, zs, intensities, "x [m]", "z [m]", "Robosense E1 XZ Projection", xz_path)

    print(f"points={len(xs)}")
    print(f"xy={xy_path}")
    print(f"xz={xz_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
