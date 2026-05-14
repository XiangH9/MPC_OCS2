#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path: Path):
    if not path.exists():
        return []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = []
        for row in reader:
            parsed = {}
            for key, value in row.items():
                try:
                    parsed[key] = float(value)
                except (TypeError, ValueError):
                    parsed[key] = value
            rows.append(parsed)
        return rows


def series(rows, key):
    return [row[key] for row in rows if key in row]


def ensure_output_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def plot_estimator(rows, output_dir: Path):
    if not rows:
        return

    t = series(rows, "time")
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(t, series(rows, "pos_x"), label="pos_x")
    axes[0].plot(t, series(rows, "pos_y"), label="pos_y")
    axes[0].plot(t, series(rows, "pos_z"), label="pos_z")
    axes[0].set_title("Estimator Position")
    axes[0].set_ylabel("m")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, series(rows, "vel_x"), label="vel_x")
    axes[1].plot(t, series(rows, "vel_y"), label="vel_y")
    axes[1].plot(t, series(rows, "vel_z"), label="vel_z")
    axes[1].set_title("Estimator Velocity")
    axes[1].set_ylabel("m/s")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(t, series(rows, "yaw"), label="yaw")
    axes[2].plot(t, series(rows, "gyro_z"), label="gyro_z")
    axes[2].set_title("Estimator Yaw / Gyro")
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylabel("rad or rad/s")
    axes[2].legend()
    axes[2].grid(True)

    fig.tight_layout()
    fig.savefig(output_dir / "estimator_summary.png", dpi=150)
    plt.close(fig)


def plot_trotting(rows, output_dir: Path):
    if not rows:
        return

    t = series(rows, "time")
    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)

    axes[0].plot(t, series(rows, "pcd_x"), label="pcd_x")
    axes[0].plot(t, series(rows, "body_x"), label="body_x")
    axes[0].plot(t, series(rows, "pcd_y"), label="pcd_y")
    axes[0].plot(t, series(rows, "body_y"), label="body_y")
    axes[0].set_title("Body Position Tracking")
    axes[0].set_ylabel("m")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, series(rows, "target_vx"), label="target_vx")
    axes[1].plot(t, series(rows, "body_vx"), label="body_vx")
    axes[1].plot(t, series(rows, "target_vy"), label="target_vy")
    axes[1].plot(t, series(rows, "body_vy"), label="body_vy")
    axes[1].set_title("Velocity Tracking")
    axes[1].set_ylabel("m/s")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(t, series(rows, "pos_err_x"), label="pos_err_x")
    axes[2].plot(t, series(rows, "pos_err_y"), label="pos_err_y")
    axes[2].plot(t, series(rows, "vel_err_x"), label="vel_err_x")
    axes[2].plot(t, series(rows, "vel_err_y"), label="vel_err_y")
    axes[2].set_title("Tracking Error")
    axes[2].legend()
    axes[2].grid(True)

    axes[3].plot(t, series(rows, "dd_pcd_x"), label="dd_pcd_x")
    axes[3].plot(t, series(rows, "dd_pcd_y"), label="dd_pcd_y")
    axes[3].plot(t, series(rows, "d_wbd_x"), label="d_wbd_x")
    axes[3].plot(t, series(rows, "d_wbd_y"), label="d_wbd_y")
    axes[3].set_title("Controller Output")
    axes[3].set_xlabel("time (s)")
    axes[3].legend()
    axes[3].grid(True)

    fig.tight_layout()
    fig.savefig(output_dir / "trotting_summary.png", dpi=150)
    plt.close(fig)


def plot_balance(rows, output_dir: Path):
    if not rows:
        return

    sample = series(rows, "sample")
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(sample, series(rows, "bd_x"), label="bd_x")
    axes[0].plot(sample, series(rows, "bd_y"), label="bd_y")
    axes[0].plot(sample, series(rows, "bd_z"), label="bd_z")
    axes[0].set_title("Desired Generalized Force")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(sample, series(rows, "force_fl_z"), label="FL_z")
    axes[1].plot(sample, series(rows, "force_fr_z"), label="FR_z")
    axes[1].plot(sample, series(rows, "force_rl_z"), label="RL_z")
    axes[1].plot(sample, series(rows, "force_rr_z"), label="RR_z")
    axes[1].set_title("Normal Contact Forces")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(sample, series(rows, "dd_pcd_x"), label="dd_pcd_x")
    axes[2].plot(sample, series(rows, "dd_pcd_y"), label="dd_pcd_y")
    axes[2].plot(sample, series(rows, "d_wbd_x"), label="d_wbd_x")
    axes[2].plot(sample, series(rows, "d_wbd_y"), label="d_wbd_y")
    axes[2].set_title("Balance Control Inputs")
    axes[2].set_xlabel("sample")
    axes[2].legend()
    axes[2].grid(True)

    fig.tight_layout()
    fig.savefig(output_dir / "balance_summary.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot phase1 CSV logs.")
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path("/home/xiangh9/xhros2/uni_ws/tmp/unitree_phase1_logs"),
        help="Directory containing estimator.csv, trotting.csv, balance.csv",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory to save png plots. Default: <log-dir>/plots",
    )
    args = parser.parse_args()

    output_dir = args.output_dir or (args.log_dir / "plots")
    ensure_output_dir(output_dir)

    estimator_rows = load_csv(args.log_dir / "estimator.csv")
    trotting_rows = load_csv(args.log_dir / "trotting.csv")
    balance_rows = load_csv(args.log_dir / "balance.csv")

    plot_estimator(estimator_rows, output_dir)
    plot_trotting(trotting_rows, output_dir)
    plot_balance(balance_rows, output_dir)

    print(f"Plots saved to: {output_dir}")


if __name__ == "__main__":
    main()
