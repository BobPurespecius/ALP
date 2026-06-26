#!/usr/bin/env python3

import argparse
import csv
import json
import re
import subprocess
import sys
from pathlib import Path


DEFAULT_VALUES = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]


def set_obstacles_inflation(param_file, value):
    text = param_file.read_text()
    pattern = r'(<param\s+name="grid_map/obstacles_inflation"\s+value=")([^"]+)("\s*/>)'
    new_text, count = re.subn(pattern, rf'\g<1>{value:.2f}\3', text)
    if count != 1:
        raise RuntimeError(f"expected one obstacles_inflation param in {param_file}, changed {count}")
    param_file.write_text(new_text)


def read_overall_summary(analysis_dir):
    summary_file = analysis_dir / "overall_summary.json"
    if not summary_file.exists():
        return {
            "total_runs": 0,
            "successful_runs": 0,
            "run_success_rate": 0.0,
            "mean_vehicle_success_rate": 0.0,
        }
    with summary_file.open() as f:
        return json.load(f)


def mean_csv_column(csv_file, column):
    if not csv_file.exists():
        return ""
    values = []
    with csv_file.open(newline="") as f:
        for row in csv.DictReader(f):
            raw = row.get(column, "")
            if raw == "":
                continue
            try:
                values.append(float(raw))
            except ValueError:
                pass
    if not values:
        return ""
    return sum(values) / len(values)


def run_batch(workspace, scene, runs, duration, startup_timeout, analysis_dir):
    cmd = [
        "./batch_run_experiments.sh",
        "--scene", scene,
        "--runs", str(runs),
        "--duration", str(duration),
        "--startup-timeout", str(startup_timeout),
        "--analysis-dir", str(analysis_dir),
    ]
    subprocess.run(cmd, cwd=workspace, check=True)


def write_results_csv(output_dir, rows):
    path = output_dir / "obstacles_inflation_sweep_summary.csv"
    columns = [
        "obstacles_inflation",
        "analysis_dir",
        "total_runs",
        "successful_runs",
        "run_success_rate",
        "mean_vehicle_success_rate",
        "mean_final_goal_error_m",
        "mean_min_obstacle_clearance_m",
        "mean_duration_s",
        "mean_rms_accel_mps2",
        "mean_max_accel_mps2",
        "mean_rms_jerk_mps3",
        "mean_max_jerk_mps3",
        "mean_mean_turn_angle_rad",
        "mean_max_turn_angle_rad",
        "mean_mean_control_delta",
        "mean_max_control_delta",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    return path


def plot_success_rate(output_dir, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    xs = [row["obstacles_inflation"] for row in rows]
    ys = [row["run_success_rate"] * 100.0 for row in rows]
    vehicle_ys = [row["mean_vehicle_success_rate"] * 100.0 for row in rows]

    fig, ax = plt.subplots(figsize=(8, 4.8))
    ax.plot(xs, ys, marker="o", linewidth=2, label="整轮成功率")
    ax.plot(xs, vehicle_ys, marker="s", linewidth=2, label="单机成功率")
    ax.set_xlabel("障碍物膨胀距离（m）")
    ax.set_ylabel("成功率（%）")
    ax.set_ylim(-2, 102)
    ax.set_xticks(xs)
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()

    path = output_dir / "success_rate_vs_obstacles_inflation.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_metric_group(output_dir, rows, filename, title, metrics, ylabel):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    xs = [row["obstacles_inflation"] for row in rows]
    fig, ax = plt.subplots(figsize=(8, 4.8))
    for key, label in metrics:
        ys = [row[key] for row in rows]
        ax.plot(xs, ys, marker="o", linewidth=2, label=label)
    ax.set_xlabel("障碍物膨胀距离（m）")
    ax.set_ylabel(ylabel)
    ax.set_xticks(xs)
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()

    path = output_dir / filename
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_smoothness(output_dir, rows):
    return [
        plot_metric_group(
            output_dir,
            rows,
            "smoothness_accel_vs_obstacles_inflation.png",
            "Acceleration Smoothness vs Obstacle Inflation",
            [
                ("mean_rms_accel_mps2", "平均 RMS 加速度"),
                ("mean_max_accel_mps2", "平均最大加速度"),
            ],
            "加速度（m/s^2）",
        ),
        plot_metric_group(
            output_dir,
            rows,
            "smoothness_jerk_vs_obstacles_inflation.png",
            "Jerk Smoothness vs Obstacle Inflation",
            [
                ("mean_rms_jerk_mps3", "平均 RMS jerk"),
                ("mean_max_jerk_mps3", "平均最大 jerk"),
            ],
            "jerk（m/s^3）",
        ),
        plot_metric_group(
            output_dir,
            rows,
            "smoothness_control_turn_vs_obstacles_inflation.png",
            "Control/Turn Smoothness vs Obstacle Inflation",
            [
                ("mean_mean_control_delta", "平均控制增量"),
                ("mean_max_control_delta", "平均最大控制增量"),
                ("mean_max_turn_angle_rad", "平均最大转角"),
            ],
            "控制增量 / 转角（rad）",
        ),
    ]


def parse_values(raw_values):
    if not raw_values:
        return DEFAULT_VALUES
    values = []
    for raw in raw_values.split(","):
        raw = raw.strip()
        if not raw:
            continue
        values.append(float(raw))
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="platform")
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--duration", type=int, default=70)
    parser.add_argument("--startup-timeout", type=int, default=180)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--values", default="")
    parser.add_argument("--restore-value", type=float, default=None)
    args = parser.parse_args()

    workspace = Path(__file__).resolve().parent
    param_file = workspace / "src/ego-planner/src/planner/plan_manage/launch/advanced_param.xml"
    output_dir = Path(args.output_dir) if args.output_dir else workspace / "analysis" / "obstacles_inflation_sweep"
    output_dir.mkdir(parents=True, exist_ok=True)

    values = parse_values(args.values)
    original_text = param_file.read_text()
    rows = []
    try:
        for value in values:
            label = f"inflation_{value:.2f}".replace(".", "p")
            analysis_dir = output_dir / label
            print(f"\n=== obstacles_inflation={value:.2f}, analysis={analysis_dir} ===", flush=True)
            set_obstacles_inflation(param_file, value)

            if not (args.skip_existing and (analysis_dir / "overall_summary.json").exists()):
                run_batch(workspace, args.scene, args.runs, args.duration, args.startup_timeout, analysis_dir)

            summary = read_overall_summary(analysis_dir)
            run_csv = analysis_dir / "run_summary.csv"
            row = {
                "obstacles_inflation": value,
                "analysis_dir": str(analysis_dir),
                "total_runs": int(summary.get("total_runs", 0)),
                "successful_runs": int(summary.get("successful_runs", 0)),
                "run_success_rate": float(summary.get("run_success_rate", 0.0)),
                "mean_vehicle_success_rate": float(summary.get("mean_vehicle_success_rate", 0.0)),
                "mean_final_goal_error_m": mean_csv_column(run_csv, "mean_final_goal_error_m"),
                "mean_min_obstacle_clearance_m": mean_csv_column(run_csv, "mean_min_obstacle_clearance_m"),
                "mean_duration_s": mean_csv_column(run_csv, "mean_duration_s"),
                "mean_rms_accel_mps2": mean_csv_column(run_csv, "mean_rms_accel_mps2"),
                "mean_max_accel_mps2": mean_csv_column(run_csv, "mean_max_accel_mps2"),
                "mean_rms_jerk_mps3": mean_csv_column(run_csv, "mean_rms_jerk_mps3"),
                "mean_max_jerk_mps3": mean_csv_column(run_csv, "mean_max_jerk_mps3"),
                "mean_mean_turn_angle_rad": mean_csv_column(run_csv, "mean_mean_turn_angle_rad"),
                "mean_max_turn_angle_rad": mean_csv_column(run_csv, "mean_max_turn_angle_rad"),
                "mean_mean_control_delta": mean_csv_column(run_csv, "mean_mean_control_delta"),
                "mean_max_control_delta": mean_csv_column(run_csv, "mean_max_control_delta"),
            }
            rows.append(row)
            write_results_csv(output_dir, rows)
            plot_success_rate(output_dir, rows)
            plot_smoothness(output_dir, rows)
    finally:
        if args.restore_value is None:
            param_file.write_text(original_text)
        else:
            set_obstacles_inflation(param_file, args.restore_value)

    csv_path = write_results_csv(output_dir, rows)
    plot_path = plot_success_rate(output_dir, rows)
    smoothness_paths = plot_smoothness(output_dir, rows)
    print(f"\nsummary_csv={csv_path}")
    print(f"plot={plot_path}")
    for path in smoothness_paths:
        print(f"smoothness_plot={path}")


if __name__ == "__main__":
    sys.exit(main())
