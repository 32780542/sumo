import argparse
import csv
import glob
import json
import os
import sys

try:
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    import numpy as np
except ModuleNotFoundError as exc:
    missing_name = exc.name or "numpy/matplotlib"
    print(
        f"缺少绘图依赖：{missing_name}\n"
        "请在当前 Python 环境中运行：python -m pip install numpy matplotlib",
        file=sys.stderr,
    )
    raise SystemExit(1) from exc


BASE_DIR = os.path.dirname(__file__)
CSV_DIR = os.path.join(BASE_DIR, "csv")
LOG_DIR = os.path.join(BASE_DIR, "logs")
MATLAB_STYLE_SPEED_CMAP = LinearSegmentedColormap.from_list(
    "matlab_speed_yellow_red",
    [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0)],
)


def load_vehicle_trace_rows(csv_path):
    """读取车辆轨迹 CSV。"""
    with open(csv_path, newline="", encoding="utf-8-sig") as csv_file:
        return list(csv.DictReader(csv_file))


def parse_sequence(value):
    """解析 CSV 单元格中的 JSON 数组。"""
    return json.loads(value) if value else []


def find_latest_trace_csv(csv_dir=CSV_DIR):
    """查找最新的车辆轨迹 CSV。"""
    pattern = os.path.join(csv_dir, "vehicle_traces_*.csv")
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(f"No vehicle trace CSV found in {csv_dir}")
    return max(files, key=os.path.getmtime)


def build_speed_heatmap_grid(
    rows,
    road_length=1200,
    space_bin_size=50,
    max_speed=30,
    target_time_bins=30,
):
    """把车辆轨迹投影到时间-空间网格，并按 MATLAB 脚本逻辑进行时间平滑。"""
    space_edges = np.arange(0, road_length + space_bin_size, space_bin_size)
    space_bins = len(space_edges) - 1
    max_steps = 0
    parsed_rows = []

    for row in rows:
        x_sequence = parse_sequence(row["X 坐标序列"])
        speed_sequence = parse_sequence(row["速度序列 (m/s)"])
        sample_count = min(len(x_sequence), len(speed_sequence))
        if sample_count <= 0:
            continue
        parsed_rows.append((x_sequence[:sample_count], speed_sequence[:sample_count]))
        max_steps = max(max_steps, sample_count)

    if max_steps == 0:
        return (
            np.full((1, space_bins), max_speed, dtype=float),
            np.array([0, 1]),
            space_edges,
        )

    speed_sums = np.zeros((max_steps, space_bins), dtype=float)
    speed_counts = np.zeros((max_steps, space_bins), dtype=float)

    for x_sequence, speed_sequence in parsed_rows:
        for step_index, (x_position, speed) in enumerate(zip(x_sequence, speed_sequence)):
            if x_position < 0 or x_position > road_length:
                continue
            space_index = min(int(x_position // space_bin_size), space_bins - 1)
            speed_sums[step_index, space_index] += speed
            speed_counts[step_index, space_index] += 1

    speed_grid = np.divide(
        speed_sums,
        speed_counts,
        out=np.full_like(speed_sums, max_speed),
        where=speed_counts > 0,
    )

    time_bin_size = max(1, int(np.ceil(max_steps / max(1, target_time_bins))))
    smoothed_rows = []
    time_edges = [0]
    for start in range(0, max_steps, time_bin_size):
        end = min(start + time_bin_size, max_steps)
        window_values = speed_grid[start:end, :]
        window_counts = speed_counts[start:end, :]
        window_result = np.full(space_bins, max_speed, dtype=float)
        for space_index in range(space_bins):
            occupied_values = window_values[window_counts[:, space_index] > 0, space_index]
            if occupied_values.size > 0:
                window_result[space_index] = float(np.mean(occupied_values))
        smoothed_rows.append(window_result)
        time_edges.append(end)

    return np.vstack(smoothed_rows), np.array(time_edges), space_edges


def plot_speed_heatmap(
    heatmap,
    time_edges,
    space_edges,
    output_path,
    title="Space-Time Speed Heatmap",
    max_speed=30,
):
    """绘制接近 MATLAB fill 风格的时空车速热力图，速度越低越红。"""
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fig, ax = plt.subplots(figsize=(14, 8))
    mesh = ax.pcolormesh(
        time_edges,
        space_edges,
        heatmap.T,
        cmap=MATLAB_STYLE_SPEED_CMAP,
        vmin=0,
        vmax=max_speed,
        shading="auto",
    )
    ax.set_xlabel("Time bin (simulation step)")
    ax.set_ylabel("Position along road (m)")
    ax.set_title(title)
    ax.set_xlim(time_edges[0], time_edges[-1])
    ax.set_ylim(space_edges[0], space_edges[-1])
    ax.set_facecolor("white")
    ax.grid(False)
    colorbar = fig.colorbar(mesh, ax=ax)
    colorbar.set_label("Average speed (m/s)")
    for spine in ax.spines.values():
        spine.set_linewidth(1.0)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def build_default_output_path(csv_path):
    """根据输入 CSV 名生成默认热力图输出路径。"""
    stem = os.path.splitext(os.path.basename(csv_path))[0]
    return os.path.join(LOG_DIR, f"{stem}_speed_heatmap.png")


def main():
    parser = argparse.ArgumentParser(description="Plot a space-time average-speed heatmap.")
    parser.add_argument("--csv", dest="csv_path", default=None, help="Vehicle trace CSV path.")
    parser.add_argument("--output", default=None, help="Output PNG path.")
    parser.add_argument("--road-length", type=float, default=1200.0)
    parser.add_argument("--space-bin-size", type=float, default=50.0)
    parser.add_argument("--max-speed", type=float, default=30.0)
    parser.add_argument("--time-bins", type=int, default=30)
    args = parser.parse_args()

    csv_path = args.csv_path or find_latest_trace_csv()
    output_path = args.output or build_default_output_path(csv_path)
    rows = load_vehicle_trace_rows(csv_path)
    heatmap, time_edges, space_edges = build_speed_heatmap_grid(
        rows,
        road_length=args.road_length,
        space_bin_size=args.space_bin_size,
        max_speed=args.max_speed,
        target_time_bins=args.time_bins,
    )
    plot_speed_heatmap(
        heatmap,
        time_edges,
        space_edges,
        output_path,
        title=os.path.basename(csv_path),
        max_speed=args.max_speed,
    )
    print(output_path)


if __name__ == "__main__":
    main()
