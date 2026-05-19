import csv
import json
import os
from datetime import datetime

import sim.config as cfg
def format_float_for_filename(value):
    """把浮点参数转成文件名安全格式。"""
    return f"{value:g}".replace(".", "p")


def build_vehicle_trace_filename(enable_swarm_rules, sim_seconds, flow_probability, timestamp=None):
    """生成包含测试时间、仿生开关、仿真时长和车流量的车辆轨迹文件名。"""
    timestamp = timestamp or datetime.now()
    swarm_label = "swarm_on" if enable_swarm_rules else "swarm_off"
    flow_label = format_float_for_filename(flow_probability)
    return (
        f"vehicle_traces_{timestamp:%Y%m%d_%H%M%S}_"
        f"{swarm_label}_{int(sim_seconds)}s_flow_{flow_label}.csv"
    )


def ensure_output_directories():
    """确保网络、日志和 CSV 输出目录存在。"""
    for directory in (cfg.NETWORK_DIR, cfg.LOG_DIR, cfg.CSV_DIR):
        os.makedirs(directory, exist_ok=True)


def make_log_path(filename):
    """生成日志输出文件路径。"""
    return os.path.join(cfg.LOG_DIR, filename)


def make_csv_path(filename):
    """生成 CSV 输出文件路径。"""
    return os.path.join(cfg.CSV_DIR, filename)


def append_vehicle_trace_sample(
    traces,
    veh_id,
    speed,
    x_position,
    y_position,
    angle,
    step_index,
    dt,
):
    """记录单车一个仿真步的轨迹样本，并由速度差计算加速度。"""
    trace = traces.setdefault(
        veh_id,
        {
            "vehicle_id": veh_id,
            "speed_sequence": [],
            "acceleration_sequence": [],
            "completed": 0,
            "x_sequence": [],
            "y_sequence": [],
            "enter_step": step_index,
            "angle_sequence": [],
            "last_speed": None,
        },
    )

    last_speed = trace["last_speed"]
    acceleration = 0.0 if last_speed is None else (speed - last_speed) / dt
    trace["speed_sequence"].append(float(speed))
    trace["acceleration_sequence"].append(float(acceleration))
    trace["x_sequence"].append(float(x_position))
    trace["y_sequence"].append(float(y_position))
    trace["angle_sequence"].append(float(angle))
    trace["last_speed"] = float(speed)
    if x_position >= cfg.TRACE_FINISH_POSITION:
        trace["completed"] = 1


def mark_vehicle_trace_completed(traces, veh_id):
    """车辆离开仿真时补充标记完成状态。"""
    if veh_id in traces:
        traces[veh_id]["completed"] = 1


def write_vehicle_trace_file(traces, output_path):
    """把车辆级轨迹序列写成一车一行的 CSV 文件。"""
    fieldnames = [
        "车辆 ID",
        "速度序列 (m/s)",
        "加速度序列 (m/s^2)",
        "完成状态",
        "X 坐标序列",
        "Y 坐标序列",
        "进入时间步",
        "角度序列",
    ]

    with open(output_path, "w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for veh_id in sorted(traces, key=lambda item: int(item) if str(item).isdigit() else str(item)):
            trace = traces[veh_id]
            writer.writerow(
                {
                    "车辆 ID": trace["vehicle_id"],
                    "速度序列 (m/s)": json.dumps(trace["speed_sequence"], ensure_ascii=False),
                    "加速度序列 (m/s^2)": json.dumps(trace["acceleration_sequence"], ensure_ascii=False),
                    "完成状态": trace["completed"],
                    "X 坐标序列": json.dumps(trace["x_sequence"], ensure_ascii=False),
                    "Y 坐标序列": json.dumps(trace["y_sequence"], ensure_ascii=False),
                    "进入时间步": trace["enter_step"],
                    "角度序列": json.dumps(trace["angle_sequence"], ensure_ascii=False),
                }
            )


def write_results(results):
    """将一组或两组实验统计结果写入 CSV 文件。"""
    fieldnames = [
        "swarm_rules",
        "mean_step_speed_mps",
        "mean_vehicle_speed_mps",
        "completed_vehicles",
        "speed_samples",
        "fcd_output",
        "vehicle_trace_output",
    ]

    with open(cfg.RESULTS_FILE, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)


def print_result(result):
    """在终端打印单次实验的关键统计指标。"""
    print(f"仿生编队规则：{result['swarm_rules']}")
    print(f"步平均速度：{result['mean_step_speed_mps']:.3f} m/s")
    print(f"车辆样本平均速度：{result['mean_vehicle_speed_mps']:.3f} m/s")
    print(f"完成行程车辆数：{result['completed_vehicles']}")
    print(f"速度样本数：{result['speed_samples']}")
    print(f"FCD 输出文件：{result['fcd_output']}")
    print(f"车辆轨迹明细文件：{result['vehicle_trace_output']}")

