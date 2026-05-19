import os
import random
import shutil
import statistics

import traci

import sim.config as cfg
from sim.outputs import append_vehicle_trace_sample, build_vehicle_trace_filename, ensure_output_directories, make_csv_path, make_log_path, mark_vehicle_trace_completed, print_result, write_results, write_vehicle_trace_file
from sim.scenario import generate_network_file, generate_obstacle_file, generate_route_file, generate_sumo_config_file, maybe_add_random_vehicles
from sim.control import compute_layered_target_speed
def resolve_sumo_binary(use_gui):
    """根据是否启用 GUI 解析 SUMO 可执行文件路径。"""
    configured_binary = cfg.SUMO_GUI_BINARY if use_gui else cfg.SUMO_BINARY
    fallback_binary = "sumo-gui" if use_gui else "sumo"
    binary = configured_binary or fallback_binary

    if os.path.isabs(binary) and os.path.exists(binary):
        return binary

    resolved_binary = shutil.which(binary)
    if resolved_binary:
        return resolved_binary

    raise FileNotFoundError(
        f"Could not find {binary}. Check cfg.SUMO_GUI_BINARY/cfg.SUMO_BINARY or add SUMO bin to PATH."
    )


def run(enable_swarm_rules=True, fcd_output="fcd_output.xml", use_gui=False):
    """启动并运行 SUMO 仿真，循环执行随机注车、分层控制和数据采集。"""
    ensure_output_directories()
    sumo_binary = resolve_sumo_binary(use_gui)
    fcd_output_path = make_log_path(fcd_output)
    trace_filename = build_vehicle_trace_filename(
        enable_swarm_rules,
        cfg.SIM_END,
        cfg.TRAFFIC_FLOW_PROBABILITY,
    )
    trace_output = make_csv_path(trace_filename)
    command = [
        sumo_binary,
        "-c",
        cfg.SUMO_CONFIG,
        "--step-length",
        str(cfg.STEP_LENGTH),
        "--fcd-output",
        fcd_output_path,
        "--lanechange.duration",
        str(cfg.LANE_CHANGE_DURATION),
        "--start",
    ]

    if use_gui:
        command.extend(["--delay", str(cfg.GUI_DELAY_MS), "--gui-settings-file", cfg.GUI_SETTINGS_FILE])

    print(f"启动 SUMO：{sumo_binary}")
    traci.start(command)
    random.seed(cfg.RANDOM_SEED)

    step_average_speeds = []
    vehicle_speed_samples = []
    completed_vehicles = 0
    next_vehicle_id = 0
    vehicle_states = {}
    vehicle_traces = {}
    step_index = 0

    try:
        while traci.simulation.getTime() < cfg.SIM_END:
            # 先推进 SUMO 一步，再按当前时刻执行注车与控制。
            traci.simulationStep()
            step_index += 1
            next_vehicle_id = maybe_add_random_vehicles(next_vehicle_id, vehicle_states)
            vehicle_ids = traci.vehicle.getIDList()
            for arrived_id in traci.simulation.getArrivedIDList():
                mark_vehicle_trace_completed(vehicle_traces, arrived_id)
            completed_vehicles += traci.simulation.getArrivedNumber()

            if not vehicle_ids:
                continue

            target_speeds = {}
            current_speeds = []

            for veh_id in vehicle_ids:
                # 每辆车独立计算目标速度；若返回 None，表示该车由 SUMO fallback 接管。
                current_speed = traci.vehicle.getSpeed(veh_id)
                current_speeds.append(current_speed)
                vehicle_speed_samples.append(current_speed)
                append_vehicle_trace_sample(
                    vehicle_traces,
                    veh_id,
                    current_speed,
                    traci.vehicle.getLanePosition(veh_id),
                    traci.vehicle.getLaneIndex(veh_id),
                    traci.vehicle.getAngle(veh_id),
                    step_index,
                    cfg.STEP_LENGTH,
                )

                target_speed = compute_layered_target_speed(
                    veh_id, vehicle_states, enable_swarm_rules
                )
                if target_speed is not None:
                    target_speeds[veh_id] = min(target_speed, cfg.MAX_SPEED)

            for veh_id, target_speed in target_speeds.items():
                # 统一在本步末尾写入速度指令，避免前后车辆读取状态时互相干扰。
                try:
                    traci.vehicle.setSpeed(veh_id, target_speed)
                except traci.TraCIException:
                    pass

            if current_speeds:
                step_average_speeds.append(statistics.fmean(current_speeds))

    finally:
        traci.close()

    write_vehicle_trace_file(vehicle_traces, trace_output)

    result = {
        "swarm_rules": "on" if enable_swarm_rules else "off",
        "mean_step_speed_mps": statistics.fmean(step_average_speeds)
        if step_average_speeds
        else 0.0,
        "mean_vehicle_speed_mps": statistics.fmean(vehicle_speed_samples)
        if vehicle_speed_samples
        else 0.0,
        "completed_vehicles": completed_vehicles,
        "speed_samples": len(vehicle_speed_samples),
        "fcd_output": fcd_output_path,
        "vehicle_trace_output": trace_output,
    }

    return result


def main():
    """程序主入口：按 cfg.RUN_MODE 执行单组或对比实验。"""
    ensure_output_directories()
    if cfg.REGENERATE_ROUTES:
        generate_network_file()
        generate_route_file()
        generate_obstacle_file()
        generate_sumo_config_file()
        print(f"已生成路网文件：{cfg.NETWORK_DIR}\\platoon.net.xml")
        print(f"已生成路线文件：{cfg.ROUTE_FILE}")
        print(f"已生成障碍物文件：{cfg.OBSTACLE_FILE}")
        print(f"已生成 SUMO 配置文件：{cfg.SUMO_CONFIG}")

    if cfg.RUN_MODE == 3:
        results = [
            run(
                enable_swarm_rules=False,
                fcd_output="fcd_output_baseline.xml",
                use_gui=cfg.USE_GUI,
            ),
            run(
                enable_swarm_rules=True,
                fcd_output="fcd_output_swarm.xml",
                use_gui=cfg.USE_GUI,
            ),
        ]
    elif cfg.RUN_MODE in (1, 2):
        enabled = cfg.RUN_MODE == 1
        fcd_output = "fcd_output_swarm.xml" if enabled else "fcd_output_baseline.xml"
        results = [run(enable_swarm_rules=enabled, fcd_output=fcd_output, use_gui=cfg.USE_GUI)]
    else:
        raise ValueError("cfg.RUN_MODE 必须为 1、2 或 3。")

    write_results(results)
    for index, result in enumerate(results):
        if index:
            print()
        print_result(result)

    print(f"\n结果已保存至：{cfg.RESULTS_FILE}")

