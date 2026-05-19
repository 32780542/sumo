import os
import random
import shutil
import subprocess
import xml.etree.ElementTree as ET

import traci

import sim.config as cfg
SCENARIO_MODE_NAMES = {
    1: "straight_obstacles",
    2: "lane_drop_3_to_2",
}


def get_current_scenario_name():
    """返回当前场景名称，优先支持数字模式，兼容旧字符串配置。"""
    if cfg.SCENARIO_NAME:
        return cfg.SCENARIO_NAME
    if cfg.SCENARIO_MODE in SCENARIO_MODE_NAMES:
        return SCENARIO_MODE_NAMES[cfg.SCENARIO_MODE]
    raise ValueError(f"未知场景编号：{cfg.SCENARIO_MODE}")


def get_current_scenario_spec():
    """返回当前场景的路网和障碍物规格。"""
    scenario_name = get_current_scenario_name()
    if scenario_name == "straight_obstacles":
        return {
            "name": "straight_obstacles",
            "route_edges": cfg.ROAD_EDGE_ID,
            "edge_ids": (cfg.ROAD_EDGE_ID,),
            "depart_lanes": cfg.LANES,
            "lane_count": 3,
            "enable_obstacles": cfg.ENABLE_OBSTACLES,
            "obstacles": cfg.OBSTACLES,
        }
    if scenario_name == "lane_drop_3_to_2":
        return {
            "name": "lane_drop_3_to_2",
            "route_edges": "west_to_merge merge_to_east",
            "edge_ids": ("west_to_merge", "merge_to_east"),
            "depart_lanes": (2, 1, 0),
            "lane_count": 3,
            "enable_obstacles": False,
            "obstacles": [],
        }
    raise ValueError(f"未知场景：{scenario_name}")


def get_active_obstacles():
    """返回当前场景中应参与 GUI 与控制逻辑的静态障碍物。"""
    spec = get_current_scenario_spec()
    return spec["obstacles"] if spec["enable_obstacles"] else []


def generate_route_file():
    """生成 SUMO 路线文件，只定义 AV 车型和直线路线，不预先写死车辆。"""
    spec = get_current_scenario_spec()
    routes = ET.Element("routes")
    ET.SubElement(
        routes,
        "vType",
        {
            "id": "AV",
            "vClass": "passenger",
            "guiShape": "passenger",
            "carFollowModel": "IDM",
            "accel": "2.6",
            "decel": "4.5",
            "minGap": "2.0",
            "maxSpeed": str(cfg.MAX_SPEED),
            "length": "4.8",
            "width": "1.8",
            "color": "0,80,255",
        },
    )

    ET.SubElement(routes, "route", {"id": cfg.ROUTE_ID, "edges": spec["route_edges"]})

    tree = ET.ElementTree(routes)
    ET.indent(tree, space="    ")
    tree.write(cfg.ROUTE_FILE, encoding="UTF-8", xml_declaration=True)


def generate_obstacle_file():
    """生成静态障碍物 additional 文件，障碍物以 POI 形式绑定到指定车道位置。"""
    spec = get_current_scenario_spec()
    additional = ET.Element("additional")

    if spec["enable_obstacles"]:
        for obstacle in spec["obstacles"]:
            position = float(obstacle["position"])
            lane_id = f"{cfg.ROAD_EDGE_ID}_{int(obstacle['lane'])}"
            ET.SubElement(
                additional,
                "poi",
                {
                    "id": obstacle["id"],
                    "type": "static_obstacle",
                    "color": "220,40,40",
                    "lane": lane_id,
                    "pos": str(position),
                    "posLat": "0",
                    "friendlyPos": "true",
                    "width": "2.4",
                    "height": "5.5",
                    "layer": "10",
                },
            )

    tree = ET.ElementTree(additional)
    ET.indent(tree, space="    ")
    tree.write(cfg.OBSTACLE_FILE, encoding="UTF-8", xml_declaration=True)


def resolve_netconvert_binary():
    """解析 netconvert 可执行文件。"""
    binary = cfg.NETCONVERT_BINARY or "netconvert"
    if os.path.isabs(binary) and os.path.exists(binary):
        return binary
    resolved = shutil.which(binary)
    if resolved:
        return resolved

    sumo_gui_dir = os.path.dirname(cfg.SUMO_GUI_BINARY) if cfg.SUMO_GUI_BINARY else ""
    candidate = os.path.join(sumo_gui_dir, "netconvert.exe")
    if candidate and os.path.exists(candidate):
        return candidate

    raise FileNotFoundError("Could not find netconvert. Set NETCONVERT_BINARY or add SUMO bin to PATH.")


def write_plain_network_files_for_scenario():
    """根据当前场景生成 plain node/edge/type 文件。"""
    spec = get_current_scenario_spec()

    nodes = ET.Element("nodes")
    edges = ET.Element("edges")
    types = ET.Element("types")
    ET.SubElement(types, "type", {"id": "three_lane_road", "numLanes": "3", "speed": str(cfg.MAX_SPEED)})
    ET.SubElement(types, "type", {"id": "two_lane_road", "numLanes": "2", "speed": str(cfg.MAX_SPEED)})

    if spec["name"] == "lane_drop_3_to_2":
        ET.SubElement(nodes, "node", {"id": "west", "x": "0", "y": "0", "type": "dead_end"})
        ET.SubElement(nodes, "node", {"id": "merge", "x": "800", "y": "0", "type": "priority"})
        ET.SubElement(nodes, "node", {"id": "east", "x": "1200", "y": "0", "type": "dead_end"})
        ET.SubElement(edges, "edge", {"id": "west_to_merge", "from": "west", "to": "merge", "type": "three_lane_road"})
        ET.SubElement(edges, "edge", {"id": "merge_to_east", "from": "merge", "to": "east", "type": "two_lane_road"})
    else:
        ET.SubElement(nodes, "node", {"id": "west", "x": "0", "y": "0", "type": "dead_end"})
        ET.SubElement(nodes, "node", {"id": "east", "x": "1200", "y": "0", "type": "dead_end"})
        ET.SubElement(edges, "edge", {"id": cfg.ROAD_EDGE_ID, "from": "west", "to": "east", "type": "three_lane_road"})

    file_map = {
        "nodes": os.path.join(cfg.NETWORK_DIR, "platoon.nod.xml"),
        "edges": os.path.join(cfg.NETWORK_DIR, "platoon.edg.xml"),
        "types": os.path.join(cfg.NETWORK_DIR, "platoon.typ.xml"),
    }
    for root, path in ((nodes, file_map["nodes"]), (edges, file_map["edges"]), (types, file_map["types"])):
        tree = ET.ElementTree(root)
        ET.indent(tree, space="    ")
        tree.write(path, encoding="UTF-8", xml_declaration=True)

    return file_map


def generate_network_file():
    """生成当前场景的 SUMO net.xml。"""
    os.makedirs(cfg.NETWORK_DIR, exist_ok=True)
    plain_files = write_plain_network_files_for_scenario()
    netconvert = resolve_netconvert_binary()
    command = [
        netconvert,
        "--node-files",
        plain_files["nodes"],
        "--edge-files",
        plain_files["edges"],
        "--type-files",
        plain_files["types"],
        "--output-file",
        os.path.join(cfg.NETWORK_DIR, "platoon.net.xml"),
        "--no-turnarounds",
        "true",
    ]
    subprocess.run(command, check=True)


def generate_sumo_config_file():
    """生成 SUMO 配置文件，引用 network 目录内的路网/路线/障碍物文件。"""
    configuration = ET.Element("configuration")
    input_node = ET.SubElement(configuration, "input")
    ET.SubElement(input_node, "net-file", {"value": "platoon.net.xml"})
    ET.SubElement(input_node, "route-files", {"value": "platoon.rou.xml"})
    ET.SubElement(input_node, "additional-files", {"value": "obstacles.add.xml"})

    time_node = ET.SubElement(configuration, "time")
    ET.SubElement(time_node, "begin", {"value": "0"})
    ET.SubElement(time_node, "end", {"value": "3600"})
    ET.SubElement(time_node, "step-length", {"value": str(cfg.STEP_LENGTH)})

    output_node = ET.SubElement(configuration, "output")
    ET.SubElement(output_node, "fcd-output", {"value": os.path.join("..", "logs", "fcd_output.xml")})

    gui_node = ET.SubElement(configuration, "gui_only")
    ET.SubElement(gui_node, "gui-settings-file", {"value": "gui-settings.xml"})

    tree = ET.ElementTree(configuration)
    ET.indent(tree, space="    ")
    tree.write(cfg.SUMO_CONFIG, encoding="UTF-8", xml_declaration=True)


def get_flow_probability(sim_time):
    """根据当前仿真时间返回车辆生成概率，支持固定流量或分时段流量梯度。"""
    if not cfg.FLOW_GRADIENTS:
        return cfg.TRAFFIC_FLOW_PROBABILITY

    probability = cfg.FLOW_GRADIENTS[0][1]
    for start_time, flow_probability in cfg.FLOW_GRADIENTS:
        if sim_time >= start_time:
            probability = flow_probability
        else:
            break
    return probability


def maybe_add_random_vehicles(next_vehicle_id, vehicle_states):
    """按概率在每条车道随机生成 AV，并为每辆车分配异质性动力学参数。"""
    spec = get_current_scenario_spec()
    flow_probability = get_flow_probability(traci.simulation.getTime())

    for lane in spec["depart_lanes"]:
        # 每条车道独立抽样：抽中才在该车道入口生成一辆 AV。
        if random.random() >= flow_probability:
            continue

        next_vehicle_id += 1
        veh_id = str(next_vehicle_id)

        try:
            traci.vehicle.add(
                vehID=veh_id,
                routeID=cfg.ROUTE_ID,
                typeID="AV",
                depart="now",
                departLane=str(lane),
                departPos="0",
                departSpeed=str(cfg.INITIAL_SPEED),
            )
        except traci.TraCIException:
            next_vehicle_id -= 1
            continue

        traci.vehicle.setSpeedMode(veh_id, 31)
        traci.vehicle.setLaneChangeMode(veh_id, cfg.SAFE_LANE_CHANGE_MODE)

        if cfg.ENABLE_VEHICLE_HETEROGENEITY:
            # 为每辆车随机生成合理范围内的动力学差异，体现车辆异质性。
            vehicle_params = {
                "max_speed": random.uniform(24.0, 30.0),
                "accel": random.uniform(1.8, 3.0),
                "decel": random.uniform(3.8, 5.2),
                "min_gap": random.uniform(1.5, 2.8),
                "tau": random.uniform(0.9, 1.6),
            }
            vehicle_params["desired_speed"] = random.uniform(20.0, vehicle_params["max_speed"])
        else:
            vehicle_params = dict(cfg.HOMOGENEOUS_VEHICLE_PARAMS)

        traci.vehicle.setMaxSpeed(veh_id, vehicle_params["max_speed"])
        traci.vehicle.setAccel(veh_id, vehicle_params["accel"])
        traci.vehicle.setDecel(veh_id, vehicle_params["decel"])
        traci.vehicle.setMinGap(veh_id, vehicle_params["min_gap"])
        traci.vehicle.setTau(veh_id, vehicle_params["tau"])

        vehicle_states[veh_id] = {
            "type": "AV",
            "control_type": 0,
            "created_at": traci.simulation.getTime(),
            "lane": lane,
            **vehicle_params,
            "fallback_active": False,
        }

    return next_vehicle_id

