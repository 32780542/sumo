import csv
import os
import random
import shutil
import statistics
import xml.etree.ElementTree as ET

import traci


# =========================
# 实验开关参数
# =========================
# 在 VSCode 中直接运行 main.py 时，修改 RUN_MODE 即可切换实验模式：
#   1 = 启用仿生编队规则
#   2 = 关闭仿生编队规则，仅运行基础自动驾驶与安全层
#   3 = 连续运行关闭/开启两组实验，并保存对比结果
RUN_MODE = 2

# 是否打开 SUMO GUI 窗口；False 表示后台无界面运行。
USE_GUI = True

# GUI 播放延迟，单位为毫秒；车辆运动太快时可以调大。
GUI_DELAY_MS = 50

# 是否在每次运行前重新生成 platoon.rou.xml 和 obstacles.add.xml。
# 修改车辆类型、路线或障碍物参数后建议保持 True。
REGENERATE_ROUTES = True

# 随机车辆生成参数：仿照你之前 MATLAB/TraCI 的逐车道随机注车结构。
# 每个仿真步，每条车道都会独立尝试生成一辆 AV。
RANDOM_SEED = 7
# 每个仿真步、每条车道生成车辆的概率；数值越大车流越密。
TRAFFIC_FLOW_PROBABILITY = 0.02
# 新生成车辆的初始速度，单位 m/s。
INITIAL_SPEED = 22.0
# 需要注入车辆的车道编号；当前道路共有 0、1、2 三条车道。
LANES = (2, 1, 0)

# 可选的分时段流量梯度，格式为 [(开始时间, 生成概率), ...]。
# 为空时，整个仿真都使用 TRAFFIC_FLOW_PROBABILITY。
FLOW_GRADIENTS = []

# 静态道路障碍物参数；position 表示从西向东的纵向位置，单位 m。
# 这些障碍物是 POI，不是车辆，因此不会被仿生编队规则当作车辆控制。
ENABLE_OBSTACLES = True
OBSTACLES = [
    {"id": "obs_0", "lane": 2, "position": 350.0},
    {"id": "obs_1", "lane": 1, "position": 650.0},
    {"id": "obs_2", "lane": 0, "position": 950.0},
]

# SUMO 可执行文件路径；None 表示自动从系统 PATH 中查找。
SUMO_BINARY = None
SUMO_GUI_BINARY = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"

# 仿真输入/输出文件路径。
SUMO_CONFIG = os.path.join(os.path.dirname(__file__), "platoon.sumocfg")
ROUTE_FILE = os.path.join(os.path.dirname(__file__), "platoon.rou.xml")
OBSTACLE_FILE = os.path.join(os.path.dirname(__file__), "obstacles.add.xml")

# 仿真步长，单位 s。
STEP_LENGTH = 0.1
# 局部感知范围，单位 m。
SENSING_RANGE = 50.0
# 静态短距离安全阈值，单位 m。
SAFE_DISTANCE = 5.0
# 时间头距安全阈值，单位 s；安全距离约为 speed * SAFE_TIME_HEADWAY。
SAFE_TIME_HEADWAY = 1.5
# 最小静态安全距离，单位 m。
STATIC_SAFE_DISTANCE = 8.0
# Fallback 解除系数；危险解除时要求距离更宽松，避免控制模式频繁抖动。
FALLBACK_RELEASE_FACTOR = 1.25
# 全局最高速度，单位 m/s。
MAX_SPEED = 30.0
# 全局最低速度，单位 m/s。
MIN_SPEED = 0.0
# 仿真结束时间，单位 s。
SIM_END = 600
# 极限危险距离，小于该距离时强制停车。
CRITICAL_DISTANCE = 3.0
# 静态障碍物前向感知距离，单位 m。
OBSTACLE_LOOKAHEAD = 70.0
# 在障碍物前预留的默认停车缓冲距离，单位 m；实际控制会与车辆 min_gap 取一致。
OBSTACLE_STOP_BUFFER = SAFE_DISTANCE
# 进入该距离后对障碍物执行更强的减速保护，单位 m。
OBSTACLE_HARD_BRAKE_DISTANCE = 18.0

# 换道持续时间，单位 s；同时传给 SUMO 启动参数和 traci.vehicle.changeLane()。
# 数值越大，GUI 中的换道过程越平滑、越不“瞬移”。
LANE_CHANGE_DURATION = 2.0
# 简化 MOBIL 换道收益阈值；目标车道收益必须超过该值才换道。
MOBIL_MIN_GAIN = 0.2
# 换道时目标车道后方安全间距，单位 m。
MOBIL_BACK_SAFE_GAP = 25.0
# 换道时目标车道前方安全间距，单位 m。
MOBIL_FRONT_SAFE_GAP = 45.0
# SUMO 安全换道模式，保留 SUMO 自带的安全检查，避免侧向碰撞。
SAFE_LANE_CHANGE_MODE = 1621
# 巡航速度误差反馈增益。
CRUISE_GAIN = 0.35
# IDM 模型中的加速度指数。
IDM_DELTA = 4
# IDM 默认时间头距，单位 s。
IDM_TIME_HEADWAY = 1.2
# 仿生编队层参数：只在基础单车自动驾驶速度上做小幅修正。
# 这些规则不直接接管安全和换道，只诱导局部速度同步与紧凑行驶。
SWARM_ALIGNMENT_GAIN = 0.08
SWARM_COHESION_GAIN = 0.25
SWARM_SEPARATION_GAIN = 0.0
SWARM_DESIRED_HEADWAY = 0.75
SWARM_MIN_PLATOON_GAP = 4.5
SWARM_MAX_SPEED_DELTA = 0.6
# 仿生层参考车辆筛选：低速/停止车辆通常代表局部受阻，不参与畅通车道的速度对齐。
SWARM_MIN_REFERENCE_SPEED = 5.0
SWARM_REFERENCE_SPEED_RATIO = 0.7

# 实验统计结果和 GUI 配置文件路径。
RESULTS_FILE = os.path.join(os.path.dirname(__file__), "platoon_results.csv")
GUI_SETTINGS_FILE = os.path.join(os.path.dirname(__file__), "gui-settings.xml")

# 路网中直线路段和路线 ID。
ROAD_EDGE_ID = "west_to_east"
ROUTE_ID = "r_west_to_east"
# 车道宽度，仅用于辅助计算/说明。
LANE_WIDTH = 3.2


def clamp(value, lower, upper):
    """将数值限制在 [lower, upper] 区间内，避免速度等控制量越界。"""
    return max(lower, min(upper, value))


def generate_route_file():
    """生成 SUMO 路线文件，只定义 AV 车型和直线路线，不预先写死车辆。"""
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
            "maxSpeed": str(MAX_SPEED),
            "length": "4.8",
            "width": "1.8",
            "color": "0,80,255",
        },
    )

    ET.SubElement(routes, "route", {"id": ROUTE_ID, "edges": ROAD_EDGE_ID})

    tree = ET.ElementTree(routes)
    ET.indent(tree, space="    ")
    tree.write(ROUTE_FILE, encoding="UTF-8", xml_declaration=True)


def generate_obstacle_file():
    """生成静态障碍物 additional 文件，障碍物以 POI 形式绑定到指定车道位置。"""
    additional = ET.Element("additional")

    if ENABLE_OBSTACLES:
        for obstacle in OBSTACLES:
            position = float(obstacle["position"])
            lane_id = f"{ROAD_EDGE_ID}_{int(obstacle['lane'])}"
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
    tree.write(OBSTACLE_FILE, encoding="UTF-8", xml_declaration=True)


def get_flow_probability(sim_time):
    """根据当前仿真时间返回车辆生成概率，支持固定流量或分时段流量梯度。"""
    if not FLOW_GRADIENTS:
        return TRAFFIC_FLOW_PROBABILITY

    probability = FLOW_GRADIENTS[0][1]
    for start_time, flow_probability in FLOW_GRADIENTS:
        if sim_time >= start_time:
            probability = flow_probability
        else:
            break
    return probability


def maybe_add_random_vehicles(next_vehicle_id, vehicle_states):
    """按概率在每条车道随机生成 AV，并为每辆车分配异质性动力学参数。"""
    flow_probability = get_flow_probability(traci.simulation.getTime())

    for lane in LANES:
        # 每条车道独立抽样：抽中才在该车道入口生成一辆 AV。
        if random.random() >= flow_probability:
            continue

        next_vehicle_id += 1
        veh_id = str(next_vehicle_id)

        try:
            traci.vehicle.add(
                vehID=veh_id,
                routeID=ROUTE_ID,
                typeID="AV",
                depart="now",
                departLane=str(lane),
                departPos="0",
                departSpeed=str(INITIAL_SPEED),
            )
        except traci.TraCIException:
            next_vehicle_id -= 1
            continue

        traci.vehicle.setSpeedMode(veh_id, 31)
        traci.vehicle.setLaneChangeMode(veh_id, SAFE_LANE_CHANGE_MODE)

        # 为每辆车随机生成合理范围内的动力学差异，体现车辆异质性。
        max_speed = random.uniform(24.0, 30.0)
        accel = random.uniform(1.8, 3.0)
        decel = random.uniform(3.8, 5.2)
        min_gap = random.uniform(1.5, 2.8)
        tau = random.uniform(0.9, 1.6)
        desired_speed = random.uniform(20.0, max_speed)

        traci.vehicle.setMaxSpeed(veh_id, max_speed)
        traci.vehicle.setAccel(veh_id, accel)
        traci.vehicle.setDecel(veh_id, decel)
        traci.vehicle.setMinGap(veh_id, min_gap)
        traci.vehicle.setTau(veh_id, tau)

        vehicle_states[veh_id] = {
            "type": "AV",
            "control_type": 0,
            "created_at": traci.simulation.getTime(),
            "lane": lane,
            "max_speed": max_speed,
            "accel": accel,
            "decel": decel,
            "min_gap": min_gap,
            "tau": tau,
            "desired_speed": desired_speed,
            "fallback_active": False,
        }

    return next_vehicle_id


def get_vehicle_params(veh_id, vehicle_states):
    """读取车辆的异质性参数；若车辆状态缺失，则返回一组默认 AV 参数。"""
    return vehicle_states.get(
        veh_id,
        {
            "max_speed": MAX_SPEED,
            "accel": 2.6,
            "decel": 4.5,
            "min_gap": 2.0,
            "tau": IDM_TIME_HEADWAY,
            "desired_speed": 24.0,
        },
    )


def get_vehicle_speed(veh_id):
    """安全读取指定车辆速度；车辆不存在或 TraCI 报错时返回 None。"""
    try:
        return traci.vehicle.getSpeed(veh_id)
    except traci.TraCIException:
        return None


def add_neighbor(neighbors, veh_id, distance):
    """把感知范围内的邻居车辆加入列表，记录车辆 ID、距离和速度。"""
    if not veh_id or distance < 0 or distance > SENSING_RANGE:
        return

    speed = get_vehicle_speed(veh_id)
    if speed is None:
        return

    neighbors.append(
        {
            "id": veh_id,
            "distance": float(distance),
            "speed": float(speed),
        }
    )


def sense_leader_and_follower(veh_id):
    """感知当前车辆前车和后车，并返回前车信息及纵向邻居列表。"""
    neighbors = []
    leader = None

    try:
        leader_info = traci.vehicle.getLeader(veh_id, SENSING_RANGE)
    except traci.TraCIException:
        leader_info = None

    if leader_info:
        leader_id, leader_distance = leader_info
        add_neighbor(neighbors, leader_id, leader_distance)
        if leader_id and leader_distance >= 0:
            leader_speed = get_vehicle_speed(leader_id)
            leader = {
                "id": leader_id,
                "distance": float(leader_distance),
                "speed": leader_speed if leader_speed is not None else 0.0,
            }

    try:
        follower_id, follower_distance = traci.vehicle.getFollower(veh_id, SENSING_RANGE)
    except traci.TraCIException:
        follower_id, follower_distance = "", -1

    add_neighbor(neighbors, follower_id, follower_distance)
    return leader, neighbors


def sense_lateral_neighbors(veh_id):
    """感知左右相邻车道中的邻居车辆，用于仿生速度对齐。"""
    neighbors = []

    # SUMO 的 getNeighbors 使用 mode 位选择左右方向和前后关系。
    # 这里遍历 0、1、2、3，尽量收集左右相邻车道的前后邻居。
    for mode in (0, 1, 2, 3):
        try:
            lateral_neighbors = traci.vehicle.getNeighbors(veh_id, mode)
        except traci.TraCIException:
            continue

        for neighbor_id, distance in lateral_neighbors:
            add_neighbor(neighbors, neighbor_id, abs(distance))

    return neighbors


def sense_static_obstacle(veh_id):
    """感知当前车道前方最近的静态障碍物；障碍物是 POI，不属于车辆列表。"""
    if not ENABLE_OBSTACLES:
        return None

    lane = traci.vehicle.getLaneIndex(veh_id)
    position = traci.vehicle.getLanePosition(veh_id)
    nearest = None

    for obstacle in OBSTACLES:
        if int(obstacle["lane"]) != lane:
            continue

        distance = float(obstacle["position"]) - position
        if distance < 0 or distance > OBSTACLE_LOOKAHEAD:
            continue

        if nearest is None or distance < nearest["distance"]:
            nearest = {
                "id": obstacle["id"],
                "distance": distance,
                "speed": 0.0,
                "lane": lane,
            }

    return nearest


def idm_target_speed(current_speed, leader, obstacle, params):
    """中层单车自动驾驶：使用 IDM 跟车/巡航模型计算基础目标速度。"""
    desired_speed = params["desired_speed"]
    max_speed = params["max_speed"]
    accel = params["accel"]
    decel = params["decel"]
    min_gap = params["min_gap"]
    tau = params["tau"]

    front_object = None
    if leader:
        front_object = leader
    if obstacle and (front_object is None or obstacle["distance"] < front_object["distance"]):
        front_object = obstacle

    # IDM 自由流加速度项：速度越接近期望速度，加速越弱。
    free_accel = accel * (1.0 - (max(current_speed, 0.0) / max(desired_speed, 0.1)) ** IDM_DELTA)

    if front_object:
        # IDM 跟车项：根据前方对象距离、相对速度和期望间距计算减速度。
        distance = max(front_object["distance"], 0.1)
        delta_v = current_speed - front_object.get("speed", 0.0)
        desired_gap = min_gap + max(
            0.0,
            current_speed * tau + current_speed * delta_v / (2.0 * (accel * decel) ** 0.5),
        )
        acceleration = free_accel - accel * (desired_gap / distance) ** 2
    else:
        acceleration = free_accel + CRUISE_GAIN * (desired_speed - current_speed)

    return current_speed + acceleration * STEP_LENGTH


def lane_has_space_for_change(veh_id, target_lane):
    """检查目标车道前后车辆和障碍物间距，判断是否满足安全换道条件。"""
    ego_pos = traci.vehicle.getLanePosition(veh_id)

    for other_id in traci.vehicle.getIDList():
        if other_id == veh_id:
            continue

        try:
            if traci.vehicle.getRoadID(other_id) != ROAD_EDGE_ID:
                continue
            if traci.vehicle.getLaneIndex(other_id) != target_lane:
                continue
        except traci.TraCIException:
            continue

        gap = traci.vehicle.getLanePosition(other_id) - ego_pos
        if -MOBIL_BACK_SAFE_GAP < gap < MOBIL_FRONT_SAFE_GAP:
            return False

    for obstacle in OBSTACLES:
        if int(obstacle["lane"]) != target_lane:
            continue
        gap = float(obstacle["position"]) - ego_pos
        if -MOBIL_BACK_SAFE_GAP < gap < OBSTACLE_LOOKAHEAD:
            return False

    return True


def maybe_execute_mobil_lane_change(veh_id, leader, obstacle):
    """简化 MOBIL 换道逻辑：当前方受阻且目标车道安全收益更高时执行换道。"""
    if not leader and not obstacle:
        return

    front_distance = min(
        [item["distance"] for item in (leader, obstacle) if item is not None],
        default=OBSTACLE_LOOKAHEAD,
    )
    if front_distance > 30.0:
        return

    current_lane = traci.vehicle.getLaneIndex(veh_id)
    best_lane = current_lane
    best_gain = 0.0

    for target_lane in (current_lane - 1, current_lane + 1):
        # 只允许换到相邻车道，并检查目标车道前后安全空间。
        if target_lane < 0 or target_lane > 2:
            continue
        if not lane_has_space_for_change(veh_id, target_lane):
            continue

        gain = OBSTACLE_LOOKAHEAD
        for obstacle_item in OBSTACLES:
            if int(obstacle_item["lane"]) == target_lane:
                obstacle_gap = float(obstacle_item["position"]) - traci.vehicle.getLanePosition(veh_id)
                if 0 <= obstacle_gap < OBSTACLE_LOOKAHEAD:
                    gain = min(gain, obstacle_gap)

        if gain > best_gain:
            best_lane = target_lane
            best_gain = gain

    if best_lane != current_lane and best_gain > front_distance + MOBIL_MIN_GAIN:
        traci.vehicle.changeLane(veh_id, best_lane, LANE_CHANGE_DURATION)


def apply_swarm_layer(base_speed, current_speed, neighbors, leader):
    """上层仿生编队规则：在有限感知内诱导局部正向协同。

    设计原则：
    1. 不替代单车自动驾驶层，只在 IDM/MOBIL 给出的 base_speed 上做小幅修正。
    2. 不直接处理危险工况，最终仍交给后续安全层和 SUMO fallback 兜底。
    3. 只使用本车有限感知到的邻居信息，让类似编队的行为自发涌现。
    4. 当前版本只做正向速度微调，不主动降速；降速由安全层和 IDM 负责。
    """
    if not neighbors:
        return base_speed

    speed_delta = 0.0
    reference_neighbors = [
        item
        for item in neighbors
        if item["speed"] >= SWARM_MIN_REFERENCE_SPEED
        and item["speed"] >= current_speed * SWARM_REFERENCE_SPEED_RATIO
    ]

    # 规则 1：正向速度对齐。低速/停车邻居不参与对齐，避免畅通车道被堵塞车道拖慢。
    if reference_neighbors:
        neighbor_mean_speed = statistics.fmean(item["speed"] for item in reference_neighbors)
        if neighbor_mean_speed > current_speed:
            speed_delta += SWARM_ALIGNMENT_GAIN * (neighbor_mean_speed - current_speed)

    # 规则 2：温和凝聚。当前车距大于期望头距时，允许略微追赶前方车群。
    if leader:
        desired_gap = max(
            SWARM_MIN_PLATOON_GAP,
            current_speed * SWARM_DESIRED_HEADWAY,
        )
        gap_error = leader["distance"] - desired_gap
        if gap_error > 0:
            speed_delta += SWARM_COHESION_GAIN * min(gap_error / desired_gap, 1.0)

        # 规则 3：分离占位。当前暂不主动降速，避免与安全层/IDM 形成重复保守控制。
        if gap_error < 0 and SWARM_SEPARATION_GAIN > 0:
            speed_delta += SWARM_SEPARATION_GAIN * max(gap_error / desired_gap, -1.0)

    speed_delta = clamp(speed_delta, 0.0, SWARM_MAX_SPEED_DELTA)
    return base_speed + speed_delta


def apply_safety_layer(target_speed, veh_id, leader, params):
    """底层安全约束：针对真实前车进行安全速度钳位。"""
    current_speed = traci.vehicle.getSpeed(veh_id)
    decel = params["decel"]
    safe_speed = target_speed

    if leader:
        distance = leader["distance"]
        front_speed = leader.get("speed", 0.0)

        if distance <= CRITICAL_DISTANCE:
            safe_speed = MIN_SPEED
        elif distance < SAFE_DISTANCE:
            safe_speed = min(safe_speed, max(MIN_SPEED, front_speed - 1.0))
        else:
            braking_safe_speed = (front_speed**2 + 2.0 * decel * max(0.0, distance - SAFE_DISTANCE)) ** 0.5
            safe_speed = min(safe_speed, braking_safe_speed)

    max_delta = decel * STEP_LENGTH
    return max(current_speed - max_delta, min(safe_speed, current_speed + params["accel"] * STEP_LENGTH))


def apply_static_obstacle_safety(target_speed, veh_id, obstacle, params):
    """底层安全约束：针对静态 POI 障碍物强制减速或停车，避免穿越障碍物。"""
    if not obstacle:
        return target_speed

    distance = obstacle["distance"]
    current_speed = traci.vehicle.getSpeed(veh_id)
    decel = params["decel"]
    stop_buffer = max(params.get("min_gap", SAFE_DISTANCE), OBSTACLE_STOP_BUFFER)

    if distance <= stop_buffer:
        return MIN_SPEED

    stop_distance = max(0.0, distance - stop_buffer)
    safe_speed = (2.0 * decel * stop_distance) ** 0.5

    if distance <= OBSTACLE_HARD_BRAKE_DISTANCE:
        safe_speed = min(safe_speed, max(MIN_SPEED, current_speed - decel * STEP_LENGTH))

    return min(target_speed, safe_speed)


def get_dynamic_safe_gap(speed):
    """根据当前速度计算时间头距安全距离，同时不低于静态安全距离。"""
    return max(STATIC_SAFE_DISTANCE, speed * SAFE_TIME_HEADWAY)


def has_leader_collision_risk(veh_id, leader, vehicle_states):
    """判断与真实前车是否存在碰撞风险；仅真实车辆触发 SUMO fallback。"""
    speed = traci.vehicle.getSpeed(veh_id)
    safe_gap = get_dynamic_safe_gap(speed)
    fallback_active = vehicle_states.get(veh_id, {}).get("fallback_active", False)
    release_factor = FALLBACK_RELEASE_FACTOR if fallback_active else 1.0

    if not leader:
        return False

    distance = leader["distance"]
    front_speed = leader.get("speed", 0.0)
    closing_speed = max(0.0, speed - front_speed)
    braking_gap = (closing_speed * closing_speed) / (2.0 * 4.5) if closing_speed > 0 else 0.0
    required_gap = max(safe_gap, STATIC_SAFE_DISTANCE + braking_gap)

    if distance < required_gap * release_factor:
        return True

    return False


def set_fallback_mode(veh_id, vehicle_states):
    """强制安全模式：释放速度控制给 SUMO，由 SUMO 内置逻辑避免追尾碰撞。"""
    traci.vehicle.setSpeedMode(veh_id, 31)
    traci.vehicle.setLaneChangeMode(veh_id, SAFE_LANE_CHANGE_MODE)
    traci.vehicle.setSpeed(veh_id, -1)
    vehicle_states.setdefault(veh_id, {})["fallback_active"] = True


def set_custom_control_mode(veh_id, vehicle_states):
    """自定义控制模式：保留 SUMO 安全检查，同时允许脚本设置目标速度和换道。"""
    traci.vehicle.setSpeedMode(veh_id, 31)
    traci.vehicle.setLaneChangeMode(veh_id, SAFE_LANE_CHANGE_MODE)
    vehicle_states.setdefault(veh_id, {})["fallback_active"] = False


def compute_layered_target_speed(veh_id, vehicle_states, enable_swarm_rules):
    """三层控制总入口：安全层、单车自动驾驶层、仿生编队层依次协同计算速度。"""
    current_speed = traci.vehicle.getSpeed(veh_id)
    params = get_vehicle_params(veh_id, vehicle_states)
    allowed_speed = min(params["max_speed"], traci.vehicle.getAllowedSpeed(veh_id))

    leader, longitudinal_neighbors = sense_leader_and_follower(veh_id)
    lateral_neighbors = sense_lateral_neighbors(veh_id)
    obstacle = sense_static_obstacle(veh_id)
    neighbors = longitudinal_neighbors + lateral_neighbors

    if has_leader_collision_risk(veh_id, leader, vehicle_states):
        # 对真实前车存在追尾风险时，释放速度控制给 SUMO fallback。
        set_fallback_mode(veh_id, vehicle_states)
        return None

    # 安全风险解除后，恢复自定义控制；但仍保留 SUMO 的安全检查模式。
    set_custom_control_mode(veh_id, vehicle_states)
    maybe_execute_mobil_lane_change(veh_id, leader, obstacle)

    target_speed = idm_target_speed(current_speed, leader, obstacle, params)
    if enable_swarm_rules:
        # 仿生编队层采用温和局部规则，最终仍受安全层限制。
        target_speed = apply_swarm_layer(target_speed, current_speed, neighbors, leader)
    target_speed = apply_safety_layer(target_speed, veh_id, leader, params)
    target_speed = apply_static_obstacle_safety(target_speed, veh_id, obstacle, params)
    return clamp(target_speed, MIN_SPEED, allowed_speed)


def resolve_sumo_binary(use_gui):
    """根据是否启用 GUI 解析 SUMO 可执行文件路径。"""
    configured_binary = SUMO_GUI_BINARY if use_gui else SUMO_BINARY
    fallback_binary = "sumo-gui" if use_gui else "sumo"
    binary = configured_binary or fallback_binary

    if os.path.isabs(binary) and os.path.exists(binary):
        return binary

    resolved_binary = shutil.which(binary)
    if resolved_binary:
        return resolved_binary

    raise FileNotFoundError(
        f"Could not find {binary}. Check SUMO_GUI_BINARY/SUMO_BINARY or add SUMO bin to PATH."
    )


def run(enable_swarm_rules=True, fcd_output="fcd_output.xml", use_gui=False):
    """启动并运行 SUMO 仿真，循环执行随机注车、分层控制和数据采集。"""
    sumo_binary = resolve_sumo_binary(use_gui)
    command = [
        sumo_binary,
        "-c",
        SUMO_CONFIG,
        "--step-length",
        str(STEP_LENGTH),
        "--fcd-output",
        fcd_output,
        "--lanechange.duration",
        str(LANE_CHANGE_DURATION),
        "--start",
    ]

    if use_gui:
        command.extend(["--delay", str(GUI_DELAY_MS), "--gui-settings-file", GUI_SETTINGS_FILE])

    print(f"启动 SUMO：{sumo_binary}")
    traci.start(command)
    random.seed(RANDOM_SEED)

    step_average_speeds = []
    vehicle_speed_samples = []
    completed_vehicles = 0
    next_vehicle_id = 0
    vehicle_states = {}

    try:
        while traci.simulation.getTime() < SIM_END:
            # 先推进 SUMO 一步，再按当前时刻执行注车与控制。
            traci.simulationStep()
            next_vehicle_id = maybe_add_random_vehicles(next_vehicle_id, vehicle_states)
            vehicle_ids = traci.vehicle.getIDList()
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

                target_speed = compute_layered_target_speed(
                    veh_id, vehicle_states, enable_swarm_rules
                )
                if target_speed is not None:
                    target_speeds[veh_id] = min(target_speed, MAX_SPEED)

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
        "fcd_output": fcd_output,
    }

    return result


def write_results(results):
    """将一组或两组实验统计结果写入 CSV 文件。"""
    fieldnames = [
        "swarm_rules",
        "mean_step_speed_mps",
        "mean_vehicle_speed_mps",
        "completed_vehicles",
        "speed_samples",
        "fcd_output",
    ]

    with open(RESULTS_FILE, "w", newline="", encoding="utf-8") as csv_file:
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


def main():
    """程序主入口：按 RUN_MODE 执行单组或对比实验。"""
    if REGENERATE_ROUTES:
        generate_route_file()
        generate_obstacle_file()
        print(f"已生成路线文件：{ROUTE_FILE}")
        print(f"已生成障碍物文件：{OBSTACLE_FILE}")

    if RUN_MODE == 3:
        results = [
            run(
                enable_swarm_rules=False,
                fcd_output="fcd_output_baseline.xml",
                use_gui=USE_GUI,
            ),
            run(
                enable_swarm_rules=True,
                fcd_output="fcd_output_swarm.xml",
                use_gui=USE_GUI,
            ),
        ]
    elif RUN_MODE in (1, 2):
        enabled = RUN_MODE == 1
        fcd_output = "fcd_output_swarm.xml" if enabled else "fcd_output_baseline.xml"
        results = [run(enable_swarm_rules=enabled, fcd_output=fcd_output, use_gui=USE_GUI)]
    else:
        raise ValueError("RUN_MODE 必须为 1、2 或 3。")

    write_results(results)
    for index, result in enumerate(results):
        if index:
            print()
        print_result(result)

    print(f"\n结果已保存至：{RESULTS_FILE}")


if __name__ == "__main__":
    main()
