import traci

from sim.scenario import get_active_obstacles
from sim.config import *
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


def add_neighbor(neighbors, veh_id, distance, signed_distance=None, lane=None, signal_direction=0):
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
            "signed_distance": float(signed_distance if signed_distance is not None else distance),
            "lane": lane,
            "signal_direction": signal_direction,
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
            try:
                neighbor_lane = traci.vehicle.getLaneIndex(neighbor_id)
            except traci.TraCIException:
                neighbor_lane = None
            add_neighbor(
                neighbors,
                neighbor_id,
                abs(distance),
                signed_distance=distance,
                lane=neighbor_lane,
                signal_direction=get_neighbor_turn_signal_direction(neighbor_id),
            )

    return neighbors


def get_neighbor_turn_signal_direction(veh_id):
    """读取侧向邻车转向灯方向，避免感知层依赖控制层。"""
    try:
        signals = traci.vehicle.getSignals(veh_id)
    except traci.TraCIException:
        return 0

    if signals & RIGHT_SIGNAL_BIT:
        return -1
    if signals & LEFT_SIGNAL_BIT:
        return 1
    return 0


def sense_static_obstacle(veh_id):
    """感知当前车道前方最近的静态障碍物；障碍物是 POI，不属于车辆列表。"""
    active_obstacles = get_active_obstacles()
    if not active_obstacles:
        return None

    lane = traci.vehicle.getLaneIndex(veh_id)
    position = traci.vehicle.getLanePosition(veh_id)
    nearest = None

    for obstacle in active_obstacles:
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

