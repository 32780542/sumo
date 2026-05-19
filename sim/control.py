import statistics

import traci

import sim.config as cfg
from sim.config import *
from sim.scenario import get_active_obstacles
from sim.utils import clamp
from sim.sensing import get_vehicle_params, sense_leader_and_follower, sense_lateral_neighbors, sense_static_obstacle
def is_valid_lane_index(veh_id, lane_index):
    """判断目标车道编号是否在车辆当前道路可用车道范围内。"""
    if lane_index < 0:
        return False
    try:
        lane_id = traci.vehicle.getLaneID(veh_id)
        edge_id = lane_id.rsplit("_", 1)[0]
        return lane_index < traci.edge.getLaneNumber(edge_id)
    except traci.TraCIException:
        return lane_index < cfg.LANE_COUNT_FALLBACK if hasattr(cfg, "LANE_COUNT_FALLBACK") else lane_index <= 2


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
    """基于局部感知检查目标车道前后车辆和障碍物间距，判断是否满足安全换道条件。"""
    ego_pos = traci.vehicle.getLanePosition(veh_id)
    current_lane = traci.vehicle.getLaneIndex(veh_id)
    direction = target_lane - current_lane

    if abs(direction) != 1:
        return False

    try:
        if not traci.vehicle.couldChangeLane(veh_id, direction):
            return False
    except traci.TraCIException:
        return False

    for mode in (0, 1, 2, 3):
        try:
            lateral_neighbors = traci.vehicle.getNeighbors(veh_id, mode)
        except traci.TraCIException:
            continue

        for other_id, gap in lateral_neighbors:
            try:
                if traci.vehicle.getLaneIndex(other_id) != target_lane:
                    continue
            except traci.TraCIException:
                continue

            if -MOBIL_BACK_SAFE_GAP < gap < MOBIL_FRONT_SAFE_GAP:
                return False

    for obstacle in get_active_obstacles():
        if int(obstacle["lane"]) != target_lane:
            continue
        gap = float(obstacle["position"]) - ego_pos
        if -MOBIL_BACK_SAFE_GAP < gap < SENSING_RANGE:
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
        if not is_valid_lane_index(veh_id, target_lane):
            continue
        if not lane_has_space_for_change(veh_id, target_lane):
            continue

        gain = OBSTACLE_LOOKAHEAD
        for obstacle_item in get_active_obstacles():
            if int(obstacle_item["lane"]) == target_lane:
                obstacle_gap = float(obstacle_item["position"]) - traci.vehicle.getLanePosition(veh_id)
                if 0 <= obstacle_gap < OBSTACLE_LOOKAHEAD:
                    gain = min(gain, obstacle_gap)

        if gain > best_gain:
            best_lane = target_lane
            best_gain = gain

    if best_lane != current_lane and best_gain > front_distance + MOBIL_MIN_GAIN:
        traci.vehicle.changeLane(veh_id, best_lane, LANE_CHANGE_DURATION)


def apply_yield_to_signaling_leader(target_speed, veh_id, leader, params):
    """单车自动驾驶层：前车打转向灯时，后车进行礼让减速。"""
    if not leader or leader["distance"] > YIELD_TO_SIGNAL_DISTANCE:
        return target_speed

    signal_direction = get_turn_signal_direction(leader["id"])
    if signal_direction == 0:
        return target_speed

    try:
        current_lane = traci.vehicle.getLaneIndex(veh_id)
        leader_lane = traci.vehicle.getLaneIndex(leader["id"])
    except traci.TraCIException:
        return target_speed
    target_lane = leader_lane + signal_direction

    # 只有当前车处在前车将要并入的车道时，才执行礼让减速。
    if current_lane != target_lane:
        return target_speed

    current_speed = traci.vehicle.getSpeed(veh_id)
    decel_step = min(params["decel"] * STEP_LENGTH, YIELD_TO_SIGNAL_DECEL)
    return min(target_speed, max(MIN_SPEED, current_speed - decel_step))


def maybe_execute_waiting_lane_change(veh_id, current_speed, obstacle, vehicle_states):
    """单车自动驾驶层：障碍物前等待过久时，更积极地尝试安全换道。"""
    state = vehicle_states.setdefault(veh_id, {})
    sim_time = traci.simulation.getTime()

    waiting_near_obstacle = (
        obstacle is not None
        and obstacle["distance"] <= WAITING_OBSTACLE_DISTANCE
        and current_speed <= WAITING_SPEED_THRESHOLD
    )

    if waiting_near_obstacle:
        state.setdefault("waiting_since", sim_time)
    else:
        state.pop("waiting_since", None)
        return False

    waiting_time = sim_time - state["waiting_since"]
    if waiting_time < WAITING_TIME_TO_FORCE_LANE_CHANGE:
        return False

    last_attempt_time = state.get("last_waiting_lane_change_attempt", -1e9)
    if sim_time - last_attempt_time < WAITING_LANE_CHANGE_COOLDOWN:
        return False
    state["last_waiting_lane_change_attempt"] = sim_time

    current_lane = traci.vehicle.getLaneIndex(veh_id)
    best_lane = current_lane
    best_clearance = obstacle["distance"]

    for target_lane in (current_lane - 1, current_lane + 1):
        if not is_valid_lane_index(veh_id, target_lane):
            continue
        if not lane_has_space_for_change(veh_id, target_lane):
            continue

        clearance = get_lane_clearance_ahead(veh_id, target_lane)
        if clearance > best_clearance:
            best_lane = target_lane
            best_clearance = clearance

    if best_lane == current_lane:
        return False

    traci.vehicle.changeLane(veh_id, best_lane, LANE_CHANGE_DURATION)
    return True


def get_lane_clearance_ahead(veh_id, target_lane):
    """仿生层辅助函数：基于局部邻居估计目标车道前方最近车辆/障碍物距离。"""
    ego_pos = traci.vehicle.getLanePosition(veh_id)
    clearance = SENSING_RANGE

    for mode in (0, 1, 2, 3):
        try:
            lateral_neighbors = traci.vehicle.getNeighbors(veh_id, mode)
        except traci.TraCIException:
            continue

        for other_id, gap in lateral_neighbors:
            try:
                if traci.vehicle.getLaneIndex(other_id) != target_lane:
                    continue
            except traci.TraCIException:
                continue

            if 0 <= gap < clearance:
                clearance = gap

    for obstacle_item in get_active_obstacles():
        if int(obstacle_item["lane"]) != target_lane:
            continue
        gap = float(obstacle_item["position"]) - ego_pos
        if 0 <= gap <= SENSING_RANGE and gap < clearance:
            clearance = gap

    return clearance


def detect_queue_blockage(current_speed, leader, obstacle):
    """仿生层辅助函数：识别前方障碍物、停车队列或低速车辆形成的阻塞。"""
    if obstacle and obstacle["distance"] <= SWARM_RECONFIG_LOOKAHEAD:
        return obstacle["distance"]

    if not leader:
        return None

    leader_speed = leader.get("speed", current_speed)
    if (
        leader["distance"] <= SWARM_RECONFIG_LOOKAHEAD
        and leader_speed <= SWARM_RECONFIG_LOW_SPEED
        and current_speed > leader_speed + 1.0
    ):
        return leader["distance"]

    return None


def choose_safe_reconfiguration_direction(veh_id, blockage_distance):
    """选择安全且更畅通的相邻车道方向；返回 -1/1/0，分别表示下/上/不换道。"""
    current_lane = traci.vehicle.getLaneIndex(veh_id)
    best_direction = 0
    best_clearance = blockage_distance

    for target_lane in (current_lane - 1, current_lane + 1):
        if not is_valid_lane_index(veh_id, target_lane):
            continue
        if not lane_has_space_for_change(veh_id, target_lane):
            continue

        clearance = get_lane_clearance_ahead(veh_id, target_lane)
        if clearance > best_clearance and clearance >= SWARM_RECONFIG_MIN_TARGET_CLEARANCE:
            best_direction = target_lane - current_lane
            best_clearance = clearance

    return best_direction


def set_turn_signal(veh_id, direction):
    """设定转向灯信号；direction=-1 表示向低编号车道，1 表示向高编号车道。"""
    if direction < 0:
        traci.vehicle.setSignals(veh_id, RIGHT_SIGNAL_BIT)
    elif direction > 0:
        traci.vehicle.setSignals(veh_id, LEFT_SIGNAL_BIT)
    else:
        traci.vehicle.setSignals(veh_id, 0)


def get_turn_signal_direction(veh_id):
    """读取前车转向灯方向；返回 -1/1/0。"""
    try:
        signals = traci.vehicle.getSignals(veh_id)
    except traci.TraCIException:
        return 0

    if signals & RIGHT_SIGNAL_BIT:
        return -1
    if signals & LEFT_SIGNAL_BIT:
        return 1
    return 0


def maybe_execute_swarm_reconfiguration(veh_id, current_speed, leader, obstacle, vehicle_states):
    """仿生队列变换：通过转向灯信息素传播拥堵与重组意图。"""
    blockage_distance = detect_queue_blockage(current_speed, leader, obstacle)
    current_lane = traci.vehicle.getLaneIndex(veh_id)
    sim_time = traci.simulation.getTime()
    state = vehicle_states.setdefault(veh_id, {})

    if blockage_distance is not None:
        direction = choose_safe_reconfiguration_direction(veh_id, blockage_distance)
        state["pheromone_direction"] = direction
        state["pheromone_time"] = sim_time
        set_turn_signal(veh_id, direction)
        if direction == 0:
            return False
    else:
        direction = 0
        if leader:
            leader_direction = get_turn_signal_direction(leader["id"])
            leader_state = vehicle_states.get(leader["id"], {})
            signal_time = leader_state.get("pheromone_time")
            signal_is_fresh = (
                signal_time is not None
                and sim_time - signal_time <= SWARM_SIGNAL_MEMORY
                and sim_time - signal_time >= SWARM_SIGNAL_DELAY
            )
            if leader_direction and signal_is_fresh:
                direction = leader_direction
                state["pheromone_direction"] = direction
                state["pheromone_time"] = sim_time
                set_turn_signal(veh_id, direction)
            else:
                state.pop("pheromone_direction", None)
                state.pop("pheromone_time", None)
                set_turn_signal(veh_id, 0)
        else:
            state.pop("pheromone_direction", None)
            state.pop("pheromone_time", None)
            set_turn_signal(veh_id, 0)

        if direction == 0:
            return False

    target_lane = current_lane + direction
    if not is_valid_lane_index(veh_id, target_lane):
        state.pop("pheromone_direction", None)
        state.pop("pheromone_time", None)
        set_turn_signal(veh_id, 0)
        return False
    if not lane_has_space_for_change(veh_id, target_lane):
        return False

    traci.vehicle.changeLane(veh_id, target_lane, LANE_CHANGE_DURATION)
    return True


def find_adjacent_formation_reference(veh_id):
    """寻找相邻车道中最近的流动车辆，作为交错长队列组织的局部参考点。"""
    best_reference = None
    best_abs_gap = SENSING_RANGE

    for mode in (0, 1, 2, 3):
        try:
            lateral_neighbors = traci.vehicle.getNeighbors(veh_id, mode)
        except traci.TraCIException:
            continue

        for other_id, gap in lateral_neighbors:
            try:
                other_speed = traci.vehicle.getSpeed(other_id)
                other_lane = traci.vehicle.getLaneIndex(other_id)
            except traci.TraCIException:
                continue

            if other_speed < SWARM_MIN_REFERENCE_SPEED:
                continue

            abs_gap = abs(gap)
            if abs_gap < best_abs_gap:
                best_reference = {
                    "id": other_id,
                    "lane": other_lane,
                    "gap": gap,
                    "speed": other_speed,
                }
                best_abs_gap = abs_gap

    return best_reference


def compute_stagger_slot_delta(ego_lane, current_speed, reference):
    """通过半车距错位形成斜向槽位，服务队列内部换道和重组。"""
    if not reference:
        return 0.0

    stagger_gap = max(SWARM_MIN_PLATOON_GAP, current_speed * SWARM_STAGGER_HEADWAY)
    desired_gap = stagger_gap if ego_lane > reference["lane"] else -stagger_gap
    gap_error = reference["gap"] - desired_gap

    if abs(gap_error) <= SWARM_STAGGER_TOLERANCE:
        return 0.0

    if abs(reference["gap"]) <= SWARM_STAGGER_SLOT_GAP:
        gap_error = -desired_gap

    normalized_error = clamp(
        gap_error / max(stagger_gap, 0.1),
        -1.0,
        1.0,
    )
    return clamp(
        -SWARM_STAGGER_GAIN * normalized_error,
        -SWARM_STAGGER_MAX_DELTA,
        SWARM_STAGGER_MAX_DELTA,
    )


def compute_staggered_formation_delta(veh_id, current_speed, obstacle, reconfigured):
    """计算交错长队列的速度微调量，让相邻车道形成可换道的斜向槽位。"""
    if not SWARM_STAGGER_ENABLE or veh_id is None or reconfigured:
        return 0.0
    if obstacle and obstacle["distance"] <= SWARM_STAGGER_CLEAR_OBSTACLE_DISTANCE:
        return 0.0

    reference = find_adjacent_formation_reference(veh_id)
    if not reference:
        return 0.0

    ego_lane = traci.vehicle.getLaneIndex(veh_id)
    return compute_stagger_slot_delta(ego_lane, current_speed, reference)


def compute_asymmetric_alignment_delta(current_speed, neighbors):
    """只向更快的流动邻居对齐，避免慢车道把拥堵横向传播过来。"""
    faster_neighbors = [
        item
        for item in neighbors
        if item["speed"] > current_speed
        and item["speed"] >= SWARM_MIN_REFERENCE_SPEED
        and item["speed"] >= current_speed * SWARM_REFERENCE_SPEED_RATIO
    ]

    if not faster_neighbors:
        return 0.0

    neighbor_mean_speed = statistics.fmean(item["speed"] for item in faster_neighbors)
    return SWARM_ALIGNMENT_GAIN * (neighbor_mean_speed - current_speed)


def compute_slingshot_delta(current_speed, leader):
    """前车中低速起步并拉开距离时，补偿 IDM 的启动迟滞。"""
    if not SWARM_SLINGSHOT_ENABLE or not leader:
        return 0.0
    if current_speed > SWARM_SLINGSHOT_MAX_EGO_SPEED:
        return 0.0
    if leader["distance"] < SWARM_SLINGSHOT_MIN_GAP:
        return 0.0

    relative_speed = leader.get("speed", current_speed) - current_speed
    if relative_speed <= SWARM_SLINGSHOT_REL_SPEED:
        return 0.0

    return SWARM_MAX_ACCEL_DELTA


def compute_lateral_signal_cooperation_delta(ego_lane, lateral_neighbors):
    """看到侧向车辆打灯并入本车道时，轻微拉开拉链式间隙。"""
    if not SWARM_ZIPPER_ENABLE:
        return 0.0

    delta = 0.0
    for neighbor in lateral_neighbors:
        neighbor_lane = neighbor.get("lane")
        signal_direction = neighbor.get("signal_direction", 0)
        if neighbor_lane is None or signal_direction == 0:
            continue
        if neighbor_lane + signal_direction != ego_lane:
            continue

        signed_distance = neighbor.get("signed_distance", neighbor.get("distance", 0.0))
        if abs(signed_distance) > SWARM_ZIPPER_SIGNAL_DISTANCE:
            continue

        if signed_distance >= 0:
            delta -= SWARM_ZIPPER_YIELD_DELTA
        else:
            delta += SWARM_ZIPPER_ADVANCE_DELTA

    return clamp(delta, -SWARM_ZIPPER_YIELD_DELTA, SWARM_ZIPPER_ADVANCE_DELTA)


def choose_vacancy_reallocation_lane(ego_lane, lateral_neighbors, lane_clearance_fn, lane_safe_fn):
    """相邻车辆想并入本车道时，选择另一侧安全通畅的空车道主动让位。"""
    if not SWARM_VACANCY_REALLOCATION_ENABLE:
        return None

    best_lane = None
    best_clearance = SWARM_VACANCY_MIN_CLEARANCE

    for neighbor in lateral_neighbors:
        neighbor_lane = neighbor.get("lane")
        signal_direction = neighbor.get("signal_direction", 0)
        if neighbor_lane is None or signal_direction == 0:
            continue
        if neighbor_lane + signal_direction != ego_lane:
            continue

        signed_distance = neighbor.get("signed_distance", neighbor.get("distance", 0.0))
        if abs(signed_distance) > SWARM_VACANCY_SIGNAL_DISTANCE:
            continue

        vacancy_lane = ego_lane + signal_direction
        if vacancy_lane < 0 or vacancy_lane > 2:
            continue
        if not lane_safe_fn(vacancy_lane):
            continue

        clearance = lane_clearance_fn(vacancy_lane)
        if clearance > best_clearance:
            best_lane = vacancy_lane
            best_clearance = clearance

    return best_lane


def maybe_execute_vacancy_reallocation(veh_id, ego_lane, lateral_neighbors, vehicle_states):
    """优先主动挪向另一侧空车道，为相邻车辆并入本车道释放空间。"""
    state = vehicle_states.setdefault(veh_id, {})
    sim_time = traci.simulation.getTime()
    last_time = state.get("last_vacancy_reallocation_time", -1e9)
    if sim_time - last_time < SWARM_VACANCY_COOLDOWN:
        return False

    target_lane = choose_vacancy_reallocation_lane(
        ego_lane,
        lateral_neighbors,
        lambda lane: get_lane_clearance_ahead(veh_id, lane),
        lambda lane: lane_has_space_for_change(veh_id, lane),
    )
    if target_lane is None:
        return False

    traci.vehicle.changeLane(veh_id, target_lane, LANE_CHANGE_DURATION)
    state["last_vacancy_reallocation_time"] = sim_time
    state["pheromone_direction"] = target_lane - ego_lane
    state["pheromone_time"] = sim_time
    set_turn_signal(veh_id, target_lane - ego_lane)
    return True


def compute_reconfiguration_momentum_delta(reconfigured, signal_direction):
    """重组或准备重组时维持动量，最终仍受机械约束和安全层限制。"""
    if reconfigured:
        return SWARM_RECONFIG_MOMENTUM_DELTA
    if signal_direction:
        return min(SWARM_RECONFIG_SPEED_BONUS, SWARM_RECONFIG_MOMENTUM_DELTA)
    return 0.0


def apply_swarm_layer(
    base_speed,
    current_speed,
    neighbors,
    leader,
    veh_id=None,
    obstacle=None,
    vehicle_states=None,
):
    """上层仿生编队规则：有限感知下的 ADEPT-Boids 混合局部规则。

    设计原则：
    1. 不替代单车自动驾驶层，只在 IDM/MOBIL 给出的 base_speed 上做小幅修正。
    2. 不直接处理危险工况，最终仍交给后续安全层和 SUMO fallback 兜底。
    3. 只使用本车有限感知到的邻居信息，让类似编队的行为自发涌现。
    4. 低速/停止邻居代表局部受阻，不作为畅通车辆的速度对齐对象。
    """
    params = get_vehicle_params(veh_id, vehicle_states or {}) if veh_id is not None else {}
    vehicle_max_speed = params.get("max_speed", MAX_SPEED)
    vehicle_accel = params.get("accel", 2.6)
    vehicle_decel = params.get("decel", 4.5)

    reconfigured = False
    signal_direction = 0
    if veh_id is not None and vehicle_states is not None:
        reconfigured = maybe_execute_swarm_reconfiguration(
            veh_id, current_speed, leader, obstacle, vehicle_states
        )
        signal_direction = vehicle_states.get(veh_id, {}).get("pheromone_direction", 0)

    speed_delta = compute_reconfiguration_momentum_delta(reconfigured, signal_direction)
    if not reconfigured and veh_id is not None and vehicle_states is not None:
        state = vehicle_states.get(veh_id, {})
        signal_time = state.get("pheromone_time")
        if signal_time is not None and traci.simulation.getTime() - signal_time <= SWARM_SIGNAL_MEMORY:
            # 看到前车信息素但暂未完成换道时，轻微预减速，为队列变换留出空间。
            speed_delta -= SWARM_SIGNAL_PREPARE_DECEL

    # 交错长队列组织：仅在畅通直道上启用，遇障碍物或重组时暂停。
    speed_delta += compute_staggered_formation_delta(
        veh_id,
        current_speed,
        obstacle,
        reconfigured,
    )

    # Boids-Alignment：只向更快的流动邻居对齐，避免慢车道造成拥堵横向传染。
    speed_delta += compute_asymmetric_alignment_delta(current_speed, neighbors)

    if veh_id is not None:
        try:
            ego_lane = traci.vehicle.getLaneIndex(veh_id)
        except traci.TraCIException:
            ego_lane = None
        if ego_lane is not None:
            reallocated = False
            if vehicle_states is not None:
                reallocated = maybe_execute_vacancy_reallocation(
                    veh_id,
                    ego_lane,
                    neighbors,
                    vehicle_states,
                )
            if reallocated:
                reconfigured = True
                signal_direction = vehicle_states.get(veh_id, {}).get("pheromone_direction", signal_direction)
                speed_delta += compute_reconfiguration_momentum_delta(reconfigured, signal_direction)
            else:
                speed_delta += compute_lateral_signal_cooperation_delta(ego_lane, neighbors)

    if leader:
        speed_delta += compute_slingshot_delta(current_speed, leader)
        # ADEPT-R1/R2/R3：围绕 d +/- delta 的局部间距触发，不需要全局领车。
        desired_gap = max(
            SWARM_MIN_PLATOON_GAP,
            current_speed * SWARM_DESIRED_HEADWAY,
        )
        lower_gap = max(SAFE_DISTANCE, desired_gap - SWARM_GAP_TOLERANCE)
        upper_gap = desired_gap + SWARM_GAP_TOLERANCE
        gap = leader["distance"]
        leader_speed = leader.get("speed", current_speed)

        if gap > upper_gap:
            # R1/Boids-Cohesion：前方局部车群较远时，轻微加速追赶。
            gap_ratio = min((gap - upper_gap) / max(desired_gap, 0.1), 1.0)
            speed_delta += SWARM_COHESION_GAIN * gap_ratio
        elif gap < lower_gap:
            # R2/Boids-Separation：间距偏小时轻微降速，但幅度受限，安全层仍是最终约束。
            gap_ratio = min((lower_gap - gap) / max(desired_gap, 0.1), 1.0)
            speed_delta -= SWARM_SEPARATION_GAIN * gap_ratio
        else:
            # R3/Match：间距处于容许带内时，弱匹配前车速度以形成局部队列。
            speed_delta += SWARM_MATCH_GAIN * (leader_speed - current_speed)

    # 仿生层最终钳位：不能突破车辆自身最大速度、最大加速度和最大减速度。
    speed_delta = clamp(speed_delta, -SWARM_MAX_DECEL_DELTA, SWARM_MAX_ACCEL_DELTA)
    raw_target_speed = clamp(base_speed + speed_delta, MIN_SPEED, vehicle_max_speed)
    lower_mechanical_speed = max(MIN_SPEED, current_speed - vehicle_decel * STEP_LENGTH)
    upper_mechanical_speed = min(vehicle_max_speed, current_speed + vehicle_accel * STEP_LENGTH)
    return clamp(raw_target_speed, lower_mechanical_speed, upper_mechanical_speed)


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
    maybe_execute_waiting_lane_change(veh_id, current_speed, obstacle, vehicle_states)

    target_speed = idm_target_speed(current_speed, leader, obstacle, params)
    target_speed = apply_yield_to_signaling_leader(target_speed, veh_id, leader, params)
    if enable_swarm_rules:
        # 仿生编队层采用温和局部规则，最终仍受安全层限制。
        target_speed = apply_swarm_layer(
            target_speed,
            current_speed,
            neighbors,
            leader,
            veh_id=veh_id,
            obstacle=obstacle,
            vehicle_states=vehicle_states,
        )
    target_speed = apply_safety_layer(target_speed, veh_id, leader, params)
    target_speed = apply_static_obstacle_safety(target_speed, veh_id, obstacle, params)
    return clamp(target_speed, MIN_SPEED, allowed_speed)

