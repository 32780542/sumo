import unittest
from datetime import datetime

import main
import plot_speed_heatmap
import sim.config as cfg
from sim import scenario


class SwarmRuleTests(unittest.TestCase):
    def test_asymmetric_alignment_ignores_slower_neighbors(self):
        neighbors = [
            {"id": "slow", "speed": 8.0},
            {"id": "stopped", "speed": 0.0},
        ]

        delta = main.compute_asymmetric_alignment_delta(12.0, neighbors)

        self.assertEqual(delta, 0.0)

    def test_asymmetric_alignment_uses_only_faster_neighbors(self):
        neighbors = [
            {"id": "slow", "speed": 10.0},
            {"id": "fast", "speed": 16.0},
            {"id": "faster", "speed": 18.0},
        ]

        delta = main.compute_asymmetric_alignment_delta(12.0, neighbors)

        self.assertGreater(delta, 0.0)
        self.assertAlmostEqual(delta, main.SWARM_ALIGNMENT_GAIN * 5.0)

    def test_slingshot_accelerates_when_low_speed_leader_pulls_away(self):
        leader = {"id": "front", "distance": 18.0, "speed": 7.0}

        delta = main.compute_slingshot_delta(4.0, leader)

        self.assertEqual(delta, main.SWARM_MAX_ACCEL_DELTA)

    def test_slingshot_does_not_accelerate_into_short_gap(self):
        leader = {"id": "front", "distance": 4.0, "speed": 7.0}

        delta = main.compute_slingshot_delta(4.0, leader)

        self.assertEqual(delta, 0.0)

    def test_lateral_signal_cooperation_yields_to_side_front_merging_in(self):
        neighbors = [
            {
                "id": "side_front",
                "lane": 1,
                "signed_distance": 12.0,
                "signal_direction": 1,
                "speed": 8.0,
            }
        ]

        delta = main.compute_lateral_signal_cooperation_delta(ego_lane=2, lateral_neighbors=neighbors)

        self.assertLess(delta, 0.0)

    def test_lateral_signal_cooperation_accelerates_for_side_rear_merging_in(self):
        neighbors = [
            {
                "id": "side_rear",
                "lane": 1,
                "signed_distance": -10.0,
                "signal_direction": 1,
                "speed": 8.0,
            }
        ]

        delta = main.compute_lateral_signal_cooperation_delta(ego_lane=2, lateral_neighbors=neighbors)

        self.assertGreater(delta, 0.0)

    def test_reconfiguration_momentum_uses_full_swarm_accel_budget(self):
        delta = main.compute_reconfiguration_momentum_delta(reconfigured=True, signal_direction=0)

        self.assertEqual(delta, main.SWARM_MAX_ACCEL_DELTA)

    def test_reconfiguration_momentum_prepares_when_signal_active(self):
        delta = main.compute_reconfiguration_momentum_delta(reconfigured=False, signal_direction=-1)

        self.assertGreater(delta, 0.0)

    def test_stagger_slot_delta_pushes_parallel_neighbor_out_of_blind_slot(self):
        reference = {"id": "side", "lane": 1, "gap": 0.5, "speed": 12.0}

        delta = main.compute_stagger_slot_delta(ego_lane=2, current_speed=12.0, reference=reference)

        self.assertGreater(delta, 0.0)

    def test_stagger_slot_delta_reduces_speed_when_too_far_ahead_of_target_slot(self):
        reference = {"id": "side", "lane": 1, "gap": 20.0, "speed": 12.0}

        delta = main.compute_stagger_slot_delta(ego_lane=2, current_speed=12.0, reference=reference)

        self.assertLess(delta, 0.0)

    def test_vacancy_reallocation_chooses_opposite_empty_lane(self):
        lateral_neighbors = [
            {
                "id": "merging",
                "lane": 0,
                "signed_distance": 8.0,
                "signal_direction": 1,
                "speed": 10.0,
            }
        ]

        target_lane = main.choose_vacancy_reallocation_lane(
            ego_lane=1,
            lateral_neighbors=lateral_neighbors,
            lane_clearance_fn=lambda lane: 50.0 if lane == 2 else 0.0,
            lane_safe_fn=lambda lane: True,
        )

        self.assertEqual(target_lane, 2)

    def test_vacancy_reallocation_rejects_unclear_lane(self):
        lateral_neighbors = [
            {
                "id": "merging",
                "lane": 0,
                "signed_distance": 8.0,
                "signal_direction": 1,
                "speed": 10.0,
            }
        ]

        target_lane = main.choose_vacancy_reallocation_lane(
            ego_lane=1,
            lateral_neighbors=lateral_neighbors,
            lane_clearance_fn=lambda lane: 10.0,
            lane_safe_fn=lambda lane: True,
        )

        self.assertIsNone(target_lane)

    def test_build_vehicle_trace_filename_contains_metadata(self):
        filename = main.build_vehicle_trace_filename(
            enable_swarm_rules=True,
            sim_seconds=80,
            flow_probability=0.025,
            timestamp=datetime(2026, 5, 10, 12, 34, 56),
        )

        self.assertEqual(filename, "vehicle_traces_20260510_123456_swarm_on_80s_flow_0p025.csv")

    def test_append_vehicle_trace_sample_derives_acceleration(self):
        traces = {}

        main.append_vehicle_trace_sample(
            traces,
            veh_id="veh_1",
            speed=10.0,
            x_position=12.0,
            y_position=1.0,
            angle=90.0,
            step_index=3,
            dt=0.1,
        )
        main.append_vehicle_trace_sample(
            traces,
            veh_id="veh_1",
            speed=10.5,
            x_position=13.0,
            y_position=1.0,
            angle=90.0,
            step_index=4,
            dt=0.1,
        )

        trace = traces["veh_1"]
        self.assertEqual(trace["enter_step"], 3)
        self.assertEqual(trace["speed_sequence"], [10.0, 10.5])
        self.assertEqual(trace["acceleration_sequence"], [0.0, 5.0])

    def test_mark_vehicle_trace_completed_sets_completed_flag(self):
        traces = {"veh_1": {"completed": 0}}

        main.mark_vehicle_trace_completed(traces, "veh_1")

        self.assertEqual(traces["veh_1"]["completed"], 1)

    def test_heatmap_binning_averages_speed_by_time_and_space(self):
        rows = [
            {
                "X 坐标序列": "[10, 60, 60, 110]",
                "速度序列 (m/s)": "[10, 20, 30, 40]",
            },
            {
                "X 坐标序列": "[20, 60, 160]",
                "速度序列 (m/s)": "[30, 40, 50]",
            },
        ]

        heatmap, time_edges, space_edges = plot_speed_heatmap.build_speed_heatmap_grid(
            rows,
            road_length=200,
            space_bin_size=50,
            max_speed=30,
            target_time_bins=2,
        )

        self.assertEqual(heatmap.shape, (2, 4))
        self.assertEqual(list(time_edges), [0, 2, 4])
        self.assertEqual(list(space_edges), [0, 50, 100, 150, 200])
        self.assertAlmostEqual(heatmap[0, 0], 20.0)
        self.assertAlmostEqual(heatmap[0, 1], 30.0)
        self.assertAlmostEqual(heatmap[1, 1], 30.0)
        self.assertAlmostEqual(heatmap[1, 2], 40.0)
        self.assertAlmostEqual(heatmap[1, 3], 50.0)

    def test_lane_drop_scenario_has_two_edges_and_no_obstacles(self):
        old_mode = cfg.SCENARIO_MODE
        old_name = cfg.SCENARIO_NAME
        try:
            cfg.SCENARIO_MODE = 2
            cfg.SCENARIO_NAME = None
            spec = scenario.get_current_scenario_spec()
        finally:
            cfg.SCENARIO_MODE = old_mode
            cfg.SCENARIO_NAME = old_name

        self.assertEqual(spec["route_edges"], "west_to_merge merge_to_east")
        self.assertFalse(spec["enable_obstacles"])
        self.assertEqual(spec["depart_lanes"], (2, 1, 0))
        self.assertEqual(spec["lane_count"], 3)

    def test_string_scenario_name_still_supported(self):
        old_mode = cfg.SCENARIO_MODE
        old_name = cfg.SCENARIO_NAME
        try:
            cfg.SCENARIO_MODE = 1
            cfg.SCENARIO_NAME = "lane_drop_3_to_2"
            spec = scenario.get_current_scenario_spec()
        finally:
            cfg.SCENARIO_MODE = old_mode
            cfg.SCENARIO_NAME = old_name

        self.assertEqual(spec["name"], "lane_drop_3_to_2")

    def test_lane_drop_scenario_has_no_active_control_obstacles(self):
        old_mode = cfg.SCENARIO_MODE
        old_name = cfg.SCENARIO_NAME
        try:
            cfg.SCENARIO_MODE = 2
            cfg.SCENARIO_NAME = None
            active_obstacles = scenario.get_active_obstacles()
        finally:
            cfg.SCENARIO_MODE = old_mode
            cfg.SCENARIO_NAME = old_name

        self.assertEqual(active_obstacles, [])

    def test_random_vehicle_generation_uses_homogeneous_params_when_disabled(self):
        class FakeSimulation:
            @staticmethod
            def getTime():
                return 0.0

        class FakeVehicle:
            def __init__(self):
                self.added = []
                self.params = {}

            def add(self, **kwargs):
                self.added.append(kwargs)
                self.params[kwargs["vehID"]] = {}

            def setSpeedMode(self, veh_id, value):
                self.params[veh_id]["speed_mode"] = value

            def setLaneChangeMode(self, veh_id, value):
                self.params[veh_id]["lane_change_mode"] = value

            def setMaxSpeed(self, veh_id, value):
                self.params[veh_id]["max_speed"] = value

            def setAccel(self, veh_id, value):
                self.params[veh_id]["accel"] = value

            def setDecel(self, veh_id, value):
                self.params[veh_id]["decel"] = value

            def setMinGap(self, veh_id, value):
                self.params[veh_id]["min_gap"] = value

            def setTau(self, veh_id, value):
                self.params[veh_id]["tau"] = value

        class FakeTraCI:
            TraCIException = Exception
            simulation = FakeSimulation()

            def __init__(self):
                self.vehicle = FakeVehicle()

        old_traci = scenario.traci
        old_probability = cfg.TRAFFIC_FLOW_PROBABILITY
        old_gradients = cfg.FLOW_GRADIENTS
        old_heterogeneity = cfg.ENABLE_VEHICLE_HETEROGENEITY
        fake_traci = FakeTraCI()
        vehicle_states = {}

        try:
            scenario.traci = fake_traci
            cfg.TRAFFIC_FLOW_PROBABILITY = 1.0
            cfg.FLOW_GRADIENTS = []
            cfg.ENABLE_VEHICLE_HETEROGENEITY = False

            next_vehicle_id = scenario.maybe_add_random_vehicles(0, vehicle_states)
        finally:
            scenario.traci = old_traci
            cfg.TRAFFIC_FLOW_PROBABILITY = old_probability
            cfg.FLOW_GRADIENTS = old_gradients
            cfg.ENABLE_VEHICLE_HETEROGENEITY = old_heterogeneity

        self.assertEqual(next_vehicle_id, 3)
        self.assertEqual(len(vehicle_states), 3)
        for veh_id, state in vehicle_states.items():
            for param_name, expected_value in cfg.HOMOGENEOUS_VEHICLE_PARAMS.items():
                self.assertEqual(state[param_name], expected_value)
            for param_name in ("max_speed", "accel", "decel", "min_gap", "tau"):
                expected_value = cfg.HOMOGENEOUS_VEHICLE_PARAMS[param_name]
                self.assertEqual(fake_traci.vehicle.params[veh_id][param_name], expected_value)


if __name__ == "__main__":
    unittest.main()
