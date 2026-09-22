#!/usr/bin/env python3
import carla
import copy
import hashlib
import heapq
import itertools
import json
import math
import pygame
import argparse
import os
import re
import xml.etree.ElementTree as ET

from roundabout_2b import (
    SCHEMA_VERSION as ROUNDABOUT_SCHEMA_VERSION,
    canonical_hash as roundabout_hash,
    gate_crossing,
    gate_signed_distance,
    required_vt1_initial_upstream_distance,
    roundabout_conflict_sync_distances,
    roundabout_condition_payload,
    roundabout_route_fingerprint,
    validate_roundabout_config,
)

from traffic_light_1d import (
    MANEUVERS,
    SCHEMA_VERSION,
    canonical_hash,
    generate_signal_case_matrix,
    update_matrix_manifest,
)

# ====================== 核心配置 ======================
SCREEN_SIZE = (1200, 800)
ZOOM = 15
OFFSET_X = 600
OFFSET_Y = 400
ZOOM_SPEED = 1.2
MIN_ZOOM = 0.2
MAX_ZOOM = 100.0
VIEWPORT_DRAW_MARGIN = 64
MAX_ROUTE_VISIBLE_TAIL_PX = 240.0
ROUNDABOUT_ARROW_SCREEN_SPACING_PX = 28.0
ROUNDABOUT_ARROW_LENGTH_PX = 14.0
ROUNDABOUT_ARROW_HEAD_LENGTH_PX = 5.0
ROUNDABOUT_CONFIRMED_ROUTE_SAMPLE_M = 4.0
ROUNDABOUT_LOCAL_TRACE_RESOLUTION_M = 1.0
ROUNDABOUT_LOCAL_TRACE_GOAL_TOLERANCE_M = 3.0
ROUNDABOUT_LOCAL_TRACE_MAX_RATIO = 3.0
ROUNDABOUT_LOCAL_TRACE_EXTRA_M = 20.0
ROUNDABOUT_LOCAL_TRACE_MAX_M = 120.0
ROUNDABOUT_LOCAL_TRACE_MAX_LANE_CHANGES = 2
ROUNDABOUT_LANE_CHANGE_PENALTY_M = 8.0
ROUNDABOUT_LANE_CHANGE_MAX_HEADING_DEG = 45.0
ROUNDABOUT_LANE_CHANGE_FORWARD_M = 8.0
ROUNDABOUT_CLICK_CANDIDATE_MIN_RADIUS_M = 5.0
ROUNDABOUT_CLICK_CANDIDATE_MAX_RADIUS_M = 12.0
ROUNDABOUT_CLICK_CANDIDATE_SCREEN_RADIUS_PX = 48.0
ROUNDABOUT_CLICK_CANDIDATE_LIMIT = 24
ROUNDABOUT_PARALLEL_LANE_LONGITUDINAL_M = 6.0
ROUNDABOUT_PARALLEL_LANE_MAX_WIDTHS = 3.2
ROUNDABOUT_PARALLEL_LANE_MIN_LATERAL_M = 8.0
ROUNDABOUT_PARALLEL_LANE_HEADING_ALIGNMENT = 0.85
ROUNDABOUT_PARALLEL_LANE_CONTIGUOUS_WIDTHS = 1.6
ROUNDABOUT_HISTORY_LIMIT = 100
ROUTE_ANCHOR_RECOMMENDED_MIN_M = 10.0
ROUTE_ANCHOR_RECOMMENDED_MAX_M = 20.0
ROUTE_ANCHOR_STRAIGHT_MAX_M = 30.0
ROUTE_ANCHOR_BRANCH_MAX_M = 10.0
VT2_DEFAULT_UPSTREAM_DISTANCE_M = 2.0
VT2_MINIMUM_GATE_CLEARANCE_M = 2.0
VUT_START_ENTRY_MARGIN_M = 3.0
VUT_ENTRY_ARRIVAL_DISTANCE_M = 3.0
VT1_ENTRY_UPSTREAM_MINIMUM_M = 3.0
VT1_SETUP_BUDGET_S = 2.0
VT1_STABLE_DURATION_S = 1.0
VT1_TIMING_BUFFER_M = 2.0
VT1_INITIAL_MINIMUM_M = 15.0
VT1_CONFLICT_MAX_OFFSET_M = 12.0
VT1_CONFLICT_HEADWAY_TARGET_S = 1.0
VT1_CONFLICT_HEADWAY_TOLERANCE_S = 0.5
VT1_RECOMMENDATION_CONFLICT_PROXIMITY_M = 3.0
VT1_RECOMMENDATION_CONFLICT_MAX_HEADING_DEG = 60.0
VT1_RECOMMENDATION_EXIT1_REVERSE_MAX_M = 160.0
VUT_APPROACH_SPEED_ASSUMPTION_KMH = 15.0
VUT_APPROACH_TIME_BUDGET_S = 15.0
VT1_RECOMMENDED_SEGMENT_EXTRA_M = 25.0
VT2_RECOMMENDED_SEGMENT_MIN_M = 2.0
VT2_RECOMMENDED_SEGMENT_MAX_M = 30.0
ROUNDABOUT_RECOMMENDATION_TRACE_STEP_M = 2.0
ROUNDABOUT_RECOMMENDATION_BRANCH_LOOKBACKS_M = (
    2.0, 4.0, 8.0, 12.0, 20.0, 30.0, 40.0)
ROUNDABOUT_COLOR_IN = (80, 255, 120)
ROUNDABOUT_COLOR_OUT = (80, 170, 255)
ROUNDABOUT_COLOR_VUT = (0, 225, 255)
ROUNDABOUT_COLOR_EXIT_1 = (255, 225, 70)
ROUNDABOUT_COLOR_EXIT_2 = (255, 160, 40)
ROUNDABOUT_COLOR_EXIT_3 = (255, 90, 70)
ROUNDABOUT_COLOR_VT1 = (255, 165, 0)
ROUNDABOUT_COLOR_VT2 = (210, 90, 255)
ROUNDABOUT_COLOR_VT1_RECOMMENDED = (255, 125, 20)
ROUNDABOUT_COLOR_VT2_RECOMMENDED = (235, 70, 255)
ROUNDABOUT_COLOR_UNCONFIRMED = (255, 220, 70)
ROUNDABOUT_COLOR_FAILED_ROUTE = (125, 125, 125)
ROUNDABOUT_COLOR_FAILED_SEGMENT = (255, 70, 95)
ROUNDABOUT_COLOR_TOPOLOGY_CONFLICT = (255, 55, 55)
ROUNDABOUT_TOPOLOGY_SAME_CROSS_SECTION_M = 3.0
ROUNDABOUT_EXIT_GATE_MARGIN_M = 0.35
ROUNDABOUT_EXIT_GATE_MIN_ALIGNMENT = 0.5
ROUNDABOUT_EXIT_GATE_LANE_WINDOW = 3
ROUNDABOUT_EXIT_GATE_TOPOLOGY_MATCH_M = 40.0
ROUNDABOUT_EXIT_GATE_DOWNSTREAM_CLEARANCE_M = 2.0
ROUTE_ID = "route_01"
ROUNDABOUT_EDITOR_BUILD = "20260819-placement-segments-v5"
# ======================================================


class RoundaboutRouteTraceError(RuntimeError):
    """Actionable failure from the bounded, forward-only 2.b route tracer."""

    def __init__(self, message, partial_route=None, segment_index=None,
                 segment_start=None, segment_target=None):
        super().__init__(message)
        self.partial_route = list(partial_route or [])
        self.segment_index = segment_index
        self.segment_start = segment_start
        self.segment_target = segment_target

class CarlaMapEditor0916:
    def __init__(self, host='127.0.0.1', port=2000, town_name="TOWN10HD_Opt",
                 scenario="1", save_dir="output", maneuver="straight", seed=41798,
                 roundabout_speed_limit_kmh=None, roundabout_variants=None,
                 roundabout_conflict_headway_s=VT1_CONFLICT_HEADWAY_TARGET_S,
                 roundabout_conflict_tolerance_s=VT1_CONFLICT_HEADWAY_TOLERANCE_S,
                 roundabout_vt1_post_exit_clearance_distance_m=25.0):
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        normalized_scenario = str(scenario).lower().replace(".", "")
        self.town_name = self._resolve_loaded_map_name(self.map, town_name)

        # ======================
        # ✅ 8 大天气类型 × 每种 8 个强度 = 64 种天气
        # ======================
        self.weather_groups = {
            "sunny": [],
            "cloudy": [],
            "overcast": [],
            "light_rain": [],
            "heavy_rain": [],
            "fog": [],
            "wind": [],
            "sandstorm": [],
            # 下面全部复制加进去
            "night": [],
            "dusk": [],
            "dawn": [],
            "cold_fog": [],
            "rainy_dusk": [],
            "snowy_light": [],
            "snowy_heavy": []
        }

        # ----------------------
        # 晴天 8 个强度
        # ----------------------
        for i in range(8):
            self.weather_groups["sunny"].append({
                "cloudiness": 0 + i*5,
                "dust_storm": 0.0,
                "fog_density": 0.0,
                "fog_distance": 100.0,
                "fog_falloff": 1.0,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 1.0,
                "sun_altitude_angle": -30 + i*15,
                "sun_azimuth_angle": 120.0,
                "wetness": 0.0,
                "wind_intensity": 5.0
            })

        # ----------------------
        # 多云 8 个强度
        # ----------------------
        for i in range(8):
            self.weather_groups["cloudy"].append({
                "cloudiness": 30 + i*8,
                "dust_storm": 0.0,
                "fog_density": 0.0,
                "fog_distance": 100.0,
                "fog_falloff": 1.0,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.95,
                "sun_altitude_angle": 45.0,
                "sun_azimuth_angle": 120.0,
                "wetness": 0.0,
                "wind_intensity": 10.0 + i*3
            })

        # ----------------------
        # 阴天 8 个强度
        # ----------------------
        for i in range(8):
            self.weather_groups["overcast"].append({
                "cloudiness": 50 + i*2.5,
                "dust_storm": 0.0,
                "fog_density": 0.0,
                "fog_distance": 100.0,
                "fog_falloff": 1.0,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.7 - i*0.05,
                "sun_altitude_angle": 25.0,
                "sun_azimuth_angle": 120.0,
                "wetness": 0.0,
                "wind_intensity": 15.0 + i*4
            })

        # ----------------------
        # 小雨 8 个强度
        # ----------------------
        for i in range(8):
            self.weather_groups["light_rain"].append({
                "cloudiness": 10.0,
                "dust_storm": 0.0,
                "fog_density": 0 + i*4,
                "fog_distance": 80 - i*5,
                "fog_falloff": 1.0,
                "mie_scattering_scale": 0.03,
                "precipitation": 5 + i*4,
                "precipitation_deposits": 10 + i*6,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.75 - i*0.06,
                "sun_altitude_angle": 20.0,
                "sun_azimuth_angle": 120.0,
                "wetness": 20 + i*8,
                "wind_intensity": 18 + i*4
            })

        # ----------------------
        # 大雨 8 个强度
        # ----------------------
        for i in range(8):
            self.weather_groups["heavy_rain"].append({
                "cloudiness": 0.0,
                "dust_storm": 0.0,
                "fog_density": 0 + i*8,
                "fog_distance": 60 - i*6,
                "fog_falloff": 1.0,
                "mie_scattering_scale": 0.03,
                "precipitation": 50 + i*7,
                "precipitation_deposits": 60 + i*8,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.5 - i*0.05,
                "sun_altitude_angle": 15.0,
                "sun_azimuth_angle": 120.0,
                "wetness": 70 + i*8,
                "wind_intensity": 25 + i*6
            })

        # ----------------------
        # 大雾 8 个强度
        # ----------------------
        for i in range(8):
            self.weather_groups["fog"].append({
                "cloudiness": 100 + i*5,
                "dust_storm": 0.0,
                "fog_density": 30 + i*10,
                "fog_distance": 70 - i*8,
                "fog_falloff": 0.8 - i*0.05,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.5 - i*0.05,
                "sun_altitude_angle": 35.0,
                "sun_azimuth_angle": 120.0,
                "wetness": 0.0,
                "wind_intensity": 5.0 + i*2
            })

        # ----------------------
        # 大风 8 个强度
        # ----------------------
        for i in range(8):
            self.weather_groups["wind"].append({
                "cloudiness": 70 + i*4,
                "dust_storm": 0.0,
                "fog_density": 0.0,
                "fog_distance": 100.0,
                "fog_falloff": 1.0,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.9,
                "sun_altitude_angle": 45.0,
                "sun_azimuth_angle": 120.0,
                "wetness": 0.0,
                "wind_intensity": 20 + i*8
            })

        # ----------------------
        # 沙尘暴 8 个强度
        # ----------------------
        for i in range(8):
            self.weather_groups["sandstorm"].append({
                "cloudiness": 0.0,
                "dust_storm": 100 + i*8,
                "fog_density": 40 + i*10,
                "fog_distance": 60 - i*7,
                "fog_falloff": 0.6 - i*0.05,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.4 - i*0.04,
                "sun_altitude_angle": 45.0,
                "sun_azimuth_angle": 120.0,
                "wetness": 0.0,
                "wind_intensity": 30 + i*10
            })

        # ----------------------
        # 黑夜 8 个强度
        # ----------------------
        for i in range(8):
            self.weather_groups["night"].append({
                "cloudiness": 100.0,
                "dust_storm": 0.0,
                "fog_density": 0.0,
                "fog_distance": 100.0,
                "fog_falloff": 1.0,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.6 - i * 0.07,
                "sun_altitude_angle": -70.0 - i * 5,
                "sun_azimuth_angle": 120.0,
                "wetness": 0.0,
                "wind_intensity": 5.0
            })

        # ----------------------
        # 黄昏 傍晚 8 强度
        # ----------------------
        for i in range(8):
            self.weather_groups["dusk"].append({
                "cloudiness": 40 + i*6,
                "dust_storm": 0.0,
                "fog_density": 0.0,
                "fog_distance": 100.0,
                "fog_falloff": 1.0,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.85 - i*0.06,
                "sun_altitude_angle": 15.0 - i*4,
                "sun_azimuth_angle": 150.0,
                "wetness": 0.0,
                "wind_intensity": 8.0 + i*2
            })

        # ----------------------
        # 黎明 清晨 8 强度
        # ----------------------
        for i in range(8):
            self.weather_groups["dawn"].append({
                "cloudiness": 30 + i*5,
                "dust_storm": 0.0,
                "fog_density": 5 + i*3,
                "fog_distance": 90 - i*6,
                "fog_falloff": 0.9,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 0.0,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.75 + i*0.03,
                "sun_altitude_angle": -10.0 + i*5,
                "sun_azimuth_angle": 60.0,
                "wetness": 10.0 + i*5,
                "wind_intensity": 6.0 + i*2
            })

        # ----------------------
        # 晨雾/冷雾 8 强度
        # ----------------------
        for i in range(8):
            self.weather_groups["cold_fog"].append({
                "cloudiness": 50 + i*5,
                "dust_storm": 0.0,
                "fog_density": 20 + i*12,
                "fog_distance": 80 - i*9,
                "fog_falloff": 0.75 - i*0.04,
                "mie_scattering_scale": 0.03,
                "precipitation": 0.0,
                "precipitation_deposits": 5.0 + i*3,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.6 - i*0.05,
                "sun_altitude_angle": 20.0,
                "sun_azimuth_angle": 90.0,
                "wetness": 15.0 + i*6,
                "wind_intensity": 4.0 + i*1.5
            })

        # ----------------------
        # 雨夜黄昏 8 强度
        # ----------------------
        for i in range(8):
            self.weather_groups["rainy_dusk"].append({
                "cloudiness": 100.0,
                "dust_storm": 0.0,
                "fog_density": 10 + i*6,
                "fog_distance": 75 - i*7,
                "fog_falloff": 0.85,
                "mie_scattering_scale": 0.03,
                "precipitation": 15 + i*5,
                "precipitation_deposits": 30 + i*8,
                "rayleigh_scattering_scale": 0.0331,
                "scattering_intensity": 0.6 - i*0.06,
                "sun_altitude_angle": 10.0 - i*3,
                "sun_azimuth_angle": 140.0,
                "wetness": 40 + i*10,
                "wind_intensity": 15 + i*4
            })

        # ----------------------
        # 小雪 8 强度
        # ----------------------
        for i in range(8):
            self.weather_groups["snowy_light"].append({
                "cloudiness": 80 + i*2.5,
                "dust_storm": 0.0,
                "fog_density": 15 + i*8,
                "fog_distance": 70 - i*6,
                "fog_falloff": 0.8,
                "mie_scattering_scale": 0.05,
                "precipitation": 10 + i*6,
                "precipitation_deposits": 20 + i*10,
                "rayleigh_scattering_scale": 0.02,
                "scattering_intensity": 0.65 - i*0.05,
                "sun_altitude_angle": 30.0,
                "sun_azimuth_angle": 120.0,
                "wetness": 30 + i*8,
                "wind_intensity": 12 + i*3
            })

        # ----------------------
        # 大雪 8 强度
        # ----------------------
        for i in range(8):
            self.weather_groups["snowy_heavy"].append({
                "cloudiness": 100.0,
                "dust_storm": 0.0,
                "fog_density": 35 + i*10,
                "fog_distance": 55 - i*8,
                "fog_falloff": 0.7 - i*0.04,
                "mie_scattering_scale": 0.07,
                "precipitation": 40 + i*8,
                "precipitation_deposits": 50 + i*10,
                "rayleigh_scattering_scale": 0.015,
                "scattering_intensity": 0.45 - i*0.05,
                "sun_altitude_angle": 25.0,
                "sun_azimuth_angle": 120.0,
                "wetness": 60 + i*8,
                "wind_intensity": 22 + i*6
            })
        # ======================
        # 原有功能不变
        # ======================
        self.traffic_light_positions = []
        for actor in self.world.get_actors().filter('*traffic*light*'):
            loc = actor.get_transform().location
            self.traffic_light_positions.append((loc.x, loc.y))
        try:
            env_objs = self.world.get_environment_objects(carla.CityObjectLabel.TrafficLight)
            for obj in env_objs:
                loc = obj.transform.location
                self.traffic_light_positions.append((loc.x, loc.y))
        except:
            pass

        self.scenario = normalized_scenario
        self.is_signal_1d = self.scenario == "1d"
        self.is_roundabout_2b = self.scenario == "2b"
        self.maneuver = maneuver
        if self.maneuver not in MANEUVERS:
            raise ValueError("maneuver must be one of: {}".format(", ".join(MANEUVERS)))
        self.base_seed = int(seed)
        if roundabout_speed_limit_kmh is not None:
            roundabout_speed_limit_kmh = float(roundabout_speed_limit_kmh)
            if not math.isfinite(roundabout_speed_limit_kmh) \
                    or roundabout_speed_limit_kmh <= 0.0:
                raise ValueError("roundabout_speed_limit_kmh must be positive")
        self.roundabout_speed_limit_kmh = roundabout_speed_limit_kmh
        self.roundabout_conflict_headway_s = float(
            roundabout_conflict_headway_s)
        self.roundabout_conflict_tolerance_s = float(
            roundabout_conflict_tolerance_s)
        self.roundabout_vt1_post_exit_clearance_distance_m = float(
            roundabout_vt1_post_exit_clearance_distance_m)
        if (not math.isfinite(self.roundabout_conflict_headway_s)
                or self.roundabout_conflict_headway_s <= 0.0):
            raise ValueError("roundabout_conflict_headway_s must be positive")
        if (not math.isfinite(self.roundabout_conflict_tolerance_s)
                or self.roundabout_conflict_tolerance_s < 0.0
                or self.roundabout_conflict_tolerance_s
                > self.roundabout_conflict_headway_s):
            raise ValueError(
                "roundabout_conflict_tolerance_s must be between 0 and headway")
        if (not math.isfinite(
                self.roundabout_vt1_post_exit_clearance_distance_m)
                or self.roundabout_vt1_post_exit_clearance_distance_m <= 0.0):
            raise ValueError(
                "roundabout_vt1_post_exit_clearance_distance_m must be positive")
        self.roundabout_variants = roundabout_variants or {
            "weather": [{"id": "clear_day", "preset": "sunny", "level": 7}],
            "vehicles": [{
                "id": "tesla_model3",
                "vt1_model": "vehicle.tesla.model3",
                "vt2_model": "vehicle.tesla.model3",
            }],
        }
        if self.is_roundabout_2b:
            self._validate_roundabout_vehicle_blueprints()
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        self.route_index = self._discover_next_route_index()

        self.waypoints = self.map.generate_waypoints(2.0)
        self.traffic_light_infos = []
        self.signal_lane_ranges = {}
        self._build_signal_route_index()
        self.trigger_point = None
        self.ego_point = None
        self.actor_points = []
        self.opposite_points = []
        self.ego_steer_target_point = None  # ego自动驾驶目标点（单个）
        self.selected_agent_idx = -1
        self.selected_mode = None
        self.agent_placement_mode = "center"  # "center" | "opposite" | "target"
        self.ego_route_points = []
        # 2.b keeps the existing single-editor workflow.  Route ownership is
        # determined by the currently selected VUT/VT1; VT2 has no route.
        self.roundabout_place_role = "vut"
        self.roundabout_capable = True
        # A configured default is not evidence that the editor operator chose
        # the formal test branch for this scenario.
        self.roundabout_branch_confirmed = False
        self.roundabout_topology_confirmed = False
        self.roundabout_topology_review_ready = False
        self.roundabout_topology_review = None
        self.roundabout_topology_review_errors = []
        self.roundabout_vut_recommendation = None
        self.roundabout_vut_recommendation_errors = []
        self.roundabout_placement_recommendations = None
        self.roundabout_placement_recommendation_key = None
        self.roundabout_placement_recommendation_errors = []
        self.roundabout_last_trace_failure = None
        self.roundabout_topology_preview = None
        self.roundabout_topology_preview_errors = []
        self.roundabout_topology_seeds = []
        self.roundabout_topology_mark_kind = "inbound"
        self.roundabout_topology_edit_target = None
        self.roundabout_topology_lane_target = None
        self.roundabout_topology_cache_loaded = False
        self.roundabout_topology_cache_path = None
        self.roundabout_show_all_overlays = False
        self.roundabout_analysis = None
        self.roundabout_errors = []
        self.roundabout_warnings = []
        # A partially drawn scenario is not a failed scenario.  Detailed
        # validation errors become visible only after the operator submits S.
        self.roundabout_validation_attempted = False
        self.roundabout_dirty = True
        self.roundabout_history = []
        self.roundabout_vut_route_confirmed = False
        self.roundabout_vt1_route_confirmed = False
        # Enter is a transaction boundary for a route.  The exact dense path
        # accepted here is rendered, analysed and later serialized; saving
        # must never ask a planner to produce a potentially different path.
        self.roundabout_vut_route_cache = None
        self.roundabout_vt1_route_cache = None
        self.roundabout_vut_route_failure = None
        self.roundabout_vt1_route_failure = None
        self.roundabout_help_expanded = False
        self.last_feedback = (
            "Complete STEP 0 and STEP 1 first; then follow the numbered workflow.")
        self.last_feedback_error = False
        self._global_route_planner = None
        self._global_route_planner_checked = False

        if self.is_roundabout_2b:
            self._load_roundabout_topology_cache()

        self.dragging = False
        self.last_mouse_pos = (0, 0)
        self.zoom = ZOOM
        self.offset_x = OFFSET_X
        self.offset_y = OFFSET_Y

        pygame.init()
        self.screen = pygame.display.set_mode(SCREEN_SIZE)
        pygame.display.set_caption(self._window_caption())
        self.clock = pygame.time.Clock()
        self.font, self.font_supports_cjk = self._load_editor_font(22)
        self.panel_font, panel_supports_cjk = self._load_editor_font(16)
        self.font_supports_cjk = self.font_supports_cjk and panel_supports_cjk

    @staticmethod
    def _load_editor_font(size):
        """Prefer a CJK font, while keeping the editor portable."""
        candidates = (
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        )
        for path in candidates:
            if os.path.isfile(path):
                try:
                    return pygame.font.Font(path, size), True
                except pygame.error:
                    continue
        return pygame.font.SysFont(None, size), False

    @staticmethod
    def _resolve_loaded_map_name(carla_map, requested_name):
        """Fail early instead of writing JSON under the wrong CARLA map key."""
        loaded_name = str(getattr(carla_map, "name", "")).rstrip("/").rsplit("/", 1)[-1]
        if not loaded_name:
            raise RuntimeError("无法读取CARLA当前地图名；不能创建可审计的场景配置。")
        if requested_name is None:
            return loaded_name
        requested_basename = str(requested_name).rstrip("/").rsplit("/", 1)[-1]
        if loaded_name.lower() != requested_basename.lower():
            raise RuntimeError(
                "编辑器地图不一致：CARLA当前加载 {!r}，但--name为 {!r}；"
                "请加载目标地图或修正--name。".format(loaded_name, requested_name))
        return loaded_name

    def _window_caption(self):
        if self.is_signal_1d:
            return "CARLA 1.d机动车信号灯编辑器 | {} | seed={}".format(
                self.maneuver, self.base_seed)
        if self.is_roundabout_2b:
            branch = "capable" if self.roundabout_capable else "incapable/ODD"
            return (
                "CARLA GB/T 41798 2.b 环形路口编辑器 | {} | seed={} | build={}"
                .format(branch, self.base_seed, ROUNDABOUT_EDITOR_BUILD))
        return "CARLA 场景编辑器 → 按S自动生成天气场景"

    def _discover_next_route_index(self):
        pattern = re.compile(r"^scenario_{}_([0-9]{{4}})_".format(re.escape(self.scenario)))
        indexes = []
        for filename in os.listdir(self.save_dir):
            match = pattern.match(filename)
            if match:
                indexes.append(int(match.group(1)))
        return max(indexes, default=-1) + 1

    def _roundabout_topology_cache_identity(self):
        """Return the map-bound identity used for persistent STEP 1 marks."""
        carla_map = getattr(self, "map", None)
        map_name = str(getattr(carla_map, "name", "")).strip()
        if not map_name:
            raise RuntimeError("CARLA地图没有可用名称")
        try:
            opendrive = carla_map.to_opendrive()
        except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
            raise RuntimeError("无法读取地图OpenDRIVE：{}".format(exc)) from exc
        if not isinstance(opendrive, str) or not opendrive.strip():
            raise RuntimeError("地图OpenDRIVE为空")
        digest = hashlib.sha256(opendrive.encode("utf-8")).hexdigest()
        basename = map_name.rstrip("/").rsplit("/", 1)[-1]
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", basename) or "map"
        cache_dir = getattr(
            self, "roundabout_topology_cache_dir",
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "config", "roundabout_topology"))
        path = os.path.join(cache_dir, "{}_{}.json".format(safe_name, digest[:16]))
        return {
            "map_name": map_name,
            "opendrive_sha256": digest,
            "path": path,
        }

    def _load_roundabout_topology_cache(self):
        """Load reviewed coordinates only when map name and XODR both match."""
        try:
            identity = self._roundabout_topology_cache_identity()
        except RuntimeError as exc:
            self._feedback("STEP 1拓扑缓存不可用：{}；请手工标定。".format(exc))
            return False
        self.roundabout_topology_cache_path = identity["path"]
        if not os.path.isfile(identity["path"]):
            self._feedback(
                "当前地图没有STEP 1拓扑缓存；确认STEP 0后请手工标定IN/OUT。")
            return False
        try:
            with open(identity["path"], "r", encoding="utf-8") as stream:
                document = json.load(stream)
        except (OSError, ValueError) as exc:
            self._feedback("STEP 1拓扑缓存读取失败：{}；请手工标定。".format(exc), True)
            return False
        if (document.get("schema_version") != 1
                or document.get("map_name") != identity["map_name"]
                or document.get("opendrive_sha256") != identity["opendrive_sha256"]
                or not isinstance(document.get("arms"), list)):
            self._feedback(
                "STEP 1拓扑缓存与当前地图/OpenDRIVE不匹配，已拒绝加载。", True)
            return False
        self.roundabout_topology_seeds = copy.deepcopy(document["arms"])
        review, errors = self._build_roundabout_topology_review()
        self.roundabout_topology_review = review
        self.roundabout_topology_review_errors = errors
        self.roundabout_topology_review_ready = not errors
        self.roundabout_topology_confirmed = False
        self.roundabout_topology_cache_loaded = not errors
        if errors:
            self._feedback(
                "已读取地图拓扑坐标，但当前CARLA审核失败：{}；请修改或重标。".format(
                    "；".join(errors[:2])), True)
            return False
        self._feedback(
            "已自动加载与当前地图唯一绑定的STEP 1拓扑：{}个arm。"
            "确认STEP 0后请检查高亮；正确按Enter，错误则左键选标记、右键重设。".format(
                review.get("arm_count", 0)))
        return True

    def _save_roundabout_topology_cache(self):
        """Persist operator-accepted STEP 1 coordinates atomically as JSON."""
        identity = self._roundabout_topology_cache_identity()
        os.makedirs(os.path.dirname(identity["path"]), exist_ok=True)
        document = {
            "schema_version": 1,
            "map_name": identity["map_name"],
            "opendrive_sha256": identity["opendrive_sha256"],
            "arms": copy.deepcopy(self.roundabout_topology_seeds),
        }
        temporary = identity["path"] + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, identity["path"])
        self.roundabout_topology_cache_path = identity["path"]
        self.roundabout_topology_cache_loaded = True
        return identity["path"]

    @staticmethod
    def _lane_key(waypoint):
        return waypoint.road_id, waypoint.section_id, waypoint.lane_id

    def _remember_signal_waypoint(self, waypoint):
        key = self._lane_key(waypoint)
        current = self.signal_lane_ranges.get(key)
        if current is None:
            self.signal_lane_ranges[key] = [waypoint.s, waypoint.s]
        else:
            current[0] = min(current[0], waypoint.s)
            current[1] = max(current[1], waypoint.s)

    def _walk_signal_lane(self, start, direction, max_distance, step=2.0):
        frontier = [(start, 0.0)]
        visited = set()
        while frontier:
            waypoint, distance = frontier.pop()
            waypoint_id = getattr(waypoint, "id", (self._lane_key(waypoint), round(waypoint.s, 2)))
            if waypoint_id in visited or distance > max_distance:
                continue
            visited.add(waypoint_id)
            self._remember_signal_waypoint(waypoint)
            next_waypoints = waypoint.previous(step) if direction < 0 else waypoint.next(step)
            frontier.extend((candidate, distance + step) for candidate in next_waypoints)

    def _build_signal_route_index(self):
        """Index lane portions controlled by every CARLA traffic light."""
        for actor in self.world.get_actors().filter('*traffic*light*'):
            try:
                trigger_location = actor.get_transform().transform(actor.trigger_volume.location)
                waypoint = self.map.get_waypoint(
                    trigger_location, project_to_road=True, lane_type=carla.LaneType.Driving)
                if waypoint is None:
                    continue
                center = waypoint.transform.location
                forward = waypoint.transform.get_forward_vector()
                perpendicular = carla.Vector3D(-forward.y, forward.x, 0.0)
                half_width = max(1.0, waypoint.lane_width * 0.5)
                left = center + carla.Location(
                    x=perpendicular.x * half_width, y=perpendicular.y * half_width)
                right = center + carla.Location(
                    x=-perpendicular.x * half_width, y=-perpendicular.y * half_width)
                self.traffic_light_infos.append({
                    "actor_location": actor.get_location(),
                    "trigger_location": trigger_location,
                    "stop_center": center,
                    "left": left,
                    "right": right,
                    "approach_yaw": waypoint.transform.rotation.yaw,
                    "road_id": waypoint.road_id,
                    "lane_id": waypoint.lane_id,
                })
                self._walk_signal_lane(waypoint, -1, 80.0)
                self._walk_signal_lane(waypoint, 1, 35.0)
            except (RuntimeError, AttributeError):
                continue

    def _waypoint_is_signal_route(self, waypoint):
        lane_range = self.signal_lane_ranges.get(self._lane_key(waypoint))
        return lane_range is not None and lane_range[0] - 1.0 <= waypoint.s <= lane_range[1] + 1.0

    def _point_is_signal_route(self, point):
        waypoint = self.map.get_waypoint(
            carla.Location(x=float(point['x']), y=float(point['y']), z=float(point.get('z', 0.0))),
            project_to_road=True, lane_type=carla.LaneType.Driving)
        return waypoint is not None and self._waypoint_is_signal_route(waypoint)

    @staticmethod
    def _point_segment_distance(point, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        if dx == 0 and dy == 0:
            return math.hypot(point[0] - start[0], point[1] - start[1])
        ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / (dx * dx + dy * dy)
        ratio = max(0.0, min(1.0, ratio))
        nearest = (start[0] + ratio * dx, start[1] + ratio * dy)
        return math.hypot(point[0] - nearest[0], point[1] - nearest[1])

    def _select_route_traffic_light(self):
        if not self.traffic_light_infos:
            raise RuntimeError("当前地图没有可用的CARLA交通信号灯Actor")
        route_xy = [(float(p['x']), float(p['y'])) for p in self.ego_route_points]
        hint = None
        if self.trigger_point:
            hint = (float(self.trigger_point['x']), float(self.trigger_point['y']))

        best = None
        best_score = float("inf")
        for info in self.traffic_light_infos:
            center = (info['stop_center'].x, info['stop_center'].y)
            route_distance = min(
                [math.hypot(center[0] - x, center[1] - y) for x, y in route_xy]
                + [self._point_segment_distance(center, a, b)
                   for a, b in zip(route_xy, route_xy[1:])]
            )
            hint_distance = math.hypot(center[0] - hint[0], center[1] - hint[1]) if hint else 0.0
            score = route_distance + 0.25 * hint_distance
            if score < best_score:
                best_score = score
                best = (info, route_distance, hint_distance)

        info, route_distance, hint_distance = best
        if route_distance > 15.0 and hint_distance > 25.0:
            raise RuntimeError("绘制路线没有经过所选交通信号灯，请让路线穿过停止线")
        return info

    # ======================
    # GB/T 41798-2022 2.b editor/topology helpers
    # ======================
    def _feedback(self, message, error=False):
        """Keep the last action visible and mirror detailed guidance to stdout."""
        self.last_feedback = str(message)
        self.last_feedback_error = bool(error)
        prefix = "❌" if error else "✅"
        print("{} {}".format(prefix, message))

    def _mark_roundabout_dirty(self):
        if self.is_roundabout_2b:
            self.roundabout_dirty = True
            self.roundabout_analysis = None
            self.roundabout_validation_attempted = False

    def _invalidate_roundabout_topology_confirmation(self):
        """Invalidate route-derived numbering when the open VUT path changes."""
        if not self.is_roundabout_2b:
            return
        # Manually paired physical arms are independent of which of their
        # inbound lanes the VUT later uses.  Keep that STEP 1 evidence, but
        # discard the derived entry/exit numbering preview.
        if not getattr(self, "roundabout_topology_seeds", []):
            self.roundabout_topology_confirmed = False
        self.roundabout_topology_preview = None
        self.roundabout_topology_preview_errors = []
        self.roundabout_last_trace_failure = None
        self.roundabout_placement_recommendations = None
        self.roundabout_placement_recommendation_key = None
        self.roundabout_placement_recommendation_errors = []

    def _push_roundabout_history(self, action):
        """Save one reversible 2.b editor state before a user mutation."""
        if not self.is_roundabout_2b:
            return
        history = getattr(self, "roundabout_history", None)
        if history is None:
            self.roundabout_history = []
            history = self.roundabout_history
        history.append({
            "action": str(action),
            "ego_point": copy.deepcopy(self.ego_point),
            "ego_route_points": copy.deepcopy(self.ego_route_points),
            "actor_points": copy.deepcopy(self.actor_points),
            "selected_agent_idx": int(self.selected_agent_idx),
            "selected_mode": self.selected_mode,
            "roundabout_place_role": self.roundabout_place_role,
            "roundabout_capable": bool(self.roundabout_capable),
            "roundabout_branch_confirmed": bool(getattr(
                self, "roundabout_branch_confirmed", False)),
            "roundabout_topology_confirmed": bool(getattr(
                self, "roundabout_topology_confirmed", False)),
            "roundabout_topology_review_ready": bool(getattr(
                self, "roundabout_topology_review_ready", False)),
            "roundabout_topology_review": copy.deepcopy(getattr(
                self, "roundabout_topology_review", None)),
            "roundabout_topology_review_errors": copy.deepcopy(getattr(
                self, "roundabout_topology_review_errors", [])),
            "roundabout_vut_recommendation": copy.deepcopy(getattr(
                self, "roundabout_vut_recommendation", None)),
            "roundabout_vut_recommendation_errors": copy.deepcopy(getattr(
                self, "roundabout_vut_recommendation_errors", [])),
            "roundabout_topology_seeds": copy.deepcopy(getattr(
                self, "roundabout_topology_seeds", [])),
            "roundabout_topology_mark_kind": getattr(
                self, "roundabout_topology_mark_kind", "inbound"),
            "roundabout_validation_attempted": bool(getattr(
                self, "roundabout_validation_attempted", False)),
            "roundabout_vut_route_confirmed": bool(getattr(
                self, "roundabout_vut_route_confirmed", False)),
            "roundabout_vt1_route_confirmed": bool(getattr(
                self, "roundabout_vt1_route_confirmed", False)),
            "roundabout_vut_route_cache": getattr(
                self, "roundabout_vut_route_cache", None),
            "roundabout_vt1_route_cache": getattr(
                self, "roundabout_vt1_route_cache", None),
            "roundabout_vut_route_failure": copy.deepcopy(getattr(
                self, "roundabout_vut_route_failure", None)),
            "roundabout_vt1_route_failure": copy.deepcopy(getattr(
                self, "roundabout_vt1_route_failure", None)),
        })
        if len(history) > ROUNDABOUT_HISTORY_LIMIT:
            del history[:-ROUNDABOUT_HISTORY_LIMIT]

    def undo_roundabout_action(self):
        """Undo the most recent accepted placement, route, delete, or branch edit."""
        if not self.is_roundabout_2b:
            return False
        history = getattr(self, "roundabout_history", [])
        if not history:
            self._feedback("没有可撤销的操作；只有已成功执行的编辑步骤会进入历史记录。", True)
            return False
        state = history.pop()
        self.ego_point = state["ego_point"]
        self.ego_route_points = state["ego_route_points"]
        self.actor_points = state["actor_points"]
        self.selected_agent_idx = state["selected_agent_idx"]
        self.selected_mode = state["selected_mode"]
        self.roundabout_place_role = state["roundabout_place_role"]
        self.roundabout_capable = state["roundabout_capable"]
        self.roundabout_branch_confirmed = state[
            "roundabout_branch_confirmed"]
        self.roundabout_topology_confirmed = state[
            "roundabout_topology_confirmed"]
        self.roundabout_topology_review_ready = state.get(
            "roundabout_topology_review_ready", False)
        self.roundabout_topology_review = state.get(
            "roundabout_topology_review")
        self.roundabout_topology_review_errors = state.get(
            "roundabout_topology_review_errors", [])
        self.roundabout_vut_recommendation = state.get(
            "roundabout_vut_recommendation")
        self.roundabout_vut_recommendation_errors = state.get(
            "roundabout_vut_recommendation_errors", [])
        self.roundabout_topology_seeds = state[
            "roundabout_topology_seeds"]
        self.roundabout_topology_mark_kind = state[
            "roundabout_topology_mark_kind"]
        self.roundabout_topology_edit_target = None
        self.roundabout_topology_lane_target = None
        self.roundabout_validation_attempted = state.get(
            "roundabout_validation_attempted", False)
        self.roundabout_topology_preview = None
        self.roundabout_topology_preview_errors = []
        self.roundabout_vut_route_confirmed = state[
            "roundabout_vut_route_confirmed"]
        self.roundabout_vt1_route_confirmed = state[
            "roundabout_vt1_route_confirmed"]
        self.roundabout_vut_route_cache = state.get(
            "roundabout_vut_route_cache")
        self.roundabout_vt1_route_cache = state.get(
            "roundabout_vt1_route_cache")
        self.roundabout_vut_route_failure = state.get(
            "roundabout_vut_route_failure")
        self.roundabout_vt1_route_failure = state.get(
            "roundabout_vt1_route_failure")
        self._mark_roundabout_dirty()
        try:
            pygame.display.set_caption(self._window_caption())
        except (pygame.error, AttributeError):
            pass
        self._feedback("已撤销：{}。".format(state["action"]))
        return True

    @staticmethod
    def _point_payload(point):
        payload = {
            "x": round(float(point["x"]), 4),
            "y": round(float(point["y"]), 4),
            "z": round(float(point.get("z", 0.0)), 4),
            "yaw": round(float(point.get("yaw", 0.0)), 4),
        }
        for key in ("road_id", "section_id", "lane_id"):
            if key in point:
                payload[key] = int(point[key])
        if "s" in point:
            payload["s"] = round(float(point["s"]), 4)
        return payload

    @classmethod
    def _route_item_payload(cls, item, road_option=None):
        """Canonicalize a CARLA waypoint or a test/serialized route point."""
        if hasattr(item, "transform"):
            return cls._waypoint_payload(item, road_option)
        payload = cls._point_payload(item)
        if road_option is not None:
            payload["road_option"] = cls._road_option_value(road_option)
        return payload

    @classmethod
    def _route_anchor_fingerprint(cls, points, actor_start=None):
        payload = []
        if actor_start is not None:
            payload.append(cls._point_payload(actor_start))
        payload.extend(cls._point_payload(point) for point in points)
        return roundabout_hash(payload)

    def _build_roundabout_route_cache(
            self, role, points, actor_start, dense, options):
        if len(dense) != len(options):
            raise RuntimeError("路线规划结果与道路指令数量不一致")
        payload = [
            self._route_item_payload(item, option)
            for item, option in zip(dense, options)
        ]
        length_m = sum(
            math.hypot(second["x"] - first["x"], second["y"] - first["y"])
            for first, second in zip(payload, payload[1:]))
        return {
            "role": role,
            "anchor_fingerprint": self._route_anchor_fingerprint(
                points, actor_start),
            "dense": list(dense),
            "options": list(options),
            "payload": payload,
            "route_fingerprint": roundabout_hash(payload),
            "length_m": length_m,
        }

    def _build_roundabout_candidate_extension(
            self, role, route, actor_start, waypoint):
        """Validate one new anchor while reusing the accepted dense prefix."""
        payload = self._route_anchor_payload(waypoint)
        candidate_route = list(route) + [payload]
        cache = getattr(
            self, "roundabout_vut_route_cache" if role == "vut"
            else "roundabout_vt1_route_cache", None)
        reusable = (
            isinstance(cache, dict)
            and cache.get("anchor_fingerprint")
            == self._route_anchor_fingerprint(route, actor_start)
            and cache.get("dense")
            and len(cache.get("dense", [])) == len(cache.get("options", [])))
        if not reusable:
            dense, options = self._densify_route(
                candidate_route, actor_start=actor_start)
            return payload, self._build_roundabout_route_cache(
                role, candidate_route, actor_start, dense, options)

        prefix = list(cache["dense"])
        current = prefix[-1]
        try:
            segment, _diagnostics = self._bounded_lane_graph_trace_segment(
                current, payload, waypoint)
        except RoundaboutRouteTraceError as exc:
            raise RoundaboutRouteTraceError(
                "第{}段(P{}→P{}){}".format(
                    len(candidate_route), len(route), len(candidate_route), exc),
                partial_route=prefix + exc.partial_route[1:],
                segment_index=len(candidate_route),
                segment_start=self._waypoint_payload(current),
                segment_target=self._point_payload(payload)) from exc
        seen_tokens = {self._waypoint_token(item) for item in prefix}
        for item in segment[1:]:
            token = self._waypoint_token(item)
            if token in seen_tokens:
                raise RoundaboutRouteTraceError(
                    "新增段重新经过已使用的CARLA路点{}；路线发生回环/重复，"
                    "请改选前方候选".format(token),
                    partial_route=prefix,
                    segment_index=len(candidate_route),
                    segment_start=self._waypoint_payload(current),
                    segment_target=self._point_payload(payload))
            prefix.append(item)
            seen_tokens.add(token)
        options = self._fallback_route_options(prefix)
        return payload, self._build_roundabout_route_cache(
            role, candidate_route, actor_start, prefix, options)

    def _confirmed_roundabout_route(self, role, required=True):
        if role == "vut":
            cache = getattr(self, "roundabout_vut_route_cache", None)
            points, actor_start = self.ego_route_points, self.ego_point
        elif role == "vt1":
            cache = getattr(self, "roundabout_vt1_route_cache", None)
            actor_start = self._roundabout_actor("vt1")
            points = actor_start.get("route_points", []) if actor_start else []
        else:
            raise ValueError("unknown roundabout route role: {}".format(role))
        if cache is None and not required:
            return None
        expected = self._route_anchor_fingerprint(points, actor_start)
        if cache is None or cache.get("anchor_fingerprint") != expected:
            if required:
                raise RuntimeError(
                    "{}路线已发生变化，必须重新按Enter确认真实规划路线".format(
                        role.upper()))
            return None
        return cache

    def _invalidate_roundabout_route(self, role):
        if role == "vut":
            self.roundabout_vut_route_confirmed = False
            self.roundabout_vut_route_cache = None
            self.roundabout_vut_route_failure = None
            self._invalidate_roundabout_topology_confirmation()
        elif role == "vt1":
            self.roundabout_vt1_route_confirmed = False
            self.roundabout_vt1_route_cache = None
            self.roundabout_vt1_route_failure = None

    def _set_roundabout_route_failure(
            self, role, error, payload=None,
            segment_start=None, segment_target=None):
        partial = payload
        if partial is None and isinstance(error, RoundaboutRouteTraceError):
            partial = [self._route_item_payload(item)
                       for item in error.partial_route]
        failure = {
            "message": str(error),
            "payload": list(partial or []),
            "segment_index": getattr(error, "segment_index", None),
            "segment_start": (
                getattr(error, "segment_start", None) or segment_start),
            "segment_target": (
                getattr(error, "segment_target", None) or segment_target),
        }
        if role == "vut":
            self.roundabout_vut_route_failure = failure
        else:
            self.roundabout_vt1_route_failure = failure

    @staticmethod
    def _waypoint_payload(waypoint, road_option=None):
        location = waypoint.transform.location
        payload = {
            "x": round(float(location.x), 4),
            "y": round(float(location.y), 4),
            "z": round(float(location.z + 0.2), 4),
            "yaw": round(float(waypoint.transform.rotation.yaw), 4),
        }
        if road_option is not None:
            value = getattr(road_option, "value", road_option)
            try:
                payload["road_option"] = int(value)
            except (TypeError, ValueError):
                payload["road_option"] = 4
        return payload

    @classmethod
    def _route_anchor_payload(cls, waypoint):
        """Persist the exact lane identity selected at an ambiguous click."""
        payload = cls._waypoint_payload(waypoint)
        payload.update({
            "road_id": int(waypoint.road_id),
            "section_id": int(waypoint.section_id),
            "lane_id": int(waypoint.lane_id),
            "s": round(float(waypoint.s), 4),
        })
        return payload

    @staticmethod
    def _road_option_value(value):
        value = getattr(value, "value", value)
        try:
            value = int(value)
        except (TypeError, ValueError):
            return 4
        return value if value in (-1, 1, 2, 3, 4, 5, 6) else 4

    @classmethod
    def _fallback_route_options(cls, waypoints):
        """Infer basic TCP commands when CARLA's GlobalRoutePlanner is absent."""
        options = [4] * len(waypoints)
        for index, waypoint in enumerate(waypoints):
            if index + 1 < len(waypoints):
                following = waypoints[index + 1]
                same_road_section = (
                    int(waypoint.road_id) == int(following.road_id)
                    and int(waypoint.section_id) == int(following.section_id))
                changed_lane = int(waypoint.lane_id) != int(following.lane_id)
                if same_road_section and changed_lane:
                    origin = waypoint.transform.location
                    destination = following.transform.location
                    yaw = math.radians(float(waypoint.transform.rotation.yaw))
                    # Positive projection onto the left normal means a left
                    # lane change in vehicle coordinates.
                    left_projection = (
                        (destination.x - origin.x) * -math.sin(yaw)
                        + (destination.y - origin.y) * math.cos(yaw))
                    options[index] = 5 if left_projection >= 0.0 else 6
                    continue
            lookahead = min(len(waypoints) - 1, index + 5)
            if lookahead <= index:
                continue
            yaw = float(waypoint.transform.rotation.yaw)
            future_yaw = float(waypoints[lookahead].transform.rotation.yaw)
            delta = (future_yaw - yaw + 180.0) % 360.0 - 180.0
            is_junction = bool(getattr(waypoint, "is_junction", False)) or any(
                bool(getattr(item, "is_junction", False))
                for item in waypoints[index:lookahead + 1])
            if not is_junction:
                continue
            if delta > 15.0:
                options[index] = 1
            elif delta < -15.0:
                options[index] = 2
            else:
                options[index] = 3
        return options

    @staticmethod
    def _waypoint_token(waypoint):
        return (
            int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id),
            round(float(waypoint.s), 1),
        )

    @staticmethod
    def _distance_locations(first, second):
        return math.hypot(first.x - second.x, first.y - second.y)

    @staticmethod
    def _normalise_angle(angle):
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        while angle > math.pi:
            angle -= 2.0 * math.pi
        return angle

    @staticmethod
    def _circular_distance(first, second):
        return abs(CarlaMapEditor0916._normalise_angle(first - second))

    def _get_global_route_planner(self):
        if self._global_route_planner_checked:
            return self._global_route_planner
        self._global_route_planner_checked = True
        try:
            from agents.navigation.global_route_planner import GlobalRoutePlanner
            try:
                planner = GlobalRoutePlanner(self.map, 1.0)
            except TypeError:
                from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
                planner = GlobalRoutePlanner(GlobalRoutePlannerDAO(self.map, 1.0))
            if hasattr(planner, "setup"):
                planner.setup()
            self._global_route_planner = planner
        except (ImportError, TypeError, AttributeError, RuntimeError) as exc:
            self._global_route_planner = None
            self.roundabout_warnings = [
                "GlobalRoutePlanner unavailable; using conservative lane-follow fallback: {}".format(exc)
            ]
        return self._global_route_planner

    def _greedy_trace_segment(self, start, target, resolution=1.0):
        """Fallback used only when CARLA's route planner is unavailable."""
        current = start
        output = [current]
        direct = self._distance_locations(start.transform.location, target.transform.location)
        maximum_steps = max(30, int((direct * 5.0 + 80.0) / resolution))
        visited = set()
        for _ in range(maximum_steps):
            close_enough = self._distance_locations(
                current.transform.location, target.transform.location) <= 2.0
            same_lane = self._lane_key(current) == self._lane_key(target)
            heading_error = abs(self._normalise_angle(math.radians(
                float(current.transform.rotation.yaw)
                - float(target.transform.rotation.yaw))))
            if close_enough and same_lane and heading_error <= math.radians(45.0):
                output.append(target)
                return output
            token = self._waypoint_token(current)
            if token in visited:
                break
            visited.add(token)
            choices = list(current.next(resolution))
            if not choices:
                break
            target_location = target.transform.location
            target_yaw = math.radians(target.transform.rotation.yaw)

            def score(candidate):
                location = candidate.transform.location
                distance = self._distance_locations(location, target_location)
                yaw = math.radians(candidate.transform.rotation.yaw)
                yaw_error = abs(self._normalise_angle(yaw - target_yaw))
                revisit = 1000.0 if self._waypoint_token(candidate) in visited else 0.0
                return distance + yaw_error * 2.0 + revisit

            current = min(choices, key=score)
            output.append(current)
        raise RuntimeError(
            "路线点之间无法沿Driving Lane连通；请在分叉、环岛入口和出口补充路线点")

    @staticmethod
    def _lane_type_is_driving(waypoint):
        lane_type = getattr(waypoint, "lane_type", None)
        driving = getattr(getattr(carla, "LaneType", None), "Driving", None)
        if lane_type is None:
            return False
        return lane_type == driving or str(lane_type).lower() == "driving"

    @staticmethod
    def _lane_change_flag_allows(waypoint, direction):
        desired = getattr(
            getattr(carla, "LaneChange", None),
            "Left" if direction == "left" else "Right", None)
        if desired is None:
            return False
        flags = getattr(waypoint, "lane_change", None)
        marking = getattr(
            waypoint,
            "left_lane_marking" if direction == "left" else "right_lane_marking",
            None)
        if flags is None and marking is not None:
            flags = getattr(marking, "lane_change", None)
        try:
            allowed = bool(flags & desired)
        except (TypeError, ValueError):
            allowed = str(flags).lower() in (
                direction, "both", "lanechange.{}".format(direction))
        if not allowed:
            return False
        # Never cross an explicitly solid boundary even if malformed custom
        # OpenDRIVE advertises a contradictory lane-change flag.
        marking_type = str(getattr(marking, "type", "")).lower()
        return marking_type not in ("solid", "solidsolid")

    def _legal_lateral_neighbors(self, waypoint):
        """Return legal same-direction adjacent Driving lanes."""
        output = []
        current_forward = waypoint.transform.get_forward_vector()
        for direction, accessor in (
                ("left", "get_left_lane"), ("right", "get_right_lane")):
            if not self._lane_change_flag_allows(waypoint, direction):
                continue
            method = getattr(waypoint, accessor, None)
            candidate = method() if callable(method) else None
            if candidate is None or not self._lane_type_is_driving(candidate):
                continue
            if (int(candidate.road_id) != int(waypoint.road_id)
                    or int(candidate.section_id) != int(waypoint.section_id)):
                continue
            candidate_forward = candidate.transform.get_forward_vector()
            alignment = (
                current_forward.x * candidate_forward.x
                + current_forward.y * candidate_forward.y)
            minimum_alignment = math.cos(math.radians(
                ROUNDABOUT_LANE_CHANGE_MAX_HEADING_DEG))
            if alignment < minimum_alignment:
                continue
            lateral_m = self._distance_locations(
                waypoint.transform.location, candidate.transform.location)
            maximum_lateral_m = max(
                float(getattr(waypoint, "lane_width", 3.5)),
                float(getattr(candidate, "lane_width", 3.5))) * 1.6
            if not 0.5 <= lateral_m <= maximum_lateral_m:
                continue
            # Land several metres ahead on the adjacent lane.  Connecting to
            # the same-s waypoint would create an unphysical sideways jump
            # for the Behavior controller; this diagonal represents a short,
            # executable lane-change transition.
            for landing in candidate.next(ROUNDABOUT_LANE_CHANGE_FORWARD_M):
                if not self._lane_type_is_driving(landing):
                    continue
                landing_forward = landing.transform.get_forward_vector()
                landing_alignment = (
                    current_forward.x * landing_forward.x
                    + current_forward.y * landing_forward.y)
                if landing_alignment < minimum_alignment:
                    continue
                output.append((landing, direction, lateral_m))
        return output

    def _bounded_lane_graph_trace_segment(
            self, start, target_point, target_waypoint,
            resolution=ROUNDABOUT_LOCAL_TRACE_RESOLUTION_M):
        """Search forward branches plus legal same-direction lane changes.

        This is a bounded A* search over CARLA waypoints.  Forward ``next``
        edges are combined with legal left/right Driving-lane edges.  Lateral
        movement is penalized, capped and never allowed across an explicitly
        solid marking or into an opposite-direction lane.
        """
        target_location = carla.Location(
            x=float(target_point["x"]), y=float(target_point["y"]),
            z=float(target_point.get("z", 0.0)))
        direct_m = self._distance_locations(
            start.transform.location, target_location)
        maximum_m = min(
            ROUNDABOUT_LOCAL_TRACE_MAX_M,
            max(30.0, direct_m * ROUNDABOUT_LOCAL_TRACE_MAX_RATIO
                + ROUNDABOUT_LOCAL_TRACE_EXTRA_M))
        counter = itertools.count()
        start_token = self._waypoint_token(start)
        start_state = (start_token, 0)
        queue = [(direct_m, next(counter), 0.0, 0.0, start_state)]
        waypoints = {start_state: start}
        parents = {start_state: None}
        best_cost = {start_state: 0.0}
        best_state = start_state
        best_distance = direct_m
        target_lane = self._lane_key(target_waypoint)
        forward_edges_considered = 0
        lateral_edges_considered = 0

        def reconstruct(state):
            output = []
            while state is not None:
                output.append(waypoints[state])
                state = parents[state]
            output.reverse()
            return output

        while queue:
            (_priority, _serial, weighted_cost,
             travelled_m, state) = heapq.heappop(queue)
            if weighted_cost > best_cost.get(state, float("inf")) + 1e-6:
                continue
            token, lane_changes = state
            current = waypoints[state]
            distance_m = self._distance_locations(
                current.transform.location, target_location)
            if distance_m < best_distance:
                best_distance, best_state = distance_m, state
            same_target_lane = self._lane_key(current) == target_lane
            current_yaw = float(current.transform.rotation.yaw)
            target_yaw = float(target_waypoint.transform.rotation.yaw)
            heading_error = abs((current_yaw - target_yaw + 180.0) % 360.0 - 180.0)
            # Spatial proximity alone is not enough near a split/merge.  Two
            # unrelated connector lanes can be less than 1.5 m apart.  The
            # terminal state must belong to the lane identity selected by the
            # operator (road/section/lane), otherwise a visually plausible
            # route can silently stop on the neighbouring branch.
            if (distance_m <= ROUNDABOUT_LOCAL_TRACE_GOAL_TOLERANCE_M
                    and same_target_lane and heading_error <= 60.0):
                return reconstruct(state), {
                    "direct_m": direct_m,
                    "travelled_m": travelled_m,
                    "maximum_m": maximum_m,
                    "closest_m": distance_m,
                    "lane_changes": lane_changes,
                    "forward_edges_considered": forward_edges_considered,
                    "lateral_edges_considered": lateral_edges_considered,
                }
            if travelled_m >= maximum_m:
                continue
            neighbors = []
            for candidate in current.next(resolution):
                forward_edges_considered += 1
                neighbors.append((candidate, lane_changes, 0.0))
            if lane_changes < ROUNDABOUT_LOCAL_TRACE_MAX_LANE_CHANGES:
                for candidate, _direction, lateral_m in \
                        self._legal_lateral_neighbors(current):
                    lateral_edges_considered += 1
                    neighbors.append((
                        candidate, lane_changes + 1,
                        ROUNDABOUT_LANE_CHANGE_PENALTY_M))
            for candidate, candidate_lane_changes, extra_penalty in neighbors:
                candidate_token = self._waypoint_token(candidate)
                step_m = self._distance_locations(
                    current.transform.location, candidate.transform.location)
                if step_m <= 0.05:
                    continue
                candidate_travelled_m = travelled_m + step_m
                if candidate_travelled_m > maximum_m:
                    continue
                candidate_cost = weighted_cost + step_m + extra_penalty
                candidate_state = (candidate_token, candidate_lane_changes)
                if candidate_cost >= best_cost.get(
                        candidate_state, float("inf")) - 1e-6:
                    continue
                candidate_distance = self._distance_locations(
                    candidate.transform.location, target_location)
                best_cost[candidate_state] = candidate_cost
                waypoints[candidate_state] = candidate
                parents[candidate_state] = state
                heapq.heappush(queue, (
                    candidate_cost + candidate_distance,
                    next(counter), candidate_cost, candidate_travelled_m,
                    candidate_state))

        partial = reconstruct(best_state)
        raise RoundaboutRouteTraceError(
            "有界车道图搜索失败：锚点直线距离{:.1f}m，"
            "最多允许搜索{:.1f}m，最近仍差{:.1f}m。"
            "已检查{}条前向边和{}条合法左右变道边（最多{}次变道）。"
            "这通常表示锚点在反向/不可变道车道、被实线隔开、分叉选择错误，"
            "或缺少分叉引导点；程序不会改用全局绕行。".format(
                direct_m, maximum_m, best_distance,
                forward_edges_considered, lateral_edges_considered,
                ROUNDABOUT_LOCAL_TRACE_MAX_LANE_CHANGES),
            partial_route=partial)

    def _bounded_forward_trace_segment(
            self, start, target_point, target_waypoint,
            resolution=ROUNDABOUT_LOCAL_TRACE_RESOLUTION_M):
        """Compatibility alias for the expanded bounded lane-graph search."""
        return self._bounded_lane_graph_trace_segment(
            start, target_point, target_waypoint, resolution=resolution)

    def _resolve_roundabout_anchor_waypoint(self, point):
        """Resolve a route anchor without losing its reviewed lane identity."""
        identity_keys = ("road_id", "section_id", "lane_id", "s")
        has_identity = all(key in point for key in identity_keys)
        get_xodr = getattr(self.map, "get_waypoint_xodr", None)
        if has_identity and callable(get_xodr):
            waypoint = get_xodr(
                int(point["road_id"]), int(point["lane_id"]), float(point["s"]))
            expected = (
                int(point["road_id"]), int(point["section_id"]),
                int(point["lane_id"]))
            if waypoint is None or self._lane_key(waypoint) != expected:
                raise RuntimeError(
                    "锚点记录的精确车道身份 road/section/lane={}/{}/{}、s={:.1f} "
                    "在当前OpenDRIVE中不存在；地图可能已变化，请删除该点后重新选择".format(
                        expected[0], expected[1], expected[2], float(point["s"])))
            return waypoint
        waypoint = self.map.get_waypoint(
            carla.Location(
                x=float(point["x"]), y=float(point["y"]),
                z=float(point.get("z", 0.0))),
            project_to_road=True, lane_type=carla.LaneType.Driving)
        if waypoint is None:
            return None
        if has_identity and self._lane_key(waypoint) != (
                int(point["road_id"]), int(point["section_id"]),
                int(point["lane_id"])):
            raise RuntimeError(
                "当前CARLA接口不能按OpenDRIVE身份恢复锚点，且坐标重投影到了另一条车道；"
                "请删除该点后重新选择")
        return waypoint

    def _densify_roundabout_route(self, points, actor_start):
        if actor_start is None or len(points) < 1:
            raise RuntimeError("路线至少需要车辆起点和一个锚点")
        start = self.map.get_waypoint(
            carla.Location(
                x=float(actor_start["x"]), y=float(actor_start["y"]),
                z=float(actor_start.get("z", 0.0))),
            project_to_road=True, lane_type=carla.LaneType.Driving)
        if start is None:
            raise RuntimeError("车辆起点不在可行驶车道")
        dense = [start]
        seen_tokens = {self._waypoint_token(start)}
        current = start
        for segment_index, point in enumerate(points, start=1):
            target = self._resolve_roundabout_anchor_waypoint(point)
            if target is None:
                raise RoundaboutRouteTraceError(
                    "第{}个锚点({:.1f},{:.1f})不在可行驶车道".format(
                        segment_index, float(point["x"]), float(point["y"])),
                    partial_route=dense, segment_index=segment_index,
                    segment_start=self._waypoint_payload(current),
                    segment_target=self._point_payload(point))
            try:
                segment, _diagnostics = self._bounded_lane_graph_trace_segment(
                    current, point, target)
            except RoundaboutRouteTraceError as exc:
                partial = dense + exc.partial_route[1:]
                raise RoundaboutRouteTraceError(
                    "第{}段(P{}→P{}){}".format(
                        segment_index,
                        "车辆" if segment_index == 1 else segment_index - 1,
                        segment_index, exc),
                    partial_route=partial, segment_index=segment_index,
                    segment_start=self._waypoint_payload(current),
                    segment_target=self._point_payload(point)) from exc
            for waypoint in segment[1:]:
                token = self._waypoint_token(waypoint)
                if token in seen_tokens:
                    raise RoundaboutRouteTraceError(
                        "第{}段重新经过已使用的CARLA路点{}；路线发生回环/重复，"
                        "请删除导致折返的锚点".format(segment_index, token),
                        partial_route=dense, segment_index=segment_index,
                        segment_start=self._waypoint_payload(current),
                        segment_target=self._point_payload(point))
                dense.append(waypoint)
                seen_tokens.add(token)
            current = dense[-1]
        if len(dense) < 3:
            raise RuntimeError("路线补全后过短；请把终点画到目标出口下游")
        return dense, self._fallback_route_options(dense)

    def _densify_route(self, points, actor_start=None):
        if getattr(self, "is_roundabout_2b", False):
            return self._densify_roundabout_route(points, actor_start)
        route_points = []
        if actor_start is not None:
            route_points.append(actor_start)
        route_points.extend(points)
        if len(route_points) < 2:
            raise RuntimeError("路线至少需要两个位置（车辆起点可作为第一个位置）")

        route_waypoints = []
        for point in route_points:
            waypoint = self.map.get_waypoint(
                carla.Location(
                    x=float(point["x"]), y=float(point["y"]),
                    z=float(point.get("z", 0.0))),
                project_to_road=True, lane_type=carla.LaneType.Driving)
            if waypoint is None:
                raise RuntimeError("路线点 ({:.1f}, {:.1f}) 不在可行驶车道".format(
                    float(point["x"]), float(point["y"])))
            route_waypoints.append(waypoint)

        planner = self._get_global_route_planner()
        dense = []
        dense_options = []
        planner_options_available = planner is not None
        for start, target in zip(route_waypoints, route_waypoints[1:]):
            if planner is not None:
                try:
                    trace = planner.trace_route(
                        start.transform.location, target.transform.location)
                    segment = [item[0] if isinstance(item, (tuple, list)) else item
                               for item in trace]
                    segment_options = [
                        self._road_option_value(item[1])
                        if isinstance(item, (tuple, list)) and len(item) > 1 else 4
                        for item in trace]
                    if not segment:
                        raise RuntimeError("empty planner route")
                except (RuntimeError, AttributeError, TypeError, ValueError):
                    segment = self._greedy_trace_segment(start, target)
                    segment_options = self._fallback_route_options(segment)
                    planner_options_available = False
            else:
                segment = self._greedy_trace_segment(start, target)
                segment_options = self._fallback_route_options(segment)
            for waypoint, road_option in zip(segment, segment_options):
                if dense and self._distance_locations(
                        dense[-1].transform.location,
                        waypoint.transform.location) < 0.15:
                    continue
                dense.append(waypoint)
                dense_options.append(road_option)
        if len(dense) < 3:
            raise RuntimeError("路线补全后过短；请把终点画到目标出口下游")
        if not planner_options_available:
            dense_options = self._fallback_route_options(dense)
        return dense, dense_options

    @staticmethod
    def _circumcircle(first, second, third):
        ax, ay = first
        bx, by = second
        cx, cy = third
        denominator = 2.0 * (
            ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(denominator) < 1e-6:
            return None
        ux = ((ax * ax + ay * ay) * (by - cy)
              + (bx * bx + by * by) * (cy - ay)
              + (cx * cx + cy * cy) * (ay - by)) / denominator
        uy = ((ax * ax + ay * ay) * (cx - bx)
              + (bx * bx + by * by) * (ax - cx)
              + (cx * cx + cy * cy) * (bx - ax)) / denominator
        radius = math.hypot(ax - ux, ay - uy)
        return ux, uy, radius

    @staticmethod
    def _median(values):
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) * 0.5

    def _ring_alignment(self, waypoint, center):
        location = waypoint.transform.location
        radial_x = location.x - center[0]
        radial_y = location.y - center[1]
        radius = math.hypot(radial_x, radial_y)
        if radius < 1e-6:
            return 1.0, radius
        forward = waypoint.transform.get_forward_vector()
        alignment = abs((forward.x * radial_x + forward.y * radial_y) / radius)
        return alignment, radius

    def _estimate_roundabout_circle(self, dense_route):
        """Fit the repeatedly curved, tangential part of the user-drawn VUT route."""
        gap = max(3, min(8, len(dense_route) // 12))
        candidates = []
        for index in range(0, len(dense_route) - 2 * gap, max(1, gap // 2)):
            points = []
            for waypoint in (
                    dense_route[index], dense_route[index + gap],
                    dense_route[index + 2 * gap]):
                location = waypoint.transform.location
                points.append((location.x, location.y))
            circle = self._circumcircle(*points)
            if circle is not None and 8.0 <= circle[2] <= 250.0:
                candidates.append(circle)
        if not candidates:
            raise RuntimeError(
                "无法从VUT路线识别环形曲线；请在入口、环岛内和出口增加路线点")

        def support(circle):
            center = circle[:2]
            radius = circle[2]
            tolerance = max(4.0, radius * 0.12)
            count = 0
            for waypoint in dense_route:
                alignment, point_radius = self._ring_alignment(waypoint, center)
                if abs(point_radius - radius) <= tolerance and alignment <= 0.62:
                    count += 1
            return count

        best = max(candidates, key=support)
        best_support = support(best)
        if best_support < 6:
            raise RuntimeError(
                "VUT路线中的环岛段不足以可靠拟合；请沿实际环岛车道增加路线点")
        neighbourhood = [candidate for candidate in candidates
                         if math.hypot(candidate[0] - best[0], candidate[1] - best[1])
                         <= max(8.0, best[2] * 0.18)
                         and abs(candidate[2] - best[2]) <= max(8.0, best[2] * 0.2)]
        center = (
            self._median([item[0] for item in neighbourhood]),
            self._median([item[1] for item in neighbourhood]),
        )
        radii = []
        for waypoint in dense_route:
            alignment, radius = self._ring_alignment(waypoint, center)
            if alignment <= 0.62 and abs(radius - best[2]) <= max(6.0, best[2] * 0.2):
                radii.append(radius)
        radius = self._median(radii) if radii else best[2]
        return {"center": center, "radius": radius, "method": "route_circle_fit"}

    def _ring_route_indices(self, dense_route, circle):
        tolerance = max(5.0, circle["radius"] * 0.16)
        supported = []
        for index, waypoint in enumerate(dense_route):
            alignment, radius = self._ring_alignment(waypoint, circle["center"])
            if alignment <= 0.68 and abs(radius - circle["radius"]) <= tolerance:
                supported.append(index)
        if len(supported) < 6:
            raise RuntimeError("路线没有足够的连续环岛车道点")
        groups = []
        current = [supported[0]]
        for index in supported[1:]:
            if index - current[-1] <= 6:
                current.append(index)
            else:
                groups.append(current)
                current = [index]
        groups.append(current)
        group = max(groups, key=lambda item: (len(item), item[-1] - item[0]))
        if len(group) < 6:
            raise RuntimeError("检测到的环岛段不连续；请按实际行驶顺序补充路线点")
        return group[0], group[-1]

    def _route_direction(self, dense_route, circle, first_index, last_index):
        previous = None
        accumulated = 0.0
        center_x, center_y = circle["center"]
        for waypoint in dense_route[first_index:last_index + 1]:
            location = waypoint.transform.location
            angle = math.atan2(location.y - center_y, location.x - center_x)
            if previous is not None:
                accumulated += self._normalise_angle(angle - previous)
            previous = angle
        if abs(accumulated) < math.radians(20.0):
            raise RuntimeError("VUT环岛路线转角不足，无法判断环岛行驶方向")
        return 1 if accumulated > 0.0 else -1

    def _make_gate(self, waypoint, source, number=None):
        location = waypoint.transform.location
        forward = waypoint.transform.get_forward_vector()
        half_width = max(1.0, float(waypoint.lane_width) * 0.5)
        # (-forward.y, forward.x) is left in CARLA's x/y world plane.
        left = {
            "x": round(location.x - forward.y * half_width, 4),
            "y": round(location.y + forward.x * half_width, 4),
            "z": round(location.z + 0.2, 4),
        }
        right = {
            "x": round(location.x + forward.y * half_width, 4),
            "y": round(location.y - forward.x * half_width, 4),
            "z": round(location.z + 0.2, 4),
        }
        gate = {
            "center": {
                "x": round(location.x, 4), "y": round(location.y, 4),
                "z": round(location.z + 0.2, 4),
            },
            "left": left,
            "right": right,
            "forward": {
                "x": round(float(forward.x), 6),
                "y": round(float(forward.y), 6),
                "z": round(float(forward.z), 6),
            },
            "approach_yaw": round(float(waypoint.transform.rotation.yaw), 4),
            "width_m": round(half_width * 2.0, 4),
            "road_id": int(waypoint.road_id),
            "section_id": int(waypoint.section_id),
            "lane_id": int(waypoint.lane_id),
            "s": round(float(waypoint.s), 4),
            "source": source,
        }
        if number is not None:
            gate["number"] = int(number)
        return gate

    def _entry_waypoint_index(self, dense_route, circle, first_ring):
        center = circle["center"]
        radius = circle["radius"]
        entry_index = max(0, first_ring - 1)
        entry_source = "route_curvature_fallback"
        candidates = []
        for index in range(1, min(len(dense_route), first_ring + 5)):
            if dense_route[index].is_junction and not dense_route[index - 1].is_junction:
                candidates.append(index)
        if candidates:
            entry_index = min(candidates, key=lambda value: abs(value - first_ring))
            entry_source = "junction_transition"
        else:
            for index in range(first_ring - 1, max(-1, first_ring - 25), -1):
                waypoint = dense_route[index]
                alignment, point_radius = self._ring_alignment(waypoint, center)
                forward = waypoint.transform.get_forward_vector()
                location = waypoint.transform.location
                radial_dot = ((forward.x * (location.x - center[0])
                              + forward.y * (location.y - center[1]))
                              / max(point_radius, 1e-6))
                if point_radius >= radius and radial_dot < -0.2 and alignment > 0.2:
                    entry_index = index
                    break
        return entry_index, entry_source

    def _entry_and_exit_waypoints(
            self, dense_route, circle, needs_entry=True, route_label="路线"):
        first_ring, last_ring = self._ring_route_indices(dense_route, circle)
        center = circle["center"]
        radius = circle["radius"]
        if needs_entry:
            entry_index, entry_source = self._entry_waypoint_index(
                dense_route, circle, first_ring)
        else:
            entry_index = max(0, first_ring - 1)
            entry_source = "route_curvature_fallback"

        exit_index = None
        for index in range(last_ring + 1, len(dense_route)):
            waypoint = dense_route[index]
            location = waypoint.transform.location
            point_radius = math.hypot(location.x - center[0], location.y - center[1])
            forward = waypoint.transform.get_forward_vector()
            radial_dot = ((forward.x * (location.x - center[0])
                          + forward.y * (location.y - center[1]))
                          / max(point_radius, 1e-6))
            if point_radius >= radius + 1.0 and radial_dot > 0.2:
                # Choose a point after the split so the preceding route sample
                # lies strictly on the negative side of the gate.
                exit_index = min(index + 1, len(dense_route) - 1)
                break
        if exit_index is None or exit_index >= len(dense_route) - 1:
            raise RuntimeError(
                "{}必须延伸到出口下游，才能生成有方向的出口门线".format(route_label))
        return entry_index, exit_index, entry_source, first_ring, last_ring

    def _trace_waypoint_to_ring(
            self, start, circle, direction, maximum_distance=45.0):
        """Trace one lane toward the fitted ring and retain failure evidence."""
        frontier = [(start, 0.0)]
        visited = set()
        maximum_explored = 0.0
        distance_limit_hit = False
        dead_end_count = 0
        radial_tolerance = max(6.0, circle["radius"] * 0.18)
        best = None
        while frontier:
            waypoint, distance = frontier.pop(0)
            token = self._waypoint_token(waypoint)
            if token in visited:
                continue
            if distance > maximum_distance:
                distance_limit_hit = True
                continue
            visited.add(token)
            maximum_explored = max(maximum_explored, distance)
            alignment, radius = self._ring_alignment(waypoint, circle["center"])
            radial_delta = radius - circle["radius"]
            radial_error = abs(radial_delta)
            score = (
                radial_error / max(radial_tolerance, 1e-6)
                + max(0.0, alignment - 0.72) / 0.28)
            if best is None or score < best["score"]:
                best = {
                    "score": score,
                    "distance_m": distance,
                    "lane_key": self._lane_key(waypoint),
                    "s": float(waypoint.s),
                    "radius_m": radius,
                    "radial_delta_m": radial_delta,
                    "radial_error_m": radial_error,
                    "alignment": alignment,
                    "waypoint": waypoint,
                }
            if (alignment <= 0.72
                    and radial_error <= radial_tolerance):
                return {
                    "reached": True,
                    "reason": "reached",
                    "visited_count": len(visited),
                    "maximum_explored_m": maximum_explored,
                    "maximum_distance_m": float(maximum_distance),
                    "distance_limit_hit": distance_limit_hit,
                    "dead_end_count": dead_end_count,
                    "radial_tolerance_m": radial_tolerance,
                    "fitted_radius_m": float(circle["radius"]),
                    "best": best,
                }
            choices = waypoint.next(2.0) if direction > 0 else waypoint.previous(2.0)
            if not choices:
                dead_end_count += 1
            frontier.extend((candidate, distance + 2.0) for candidate in choices)
        if distance_limit_hit:
            reason = "distance_limit"
        elif dead_end_count:
            reason = "topology_dead_end"
        elif best and best["radial_error_m"] > radial_tolerance:
            reason = "outside_fitted_ring_band"
        else:
            reason = "not_tangential_to_fitted_ring"
        return {
            "reached": False,
            "reason": reason,
            "visited_count": len(visited),
            "maximum_explored_m": maximum_explored,
            "maximum_distance_m": float(maximum_distance),
            "distance_limit_hit": distance_limit_hit,
            "dead_end_count": dead_end_count,
            "radial_tolerance_m": radial_tolerance,
            "fitted_radius_m": float(circle["radius"]),
            "best": best,
        }

    def _waypoint_reaches_ring(self, start, circle, direction, maximum_distance=45.0):
        return self._trace_waypoint_to_ring(
            start, circle, direction, maximum_distance)["reached"]

    @staticmethod
    def _describe_ring_trace_failure(
            arm_id, lane_direction, seed_lane, snapped_lane, trace):
        """Turn one failed lane trace into an operator-actionable diagnosis."""
        direction_label = "next（朝行驶方向）" if lane_direction == "进口" \
            else "previous（逆行驶方向）"
        reason_labels = {
            "distance_limit": "搜索达到固定距离上限",
            "topology_dead_end": "CARLA车道拓扑在到达环岛前中断",
            "outside_fitted_ring_band": "搜索路径没有进入VUT路线拟合出的环形半径带",
            "not_tangential_to_fitted_ring": "搜索路径进入环形区域但方向不满足切向条件",
        }
        parts = [
            "{}的{}车道检查失败 [{}]".format(
                arm_id, lane_direction,
                reason_labels.get(trace["reason"], trace["reason"])),
            "STEP 1记录车道={}；门线中心重新吸附车道={}".format(
                seed_lane, snapped_lane),
            "沿{}以2m步长访问{}个路点，最远追踪{:.1f}m（上限{:.1f}m）".format(
                direction_label, trace["visited_count"],
                trace["maximum_explored_m"], trace["maximum_distance_m"]),
        ]
        if seed_lane != snapped_lane:
            parts.append("注意：重新吸附后的车道与STEP 1记录不一致，可能吸附到了相邻道路")
        best = trace.get("best")
        if best:
            radial_side = "拟合环外侧" if best["radial_delta_m"] > 0.0 else "拟合环内侧"
            parts.append(
                "最接近判定条件的路点：距起点{:.1f}m，lane={}，半径{:.1f}m；"
                "拟合环半径{:.1f}m，径向差{:+.1f}m（允许±{:.1f}m，{}）；"
                "切向指标={:.3f}（要求≤0.720）".format(
                    best["distance_m"], best["lane_key"], best["radius_m"],
                    trace["fitted_radius_m"], best["radial_delta_m"],
                    trace["radial_tolerance_m"], radial_side, best["alignment"]))
        if trace["reason"] == "distance_limit":
            parts.append("无法判断更远处是否可达环岛；当前100m工程搜索上限可能过小")
        elif trace["reason"] == "topology_dead_end":
            parts.append("请检查该lane的OpenDRIVE successor/predecessor，或确认IN/OUT方向是否标反")
        elif trace["reason"] == "outside_fitted_ring_band":
            parts.append("可能是拟合到了内/外另一条环形车道，或该接入口半径与VUT路线差异过大")
        else:
            parts.append("可能是IN/OUT方向标反，或连接段在判定区域内仍近似径向而非环向")
        return "；".join(parts)

    def _cluster_arm_records(self, records, ring_radius):
        if not records:
            return []
        maximum_lane_width = max(
            float(getattr(item["waypoint"], "lane_width", 3.5)) for item in records)
        separation_arc_m = max(8.0, maximum_lane_width * 3.0)
        threshold = max(
            math.radians(6.0),
            min(math.radians(18.0), math.atan2(separation_arc_m, max(ring_radius, 1.0))))
        ordered = sorted(records, key=lambda item: item["angle"])
        clusters = [[ordered[0]]]
        for record in ordered[1:]:
            if self._circular_distance(record["angle"], clusters[-1][-1]["angle"]) <= threshold:
                clusters[-1].append(record)
            else:
                clusters.append([record])
        if len(clusters) > 1 and self._circular_distance(
                clusters[0][0]["angle"], clusters[-1][-1]["angle"]) <= threshold:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()
        return clusters

    def _pair_split_carriageway_clusters(self, clusters, ring_radius):
        """Pair nearby one-way halves which belong to the same access arm.

        On large or divided approaches the inbound and outbound carriageways
        can be farther apart than the conservative first-pass clustering
        threshold.  CARLA then exposes two one-way clusters even though they
        form one physical roundabout arm.  Only mutually-nearest,
        complementary (inbound-only/outbound-only) clusters are paired here;
        complete arms and two clusters with the same direction are never
        merged.
        """
        clusters = [list(cluster) for cluster in clusters]
        repair_count = 0

        def directions(cluster):
            return {item["direction"] for item in cluster}

        def cluster_angle(cluster):
            sin_sum = sum(math.sin(item["angle"]) for item in cluster)
            cos_sum = sum(math.cos(item["angle"]) for item in cluster)
            return math.atan2(sin_sum, cos_sum)

        while True:
            incomplete = [
                index for index, cluster in enumerate(clusters)
                if len(directions(cluster)) == 1
            ]
            if len(incomplete) < 2:
                break
            angles = {index: cluster_angle(clusters[index]) for index in incomplete}
            nearest = {}
            for index in incomplete:
                own_direction = next(iter(directions(clusters[index])))
                candidates = [
                    other for other in incomplete
                    if other != index
                    and next(iter(directions(clusters[other]))) != own_direction
                ]
                if candidates:
                    nearest[index] = min(
                        candidates,
                        key=lambda other: self._circular_distance(
                            angles[index], angles[other]))

            pair = None
            pair_distance = None
            for index, other in nearest.items():
                if nearest.get(other) != index or index > other:
                    continue
                widths = [
                    float(getattr(item["waypoint"], "lane_width", 3.5))
                    for item in clusters[index] + clusters[other]
                ]
                # Permit a divided carriageway of roughly six lane widths,
                # while keeping distinct neighbouring approaches separate.
                maximum_arc_m = max(18.0, max(widths) * 6.0)
                threshold = max(
                    math.radians(10.0),
                    min(math.radians(28.0),
                        math.atan2(maximum_arc_m, max(ring_radius, 1.0))))
                distance = self._circular_distance(angles[index], angles[other])
                if distance <= threshold and (
                        pair_distance is None or distance < pair_distance):
                    pair = (index, other)
                    pair_distance = distance
            if pair is None:
                break
            first, second = pair
            clusters[first].extend(clusters[second])
            clusters.pop(second)
            repair_count += 1

        # A five-arm roundabout can have a widely divided approach for which
        # the two carriageway mouths are not each other's locally nearest
        # opposite-direction cluster.  Pair the remaining one-way halves with
        # an order-preserving cyclic assignment.  The admissible angle is
        # derived from the observed spacing between physical approaches and
        # capped so unrelated neighbouring arms cannot be silently combined.
        incomplete_inbound = [
            index for index, cluster in enumerate(clusters)
            if directions(cluster) == {"inbound"}
        ]
        incomplete_outbound = [
            index for index, cluster in enumerate(clusters)
            if directions(cluster) == {"outbound"}
        ]
        if (incomplete_inbound
                and len(incomplete_inbound) == len(incomplete_outbound)):
            def direction_spacing(direction):
                values = sorted(
                    cluster_angle(cluster) % (2.0 * math.pi)
                    for cluster in clusters if direction in directions(cluster))
                if len(values) < 2:
                    return math.radians(60.0)
                gaps = [
                    (values[(index + 1) % len(values)] - value)
                    % (2.0 * math.pi)
                    for index, value in enumerate(values)
                ]
                return self._median(gaps)

            typical_arm_spacing = min(
                direction_spacing("inbound"),
                direction_spacing("outbound"))
            global_threshold = max(
                math.radians(28.0),
                min(math.radians(50.0), typical_arm_spacing * 0.62))
            inbound_order = sorted(
                incomplete_inbound,
                key=lambda index: cluster_angle(clusters[index]) % (2.0 * math.pi))
            outbound_order = sorted(
                incomplete_outbound,
                key=lambda index: cluster_angle(clusters[index]) % (2.0 * math.pi))
            best_pairs = None
            best_score = None
            for shift in range(len(outbound_order)):
                pairs = [
                    (inbound_index,
                     outbound_order[(position + shift) % len(outbound_order)])
                    for position, inbound_index in enumerate(inbound_order)
                ]
                distances = [
                    self._circular_distance(
                        cluster_angle(clusters[first]),
                        cluster_angle(clusters[second]))
                    for first, second in pairs
                ]
                if any(distance > global_threshold for distance in distances):
                    continue
                score = sum(distance * distance for distance in distances)
                if best_score is None or score < best_score:
                    best_score = score
                    best_pairs = pairs
            if best_pairs:
                paired_indexes = {
                    index for pair in best_pairs for index in pair
                }
                merged = [
                    cluster for index, cluster in enumerate(clusters)
                    if index not in paired_indexes
                ]
                merged.extend(
                    clusters[first] + clusters[second]
                    for first, second in best_pairs)
                clusters = merged
                repair_count += len(best_pairs)
        return clusters, repair_count

    def _manual_roundabout_arms(self, circle):
        """Resolve STEP 1 lane seeds against the VUT-fitted ring topology."""
        self.roundabout_last_trace_failure = None
        seeds = getattr(self, "roundabout_topology_seeds", [])
        if len(seeds) < 3 or any(
                not arm.get("inbound") or not arm.get("outbound") for arm in seeds):
            raise RuntimeError("STEP 1拓扑标定至少需要3个完整的进口/出口arm")
        center_x, center_y = circle["center"]
        arms = []
        for seed_arm in seeds:
            records = {}
            for direction, trace_direction in (("inbound", 1), ("outbound", -1)):
                gate = seed_arm[direction]["gate"]
                waypoint = self.map.get_waypoint(
                    carla.Location(
                        x=float(gate["center"]["x"]),
                        y=float(gate["center"]["y"]),
                        z=float(gate["center"].get("z", 0.0))),
                    project_to_road=True, lane_type=carla.LaneType.Driving)
                if waypoint is None:
                    self.roundabout_last_trace_failure = {
                        "arm_id": seed_arm["arm_id"],
                        "lane_direction": direction,
                        "reason": "snap_failed",
                        "seed_gate": copy.deepcopy(gate),
                        "closest_gate": None,
                    }
                    raise RuntimeError(
                        "{}的{}门线中心无法重新吸附到Driving Lane；STEP 1记录车道={}".format(
                            seed_arm["arm_id"],
                            "进口" if direction == "inbound" else "出口",
                            (
                                int(seed_arm[direction]["lane"]["road_id"]),
                                int(seed_arm[direction]["lane"]["section_id"]),
                                int(seed_arm[direction]["lane"]["lane_id"]),
                            )))
                trace = self._trace_waypoint_to_ring(
                    waypoint, circle, trace_direction, maximum_distance=100.0)
                if not trace["reached"]:
                    seed_lane = (
                        int(seed_arm[direction]["lane"]["road_id"]),
                        int(seed_arm[direction]["lane"]["section_id"]),
                        int(seed_arm[direction]["lane"]["lane_id"]),
                    )
                    self.roundabout_last_trace_failure = {
                        "arm_id": seed_arm["arm_id"],
                        "lane_direction": direction,
                        "reason": trace["reason"],
                        "seed_gate": copy.deepcopy(gate),
                        "closest_gate": (
                            self._make_gate(
                                trace["best"]["waypoint"],
                                "failed_ring_trace_closest_point")
                            if trace.get("best") else None),
                    }
                    raise RuntimeError(self._describe_ring_trace_failure(
                        seed_arm["arm_id"],
                        "进口" if direction == "inbound" else "出口",
                        seed_lane, self._lane_key(waypoint), trace))
                lane_waypoints = [
                    candidate for candidate in self._parallel_same_direction_lanes(waypoint)
                    if self._waypoint_reaches_ring(
                        candidate, circle, trace_direction, maximum_distance=100.0)
                ]
                records[direction] = []
                for lane_waypoint in lane_waypoints:
                    location = lane_waypoint.transform.location
                    records[direction].append({
                        "direction": direction,
                        "lane_key": self._lane_key(lane_waypoint),
                        "waypoint": lane_waypoint,
                        "angle": math.atan2(
                            location.y - center_y, location.x - center_x),
                    })

            def mean_angle(items):
                return math.atan2(
                    sum(math.sin(item["angle"]) for item in items),
                    sum(math.cos(item["angle"]) for item in items))

            inbound_angle = mean_angle(records["inbound"])
            outbound_angle = mean_angle(records["outbound"])
            arms.append({
                "arm_id": seed_arm["arm_id"],
                "angle": math.atan2(
                    math.sin(inbound_angle) + math.sin(outbound_angle),
                    math.cos(inbound_angle) + math.cos(outbound_angle)),
                "inbound_angle": inbound_angle,
                "outbound_angle": outbound_angle,
                "inbound": records["inbound"],
                "outbound": records["outbound"],
                "pairing_method": "operator_seeded_carla_connectivity",
            })
        self.roundabout_topology_repair_count = 0
        return arms

    def _parallel_same_direction_lanes(self, seed_waypoint):
        """Expand a seed across one contiguous same-direction carriageway.

        Custom OpenDRIVE maps often model every physical lane as a separate
        road whose lane_id is always -1.  CARLA's get_left/right_lane() then
        cannot expose the neighbour, and requiring identical road/section
        reduces every reviewed direction to one lane.  Combine API adjacency
        with a local cross-section scan over generated Driving waypoints.
        """
        seed_forward = seed_waypoint.transform.get_forward_vector()
        seed_location = seed_waypoint.transform.location
        seed_width = float(getattr(seed_waypoint, "lane_width", 3.5))
        driving_type = getattr(getattr(carla, "LaneType", None), "Driving", None)
        maximum_lateral_m = max(
            ROUNDABOUT_PARALLEL_LANE_MIN_LATERAL_M,
            seed_width * ROUNDABOUT_PARALLEL_LANE_MAX_WIDTHS)
        records = {}

        def candidate_record(candidate, source):
            if candidate is None:
                return None
            candidate_type = getattr(candidate, "lane_type", driving_type)
            if driving_type is not None and candidate_type != driving_type:
                return None
            forward = candidate.transform.get_forward_vector()
            alignment = seed_forward.x * forward.x + seed_forward.y * forward.y
            if alignment < ROUNDABOUT_PARALLEL_LANE_HEADING_ALIGNMENT:
                return None
            location = candidate.transform.location
            dx = location.x - seed_location.x
            dy = location.y - seed_location.y
            longitudinal_m = dx * seed_forward.x + dy * seed_forward.y
            lateral_m = dx * -seed_forward.y + dy * seed_forward.x
            if (abs(longitudinal_m) > ROUNDABOUT_PARALLEL_LANE_LONGITUDINAL_M
                    or abs(lateral_m) > maximum_lateral_m):
                return None
            return {
                "waypoint": candidate,
                "source": source,
                "longitudinal_m": longitudinal_m,
                "lateral_m": lateral_m,
                "score": abs(longitudinal_m) + abs(lateral_m) * 0.05,
                "lane_width": float(getattr(candidate, "lane_width", seed_width)),
            }

        def remember(candidate, source):
            record = candidate_record(candidate, source)
            if record is None:
                return False
            key = self._lane_key(candidate)
            previous = records.get(key)
            if previous is None or record["score"] < previous["score"]:
                records[key] = record
            return True

        remember(seed_waypoint, "operator_seed")

        for accessor in ("get_left_lane", "get_right_lane"):
            current = seed_waypoint
            visited = {self._lane_key(seed_waypoint)}
            for _ in range(8):
                method = getattr(current, accessor, None)
                candidate = method() if callable(method) else None
                if candidate is None:
                    break
                lane_key = self._lane_key(candidate)
                if lane_key in visited:
                    break
                visited.add(lane_key)
                if not remember(candidate, "carla_adjacent_lane"):
                    break
                current = candidate

        # The generated 2 m waypoint cloud supplies lanes that CARLA cannot
        # expose through get_left/right_lane because each was authored as an
        # independent OpenDRIVE road.  Keep the closest sample from each lane
        # identity at the seed's cross-section.
        for candidate in getattr(self, "waypoints", []):
            remember(candidate, "spatial_cross_section")

        # A road/section boundary may create two lane identities on the same
        # physical centreline.  Collapse those before counting lanes.
        centreline_tolerance_m = max(0.8, min(1.5, seed_width * 0.35))
        physical = []
        for record in sorted(
                records.values(),
                key=lambda item: (abs(item["lateral_m"]), item["score"])):
            if any(abs(record["lateral_m"] - chosen["lateral_m"])
                   < centreline_tolerance_m for chosen in physical):
                continue
            physical.append(record)

        # Only retain the contiguous group containing the operator seed.  A
        # similarly oriented service road or another arm can be spatially
        # close, but it should not jump a median-sized lateral gap.
        physical.sort(key=lambda item: item["lateral_m"])
        seed_index = min(
            range(len(physical)),
            key=lambda index: abs(physical[index]["lateral_m"]))
        first = seed_index
        while first > 0:
            left, right = physical[first - 1], physical[first]
            allowed_gap_m = max(
                left["lane_width"], right["lane_width"], seed_width)
            allowed_gap_m *= ROUNDABOUT_PARALLEL_LANE_CONTIGUOUS_WIDTHS
            if right["lateral_m"] - left["lateral_m"] > allowed_gap_m:
                break
            first -= 1
        last = seed_index
        while last + 1 < len(physical):
            left, right = physical[last], physical[last + 1]
            allowed_gap_m = max(
                left["lane_width"], right["lane_width"], seed_width)
            allowed_gap_m *= ROUNDABOUT_PARALLEL_LANE_CONTIGUOUS_WIDTHS
            if right["lateral_m"] - left["lateral_m"] > allowed_gap_m:
                break
            last += 1
        return [
            record["waypoint"] for record in physical[first:last + 1]
        ]

    def _topology_seed_waypoint(self, seed):
        gate = seed["gate"]
        lane = seed.get("lane", {})
        road_id = lane.get("road_id", gate.get("road_id"))
        section_id = lane.get("section_id", gate.get("section_id"))
        lane_id = lane.get("lane_id", gate.get("lane_id"))
        station = gate.get("s")
        get_xodr = getattr(self.map, "get_waypoint_xodr", None)
        if (callable(get_xodr) and road_id is not None
                and section_id is not None and lane_id is not None
                and station is not None):
            exact = get_xodr(int(road_id), int(lane_id), float(station))
            if (exact is not None and self._lane_key(exact) == (
                    int(road_id), int(section_id), int(lane_id))):
                return exact
        return self.map.get_waypoint(
            carla.Location(
                x=float(gate["center"]["x"]),
                y=float(gate["center"]["y"]),
                z=float(gate["center"].get("z", 0.0))),
            project_to_road=True, lane_type=carla.LaneType.Driving)

    def _build_roundabout_topology_review(self):
        """Expand marked representatives and build the P-stage visual audit."""
        review_arms = []
        errors = []
        warnings = []
        diagnostics = []
        # One OpenDRIVE lane identity can legitimately pass more than one
        # roundabout mouth.  Ownership therefore belongs to a lane *at a
        # cross-section*, not globally to (road_id, section_id, lane_id).
        lane_uses = {}
        seeds = getattr(self, "roundabout_topology_seeds", [])
        selected_gates = [
            arm[direction]["gate"]
            for arm in seeds
            for direction in ("inbound", "outbound")
            if arm.get(direction)
        ]
        estimated_center = None
        if selected_gates:
            estimated_center = {
                "x": sum(gate["center"]["x"] for gate in selected_gates)
                / len(selected_gates),
                "y": sum(gate["center"]["y"] for gate in selected_gates)
                / len(selected_gates),
            }

        for seed_arm in seeds:
            review_arm = {"arm_id": seed_arm["arm_id"], "inbound": [], "outbound": []}
            for direction in ("inbound", "outbound"):
                seed = seed_arm.get(direction)
                if not seed:
                    errors.append("{}缺少{}标记".format(
                        seed_arm["arm_id"], "进口" if direction == "inbound" else "出口"))
                    continue
                waypoint = self._topology_seed_waypoint(seed)
                if waypoint is None:
                    errors.append("{}的{}无法吸附到Driving Lane".format(
                        seed_arm["arm_id"], "进口" if direction == "inbound" else "出口"))
                    continue
                for lane_waypoint in self._parallel_same_direction_lanes(waypoint):
                    lane_key = self._lane_key(lane_waypoint)
                    lane_gate = self._make_gate(
                        lane_waypoint,
                        "operator_topology_review_{}".format(direction))
                    lane_center = lane_gate["center"]
                    excluded_here = False
                    for exclusion in seed.get("excluded_cross_sections", []):
                        excluded_lane = exclusion.get("lane", {})
                        excluded_key = (
                            int(excluded_lane.get("road_id", -999999)),
                            int(excluded_lane.get("section_id", -999999)),
                            int(excluded_lane.get("lane_id", -999999)),
                        )
                        excluded_center = exclusion.get("center", {})
                        if excluded_key != lane_key:
                            continue
                        separation_m = math.hypot(
                            float(lane_center["x"])
                            - float(excluded_center.get("x", 1e9)),
                            float(lane_center["y"])
                            - float(excluded_center.get("y", 1e9)))
                        if separation_m <= ROUNDABOUT_TOPOLOGY_SAME_CROSS_SECTION_M:
                            excluded_here = True
                            break
                    if excluded_here:
                        continue
                    current_use = {
                        "arm_id": seed_arm["arm_id"],
                        "direction": direction,
                        "gate": lane_gate,
                        "s": float(getattr(lane_waypoint, "s", 0.0)),
                    }
                    overlapping_uses = []
                    for previous_use in lane_uses.get(lane_key, []):
                        previous_center = previous_use["gate"]["center"]
                        separation_m = math.hypot(
                            float(lane_center["x"]) - float(previous_center["x"]),
                            float(lane_center["y"]) - float(previous_center["y"]))
                        if separation_m <= ROUNDABOUT_TOPOLOGY_SAME_CROSS_SECTION_M:
                            overlapping_uses.append((previous_use, separation_m))
                    if overlapping_uses:
                        for previous_use, separation_m in overlapping_uses:
                            errors.append(
                                "车道{}在同一横断面被{} {}和{} {}重复使用："
                                "门线距离{:.1f}m，s={:.1f}/{:.1f}".format(
                                    lane_key,
                                    previous_use["arm_id"], previous_use["direction"],
                                    current_use["arm_id"], current_use["direction"],
                                    separation_m, previous_use["s"], current_use["s"]))
                            diagnostics.append({
                                "kind": "lane_cross_section_conflict",
                                "lane": {
                                    "road_id": lane_key[0],
                                    "section_id": lane_key[1],
                                    "lane_id": lane_key[2],
                                },
                                "separation_m": separation_m,
                                "owners": [copy.deepcopy(previous_use),
                                           copy.deepcopy(current_use)],
                            })
                        continue
                    lane_uses.setdefault(lane_key, []).append(current_use)
                    review_arm[direction].append({
                        "gate": lane_gate,
                        "lane": self._lane_descriptor({"waypoint": lane_waypoint}),
                    })

                if estimated_center is not None:
                    gate = seed["gate"]
                    to_center_x = estimated_center["x"] - gate["center"]["x"]
                    to_center_y = estimated_center["y"] - gate["center"]["y"]
                    distance = math.hypot(to_center_x, to_center_y)
                    forward = gate["forward"]
                    alignment = ((forward["x"] * to_center_x + forward["y"] * to_center_y)
                                 / max(distance, 1e-6))
                    direction_ok = (
                        alignment > 0.05 if direction == "inbound" else alignment < -0.05)
                    if not direction_ok:
                        warnings.append(
                            "{} {}箭头相对估计环岛中心方向可疑，请人工重点核对".format(
                                seed_arm["arm_id"],
                                "IN" if direction == "inbound" else "OUT"))
            if not review_arm["inbound"] or not review_arm["outbound"]:
                errors.append("{}未形成至少1条IN和1条OUT Driving Lane".format(
                    seed_arm["arm_id"]))
            review_arms.append(review_arm)

        return {
            "arms": review_arms,
            "arm_count": len(review_arms),
            "lane_count": sum(
                len(arm[direction]) for arm in review_arms
                for direction in ("inbound", "outbound")),
            "warnings": warnings,
            "diagnostics": diagnostics,
        }, errors

    @staticmethod
    def _gate_group_angle(items, center):
        """Return the circular mean angle of one reviewed lane-mouth group."""
        angles = [
            math.atan2(
                float(item["gate"]["center"]["y"]) - center[1],
                float(item["gate"]["center"]["x"]) - center[0])
            for item in items
        ]
        if not angles:
            return None
        return math.atan2(
            sum(math.sin(angle) for angle in angles),
            sum(math.cos(angle) for angle in angles))

    def _recommend_roundabout_from_vut_start(self, vut_waypoint):
        """Build a preliminary merge/exit proposal from the selected VUT lane.

        This intentionally uses only the operator-reviewed STEP 1 topology.
        The route-derived analysis remains authoritative after STEP 3.
        """
        review = getattr(self, "roundabout_topology_review", None)
        if not review or not getattr(self, "roundabout_topology_confirmed", False):
            return None, ["STEP 1拓扑尚未审核确认。"]
        review_arms = review.get("arms", [])
        if len(review_arms) < 3:
            return None, ["已审核拓扑不足3个物理接入方向。"]

        arm_centers = []
        for arm in review_arms:
            gates = [
                item["gate"] for direction in ("inbound", "outbound")
                for item in arm.get(direction, [])
            ]
            if gates:
                arm_centers.append((
                    sum(float(gate["center"]["x"]) for gate in gates) / len(gates),
                    sum(float(gate["center"]["y"]) for gate in gates) / len(gates),
                ))
        if len(arm_centers) < 3:
            return None, ["审核结果缺少可用于出口排序的IN/OUT门线。"]
        estimated_center = (
            sum(point[0] for point in arm_centers) / len(arm_centers),
            sum(point[1] for point in arm_centers) / len(arm_centers),
        )

        arms = []
        for review_arm in review_arms:
            inbound_items = review_arm.get("inbound", [])
            outbound_items = review_arm.get("outbound", [])
            inbound_angle = self._gate_group_angle(inbound_items, estimated_center)
            outbound_angle = self._gate_group_angle(outbound_items, estimated_center)
            if inbound_angle is None or outbound_angle is None:
                return None, ["{}缺少完整IN/OUT车道。".format(
                    review_arm.get("arm_id", "unknown arm"))]
            arms.append({
                "arm_id": review_arm["arm_id"],
                "angle": math.atan2(
                    math.sin(inbound_angle) + math.sin(outbound_angle),
                    math.cos(inbound_angle) + math.cos(outbound_angle)),
                "inbound_angle": inbound_angle,
                "outbound_angle": outbound_angle,
                "inbound": inbound_items,
                "outbound": outbound_items,
            })

        vut_lane_key = self._lane_key(vut_waypoint)
        exact = [
            arm for arm in arms
            if any((
                int(item["lane"]["road_id"]),
                int(item["lane"]["section_id"]),
                int(item["lane"]["lane_id"]),
            ) == vut_lane_key for item in arm["inbound"])
        ]
        candidates = exact
        reachable = []
        if not candidates:
            for arm in arms:
                records = []
                for item in arm["inbound"]:
                    waypoint = self._topology_seed_waypoint(item)
                    if waypoint is not None:
                        records.append({
                            "lane_key": (
                                int(item["lane"]["road_id"]),
                                int(item["lane"]["section_id"]),
                                int(item["lane"]["lane_id"]),
                            ),
                            "waypoint": waypoint,
                        })
                if not records:
                    continue
                trace = self._trace_waypoint_to_records(
                    vut_waypoint, records, maximum_distance=150.0,
                    trace_direction="forward", allow_spatial_match=False)
                if trace["reached"] and trace.get("best"):
                    reachable.append((
                        float(trace["best"]["distance_m"]), arm, trace))
            candidates = []
            if reachable:
                first_distance_m = min(item[0] for item in reachable)
                # BFS advances in 2 m steps.  Only arms reached in the same
                # first layer are genuinely ambiguous; other IN lanes reached
                # later are simply downstream arms encountered after entry.
                candidates = [
                    arm for distance_m, arm, _trace in reachable
                    if abs(distance_m - first_distance_m) <= 0.1
                ]
        if len(candidates) != 1:
            detail = ""
            if not exact and reachable:
                detail = "；首达距离=" + ", ".join(
                    "{}:{:.1f}m".format(arm["arm_id"], distance_m)
                    for distance_m, arm, _trace in sorted(
                        reachable, key=lambda item: item[0]))
            return None, [
                "VUT起点进口车道{}的首个可达IN门线关联到{}个arm{}；"
                "只有最短拓扑距离相同才视为歧义，请把VUT放在分流点之前的明确上游"
                "进口车道。".format(
                    vut_lane_key, len(candidates), detail)]
        entry_arm = candidates[0]

        # In a right-driving roundabout the paired outbound mouth lies just
        # behind the inbound mouth in the actual circulation direction.  The
        # sign of their shortest angular offset therefore reveals circulation.
        direction_score = sum(
            math.sin(arm["outbound_angle"] - arm["inbound_angle"])
            for arm in arms)
        if abs(direction_score) < 1e-3:
            return None, ["无法仅从已标定IN/OUT门线稳定推断环流方向。"]
        direction = -1 if direction_score > 0.0 else 1
        exit_order, entry_order = self._roundabout_arm_orders(
            arms, entry_arm, direction)
        if len(exit_order) < 3:
            return None, ["从该VUT汇入口无法枚举出口1、2、3。"]

        exits = {}
        for number, arm in enumerate(exit_order[:3], start=1):
            exits[str(number)] = {
                "arm_id": arm["arm_id"],
                "gates": [copy.deepcopy(item["gate"]) for item in arm["outbound"]],
                "vut_allowed": number in (2, 3),
            }
        return {
            "preliminary": True,
            "entry_arm_id": entry_arm["arm_id"],
            "entry_gates": [
                copy.deepcopy(item["gate"]) for item in entry_arm["inbound"]],
            "exit_gates": exits,
            "downstream_entry_arm_id": (
                entry_order[0]["arm_id"] if entry_order else None),
            "travel_direction": (
                "counterclockwise" if direction > 0 else "clockwise"),
        }, []

    def _scan_roundabout_arms(self, circle):
        """Infer connected inbound/outbound lanes around the fitted ring."""
        if (getattr(self, "roundabout_topology_confirmed", False)
                and getattr(self, "roundabout_topology_seeds", [])):
            return self._manual_roundabout_arms(circle)
        center_x, center_y = circle["center"]
        radius = circle["radius"]
        lower = max(1.0, radius - max(4.0, radius * 0.08))
        upper = radius + max(25.0, radius * 0.45)
        by_lane_direction = {}
        for waypoint in self.waypoints:
            location = waypoint.transform.location
            radial_x = location.x - center_x
            radial_y = location.y - center_y
            point_radius = math.hypot(radial_x, radial_y)
            if point_radius < lower or point_radius > upper:
                continue
            forward = waypoint.transform.get_forward_vector()
            radial_dot = (forward.x * radial_x + forward.y * radial_y) / max(point_radius, 1e-6)
            if radial_dot >= 0.48:
                lane_direction = "outbound"
                trace_direction = -1
            elif radial_dot <= -0.48:
                lane_direction = "inbound"
                trace_direction = 1
            else:
                continue
            lane_key = self._lane_key(waypoint)
            key = (lane_direction, lane_key)
            score = abs(point_radius - (radius + max(3.0, waypoint.lane_width)))
            current = by_lane_direction.get(key)
            if current is None or score < current[0]:
                by_lane_direction[key] = (score, waypoint, trace_direction)

        records = []
        for (lane_direction, lane_key), (_, waypoint, trace_direction) in by_lane_direction.items():
            if not self._waypoint_reaches_ring(waypoint, circle, trace_direction):
                continue
            location = waypoint.transform.location
            records.append({
                "direction": lane_direction,
                "lane_key": lane_key,
                "waypoint": waypoint,
                "angle": math.atan2(location.y - center_y, location.x - center_x),
            })
        clusters = self._cluster_arm_records(records, circle["radius"])
        for cluster_index, cluster in enumerate(clusters):
            for item in cluster:
                item["initial_cluster"] = cluster_index
        clusters, repair_count = self._pair_split_carriageway_clusters(
            clusters, circle["radius"])
        self.roundabout_topology_repair_count = repair_count
        arms = []
        for cluster in clusters:
            inbound = {}
            outbound = {}
            sin_sum = sum(math.sin(item["angle"]) for item in cluster)
            cos_sum = sum(math.cos(item["angle"]) for item in cluster)
            angle = math.atan2(sin_sum, cos_sum)
            for item in cluster:
                target = inbound if item["direction"] == "inbound" else outbound
                target[item["lane_key"]] = item
            inbound_values = sorted(inbound.values(), key=lambda item: item["lane_key"])
            outbound_values = sorted(outbound.values(), key=lambda item: item["lane_key"])

            def direction_angle(values):
                if not values:
                    return None
                return math.atan2(
                    sum(math.sin(item["angle"]) for item in values),
                    sum(math.cos(item["angle"]) for item in values))

            arms.append({
                "angle": angle,
                "inbound_angle": direction_angle(inbound_values),
                "outbound_angle": direction_angle(outbound_values),
                "inbound": inbound_values,
                "outbound": outbound_values,
                "pairing_method": (
                    "split_carriageway_pairing"
                    if len({item.get("initial_cluster") for item in cluster}) > 1
                    else "angular_cluster"),
            })
        arms.sort(key=lambda item: (item["angle"] + 2.0 * math.pi) % (2.0 * math.pi))
        for index, arm in enumerate(arms, start=1):
            arm["arm_id"] = "arm_{:02d}".format(index)
        return arms

    def _roundabout_arm_orders(self, arms, entry_arm, direction):
        """Order actual lane mouths, not the average angle of a divided arm."""
        exit_order = sorted(
            [arm for arm in arms if arm["outbound"]],
            key=lambda arm: (
                2.0 * math.pi if arm is entry_arm else
                self._travel_delta(
                    entry_arm["inbound_angle"], arm["outbound_angle"], direction)))
        entry_order = sorted(
            [arm for arm in arms if arm["inbound"] and arm is not entry_arm],
            key=lambda arm: self._travel_delta(
                entry_arm["inbound_angle"], arm["inbound_angle"], direction))
        return exit_order, entry_order

    def _roundabout_route_speed_limits(self, route_waypoints):
        """Return an auditable per-road km/h fallback for runtime scoring."""
        route_road_ids = {int(waypoint.road_id) for waypoint in route_waypoints}
        discovered = {}
        try:
            root = ET.fromstring(self.map.to_opendrive())
            for road in root.findall(".//road"):
                try:
                    road_id = int(road.get("id"))
                except (TypeError, ValueError):
                    continue
                if road_id not in route_road_ids:
                    continue
                values = []
                for road_type in road.findall("type"):
                    speed = road_type.find("speed")
                    if speed is None:
                        continue
                    try:
                        value = float(speed.get("max"))
                    except (TypeError, ValueError):
                        continue
                    unit = str(speed.get("unit", "m/s")).strip().lower()
                    if unit in ("m/s", "mps", "meter_per_second"):
                        value *= 3.6
                    elif unit in ("mph", "mile_per_hour"):
                        value *= 1.609344
                    if math.isfinite(value) and value > 0.0:
                        values.append(value)
                if values:
                    discovered[road_id] = min(values)
        except (AttributeError, ET.ParseError, RuntimeError, TypeError, ValueError):
            discovered = {}

        missing = route_road_ids - set(discovered)
        source = "opendrive"
        if missing and discovered:
            conservative_limit = min(discovered.values())
            for road_id in missing:
                discovered[road_id] = conservative_limit
            missing = set()
            source = "opendrive+conservative_connector_inheritance"
        if missing and self.roundabout_speed_limit_kmh is None:
            raise RuntimeError(
                "目标地图无法为VUT路线道路 {} 提供可审计限速；请修复OpenDRIVE speed，"
                "或用 --roundabout_speed_limit_kmh 显式提供工程回退值".format(
                    ", ".join(str(value) for value in sorted(missing))))
        if missing:
            for road_id in missing:
                discovered[road_id] = float(self.roundabout_speed_limit_kmh)
            source = "opendrive+editor_override"
        return ({str(key): round(value, 4) for key, value in sorted(discovered.items())},
                source)

    @staticmethod
    def _travel_delta(start_angle, target_angle, direction):
        if direction > 0:
            return (target_angle - start_angle) % (2.0 * math.pi)
        return (start_angle - target_angle) % (2.0 * math.pi)

    def _nearest_arm_with_diagnostics(self, waypoint, arms, direction):
        location = waypoint.transform.location
        center = self.roundabout_analysis_circle["center"]
        angle = math.atan2(location.y - center[1], location.x - center[0])
        eligible = [arm for arm in arms if arm[direction]]
        if not eligible:
            return None, []
        lane_key = self._lane_key(waypoint)
        exact = [arm for arm in eligible
                 if any(record["lane_key"] == lane_key for record in arm[direction])]
        if exact:
            return min(
                exact,
                key=lambda arm: self._circular_distance(arm["angle"], angle)), []
        diagnostics = []
        connected = []
        for arm in eligible:
            trace = self._trace_waypoint_to_records(
                waypoint, arm[direction], maximum_distance=60.0,
                trace_direction="backward")
            diagnostics.append({"arm": arm, "trace": trace})
            if trace["reached"]:
                connected.append(arm)
        if connected:
            return min(
                connected,
                key=lambda arm: self._circular_distance(arm["angle"], angle)), diagnostics
        return None, diagnostics

    def _nearest_arm(self, waypoint, arms, direction):
        arm, _ = self._nearest_arm_with_diagnostics(waypoint, arms, direction)
        return arm

    @staticmethod
    def _lane_descriptor(record):
        waypoint = record["waypoint"]
        return CarlaMapEditor0916._waypoint_lane_descriptor(waypoint)

    @staticmethod
    def _waypoint_lane_descriptor(waypoint):
        return {
            "road_id": int(waypoint.road_id),
            "section_id": int(waypoint.section_id),
            "lane_id": int(waypoint.lane_id),
        }

    def _downstream_gate_for_vt2(
            self, vt2_entry_record, downstream_arm, vt2_waypoint):
        """Serialize both the reviewed mouth lane and VT2's proven approach."""
        downstream_gate = copy.deepcopy(vt2_entry_record["gate"])
        vt2_lane_descriptor = self._waypoint_lane_descriptor(vt2_waypoint)
        reviewed_inbound_lanes = [
            self._lane_descriptor(item) for item in downstream_arm["inbound"]]
        permitted_inbound_lanes = list(reviewed_inbound_lanes)
        if vt2_lane_descriptor not in permitted_inbound_lanes:
            permitted_inbound_lanes.append(vt2_lane_descriptor)
        downstream_gate.update({
            "arm_id": downstream_arm["arm_id"],
            "source": "operator_reviewed_vt2_downstream_inbound_lane",
            "selected_lane": vt2_lane_descriptor,
            "gate_lane": self._lane_descriptor(vt2_entry_record),
            "inbound_lanes": permitted_inbound_lanes,
            "lane_membership_proof": "carla_directed_topology_to_reviewed_in_gate",
        })
        return downstream_gate

    def _trace_waypoint_to_records(
            self, start, records, maximum_distance=100.0,
            trace_direction="forward", allow_spatial_match=True):
        """Trace to lane records and retain evidence if association fails.

        Fixture placement (for example VT2 upstream of an entry) is checked
        forward.  A route-derived gate is normally downstream of the arm
        sample, so arm association must trace backward through junction road
        ID changes.  Keeping the direction explicit prevents valid custom-map
        connectors from being reported as disconnected.
        """
        if trace_direction not in ("forward", "backward", "either"):
            raise ValueError("trace_direction must be forward, backward, or either")
        target_keys = {record["lane_key"] for record in records}
        target_locations = [record["waypoint"].transform.location for record in records]
        frontier = [(start, 0.0)]
        visited = set()
        maximum_explored = 0.0
        distance_limit_hit = False
        dead_end_count = 0
        best = None
        while frontier:
            waypoint, distance = frontier.pop(0)
            token = self._waypoint_token(waypoint)
            if token in visited:
                continue
            if distance > maximum_distance:
                distance_limit_hit = True
                continue
            visited.add(token)
            maximum_explored = max(maximum_explored, distance)
            distances = [
                self._distance_locations(waypoint.transform.location, location)
                for location in target_locations]
            nearest_index = min(range(len(distances)), key=distances.__getitem__)
            spatial_distance = distances[nearest_index]
            if best is None or spatial_distance < best["spatial_distance_m"]:
                best = {
                    "distance_m": distance,
                    "spatial_distance_m": spatial_distance,
                    "lane_key": self._lane_key(waypoint),
                    "waypoint": waypoint,
                    "target_record": records[nearest_index],
                }
            if (self._lane_key(waypoint) in target_keys
                    or (allow_spatial_match and spatial_distance <= 3.0)):
                return {
                    "reached": True,
                    "reason": "reached",
                    "visited_count": len(visited),
                    "maximum_explored_m": maximum_explored,
                    "maximum_distance_m": float(maximum_distance),
                    "distance_limit_hit": distance_limit_hit,
                    "dead_end_count": dead_end_count,
                    "best": best,
                }
            candidates = []
            if trace_direction in ("forward", "either"):
                candidates.extend(waypoint.next(2.0))
            if trace_direction in ("backward", "either"):
                candidates.extend(waypoint.previous(2.0))
            if not candidates:
                dead_end_count += 1
            frontier.extend((candidate, distance + 2.0) for candidate in candidates)
        reason = (
            "distance_limit" if distance_limit_hit else
            ("topology_dead_end" if dead_end_count else "no_matching_lane"))
        return {
            "reached": False,
            "reason": reason,
            "visited_count": len(visited),
            "maximum_explored_m": maximum_explored,
            "maximum_distance_m": float(maximum_distance),
            "distance_limit_hit": distance_limit_hit,
            "dead_end_count": dead_end_count,
            "best": best,
        }

    def _route_reaches_records(
            self, start, records, maximum_distance=100.0,
            trace_direction="forward", allow_spatial_match=True):
        return self._trace_waypoint_to_records(
            start, records, maximum_distance, trace_direction,
            allow_spatial_match=allow_spatial_match)["reached"]

    def _prepare_vt2_waypoint(self, clicked_waypoint, downstream_arm):
        """Validate VT2's physical approach and keep it before its IN gate.

        The click chooses a lane.  The exact longitudinal fixture position is
        engineering-defined: a valid upstream click is preserved, while a
        click on the correct lane after/too close to the reviewed gate is
        moved to a fixed point upstream.  An OUT lane or another physical arm
        is rejected rather than silently projected across carriageways.
        """
        if downstream_arm is None or not downstream_arm.get("inbound"):
            raise RuntimeError("程序尚未得到下游第1入口的进口车道")
        records = downstream_arm["inbound"]
        forward_trace = self._trace_waypoint_to_records(
            clicked_waypoint, records, maximum_distance=100.0,
            trace_direction="forward", allow_spatial_match=False)
        backward_trace = self._trace_waypoint_to_records(
            clicked_waypoint, records, maximum_distance=100.0,
            trace_direction="backward", allow_spatial_match=False)
        association = forward_trace if forward_trace["reached"] else backward_trace
        if not association["reached"] or not association.get("best"):
            nearest = min(
                records,
                key=lambda record: self._distance_locations(
                    clicked_waypoint.transform.location,
                    record["waypoint"].transform.location))
            nearest_distance = self._distance_locations(
                clicked_waypoint.transform.location,
                nearest["waypoint"].transform.location)
            raise RuntimeError(
                "点击吸附到lane={}，它沿next/previous均不能连接到{}的进口lane；"
                "最近审核进口lane={}，直线距离{:.1f}m。请在绿色{} IN箭头所在"
                "进口车道上右键，不要点蓝色OUT车道。".format(
                    self._lane_key(clicked_waypoint), downstream_arm["arm_id"],
                    nearest["lane_key"], nearest_distance,
                    downstream_arm["arm_id"].replace("arm_", "A")))

        record = association["best"]["target_record"]
        signed_distance = gate_signed_distance(
            self._waypoint_payload(clicked_waypoint), record["gate"])
        if (forward_trace["reached"]
                and signed_distance <= -VT2_MINIMUM_GATE_CLEARANCE_M):
            return clicked_waypoint, {
                "adjusted": False,
                "arm_id": downstream_arm["arm_id"],
                "lane_key": self._lane_key(clicked_waypoint),
                "entry_record": record,
                "gate_signed_distance_m": signed_distance,
            }

        try:
            candidates = list(record["waypoint"].previous(
                VT2_DEFAULT_UPSTREAM_DISTANCE_M))
        except (AttributeError, RuntimeError, TypeError, ValueError):
            candidates = []
        candidates = [
            candidate for candidate in candidates
            if self._route_reaches_records(
                candidate, records, maximum_distance=100.0,
                trace_direction="forward", allow_spatial_match=False)
        ]
        if not candidates:
            side = "已经越过" if backward_trace["reached"] else "距离过近"
            raise RuntimeError(
                "点击点{}{}进口门线，但CARLA拓扑无法在该门线上游{:.1f}m找到"
                "可用Driving Lane。请沿绿色IN箭头反方向前移后再右键。".format(
                    side, downstream_arm["arm_id"],
                    VT2_DEFAULT_UPSTREAM_DISTANCE_M))
        placement = min(
            candidates,
            key=lambda candidate: self._distance_locations(
                candidate.transform.location,
                clicked_waypoint.transform.location))
        return placement, {
            "adjusted": True,
            "arm_id": downstream_arm["arm_id"],
            "lane_key": self._lane_key(placement),
            "entry_record": record,
            "clicked_lane_key": self._lane_key(clicked_waypoint),
            "clicked_gate_signed_distance_m": signed_distance,
            "upstream_distance_m": VT2_DEFAULT_UPSTREAM_DISTANCE_M,
        }

    def _describe_arm_association_failure(
            self, waypoint, direction, diagnostics, entry_source=None):
        """Explain why a route-derived gate did not match a reviewed arm."""
        location = waypoint.transform.location
        lane_key = self._lane_key(waypoint)
        if not diagnostics:
            return (
                "自动{}点lane={}，位置=({:.1f},{:.1f})；没有可参与匹配的{}arm".format(
                    "汇入口" if direction == "inbound" else "出口",
                    lane_key, location.x, location.y,
                    "进口" if direction == "inbound" else "出口"))
        ordered = sorted(
            diagnostics,
            key=lambda item: (
                item["trace"]["best"]["spatial_distance_m"]
                if item["trace"].get("best") else float("inf")))
        nearest = ordered[0]
        trace = nearest["trace"]
        best = trace.get("best")
        reason = {
            "distance_limit": "反向搜索达到{:.0f}m工程上限".format(
                trace["maximum_distance_m"]),
            "topology_dead_end": "CARLA predecessor在到达STEP 1进口前中断",
            "no_matching_lane": "搜索未到达任何已审核进口lane",
        }.get(trace["reason"], trace["reason"])
        parts = [
            "自动VUT汇入口无法关联到物理arm [{}]".format(reason),
            "入口来源={}；入口lane={}，位置=({:.1f},{:.1f})".format(
                entry_source or "unknown", lane_key, location.x, location.y),
            "已检查{}个进口arm；最接近{}".format(
                len(diagnostics), nearest["arm"]["arm_id"]),
            "沿previous以2m步长访问{}个路点，最远追踪{:.1f}m（上限{:.1f}m）".format(
                trace["visited_count"], trace["maximum_explored_m"],
                trace["maximum_distance_m"]),
        ]
        if best:
            target_lane = best["target_record"]["lane_key"]
            parts.append(
                "最接近点：距自动入口沿拓扑{:.1f}m，当前lane={}；"
                "距离{}目标lane={}仍有直线距离{:.1f}m".format(
                    best["distance_m"], best["lane_key"],
                    nearest["arm"]["arm_id"], target_lane,
                    best["spatial_distance_m"]))
        if trace["reason"] == "distance_limit":
            parts.append(
                "可能是自动入口点落在环岛内过远位置，或当前{:.0f}m关联上限过小".format(
                    trace["maximum_distance_m"]))
        elif trace["reason"] == "topology_dead_end":
            parts.append("可能是OpenDRIVE predecessor断开、入口连接方向异常，或自动入口点位于无法回溯的环岛lane")
        else:
            parts.append("可能是STEP 1进口标记吸附到相邻lane，或自动入口点选择到了错误的内/外环车道")
        return "；".join(parts)

    @staticmethod
    def _exit_allowed_lanes(start, maximum_distance=20.0):
        """Collect connector and immediate downstream lane IDs for exit scoring."""
        frontier = [(start, 0.0)]
        visited = set()
        descriptors = {}
        while frontier:
            waypoint, distance = frontier.pop(0)
            token = (
                int(waypoint.road_id), int(waypoint.section_id),
                int(waypoint.lane_id), round(float(waypoint.s), 1))
            if token in visited or distance > maximum_distance:
                continue
            visited.add(token)
            lane_key = token[:3]
            descriptors[lane_key] = {
                "road_id": lane_key[0], "section_id": lane_key[1],
                "lane_id": lane_key[2],
            }
            frontier.extend(
                (candidate, distance + 2.0) for candidate in waypoint.next(2.0))
        return [descriptors[key] for key in sorted(descriptors)]

    def add_roundabout_topology_seed(self, wx, wy):
        """Mark one inbound/outbound lane mouth for the current physical arm."""
        if not getattr(self, "roundabout_branch_confirmed", False):
            self._feedback("请先按Enter或R确认STEP 0测试分支。", True)
            return False
        if getattr(self, "roundabout_topology_confirmed", False):
            self._feedback("STEP 1拓扑已确认；如需重标请按X清除拓扑标定。", True)
            return False
        if getattr(self, "roundabout_topology_review_ready", False):
            self._feedback(
                "程序拓扑审核已经完成；请先检查地图上的全部IN/OUT高亮。"
                "确认正确按Enter，需修改按X或Backspace。", True)
            return False
        waypoint = self.get_nearest_waypoint(wx, wy)
        if waypoint is None:
            self._feedback("该位置没有可用Driving Lane。", True)
            return False
        lane = self._lane_descriptor({"waypoint": waypoint})
        candidate_gate = self._make_gate(
            waypoint, "operator_topology_candidate_seed")
        owner = self._roundabout_seed_cross_section_owner(lane, candidate_gate)
        if owner is not None:
            self._feedback(
                "该横断面已经用于{} {}（门线距离{:.1f}m）；"
                "同一车道编号在较远位置可以重复使用，但同一门线位置不能重复。".format(
                    owner[0], owner[1].upper(), owner[2]), True)
            return False

        seeds = self.roundabout_topology_seeds
        new_arm = not seeds or seeds[-1].get("outbound") is not None
        arm_id = ("arm_{:02d}".format(len(seeds) + 1)
                  if new_arm else seeds[-1]["arm_id"])
        kind = "inbound" if new_arm else self.roundabout_topology_mark_kind
        self._push_roundabout_history(
            "标定{} {}".format(arm_id, "进口" if kind == "inbound" else "出口"))
        if new_arm:
            seeds.append({
                "arm_id": arm_id,
                "inbound": None,
                "outbound": None,
            })
        arm = seeds[-1]
        if kind == "outbound" and arm.get("inbound") is None:
            kind = "inbound"
        candidate_gate["source"] = "operator_topology_{}_seed".format(kind)
        arm[kind] = {
            "gate": candidate_gate,
            "lane": lane,
        }
        self.roundabout_topology_mark_kind = (
            "outbound" if kind == "inbound" else "inbound")
        self.roundabout_topology_preview = None
        self.roundabout_topology_preview_errors = []
        self.roundabout_topology_review_ready = False
        self.roundabout_topology_review = None
        self.roundabout_topology_review_errors = []
        self.roundabout_vut_recommendation = None
        self.roundabout_vut_recommendation_errors = []
        self.roundabout_last_trace_failure = None
        self._mark_roundabout_dirty()
        complete_count = sum(
            1 for item in seeds if item.get("inbound") and item.get("outbound"))
        if kind == "inbound":
            self._feedback(
                "{}进口已标记（绿色）；现在在同一物理接入方向的出口车道右键。".format(
                    arm["arm_id"]))
        else:
            self._feedback(
                "{}进出口配对完成；已完成{}个arm。继续右键标记下一个arm进口，"
                "全部完成后按P运行程序拓扑审核。".format(arm["arm_id"], complete_count))
        return True

    def _roundabout_seed_cross_section_owner(
            self, lane, gate, ignored_target=None):
        """Find a seed using the same lane identity at the same mouth."""
        lane_key = (
            lane.get("road_id"), lane.get("section_id"), lane.get("lane_id"))
        center = gate.get("center", {})
        for arm_index, arm in enumerate(
                getattr(self, "roundabout_topology_seeds", [])):
            for direction in ("inbound", "outbound"):
                if ignored_target == (arm_index, direction):
                    continue
                seed = arm.get(direction)
                if not seed:
                    continue
                seed_lane = seed.get("lane", {})
                seed_key = (
                    seed_lane.get("road_id"), seed_lane.get("section_id"),
                    seed_lane.get("lane_id"))
                if seed_key != lane_key:
                    continue
                seed_center = seed.get("gate", {}).get("center", {})
                distance_m = math.hypot(
                    float(center.get("x", 1e9))
                    - float(seed_center.get("x", -1e9)),
                    float(center.get("y", 1e9))
                    - float(seed_center.get("y", -1e9)))
                if distance_m <= ROUNDABOUT_TOPOLOGY_SAME_CROSS_SECTION_M:
                    return arm["arm_id"], direction, distance_m
        return None

    def select_roundabout_topology_seed(self, sx, sy):
        """Select an exact reviewed gate for editing or exclusion."""
        if (not getattr(self, "roundabout_branch_confirmed", False)
                or getattr(self, "roundabout_topology_confirmed", False)):
            return False
        candidates = []
        review = getattr(self, "roundabout_topology_review", None)
        if review and review.get("arms"):
            for arm_index, arm in enumerate(review.get("arms", [])):
                for direction in ("inbound", "outbound"):
                    for item in arm.get(direction, []):
                        center = item["gate"]["center"]
                        marker = self.world_to_screen(center["x"], center["y"])
                        distance = math.hypot(marker[0] - sx, marker[1] - sy)
                        candidates.append((distance, arm_index, direction, item))
        else:
            for arm_index, arm in enumerate(
                    getattr(self, "roundabout_topology_seeds", [])):
                for direction in ("inbound", "outbound"):
                    seed = arm.get(direction)
                    if not seed:
                        continue
                    center = seed["gate"]["center"]
                    marker = self.world_to_screen(center["x"], center["y"])
                    distance = math.hypot(marker[0] - sx, marker[1] - sy)
                    candidates.append((distance, arm_index, direction, seed))
        if not candidates:
            return False
        distance, arm_index, direction, selected_item = min(
            candidates, key=lambda item: item[0])
        if distance > 36.0:
            return False
        self.roundabout_topology_edit_target = (arm_index, direction)
        arm = self.roundabout_topology_seeds[arm_index]
        seed = arm.get(direction) or {}
        selected_lane = selected_item.get("lane", {})
        selected_gate = selected_item.get("gate", {})
        seed_lane = seed.get("lane", {})
        seed_gate = seed.get("gate", {})
        selected_key = (
            selected_lane.get("road_id"), selected_lane.get("section_id"),
            selected_lane.get("lane_id"))
        seed_key = (
            seed_lane.get("road_id"), seed_lane.get("section_id"),
            seed_lane.get("lane_id"))
        selected_center = selected_gate.get("center", {})
        seed_center = seed_gate.get("center", {})
        seed_distance_m = math.hypot(
            float(selected_center.get("x", 1e9))
            - float(seed_center.get("x", -1e9)),
            float(selected_center.get("y", 1e9))
            - float(seed_center.get("y", -1e9)))
        is_operator_seed = (
            selected_key == seed_key
            and seed_distance_m <= ROUNDABOUT_TOPOLOGY_SAME_CROSS_SECTION_M)
        self.roundabout_topology_lane_target = {
            "arm_index": arm_index,
            "arm_id": arm["arm_id"],
            "direction": direction,
            "lane": copy.deepcopy(selected_lane),
            "gate": copy.deepcopy(selected_gate),
            "is_operator_seed": is_operator_seed,
        }
        if is_operator_seed:
            instruction = "这是人工代表点：右键可移动；如需删除整个arm请先撤销或按X重标"
        else:
            instruction = "这是程序展开车道：按Delete排除；右键可移动该arm的人工代表点"
        self._feedback(
            "已选中{} {}，lane=({},{},{})；{}。".format(
                arm["arm_id"], "IN" if direction == "inbound" else "OUT",
                selected_lane.get("road_id"), selected_lane.get("section_id"),
                selected_lane.get("lane_id"), instruction))
        return True

    def replace_selected_roundabout_topology_seed(self, wx, wy):
        """Move the selected STEP 1 representative and immediately re-audit."""
        if not getattr(self, "roundabout_branch_confirmed", False):
            self._feedback("请先按Enter或R确认STEP 0测试分支。", True)
            return False
        target = getattr(self, "roundabout_topology_edit_target", None)
        if target is None:
            self._feedback(
                "拓扑已加载/审核；需要修改时先左键选中一个IN或OUT标记，再右键重设。",
                True)
            return False
        arm_index, direction = target
        if not (0 <= arm_index < len(self.roundabout_topology_seeds)):
            self.roundabout_topology_edit_target = None
            self.roundabout_topology_lane_target = None
            self._feedback("待修改拓扑标记已经失效，请重新左键选择。", True)
            return False
        waypoint = self.get_nearest_waypoint(wx, wy)
        if waypoint is None:
            self._feedback("修改位置没有可用Driving Lane。", True)
            return False
        lane = self._lane_descriptor({"waypoint": waypoint})
        candidate_gate = self._make_gate(
            waypoint, "operator_topology_{}_seed".format(direction))
        owner = self._roundabout_seed_cross_section_owner(
            lane, candidate_gate, ignored_target=target)
        if owner is not None:
            self._feedback(
                "该横断面已被{} {}使用（门线距离{:.1f}m）。".format(
                    owner[0], owner[1].upper(), owner[2]), True)
            return False
        arm = self.roundabout_topology_seeds[arm_index]
        self._push_roundabout_history(
            "修改{} {}标记".format(
                arm["arm_id"], "IN" if direction == "inbound" else "OUT"))
        arm[direction] = {
            "gate": candidate_gate,
            "lane": lane,
        }
        review, errors = self._build_roundabout_topology_review()
        self.roundabout_topology_review = review
        self.roundabout_topology_review_errors = errors
        self.roundabout_topology_review_ready = not errors
        self.roundabout_topology_confirmed = False
        self.roundabout_topology_cache_loaded = False
        self.roundabout_topology_edit_target = None
        self.roundabout_topology_lane_target = None
        self.roundabout_topology_preview = None
        self.roundabout_topology_preview_errors = []
        self.roundabout_vut_recommendation = None
        self.roundabout_vut_recommendation_errors = []
        self._mark_roundabout_dirty()
        if errors:
            self._feedback(
                "{} {}已修改，但程序审核发现：{}。请继续修改或按P复核。".format(
                    arm["arm_id"], direction.upper(), "；".join(errors[:2])), True)
            return False
        self._feedback(
            "{} {}已修改并重新审核通过；请检查高亮，全部正确后按Enter确认并覆盖缓存。".format(
                arm["arm_id"], direction.upper()))
        return True

    def clear_roundabout_topology_seeds(self):
        if not getattr(self, "roundabout_topology_seeds", []):
            self._feedback("当前没有拓扑标定可清除。", True)
            return False
        self._push_roundabout_history("清除STEP 1拓扑标定")
        self.roundabout_topology_seeds = []
        self.roundabout_topology_mark_kind = "inbound"
        self.roundabout_topology_edit_target = None
        self.roundabout_topology_lane_target = None
        self.roundabout_topology_confirmed = False
        self.roundabout_topology_review_ready = False
        self.roundabout_topology_review = None
        self.roundabout_topology_review_errors = []
        self.roundabout_vut_recommendation = None
        self.roundabout_vut_recommendation_errors = []
        self.roundabout_last_trace_failure = None
        self.roundabout_topology_preview = None
        self.roundabout_topology_preview_errors = []
        self._mark_roundabout_dirty()
        self._feedback("拓扑标定已清除；请从第1个arm进口重新开始右键标记。")
        return True

    def _draw_roundabout_topology_seeds(self):
        review = getattr(self, "roundabout_topology_review", None)
        show_expanded_review = bool(
            review
            and (getattr(self, "roundabout_topology_review_ready", False)
                 or review.get("diagnostics"))
            and (not getattr(self, "roundabout_topology_confirmed", False)
                 or getattr(self, "roundabout_show_all_overlays", False)))
        if show_expanded_review:
            for arm in review["arms"]:
                short_id = arm["arm_id"].replace("arm_", "A")
                for item in arm["inbound"]:
                    self._draw_roundabout_topology_marker(
                        item["gate"], ROUNDABOUT_COLOR_IN, "{} IN".format(short_id))
                for item in arm["outbound"]:
                    self._draw_roundabout_topology_marker(
                        item["gate"], ROUNDABOUT_COLOR_OUT, "{} OUT".format(short_id))
            target = getattr(self, "roundabout_topology_edit_target", None)
            lane_target = getattr(self, "roundabout_topology_lane_target", None)
            if lane_target is not None and lane_target.get("gate"):
                target_label = ("已选/右键修改" if lane_target.get("is_operator_seed")
                                else "已选/Delete排除")
                self._draw_roundabout_topology_marker(
                    lane_target["gate"], (255, 230, 60), target_label,
                    pending=True)
            elif target is not None:
                arm_index, direction = target
                if 0 <= arm_index < len(self.roundabout_topology_seeds):
                    seed = self.roundabout_topology_seeds[arm_index].get(direction)
                    if seed:
                        self._draw_roundabout_topology_marker(
                            seed["gate"], (255, 230, 60), "已选/右键修改", pending=True)
            self._draw_roundabout_topology_diagnostics()
            return
        for arm in getattr(self, "roundabout_topology_seeds", []):
            if arm.get("inbound"):
                self._draw_roundabout_topology_marker(
                    arm["inbound"]["gate"], ROUNDABOUT_COLOR_IN,
                    "{} IN".format(arm["arm_id"].replace("arm_", "A")))
            if arm.get("outbound"):
                self._draw_roundabout_topology_marker(
                    arm["outbound"]["gate"], ROUNDABOUT_COLOR_OUT,
                    "{} OUT".format(arm["arm_id"].replace("arm_", "A")))
        self._draw_roundabout_topology_diagnostics()

    def _draw_roundabout_topology_diagnostics(self):
        """Highlight the exact cross-sections responsible for STEP 1 errors."""
        review = getattr(self, "roundabout_topology_review", None) or {}
        for diagnostic in review.get("diagnostics", []):
            if diagnostic.get("kind") != "lane_cross_section_conflict":
                continue
            owners = diagnostic.get("owners", [])
            owner_names = [
                "{} {}".format(
                    owner.get("arm_id", "arm_??").replace("arm_", "A"),
                    "IN" if owner.get("direction") == "inbound" else "OUT")
                for owner in owners
            ]
            label = "冲突 {}".format(" / ".join(owner_names))
            drawn_centers = []
            for owner in owners:
                gate = owner.get("gate")
                if not gate:
                    continue
                center = gate.get("center", {})
                center_xy = (float(center.get("x", 0.0)),
                             float(center.get("y", 0.0)))
                if any(math.hypot(
                        center_xy[0] - previous[0],
                        center_xy[1] - previous[1]) < 0.5
                       for previous in drawn_centers):
                    continue
                drawn_centers.append(center_xy)
                self._draw_roundabout_topology_marker(
                    gate, ROUNDABOUT_COLOR_TOPOLOGY_CONFLICT, label,
                    pending=True)

    def _roundabout_map_labels_visible(self):
        """Show STEP 1 identities; declutter only after topology acceptance."""
        if not getattr(self, "is_roundabout_2b", False):
            return True
        if not getattr(self, "roundabout_topology_confirmed", False):
            return True
        return bool(getattr(self, "roundabout_help_expanded", False))

    def _draw_roundabout_topology_marker(
            self, gate, color, label, pending=False):
        """Draw a transverse position gate plus a longitudinal travel arrow."""
        if not gate:
            return
        center = self.world_to_screen(gate["center"]["x"], gate["center"]["y"])
        # Every component of this marker is screen-space bounded (arrow,
        # 48 px gate, and a right-side label).  A centre farther away than
        # this margin cannot contribute visible pixels, so never send its
        # large zoomed coordinates to pygame/SDL.
        if not self._screen_point_visible(center, margin=220):
            return
        left, right = self._fixed_screen_gate_segment(gate, center)
        outline = (255, 255, 255) if pending else (15, 15, 15)
        self._draw_clipped_line(outline, left, right, 12 if pending else 10)
        self._draw_clipped_line(color, left, right, 7 if pending else 6)
        yaw_degrees = float(gate.get("approach_yaw", 0.0))
        yaw = math.radians(yaw_degrees)
        arrow_origin = (
            center[0] - math.cos(yaw) * 32.0,
            center[1] + math.sin(yaw) * 32.0,
        )
        self._draw_direction_arrow(
            outline, arrow_origin, yaw_degrees, width=8,
            length_px=64.0, head_length_px=14.0)
        self._draw_direction_arrow(
            color, arrow_origin, yaw_degrees, width=4,
            length_px=64.0, head_length_px=14.0)
        if not self._screen_point_visible(center, margin=0):
            return
        pygame.draw.circle(self.screen, outline, center, 8)
        pygame.draw.circle(self.screen, color, center, 5)
        if not self._roundabout_map_labels_visible():
            return
        # Keep map labels compact.  Direction semantics live in the persistent
        # panel legend, so text never needs to sit across the arrow shaft.
        suffix = " · 右键" if pending else ""
        text_surface = self.font.render(label + suffix, True, color)
        # Keep the label in a fixed screen position: to the right of the
        # arrow centre.  A direction-relative offset can cross the shaft when
        # the lane points diagonally or vertically.
        screen_width, screen_height = self.screen.get_size()
        text_position = (
            int(min(
                max(8, center[0] + 80),
                max(8, screen_width - text_surface.get_width() - 8))),
            int(min(
                max(8, center[1] - text_surface.get_height() * 0.5),
                max(8, screen_height - text_surface.get_height() - 8))),
        )
        background = text_surface.get_rect(topleft=text_position).inflate(8, 6)
        pygame.draw.rect(self.screen, (0, 0, 0), background)
        pygame.draw.rect(self.screen, outline, background, 1)
        self.screen.blit(text_surface, text_position)

    def _draw_roundabout_topology_hover(self):
        """Preview the exact lane gate that the next STEP 1 right-click selects."""
        if (not getattr(self, "roundabout_branch_confirmed", False)
                or getattr(self, "roundabout_topology_confirmed", False)
                or getattr(self, "roundabout_topology_review_ready", False)):
            return
        try:
            mouse_x, mouse_y = pygame.mouse.get_pos()
        except pygame.error:
            return
        # Do not snap a hidden candidate while the pointer is over the panel.
        if mouse_y < 125:
            return
        world_x, world_y = self.screen_to_world(mouse_x, mouse_y)
        waypoint = self.get_nearest_waypoint(world_x, world_y)
        if waypoint is None:
            return
        seeds = getattr(self, "roundabout_topology_seeds", [])
        incomplete = bool(seeds and not seeds[-1].get("outbound"))
        arm_number = len(seeds) if incomplete else len(seeds) + 1
        kind = "OUT" if incomplete else "IN"
        color = ROUNDABOUT_COLOR_OUT if incomplete else ROUNDABOUT_COLOR_IN
        gate = self._make_gate(waypoint, "operator_topology_hover")
        self._draw_roundabout_topology_marker(
            gate, color, "NEXT A{:02d} {}".format(arm_number, kind), pending=True)

    def _draw_roundabout_vut_recommendation(self):
        """Show the preliminary entry and permitted exits after VUT placement."""
        proposal = getattr(self, "roundabout_vut_recommendation", None)
        if not proposal:
            return
        entry_label = "建议 VUT汇入 {}".format(proposal["entry_arm_id"])
        for gate in proposal["entry_gates"]:
            self._draw_roundabout_topology_marker(
                gate, ROUNDABOUT_COLOR_VUT, entry_label)
        exit_colors = {
            "1": ROUNDABOUT_COLOR_EXIT_1,
            "2": ROUNDABOUT_COLOR_EXIT_2,
            "3": ROUNDABOUT_COLOR_EXIT_3,
        }
        for number in ("1", "2", "3"):
            exit_info = proposal["exit_gates"].get(number)
            if not exit_info:
                continue
            usage = "VT1" if number == "1" else "VUT候选"
            label = "建议 EXIT {} / {} / {}".format(
                number, usage, exit_info["arm_id"])
            for gate in exit_info["gates"]:
                self._draw_roundabout_topology_marker(
                    gate, exit_colors[number], label)

    def _draw_roundabout_placement_recommendations(self):
        """Recolor topology-proven VT1/VT2 placement lane segments."""
        recommendations = self._ensure_roundabout_placement_recommendations()
        if not recommendations:
            return
        for role, color in (
                ("vt1", ROUNDABOUT_COLOR_VT1_RECOMMENDED),
                ("vt2", ROUNDABOUT_COLOR_VT2_RECOMMENDED)):
            screen_bins = set()
            for waypoint, _distance_m in recommendations.get(role, []):
                location = waypoint.transform.location
                point = self.world_to_screen(location.x, location.y)
                if not self._screen_point_visible(point, margin=16):
                    continue
                screen_bin = (int(point[0] // 12), int(point[1] // 12))
                if screen_bin in screen_bins:
                    continue
                screen_bins.add(screen_bin)
                pygame.draw.circle(self.screen, color, point, 4)
                self._draw_direction_arrow(
                    color, point, waypoint.transform.rotation.yaw,
                    width=3, length_px=12, head_length_px=5)

    def _draw_roundabout_trace_failure(self):
        """Highlight the seed and closest traced point from the last failure."""
        failure = getattr(self, "roundabout_last_trace_failure", None)
        if not failure:
            return
        short_id = failure["arm_id"].replace("arm_", "A")
        direction = "IN" if failure["lane_direction"] == "inbound" else "OUT"
        self._draw_roundabout_topology_marker(
            failure["seed_gate"], (255, 65, 65),
            failure.get(
                "start_label", "失败起点 {} {}".format(short_id, direction)))
        if failure.get("closest_gate"):
            self._draw_roundabout_topology_marker(
                failure["closest_gate"], (255, 80, 230),
                failure.get(
                    "closest_label", "追踪最接近点 / {}".format(failure["reason"])))
        if failure.get("target_gate"):
            self._draw_roundabout_topology_marker(
                failure["target_gate"], (255, 190, 60),
                failure.get("target_label", "最近arm目标"))

    def _reviewed_roundabout_topology(self):
        """Build directed arm records from the operator-reviewed STEP 1 gates.

        The result deliberately contains no fitted circle.  A roundabout is a
        cyclic lane topology; a VUT route is only one open path through it.
        Gate angles are used solely to order the already reviewed physical
        arms around their centroid, never to decide whether a lane is part of
        the roundabout.
        """
        review = getattr(self, "roundabout_topology_review", None)
        if not review or not getattr(self, "roundabout_topology_confirmed", False):
            raise RuntimeError("STEP 1物理arm尚未完成程序审核和用户确认")
        gate_items = [
            item for arm in review.get("arms", [])
            for direction in ("inbound", "outbound")
            for item in arm.get(direction, [])
            if isinstance(item, dict) and isinstance(item.get("gate"), dict)
        ]
        if len(gate_items) < 6:
            raise RuntimeError("STEP 1审核结果不足3个完整IN/OUT物理arm")
        center = (
            sum(float(item["gate"]["center"]["x"]) for item in gate_items)
            / len(gate_items),
            sum(float(item["gate"]["center"]["y"]) for item in gate_items)
            / len(gate_items),
        )
        arms = []
        for review_arm in review.get("arms", []):
            records = {"inbound": [], "outbound": []}
            for direction in ("inbound", "outbound"):
                for item in review_arm.get(direction, []):
                    waypoint = self._topology_seed_waypoint(item)
                    if waypoint is None:
                        raise RuntimeError(
                            "{}的{}审核门线无法重新吸附到Driving Lane".format(
                                review_arm.get("arm_id", "unknown"),
                                "进口" if direction == "inbound" else "出口"))
                    gate = copy.deepcopy(item["gate"])
                    location = gate["center"]
                    reviewed_lane = item.get("lane", {})
                    reviewed_lane_key = (
                        int(reviewed_lane["road_id"]),
                        int(reviewed_lane["section_id"]),
                        int(reviewed_lane["lane_id"]),
                    ) if all(key in reviewed_lane for key in (
                        "road_id", "section_id", "lane_id")) else self._lane_key(waypoint)
                    records[direction].append({
                        "direction": direction,
                        "lane_key": reviewed_lane_key,
                        "waypoint": waypoint,
                        "gate": gate,
                        "angle": math.atan2(
                            float(location["y"]) - center[1],
                            float(location["x"]) - center[0]),
                    })
            if not records["inbound"] or not records["outbound"]:
                raise RuntimeError(
                    "{}没有完整的进口和出口车道".format(
                        review_arm.get("arm_id", "unknown")))

            def mean_angle(values):
                return math.atan2(
                    sum(math.sin(value["angle"]) for value in values),
                    sum(math.cos(value["angle"]) for value in values))

            inbound_angle = mean_angle(records["inbound"])
            outbound_angle = mean_angle(records["outbound"])
            arms.append({
                "arm_id": str(review_arm["arm_id"]),
                "angle": math.atan2(
                    math.sin(inbound_angle) + math.sin(outbound_angle),
                    math.cos(inbound_angle) + math.cos(outbound_angle)),
                "inbound_angle": inbound_angle,
                "outbound_angle": outbound_angle,
                "inbound": records["inbound"],
                "outbound": records["outbound"],
                "pairing_method": "operator_reviewed_lane_groups",
            })
        direction_score = sum(
            math.sin(arm["outbound_angle"] - arm["inbound_angle"])
            for arm in arms)
        if abs(direction_score) < 1e-3:
            raise RuntimeError(
                "STEP 1的IN/OUT门线无法稳定确定环流方向；请检查是否有arm标反")
        direction = -1 if direction_score > 0.0 else 1
        return arms, center, direction

    @staticmethod
    def _route_arm_match_indices(route, arm, direction):
        """Return route indexes that occupy one reviewed arm lane group."""
        target_keys = {record["lane_key"] for record in arm[direction]}
        return [
            index for index, waypoint in enumerate(route)
            if CarlaMapEditor0916._lane_key(waypoint) in target_keys
        ]

    def _route_entry_event(self, route, arms):
        """Resolve VUT entry from a finite IN-gate crossing.

        Custom OpenDRIVE approaches frequently change road/lane identity at
        the connector.  STEP 2 already proves the first reachable physical
        arm from the VUT start, so that proof may replace strict lane-key
        equality; finite gate geometry and heading are still mandatory.
        """
        payload = [self._waypoint_payload(waypoint) for waypoint in route]
        recommendation = getattr(self, "roundabout_vut_recommendation", None)
        recommended_arm_id = (
            recommendation.get("entry_arm_id")
            if isinstance(recommendation, dict) else None)
        candidates = []
        rejected = []
        for arm in arms:
            if recommended_arm_id and arm["arm_id"] != recommended_arm_id:
                continue
            for record in arm["inbound"]:
                for index in range(1, len(route)):
                    accepted, evidence = self._route_gate_crossing_evidence(
                        route, payload, index, record,
                        require_lane=not bool(recommended_arm_id))
                    if accepted:
                        candidates.append((index, arm, record))
                    elif evidence.get("geometric_crossing"):
                        rejected.append((index, arm, evidence))
        if not candidates:
            lane = self._lane_key(route[0]) if route else "unknown"
            detail = ""
            if rejected:
                index, arm, evidence = min(rejected, key=lambda item: item[0])
                detail = (
                    "；稠密点{}几何上接触{} IN，但方向一致度为{:.3f}，"
                    "交点附近lane={}、审核lane={}".format(
                        index, arm["arm_id"], evidence.get("alignment", -1.0),
                        evidence.get("route_lane_keys"),
                        evidence.get("gate_lane_key")))
            elif recommended_arm_id:
                detail = "；STEP 2已关联{}，但路线未穿过其有限IN横门线".format(
                    recommended_arm_id)
            raise RuntimeError(
                "VUT稠密路线未形成有效进口门线穿越；起点lane={}{}。"
                "请让路线沿推荐进口车道穿过绿色IN横门线".format(
                    lane, detail))
        first_index = min(item[0] for item in candidates)
        earliest = [item for item in candidates if item[0] == first_index]
        if len({item[1]["arm_id"] for item in earliest}) != 1:
            raise RuntimeError("VUT在同一稠密路点穿过多个物理arm的IN门线，请检查STEP 1标记")
        return min(earliest, key=lambda item: item[1]["arm_id"])

    def _route_entry_arm(self, route, arms):
        """Compatibility wrapper returning only the resolved physical arm."""
        return self._route_entry_event(route, arms)[1]

    def _route_exit_arm(self, route, arms, after_index=0, route_label="VUT"):
        """Resolve the physical exit by a forward crossing of a reviewed OUT gate.

        Lane IDs alone are insufficient on custom junctions: a circulating
        connector may briefly reuse an access-road identity before the route
        actually leaves the roundabout.  A route therefore counts as exiting
        only after its geometry crosses the finite, directed STEP 1 gate.
        """
        payload = [self._waypoint_payload(waypoint) for waypoint in route]
        candidates_by_arm = {}
        rejected_geometric_crossings = []
        for arm in arms:
            for record in arm["outbound"]:
                crossing_indexes = []
                for index in range(max(1, after_index + 1), len(payload)):
                    accepted, evidence = self._route_gate_crossing_evidence(
                        route, payload, index, record)
                    if accepted:
                        crossing_indexes.append(index)
                    elif evidence.get("geometric_crossing"):
                        rejected_geometric_crossings.append({
                            "arm_id": arm["arm_id"],
                            "index": index,
                            "evidence": evidence,
                        })
                if crossing_indexes:
                    last = max(crossing_indexes)
                    previous = candidates_by_arm.get(arm["arm_id"])
                    if previous is None or last > previous[0]:
                        candidates_by_arm[arm["arm_id"]] = (
                            last, arm, record)
        candidates = list(candidates_by_arm.values())
        if not candidates:
            lane = self._lane_key(route[-1]) if route else "unknown"
            endpoint = route[-1].transform.location if route else None
            gate_records = [
                (arm, record) for arm in arms for record in arm["outbound"]
            ]
            nearest_text = "无可用OUT门线"
            if endpoint is not None and gate_records:
                nearest_arm, nearest_record = min(
                    gate_records,
                    key=lambda item: self._distance_locations(
                        endpoint, item[1]["waypoint"].transform.location))
                nearest_distance = self._distance_locations(
                    endpoint, nearest_record["waypoint"].transform.location)
                nearest_text = "最近{} OUT门线直线距离{:.1f}m".format(
                    nearest_arm["arm_id"], nearest_distance)
            lane_matches = sorted({
                arm["arm_id"] for arm in arms
                if any(index > after_index for index in
                       self._route_arm_match_indices(route, arm, "outbound"))
            })
            reused_text = (
                "；路线曾出现OUT lane identity={}，但未穿越相应门线，"
                "不能据此判定已驶出".format(",".join(lane_matches))
                if lane_matches else "")
            rejected_text = ""
            if rejected_geometric_crossings:
                nearest_rejected = min(
                    rejected_geometric_crossings,
                    key=lambda item: item["evidence"].get(
                        "intersection_offset_m", float("inf")))
                evidence = nearest_rejected["evidence"]
                reasons = []
                if not evidence.get("lane_match", True):
                    reasons.append(
                        "交点附近路线lane={}，目标OUT lane={}，且在{:.0f}m内"
                        "未证明二者沿车道拓扑连通".format(
                            evidence.get("route_lane_keys"),
                            evidence.get("gate_lane_key"),
                            ROUNDABOUT_EXIT_GATE_TOPOLOGY_MATCH_M))
                if not evidence.get("physical_gate_match", False):
                    reasons.append(
                        "路线终点位于门线下游{:.1f}m（需≥{:.1f}m才确认已驶出）".format(
                            evidence.get("downstream_signed_m", float("nan")),
                            ROUNDABOUT_EXIT_GATE_DOWNSTREAM_CLEARANCE_M))
                if not evidence.get("heading_match", True):
                    reasons.append(
                        "路线方向与OUT箭头一致度仅{:.3f}（要求≥{:.3f}）".format(
                            evidence.get("alignment", -1.0),
                            ROUNDABOUT_EXIT_GATE_MIN_ALIGNMENT))
                rejected_text = (
                    "；路线在稠密点{}附近几何上碰到{} OUT门线，但不计为驶出：{}".format(
                        nearest_rejected["index"], nearest_rejected["arm_id"],
                        "；".join(reasons) or "缺少有效出口车道证据"))
            raise RuntimeError(
                "{}路线尚未沿箭头方向穿过任何有效的STEP 1审核OUT门线；"
                "终点lane={}，{}{}{}。请沿目标出口车道继续画到目标OUT横门线下游；"
                "候选出口阶段该标记可能显示为黄色、橙色或红色，而不是蓝色".format(
                    route_label, lane, nearest_text, reused_text, rejected_text))
        # A continuous custom-map road can pass more than one reviewed OUT
        # cross-section.  The route endpoint expresses the operator's intent,
        # so use the final forward crossing instead of treating an earlier
        # auxiliary crossing as a completed exit or a forbidden re-entry.
        final_index = max(item[0] for item in candidates)
        final_candidates = [item for item in candidates if item[0] == final_index]
        route_location = route[final_index].transform.location
        _index, exit_arm, _record = min(
            final_candidates,
            key=lambda item: (
                math.hypot(
                    float(route_location.x)
                    - float(item[2]["gate"]["center"]["x"]),
                    float(route_location.y)
                    - float(item[2]["gate"]["center"]["y"])),
                item[1]["arm_id"]))
        return exit_arm, final_index

    def _route_gate_crossing_evidence(
            self, route, payload, index, record, require_lane=True):
        """Require finite geometry, heading and audited lane continuity.

        Custom OpenDRIVE junction connectors commonly change ``road_id`` at
        the physical gate. Exact lane identity is preferred, but a bounded
        directed CARLA-topology proof is equally valid. Geometry alone is
        still insufficient, so a nearby unrelated road cannot count as an
        exit merely because it crosses the same screen position.
        """
        gate = record["gate"]
        geometric_crossing = gate_crossing(
            payload[index - 1], payload[index], gate,
            direction="negative_to_positive",
            margin_m=ROUNDABOUT_EXIT_GATE_MARGIN_M)
        evidence = {
            "geometric_crossing": geometric_crossing,
            "lane_match": False,
            "exact_lane_match": False,
            "topology_lane_match": False,
            "heading_match": False,
        }
        if not geometric_crossing:
            return False, evidence

        previous_location = route[index - 1].transform.location
        current_location = route[index].transform.location
        dx = float(current_location.x) - float(previous_location.x)
        dy = float(current_location.y) - float(previous_location.y)
        segment_length = math.hypot(dx, dy)
        forward = gate.get("forward", {})
        fx = float(forward.get("x", 0.0))
        fy = float(forward.get("y", 0.0))
        forward_length = math.hypot(fx, fy)
        alignment = (
            (dx * fx + dy * fy) / (segment_length * forward_length)
            if segment_length > 1e-6 and forward_length > 1e-6 else -1.0)
        evidence["alignment"] = alignment
        evidence["heading_match"] = (
            alignment >= ROUNDABOUT_EXIT_GATE_MIN_ALIGNMENT)

        gate_lane_key = record.get("lane_key")
        if gate_lane_key is None and all(
                key in gate for key in ("road_id", "section_id", "lane_id")):
            gate_lane_key = (
                int(gate["road_id"]), int(gate["section_id"]),
                int(gate["lane_id"]))
        first = max(0, index - ROUNDABOUT_EXIT_GATE_LANE_WINDOW)
        last = min(len(route), index + ROUNDABOUT_EXIT_GATE_LANE_WINDOW + 1)
        route_lane_keys = {
            CarlaMapEditor0916._lane_key(waypoint)
            for waypoint in route[first:last]
        }
        evidence["gate_lane_key"] = gate_lane_key
        evidence["route_lane_keys"] = sorted(route_lane_keys)
        # Legacy/test gates without lane metadata can still use the geometric
        # and heading checks.  Real STEP 1 records always carry lane_key.
        evidence["exact_lane_match"] = (
            gate_lane_key is None or gate_lane_key in route_lane_keys)
        topology_proof = None
        if (not evidence["exact_lane_match"]
                and gate_lane_key is not None
                and record.get("waypoint") is not None):
            # OpenDRIVE can change connector identity immediately before or
            # after the reviewed cross-section. Search both directed sides
            # of a small route window, but require exact topology membership
            # at the target; spatial proximity alone is not accepted.
            for route_index in range(first, last):
                waypoint = route[route_index]
                for trace_direction in ("forward", "backward"):
                    try:
                        trace = self._trace_waypoint_to_records(
                            waypoint, [record],
                            maximum_distance=
                            ROUNDABOUT_EXIT_GATE_TOPOLOGY_MATCH_M,
                            trace_direction=trace_direction,
                            allow_spatial_match=False)
                    except (AttributeError, RuntimeError, TypeError, ValueError):
                        continue
                    if trace.get("reached"):
                        topology_proof = {
                            "route_index": route_index,
                            "direction": trace_direction,
                            "distance_m": float(
                                (trace.get("best") or {}).get(
                                    "distance_m", 0.0)),
                            "visited_count": int(trace.get("visited_count", 0)),
                        }
                        break
                if topology_proof:
                    break
        evidence["topology_lane_match"] = topology_proof is not None
        evidence["topology_proof"] = topology_proof
        evidence["lane_match"] = bool(
            evidence["exact_lane_match"] or evidence["topology_lane_match"])

        # The operator-reviewed finite gate is the physical authority.  A
        # custom OpenDRIVE map may change road_id at the cross-section or may
        # omit a successor link even though the rendered Driving Lane is
        # continuous.  Once a continuous route crosses the finite gate in its
        # arrow direction and finishes clearly downstream, the crossing is
        # valid; lane identities remain useful diagnostics, not a veto.
        downstream_signed_m = gate_signed_distance(payload[-1], gate)
        evidence["downstream_signed_m"] = float(downstream_signed_m)
        evidence["physical_gate_match"] = bool(
            downstream_signed_m
            >= ROUNDABOUT_EXIT_GATE_DOWNSTREAM_CLEARANCE_M)

        center = gate.get("center", {})
        evidence["intersection_offset_m"] = math.hypot(
            float(current_location.x) - float(center.get("x", current_location.x)),
            float(current_location.y) - float(center.get("y", current_location.y)))
        accepted_lane = (
            evidence["lane_match"]
            or evidence["physical_gate_match"]
            or not require_lane)
        return bool(accepted_lane and evidence["heading_match"]), evidence

    def _route_touches_directed_out_gate(self, route, arms):
        """Whether a draft reached an OUT cross-section in its arrow direction."""
        payload = [self._waypoint_payload(waypoint) for waypoint in route]
        for arm in arms:
            for record in arm.get("outbound", []):
                for index in range(1, len(route)):
                    _accepted, evidence = self._route_gate_crossing_evidence(
                        route, payload, index, record)
                    if (evidence.get("geometric_crossing")
                            and evidence.get("heading_match")):
                        return True
        return False

    def _route_gate_index(self, route, gate, route_label):
        payload = [self._waypoint_payload(waypoint) for waypoint in route]
        for index in range(1, len(payload)):
            if gate_crossing(
                    payload[index - 1], payload[index], gate,
                    direction="negative_to_positive", margin_m=1.0):
                return index
        raise RuntimeError(
            "{}没有沿正方向穿过STEP 1审核门线；请确认起点在门线上游，"
            "并把路线画到门线下游".format(route_label))

    @staticmethod
    def _route_record_for_lane(route, records, prefer_last=False):
        matches = []
        for record in records:
            indexes = [
                index for index, waypoint in enumerate(route)
                if CarlaMapEditor0916._lane_key(waypoint) == record["lane_key"]
            ]
            if indexes:
                matches.append((max(indexes) if prefer_last else min(indexes), record))
        if not matches:
            return records[0]
        selector = max if prefer_last else min
        return selector(matches, key=lambda item: item[0])[1]

    def _topology_preview_from_vut_route(self, vut_dense):
        arms, ordering_center, direction = self._reviewed_roundabout_topology()
        entry_index, entry_arm, entry_record = self._route_entry_event(
            vut_dense, arms)
        exit_order, entry_order = self._roundabout_arm_orders(
            arms, entry_arm, direction)
        if len(exit_order) < 3:
            raise RuntimeError("从VUT汇入口沿环流方向无法枚举至少3个出口")
        if not entry_order:
            raise RuntimeError("无法确定沿环流方向的下游第1入口")
        entry_gate = copy.deepcopy(entry_record["gate"])
        entry_gate["source"] = "operator_reviewed_vut_entry"
        exit_arm, exit_index = self._route_exit_arm(
            vut_dense, arms, after_index=entry_index, route_label="VUT")
        planned_exit = exit_order.index(exit_arm) + 1 \
            if exit_arm in exit_order else None
        if planned_exit not in (2, 3):
            raise RuntimeError(
                "VUT实际驶出{}，按STEP 1物理arm顺序属于出口{}；"
                "国标要求从出口2或出口3驶出".format(
                    exit_arm["arm_id"], planned_exit or "unknown"))
        exit_number_by_arm = {
            arm["arm_id"]: number
            for number, arm in enumerate(exit_order, start=1)
        }
        arm_gates = []
        for arm in arms:
            arm_gates.append({
                "arm_id": arm["arm_id"],
                "inbound_gate": copy.deepcopy(arm["inbound"][0]["gate"]),
                "outbound_gate": copy.deepcopy(arm["outbound"][0]["gate"]),
                "exit_number": exit_number_by_arm[arm["arm_id"]],
                "inbound_lane_count": len(arm["inbound"]),
                "outbound_lane_count": len(arm["outbound"]),
                "is_vut_entry_arm": arm is entry_arm,
                "is_downstream_entry_arm": arm is entry_order[0],
            })
        return {
            "direction": direction,
            "ordering_center": ordering_center,
            "entry_gate": entry_gate,
            "entry_index": entry_index,
            "entry_arm": entry_arm,
            "entry_arm_id": entry_arm["arm_id"],
            "exit_arm": exit_arm,
            "exit_index": exit_index,
            "planned_exit": planned_exit,
            "exit_order": exit_order,
            "entry_order": entry_order,
            "downstream_entry_arm_id": entry_order[0]["arm_id"],
            "arms": arms,
            "arm_gates": arm_gates,
            "arm_count": len(arms),
            "exit_arm_ids": {
                str(number): arm["arm_id"]
                for number, arm in enumerate(exit_order[:3], start=1)
            },
        }

    def _analyse_roundabout_topology_preview(self):
        """Build formal topology from reviewed arms plus the open VUT path."""
        if self.ego_point is None:
            return None, ["STEP 1拓扑确认需要先在STEP 2放置VUT。"]
        if not getattr(self, "roundabout_vut_route_confirmed", False):
            return None, ["STEP 1拓扑确认需要先完成STEP 3并确认VUT路线。"]
        try:
            cache = self._confirmed_roundabout_route("vut")
            vut_dense = cache["dense"]
            return self._topology_preview_from_vut_route(vut_dense), []
        except RuntimeError as exc:
            return None, ["拓扑预览无法生成：{}".format(exc)]

    def _ensure_roundabout_topology_preview(self, force=False):
        if not self.is_roundabout_2b:
            return None
        if (not force and self.roundabout_topology_preview is None
                and self.roundabout_topology_preview_errors):
            return None
        if force or self.roundabout_topology_preview is None:
            preview, errors = self._analyse_roundabout_topology_preview()
            self.roundabout_topology_preview = preview
            self.roundabout_topology_preview_errors = errors
        return self.roundabout_topology_preview

    def _reverse_waypoint_distance_band(
            self, starts, minimum_m, maximum_m, excluded_route=None,
            blocked_lane_keys=None):
        """Collect directed upstream lane points inside a distance band."""
        excluded_route = list(excluded_route or [])
        blocked_lane_keys = {
            tuple(item) for item in (blocked_lane_keys or [])
        }

        def belongs_to_excluded_route(waypoint):
            location = waypoint.transform.location
            lane_key = self._lane_key(waypoint)
            return any(
                self._lane_key(route_waypoint) == lane_key
                and self._distance_locations(
                    location, route_waypoint.transform.location) <= 3.0
                for route_waypoint in excluded_route)

        frontier = []
        counter = itertools.count()
        for item in starts:
            if isinstance(item, tuple):
                waypoint, distance_m = item
            else:
                waypoint, distance_m = item, 0.0
            heapq.heappush(
                frontier, (float(distance_m), next(counter), waypoint))
        best_distance = {}
        selected = []
        while frontier and len(best_distance) < 5000:
            distance_m, _order, waypoint = heapq.heappop(frontier)
            if distance_m > maximum_m:
                continue
            token = self._waypoint_token(waypoint)
            if distance_m >= best_distance.get(token, float("inf")):
                continue
            best_distance[token] = distance_m
            # At every roundabout arm, reverse topology exposes both the
            # circulating continuation and the public-road approach.  VT1 is
            # already circulating, so never enter a STEP 1-reviewed inbound
            # lane while expanding its recommendation band.
            if self._lane_key(waypoint) in blocked_lane_keys:
                continue
            if belongs_to_excluded_route(waypoint):
                continue
            if minimum_m <= distance_m <= maximum_m:
                selected.append((waypoint, distance_m))
            try:
                previous_waypoints = waypoint.previous(
                    ROUNDABOUT_RECOMMENDATION_TRACE_STEP_M)
            except (AttributeError, RuntimeError, TypeError, ValueError):
                previous_waypoints = []
            for previous in previous_waypoints:
                step_m = self._distance_locations(
                    waypoint.transform.location,
                    previous.transform.location)
                if step_m <= 0.05:
                    step_m = ROUNDABOUT_RECOMMENDATION_TRACE_STEP_M
                heapq.heappush(
                    frontier,
                    (distance_m + step_m, next(counter), previous))
        return selected

    def _vut_circulation_predecessors(
            self, vut_dense, entry_index, exit_index, entry_gate=None):
        """Find a circulating predecessor without assuming a 2 m merge.

        ``Waypoint.previous(d)`` reports paths exactly ``d`` metres upstream.
        On custom OpenDRIVE maps, the two incoming paths often remain a single
        connector for several metres after the visible merge.  Asking only at
        2 m therefore hides the circulating predecessor.  Probe several
        bounded look-back horizons and compare them with the VUT's actual
        approach prefix.
        """
        upper = min(len(vut_dense) - 1, max(entry_index + 1, exit_index))
        for index in range(max(1, entry_index + 1), upper + 1):
            waypoint = vut_dense[index]
            route_prefix = vut_dense[:index]
            for lookback_m in ROUNDABOUT_RECOMMENDATION_BRANCH_LOOKBACKS_M:
                try:
                    predecessors = list(waypoint.previous(lookback_m))
                except (AttributeError, RuntimeError, TypeError, ValueError):
                    continue
                unique = {}
                for predecessor in predecessors:
                    unique[self._waypoint_token(predecessor)] = predecessor
                predecessors = list(unique.values())
                if len(predecessors) < 2:
                    continue

                # Locate where the confirmed VUT route was approximately the
                # same path-distance upstream.  This is more reliable than
                # comparing every candidate with the immediately previous
                # one-metre sample when the branch is 10-40 m behind.
                travelled_m = 0.0
                expected = route_prefix[-1]
                for route_index in range(index - 1, 0, -1):
                    travelled_m += self._distance_locations(
                        vut_dense[route_index].transform.location,
                        vut_dense[route_index - 1].transform.location)
                    expected = vut_dense[route_index - 1]
                    if travelled_m >= lookback_m:
                        break
                route_predecessor = min(
                    predecessors,
                    key=lambda item: self._distance_locations(
                        item.transform.location,
                        expected.transform.location))
                alternatives = [
                    item for item in predecessors
                    if self._waypoint_token(item) != self._waypoint_token(
                        route_predecessor)
                    and self._distance_locations(
                        item.transform.location,
                        route_predecessor.transform.location) > 0.25
                ]
                if entry_gate and alternatives:
                    # The circulating predecessor is normally on the
                    # roundabout side of the reviewed IN gate.  Use this only
                    # to rank/filter when it yields evidence; do not make it a
                    # new hard requirement for unusually shaped approaches.
                    ring_side = []
                    for candidate in alternatives:
                        location = candidate.transform.location
                        if gate_signed_distance({
                                "x": float(location.x),
                                "y": float(location.y),
                                "z": float(location.z)}, entry_gate) >= -2.0:
                            ring_side.append(candidate)
                    if ring_side:
                        alternatives = ring_side
                if alternatives:
                    return index, alternatives, lookback_m
        raise RuntimeError(
            "CARLA拓扑在VUT入口后2/4/8/12/20/30/40m回溯范围内都没有"
            "暴露独立环流分支；这通常是OpenDRIVE合流连接缺失，而不是"
            "VUT锚点错误")

    def _vt1_exit1_conflict_seed(self, preview, vut_dense):
        """Trace exit 1 backwards and locate its physical VUT conflict.

        Some custom OpenDRIVE junctions model the VUT and circulating vehicle
        as separate, nearly overlapping connector lanes.  They never appear
        as two predecessors of one waypoint.  GB/T nevertheless tells us that
        VT1 must leave through exit 1, so its reviewed outbound lane is a
        stronger seed: walk that lane graph backwards and find the first
        close, direction-compatible encounter with the confirmed VUT path.
        """
        exit_order = preview.get("exit_order") or []
        if not exit_order or not exit_order[0].get("outbound"):
            raise RuntimeError("STEP 1没有可用于反向追踪的出口1 OUT车道")
        starts = [
            record.get("waypoint")
            for record in exit_order[0]["outbound"]
            if record.get("waypoint") is not None
        ]
        if not starts:
            raise RuntimeError("出口1 OUT门线无法重新吸附到CARLA Driving Lane")
        lower = max(0, int(preview.get("entry_index", 0)))
        upper = min(
            len(vut_dense) - 1,
            max(lower, int(preview.get("exit_index", len(vut_dense) - 1))))
        route_window = list(enumerate(vut_dense[lower:upper + 1], lower))
        if not route_window:
            raise RuntimeError("VUT入口至目标出口之间没有可比较的真实路线")

        frontier = []
        counter = itertools.count()
        for waypoint in starts:
            heapq.heappush(frontier, (0.0, next(counter), waypoint))
        best_distance = {}
        candidates = []
        while frontier and len(best_distance) < 5000:
            travelled_m, _order, waypoint = heapq.heappop(frontier)
            if travelled_m > VT1_RECOMMENDATION_EXIT1_REVERSE_MAX_M:
                continue
            token = self._waypoint_token(waypoint)
            if travelled_m >= best_distance.get(token, float("inf")):
                continue
            best_distance[token] = travelled_m
            location = waypoint.transform.location
            route_index, route_waypoint = min(
                route_window,
                key=lambda item: self._distance_locations(
                    location, item[1].transform.location))
            proximity_m = self._distance_locations(
                location, route_waypoint.transform.location)
            heading_error_deg = abs((
                float(waypoint.transform.rotation.yaw)
                - float(route_waypoint.transform.rotation.yaw)
                + 180.0) % 360.0 - 180.0)
            if (proximity_m <= VT1_RECOMMENDATION_CONFLICT_PROXIMITY_M
                    and heading_error_deg
                    <= VT1_RECOMMENDATION_CONFLICT_MAX_HEADING_DEG
                    and self._lane_key(waypoint)
                    != self._lane_key(route_waypoint)):
                candidates.append((
                    route_index, proximity_m, heading_error_deg,
                    travelled_m, waypoint))
            try:
                predecessors = waypoint.previous(
                    ROUNDABOUT_RECOMMENDATION_TRACE_STEP_M)
            except (AttributeError, RuntimeError, TypeError, ValueError):
                predecessors = []
            for predecessor in predecessors:
                step_m = self._distance_locations(
                    location, predecessor.transform.location)
                if step_m <= 0.05:
                    step_m = ROUNDABOUT_RECOMMENDATION_TRACE_STEP_M
                heapq.heappush(frontier, (
                    travelled_m + step_m, next(counter), predecessor))
        if not candidates:
            raise RuntimeError(
                "从出口1反向追踪{:.0f}m后，仍未找到与VUT入口后路线距离≤{:.1f}m、"
                "方向差≤{:.0f}°的独立车道；请检查出口1 OUT标定或OpenDRIVE连接"
                .format(
                    VT1_RECOMMENDATION_EXIT1_REVERSE_MAX_M,
                    VT1_RECOMMENDATION_CONFLICT_PROXIMITY_M,
                    VT1_RECOMMENDATION_CONFLICT_MAX_HEADING_DEG))
        # The first VUT encounter after its IN gate is the merge conflict;
        # proximity and heading only break ties between parallel exit lanes.
        route_index, proximity_m, heading_error_deg, _travelled_m, waypoint = \
            min(candidates, key=lambda item: (item[0], item[1], item[2]))
        return route_index, [waypoint], proximity_m

    def _build_roundabout_placement_recommendations(self, preview, vut_cache):
        """Derive VT1/VT2 placement bands from directed CARLA topology."""
        vut_dense = vut_cache["dense"]
        recommendations = {
            "vt1": [],
            "vt2": [],
            "vt1_required_m": None,
            "vt1_conflict_index": None,
        }
        errors = []

        try:
            recommendation_method = "vut_previous_branch"
            try:
                conflict_index, ring_predecessors, branch_lookback_m = \
                    self._vut_circulation_predecessors(
                        vut_dense, preview["entry_index"],
                        preview["exit_index"],
                        entry_gate=preview.get("entry_gate"))
            except RuntimeError:
                conflict_index, ring_predecessors, _proximity_m = \
                    self._vt1_exit1_conflict_seed(preview, vut_dense)
                branch_lookback_m = 0.0
                recommendation_method = "exit1_reverse_conflict_proximity"
            approach_distance_m = sum(
                self._distance_locations(
                    first.transform.location, second.transform.location)
                for first, second in zip(
                    vut_dense[:conflict_index],
                    vut_dense[1:conflict_index + 1]))
            required_m = required_vt1_initial_upstream_distance(
                approach_distance_m,
                target_speed_kmh=15.0,
                vut_approach_speed_mps=
                VUT_APPROACH_SPEED_ASSUMPTION_KMH / 3.6,
                stable_duration_s=VT1_STABLE_DURATION_S,
                setup_budget_s=VT1_SETUP_BUDGET_S,
                entry_upstream_margin_m=VT1_ENTRY_UPSTREAM_MINIMUM_M,
                timing_buffer_m=VT1_TIMING_BUFFER_M,
                minimum_distance_m=VT1_INITIAL_MINIMUM_M,
                conflict_headway_target_s=getattr(
                    self, "roundabout_conflict_headway_s",
                    VT1_CONFLICT_HEADWAY_TARGET_S))
            starts = [
                (waypoint, branch_lookback_m)
                for waypoint in ring_predecessors]
            inbound_lane_keys = {
                tuple(record["lane_key"])
                for arm in preview.get("arms", [])
                for record in arm.get("inbound", [])
                if record.get("lane_key") is not None
            }
            recommendations["vt1"] = self._reverse_waypoint_distance_band(
                starts, required_m,
                required_m + VT1_RECOMMENDED_SEGMENT_EXTRA_M,
                excluded_route=vut_dense[:conflict_index],
                blocked_lane_keys=inbound_lane_keys)
            recommendations["vt1_required_m"] = required_m
            recommendations["vt1_conflict_index"] = conflict_index
            recommendations["vt1_method"] = recommendation_method
            if not recommendations["vt1"]:
                errors.append(
                    "未在冲突点上游{:.1f}-{:.1f}m找到可达环流车道".format(
                        required_m,
                        required_m + VT1_RECOMMENDED_SEGMENT_EXTRA_M))
        except RuntimeError as exc:
            errors.append("VT1推荐不可用：{}".format(exc))

        downstream_arm = (
            preview["entry_order"][0]
            if preview.get("entry_order") else None)
        if downstream_arm and downstream_arm.get("inbound"):
            vt2_starts = [record["waypoint"]
                          for record in downstream_arm["inbound"]
                          if record.get("waypoint") is not None]
            recommendations["vt2"] = self._reverse_waypoint_distance_band(
                vt2_starts,
                VT2_RECOMMENDED_SEGMENT_MIN_M,
                VT2_RECOMMENDED_SEGMENT_MAX_M)
            if not recommendations["vt2"]:
                errors.append("VT2推荐不可用：下游第1入口门线上游没有可追踪车道")
        else:
            errors.append("VT2推荐不可用：缺少下游第1入口进口车道")
        return recommendations, errors

    def _ensure_roundabout_placement_recommendations(self):
        if (not self.is_roundabout_2b
                or not getattr(self, "roundabout_vut_route_confirmed", False)):
            return None
        try:
            vut_cache = self._confirmed_roundabout_route("vut")
            preview = self._ensure_roundabout_topology_preview()
        except RuntimeError:
            return None
        if preview is None:
            return None
        topology_fingerprint = roundabout_hash({
            "direction": preview.get("direction"),
            "entry_gate": preview.get("entry_gate"),
            "arm_gates": preview.get("arm_gates", []),
        })
        key = (
            vut_cache.get("route_fingerprint")
            or vut_cache.get("anchor_fingerprint"),
            topology_fingerprint,
            preview.get("entry_arm_id"),
            preview.get("downstream_entry_arm_id"),
            int(preview.get("entry_index", -1)),
            int(preview.get("exit_index", -1)),
        )
        if key != getattr(
                self, "roundabout_placement_recommendation_key", None):
            recommendations, errors = \
                self._build_roundabout_placement_recommendations(
                    preview, vut_cache)
            self.roundabout_placement_recommendations = recommendations
            self.roundabout_placement_recommendation_errors = errors
            self.roundabout_placement_recommendation_key = key
        return self.roundabout_placement_recommendations

    def _roundabout_actor(self, role):
        matches = [point for point in self.actor_points if point.get("role") == role]
        return matches[0] if len(matches) == 1 else None

    def _vut_vt1_merge_geometry(
            self, vut_dense, vt1_dense, route_entry_index):
        """Derive the final path-to-path merge as soon as both routes exist."""
        merge_candidates = []
        for index, waypoint in enumerate(
                vut_dense[route_entry_index:], start=route_entry_index):
            location = waypoint.transform.location
            forward = waypoint.transform.get_forward_vector()
            compatible = []
            for target_waypoint in vt1_dense:
                target = target_waypoint.transform.location
                target_forward = target_waypoint.transform.get_forward_vector()
                heading_alignment = (
                    forward.x * target_forward.x
                    + forward.y * target_forward.y)
                if heading_alignment < 0.5:
                    continue
                compatible.append(math.hypot(
                    location.x - target.x, location.y - target.y))
            if compatible and min(compatible) <= 12.0:
                merge_candidates.append((index, min(compatible)))
        if not merge_candidates:
            raise RuntimeError(
                "VUT路线没有汇入VT1所处的环形行车路径；两条稠密路线最近距离超过12m。"
                "请检查VUT入口和VT1环流路线")
        vut_entry_index, merge_offset_m = merge_candidates[0]
        entry_waypoint = vut_dense[vut_entry_index]
        entry_location = entry_waypoint.transform.location
        approach_distance_m = sum(
            self._distance_locations(
                first.transform.location, second.transform.location)
            for first, second in zip(
                vut_dense[:vut_entry_index], vut_dense[1:vut_entry_index + 1]))
        entry_forward = entry_waypoint.transform.get_forward_vector()
        start_location = vut_dense[0].transform.location
        start_gate_signed_m = (
            (start_location.x - entry_location.x) * entry_forward.x
            + (start_location.y - entry_location.y) * entry_forward.y)
        return {
            "entry_index": vut_entry_index,
            "entry_waypoint": entry_waypoint,
            "entry_location": entry_location,
            "merge_offset_m": merge_offset_m,
            "approach_distance_m": approach_distance_m,
            "start_gate_signed_m": start_gate_signed_m,
        }

    def _vut_vt1_preflight_error(self):
        """Return an actionable STEP 5 error, before VT2 or saving is needed."""
        vt1 = self._roundabout_actor("vt1")
        if vt1 is None:
            return "尚未放置VT1。"
        try:
            vut_cache = self._confirmed_roundabout_route("vut")
            vt1_cache = self._confirmed_roundabout_route("vt1", required=False)
            if vt1_cache is None:
                vt1_dense, vt1_options = self._densify_route(
                    vt1.get("route_points", []), vt1)
                vt1_cache = self._build_roundabout_route_cache(
                    "vt1", vt1.get("route_points", []), vt1,
                    vt1_dense, vt1_options)
                self.roundabout_vt1_route_cache = vt1_cache
            vut_dense = vut_cache["dense"]
            vt1_dense = vt1_cache["dense"]
            preview = self._ensure_roundabout_topology_preview(force=True)
            if preview is None:
                return "无法生成VUT入口拓扑：{}".format(
                    "；".join(self.roundabout_topology_preview_errors))
            merge = self._vut_vt1_merge_geometry(
                vut_dense, vt1_dense, preview["entry_index"])
        except RuntimeError as exc:
            return str(exc)
        clearance_m = max(0.0, -merge["start_gate_signed_m"])
        if merge["start_gate_signed_m"] > -VUT_START_ENTRY_MARGIN_M:
            move_m = VUT_START_ENTRY_MARGIN_M - clearance_m
            return (
                "VUT起点距离VUT–VT1最终汇合门线只有{:.1f}m，要求至少{:.1f}m；"
                "请把VUT沿当前进口车道向上游移动至少{:.1f}m，然后重新确认VUT和VT1路线。"
                "该余量用于保证车辆前端在试验开始时位于入口门线之前。".format(
                    clearance_m, VUT_START_ENTRY_MARGIN_M, move_m))

        timing = self._vt1_upstream_timing(vt1_dense, merge)
        timing_error = self._vt1_upstream_timing_error(timing)
        if timing_error:
            return timing_error

        try:
            vt1_exit_arm, _record = self._route_exit_arm(
                vt1_dense, preview["arms"], route_label="VT1")
        except RuntimeError as exc:
            return str(exc)
        exit_order = preview.get("exit_order", [])
        vt1_exit_number = (exit_order.index(vt1_exit_arm) + 1
                           if vt1_exit_arm in exit_order else None)
        if vt1_exit_number != 1:
            return "VT1路线驶出出口{}；国标场景要求VT1从出口1驶出。".format(
                vt1_exit_number if vt1_exit_number is not None else "未知")
        return None

    def _vt1_upstream_timing(self, vt1_dense, merge):
        """Measure the VT1 staging budget on the same path used at save time."""
        entry_location = merge["entry_location"]
        cumulative = [0.0]
        for first, second in zip(vt1_dense, vt1_dense[1:]):
            cumulative.append(
                cumulative[-1] + self._distance_locations(
                    first.transform.location, second.transform.location))
        conflict_index = min(
            range(len(vt1_dense)),
            key=lambda index: self._distance_locations(
                vt1_dense[index].transform.location, entry_location))
        conflict_offset_m = self._distance_locations(
            vt1_dense[conflict_index].transform.location, entry_location)
        upstream_arc_m = cumulative[conflict_index]
        approach_distance_m = float(merge["approach_distance_m"])
        # Conflict synchronisation uses the complete VUT route distance to the
        # shared merge point.  The upstream observation plane is derived from
        # the requested VT1 lead time; it is not subtracted here.
        sync_approach_distance_m = max(0.0, approach_distance_m)
        assumed_vut_speed_mps = VUT_APPROACH_SPEED_ASSUMPTION_KMH / 3.6
        route_approach_time_s = sync_approach_distance_m / assumed_vut_speed_mps
        # The 15 s budget is a runtime timeout, not a minimum expected travel
        # time.  Using max(route_time, timeout) here used to force every VT1
        # staging point to reserve 75 m of motion even for a short approach.
        effective_approach_time_s = route_approach_time_s
        required_m = required_vt1_initial_upstream_distance(
            sync_approach_distance_m,
            target_speed_kmh=15.0,
            vut_approach_speed_mps=assumed_vut_speed_mps,
            stable_duration_s=VT1_STABLE_DURATION_S,
            setup_budget_s=VT1_SETUP_BUDGET_S,
            entry_upstream_margin_m=VT1_ENTRY_UPSTREAM_MINIMUM_M,
            timing_buffer_m=VT1_TIMING_BUFFER_M,
            minimum_distance_m=VT1_INITIAL_MINIMUM_M,
            conflict_headway_target_s=getattr(
                self, "roundabout_conflict_headway_s",
                VT1_CONFLICT_HEADWAY_TARGET_S))
        sync = roundabout_conflict_sync_distances(
            sync_approach_distance_m,
            target_speed_kmh=15.0,
            vut_approach_speed_mps=assumed_vut_speed_mps,
            target_headway_s=getattr(
                self, "roundabout_conflict_headway_s",
                VT1_CONFLICT_HEADWAY_TARGET_S),
            headway_tolerance_s=getattr(
                self, "roundabout_conflict_tolerance_s",
                VT1_CONFLICT_HEADWAY_TOLERANCE_S),
            entry_upstream_min_distance_m=VT1_ENTRY_UPSTREAM_MINIMUM_M)
        return {
            "cumulative": cumulative,
            "conflict_index": conflict_index,
            "conflict_offset_m": conflict_offset_m,
            "upstream_arc_m": upstream_arc_m,
            "required_m": required_m,
            "approach_distance_m": approach_distance_m,
            "sync_approach_distance_m": sync_approach_distance_m,
            "route_approach_time_s": route_approach_time_s,
            "effective_approach_time_s": effective_approach_time_s,
            "conflict_sync": sync,
            "conflict_headway_target_s": getattr(
                self, "roundabout_conflict_headway_s",
                VT1_CONFLICT_HEADWAY_TARGET_S),
            "conflict_headway_tolerance_s": getattr(
                self, "roundabout_conflict_tolerance_s",
                VT1_CONFLICT_HEADWAY_TOLERANCE_S),
        }

    @staticmethod
    def _vt1_upstream_timing_error(timing):
        if timing["conflict_offset_m"] > VT1_CONFLICT_MAX_OFFSET_M:
            return (
                "VT1路线没有经过VUT汇入冲突点；最近横向距离为{:.1f}m，要求≤{:.0f}m。"
                .format(timing["conflict_offset_m"], VT1_CONFLICT_MAX_OFFSET_M))
        if timing["conflict_index"] <= 0:
            return "VT1起点没有位于VUT汇入口上游；请沿环流反方向前移VT1。"
        if timing["upstream_arc_m"] < timing["required_m"]:
            move_m = timing["required_m"] - timing["upstream_arc_m"]
            return (
                "VT1起点到VUT汇入冲突点沿真实路线只有{:.1f}m，至少需要{:.1f}m；"
                "请将VT1沿环流反方向向上游移动至少{:.1f}m。该值采用工程参数："
                "VT1目标速度15km/h、建场+稳速{:.1f}s、VUT接近窗口{:.1f}s、"
                "目标冲突时间差{:.1f}s、入口余量{:.1f}m和时序缓冲{:.1f}m"
                "（不是国标原始距离参数）。"
                .format(
                    timing["upstream_arc_m"], timing["required_m"], move_m,
                    VT1_SETUP_BUDGET_S + VT1_STABLE_DURATION_S,
                    timing["effective_approach_time_s"],
                    timing.get("conflict_headway_target_s",
                               VT1_CONFLICT_HEADWAY_TARGET_S),
                    VT1_ENTRY_UPSTREAM_MINIMUM_M, VT1_TIMING_BUFFER_M))
        return None

    def _analyse_roundabout(self):
        errors = []
        warnings = []
        vt1_matches = [point for point in self.actor_points if point.get("role") == "vt1"]
        vt2_matches = [point for point in self.actor_points if point.get("role") == "vt2"]
        unassigned = [point for point in self.actor_points
                      if point.get("role") not in ("vt1", "vt2")]
        if self.ego_point is None:
            errors.append("Set VUT start with Ctrl+Left on the inbound lane.")
        if len(vt1_matches) != 1:
            errors.append("Place exactly one VT1 (T then Right-click).")
        if len(vt2_matches) != 1:
            errors.append("Place exactly one VT2 (Y then Right-click).")
        if unassigned:
            errors.append("2.b only permits the VT1 and VT2 target actors.")
        if len(self.ego_route_points) < 2:
            errors.append("Select VUT and draw its route with Alt+Right-click (2+ points).")
        vt1 = vt1_matches[0] if len(vt1_matches) == 1 else None
        vt2 = vt2_matches[0] if len(vt2_matches) == 1 else None
        if vt1 is not None and len(vt1.get("route_points", [])) < 2:
            errors.append("Select VT1 and draw its exit-1 route (2+ points).")
        if errors:
            return None, errors, warnings

        try:
            vut_cache = self._confirmed_roundabout_route(
                "vut", required=bool(getattr(
                    self, "roundabout_vut_route_confirmed", False)))
            if vut_cache is None:
                vut_dense, vut_route_options = self._densify_route(
                    self.ego_route_points, self.ego_point)
            else:
                vut_dense = vut_cache["dense"]
                vut_route_options = vut_cache["options"]
            vt1_cache = self._confirmed_roundabout_route(
                "vt1", required=bool(getattr(
                    self, "roundabout_vt1_route_confirmed", False)))
            if vt1_cache is None:
                vt1_dense, vt1_route_options = self._densify_route(
                    vt1["route_points"], vt1)
            else:
                vt1_dense = vt1_cache["dense"]
                vt1_route_options = vt1_cache["options"]
            speed_limit_by_road_id, speed_limit_source = \
                self._roundabout_route_speed_limits(vut_dense)
            preview = self._topology_preview_from_vut_route(vut_dense)
            arms = preview["arms"]
            entry_arm = preview["entry_arm"]
            exit_order = preview["exit_order"]
            entry_order = preview["entry_order"]
            direction = preview["direction"]
            planned_exit = preview["planned_exit"]
            vt1_exit_arm, _ = self._route_exit_arm(
                vt1_dense, arms, route_label="VT1")
        except RuntimeError as exc:
            return None, [str(exc)], warnings + self.roundabout_warnings

        # The VUT route is an open path, not a circle.  Its entry event is the
        # first point at which it joins the already drawn VT1 circulating
        # path.  This path-to-path merge is the runtime entry gate and the VT1
        # conflict reference; no global circle or radius band is involved.
        try:
            merge = self._vut_vt1_merge_geometry(
                vut_dense, vt1_dense, preview["entry_index"])
        except RuntimeError as exc:
            return None, [str(exc)], warnings + self.roundabout_warnings
        vut_entry_index = merge["entry_index"]
        merge_offset_m = merge["merge_offset_m"]
        if vut_entry_index <= 0:
            errors.append("VUT起点已经位于环流冲突区；请把VUT移到进口上游。")
        entry_waypoint = merge["entry_waypoint"]
        vut_approach_distance_m = merge["approach_distance_m"]
        vut_start_entry_margin_m = VUT_START_ENTRY_MARGIN_M
        vut_start_gate_signed_m = merge["start_gate_signed_m"]
        if vut_start_gate_signed_m > -vut_start_entry_margin_m:
            clearance_m = max(0.0, -vut_start_gate_signed_m)
            errors.append(
                "VUT起点距离VUT–VT1最终汇合门线只有{:.1f}m，要求至少{:.1f}m；"
                "请沿进口车道向上游移动至少{:.1f}m。".format(
                    clearance_m, vut_start_entry_margin_m,
                    vut_start_entry_margin_m - clearance_m))
        vt1_exit_number = exit_order.index(vt1_exit_arm) + 1 \
            if vt1_exit_arm in exit_order else None
        if vt1_exit_number != 1:
            errors.append(
                "VT1 route is inferred as exit {}; it must use exit 1.".format(
                    vt1_exit_number if vt1_exit_number is not None else "unknown"))

        # VT1 starts upstream on its actual circulating path.  STEP 5 and the
        # final save deliberately share this calculation, so a placement that
        # cannot satisfy the timing budget is never accepted earlier and then
        # rejected only at STEP 7.
        vt1_start_wp = self.map.get_waypoint(
            carla.Location(x=vt1["x"], y=vt1["y"], z=vt1.get("z", 0.0)),
            project_to_road=True, lane_type=carla.LaneType.Driving)
        vt1_timing = self._vt1_upstream_timing(vt1_dense, merge)
        vt1_cumulative = vt1_timing["cumulative"]
        upstream_arc_m = vt1_timing["upstream_arc_m"]
        vt1_initial_upstream_minimum_m = vt1_timing["required_m"]
        vt1_entry_upstream_minimum_m = VT1_ENTRY_UPSTREAM_MINIMUM_M
        vt1_setup_budget_s = VT1_SETUP_BUDGET_S
        vut_approach_speed_assumption_kmh = VUT_APPROACH_SPEED_ASSUMPTION_KMH
        vut_approach_time_budget_s = VUT_APPROACH_TIME_BUDGET_S
        timing_buffer_m = VT1_TIMING_BUFFER_M
        timing_error = self._vt1_upstream_timing_error(vt1_timing)
        if timing_error:
            errors.append(timing_error)

        downstream_arm = entry_order[0] if entry_order else None
        vt2_entry_record = None
        vt2_wp = self.map.get_waypoint(
            carla.Location(x=vt2["x"], y=vt2["y"], z=vt2.get("z", 0.0)),
            project_to_road=True, lane_type=carla.LaneType.Driving)
        if vt2_wp is None:
            errors.append("VT2位置无法重新吸附到CARLA Driving Lane。")
        else:
            try:
                prepared_vt2_wp, placement = self._prepare_vt2_waypoint(
                    vt2_wp, downstream_arm)
                vt2_entry_record = placement["entry_record"]
                if placement["adjusted"]:
                    adjusted_location = prepared_vt2_wp.transform.location
                    vt2.update({
                        "x": float(adjusted_location.x),
                        "y": float(adjusted_location.y),
                        "z": float(adjusted_location.z + 0.3),
                        "yaw": float(prepared_vt2_wp.transform.rotation.yaw),
                    })
                    vt2_wp = prepared_vt2_wp
                    warnings.append(
                        "VT2原点击点位于{}进口门线之后或距门线不足{:.1f}m；"
                        "已沿同一进口车道自动调整到门线上游{:.1f}m。".format(
                            placement["arm_id"],
                            VT2_MINIMUM_GATE_CLEARANCE_M,
                            placement["upstream_distance_m"]))
            except RuntimeError as exc:
                errors.append("VT2位置无效：{}".format(exc))

        # Heading is part of the fixture geometry.  Re-snap it on every
        # analysis so legacy/manual edits cannot leak into generated trials.
        if vt1_start_wp is not None:
            vt1["yaw"] = float(vt1_start_wp.transform.rotation.yaw)
        if vt2_wp is not None:
            vt2["yaw"] = float(vt2_wp.transform.rotation.yaw)

        if errors:
            return None, errors, warnings + self.roundabout_warnings

        entry_gate = self._make_gate(entry_waypoint, "vut_vt1_path_merge")
        entry_gate["allowed_lanes"] = [
            self._lane_descriptor(item) for item in entry_arm["inbound"]]
        exit_gates = {}
        for number, arm in enumerate(exit_order[:3], start=1):
            route_for_gate = []
            if number == planned_exit:
                route_for_gate = vut_dense
            elif number == 1:
                route_for_gate = vt1_dense
            record = self._route_record_for_lane(
                route_for_gate, arm["outbound"], prefer_last=True)
            gate = copy.deepcopy(record["gate"])
            gate.update({
                "number": number,
                "source": "operator_reviewed_outbound_gate",
                "allowed_lanes": [
                    self._lane_descriptor(item) for item in arm["outbound"]],
            })
            if route_for_gate:
                try:
                    self._route_gate_index(
                        route_for_gate, gate,
                        "VUT路线" if number == planned_exit else "VT1路线")
                except RuntimeError as exc:
                    errors.append(str(exc))
            exit_gates[str(number)] = gate
        # A physical arm can have multiple inbound lanes.  The gate used for
        # VT2 geometry must belong to VT2's actual lane; always taking the
        # first reviewed lane makes a valid actor on the adjacent inbound
        # lane appear laterally outside the downstream-entry gate.
        if vt2_entry_record is None:
            vt2_entry_record = self._route_record_for_lane(
                [vt2_wp], downstream_arm["inbound"])
        # The upstream fixture point may be on a predecessor road/lane ID
        # even though CARLA's directed topology proves that it reaches the
        # reviewed inbound gate.  Preserve both identities: ``gate_lane`` is
        # the reviewed mouth, while ``selected_lane`` is where VT2 actually
        # stands.  The validator can then check the same proven approach
        # membership instead of rejecting a valid predecessor lane merely
        # because OpenDRIVE changes road_id at the connector.
        downstream_gate = self._downstream_gate_for_vt2(
            vt2_entry_record, downstream_arm, vt2_wp)
        if errors:
            return None, errors, warnings + self.roundabout_warnings

        topology_arms = []
        for arm in arms:
            topology_arms.append({
                "arm_id": arm["arm_id"],
                "angle_deg": round(math.degrees(arm["angle"]), 4),
                "inbound_angle_deg": round(math.degrees(arm["inbound_angle"]), 4),
                "outbound_angle_deg": round(math.degrees(arm["outbound_angle"]), 4),
                "inbound_lane_count": len(arm["inbound"]),
                "outbound_lane_count": len(arm["outbound"]),
                "inbound_lanes": [self._lane_descriptor(item) for item in arm["inbound"]],
                "outbound_lanes": [self._lane_descriptor(item) for item in arm["outbound"]],
                "pairing_method": arm.get("pairing_method", "angular_cluster"),
            })
        topology_core = {
            "map_name": str(self.map.name),
            "arm_count": len(topology_arms),
            "arms": topology_arms,
            "vut_entry_arm_id": entry_arm["arm_id"],
            "downstream_entry_arm_id": downstream_arm["arm_id"],
            "exit_arm_ids": {
                str(number): arm["arm_id"]
                for number, arm in enumerate(exit_order[:3], start=1)
            },
            "circulation_order": [
                arm["arm_id"] for arm in [entry_arm] + entry_order
            ],
            "ordering_reference_center": {
                "x": round(preview["ordering_center"][0], 4),
                "y": round(preview["ordering_center"][1], 4),
            },
            "travel_direction": "counterclockwise" if direction > 0 else "clockwise",
            "operator_confirmation": {
                "confirmed": bool(getattr(
                    self, "roundabout_topology_confirmed", False)),
                "method": "operator_reviewed_inbound_outbound_lane_groups",
            },
            "route_model": "open_paths_through_directed_roundabout_topology",
            "circle_fit_used_for_validation": False,
            "vt1_initial_upstream_arc_m": round(upstream_arc_m, 4),
            "vt1_initial_upstream_path_m": round(upstream_arc_m, 4),
            "vt1_initial_minimum_upstream_arc_m": round(
                vt1_initial_upstream_minimum_m, 4),
            "vt1_initial_minimum_upstream_path_m": round(
                vt1_initial_upstream_minimum_m, 4),
            "vt1_entry_upstream_minimum_m": round(
                vt1_entry_upstream_minimum_m, 4),
            "vut_approach_distance_m": round(vut_approach_distance_m, 4),
            "vut_start_gate_signed_m": round(vut_start_gate_signed_m, 4),
            "vut_vt1_merge_offset_m": round(merge_offset_m, 4),
        }
        topology_hash = roundabout_hash(topology_core)
        topology = dict(topology_core)
        topology.update({
            "verified": True,
            "topology_hash": topology_hash,
            "detection_method": (
                "operator_reviewed_lane_groups+route_lane_identity+gate_crossing"),
        })
        analysis = {
            "planned_exit": planned_exit,
            "entry_gate": entry_gate,
            "exit_gates": exit_gates,
            "downstream_entry": downstream_gate,
            "topology": topology,
            "topology_hash": topology_hash,
            "vut_dense": vut_dense,
            "vut_route_options": vut_route_options,
            "vt1_dense": vt1_dense,
            "vt1_route_options": vt1_route_options,
            "speed_limit_by_road_id": speed_limit_by_road_id,
            "speed_limit_source": speed_limit_source,
            "vt1": vt1,
            "vt2": vt2,
            "vt2_waypoint": vt2_wp,
            "downstream_arm": downstream_arm,
            "entry_arm": entry_arm,
            "direction": direction,
            "vt1_upstream_arc_m": upstream_arc_m,
            "vt1_upstream_path_m": upstream_arc_m,
            "vt1_route_length_m": vt1_cumulative[-1],
            "vut_vt1_merge_offset_m": merge_offset_m,
            "vt1_initial_upstream_minimum_m": vt1_initial_upstream_minimum_m,
            "vt1_entry_upstream_minimum_m": vt1_entry_upstream_minimum_m,
            "vt1_setup_budget_s": vt1_setup_budget_s,
            "vt1_timing_buffer_m": timing_buffer_m,
            "vut_approach_speed_assumption_kmh": vut_approach_speed_assumption_kmh,
            "vut_approach_time_budget_s": vut_approach_time_budget_s,
            "vut_approach_distance_m": vut_approach_distance_m,
            "vut_sync_approach_distance_m":
                vt1_timing["sync_approach_distance_m"],
            "vut_start_entry_margin_m": vut_start_entry_margin_m,
            "vt1_conflict_headway_target_s":
                vt1_timing["conflict_headway_target_s"],
            "vt1_conflict_headway_tolerance_s":
                vt1_timing["conflict_headway_tolerance_s"],
            "vt1_conflict_sync": vt1_timing["conflict_sync"],
        }
        return analysis, [], warnings + self.roundabout_warnings

    def _ensure_roundabout_analysis(self, force=False):
        if not self.is_roundabout_2b:
            return None
        if force or self.roundabout_dirty:
            analysis, errors, warnings = self._analyse_roundabout()
            self.roundabout_analysis = analysis
            self.roundabout_errors = errors
            self.roundabout_warnings = warnings
            self.roundabout_dirty = False
        return self.roundabout_analysis

    def world_to_screen(self, x, y):
        sx = int(self.offset_x + x * self.zoom)
        sy = int(self.offset_y - y * self.zoom)
        return sx, sy

    def screen_to_world(self, sx, sy):
        x = (sx - self.offset_x) / self.zoom
        y = (self.offset_y - sy) / self.zoom
        return x, y

    def _screen_point_visible(self, point, margin=VIEWPORT_DRAW_MARGIN):
        """Cull off-screen primitives before passing large coordinates to SDL."""
        x, y = point
        width, height = self.screen.get_size()
        return (-margin <= x <= width + margin
                and -margin <= y <= height + margin)

    def _draw_clipped_line(self, color, start, end, width=1):
        """Draw a safely clipped segment without passing huge values to SDL."""
        screen_width, screen_height = self.screen.get_size()
        if screen_width <= 0 or screen_height <= 0:
            return
        x0, y0 = float(start[0]), float(start[1])
        x1, y1 = float(end[0]), float(end[1])
        if not all(math.isfinite(value) for value in (x0, y0, x1, y1)):
            return
        dx, dy = x1 - x0, y1 - y0
        lower, upper = 0.0, 1.0
        bounds = (
            (-dx, x0),
            (dx, float(screen_width - 1) - x0),
            (-dy, y0),
            (dy, float(screen_height - 1) - y0),
        )
        for coefficient, distance in bounds:
            if abs(coefficient) < 1e-12:
                if distance < 0.0:
                    return
                continue
            ratio = distance / coefficient
            if coefficient < 0.0:
                lower = max(lower, ratio)
            else:
                upper = min(upper, ratio)
            if lower > upper:
                return
        clipped_start = (
            int(round(x0 + lower * dx)),
            int(round(y0 + lower * dy)),
        )
        clipped_end = (
            int(round(x0 + upper * dx)),
            int(round(y0 + upper * dy)),
        )
        pygame.draw.line(
            self.screen, color, clipped_start, clipped_end, width)

    def _draw_clipped_polyline(self, points, color, width=1):
        for start, end in zip(points, points[1:]):
            # Route anchors are intentionally sparse.  At high zoom a normal
            # 10-20 m interval can span thousands of pixels.  Clipping such a
            # chord to both viewport edges creates a misleading full-screen
            # coloured band.  Only draw a bounded tail around an anchor that
            # is actually visible; two off-screen anchors contribute nothing.
            start_visible = self._screen_point_visible(start, margin=0)
            end_visible = self._screen_point_visible(end, margin=0)
            if not start_visible and not end_visible:
                continue
            dx = float(end[0]) - float(start[0])
            dy = float(end[1]) - float(start[1])
            length = math.hypot(dx, dy)
            if length <= MAX_ROUTE_VISIBLE_TAIL_PX:
                self._draw_clipped_line(color, start, end, width)
                continue
            unit_x, unit_y = dx / length, dy / length
            if start_visible:
                bounded_end = (
                    float(start[0]) + unit_x * MAX_ROUTE_VISIBLE_TAIL_PX,
                    float(start[1]) + unit_y * MAX_ROUTE_VISIBLE_TAIL_PX,
                )
                self._draw_clipped_line(color, start, bounded_end, width)
            if end_visible:
                bounded_start = (
                    float(end[0]) - unit_x * MAX_ROUTE_VISIBLE_TAIL_PX,
                    float(end[1]) - unit_y * MAX_ROUTE_VISIBLE_TAIL_PX,
                )
                self._draw_clipped_line(color, bounded_start, end, width)

    def _fixed_screen_gate_segment(self, gate, center=None, half_width_px=24.0):
        """Return a gate bar with bounded screen dimensions at every zoom."""
        if center is None:
            center = self.world_to_screen(
                gate["center"]["x"], gate["center"]["y"])
        left_world = self.world_to_screen(
            gate["left"]["x"], gate["left"]["y"])
        right_world = self.world_to_screen(
            gate["right"]["x"], gate["right"]["y"])
        gate_dx = right_world[0] - left_world[0]
        gate_dy = right_world[1] - left_world[1]
        gate_length = math.hypot(gate_dx, gate_dy)
        if gate_length < 1e-6:
            yaw = math.radians(float(gate.get("approach_yaw", 0.0)))
            gate_dx, gate_dy = -math.sin(yaw), -math.cos(yaw)
            gate_length = 1.0
        unit_x, unit_y = gate_dx / gate_length, gate_dy / gate_length
        return (
            (center[0] - unit_x * half_width_px,
             center[1] - unit_y * half_width_px),
            (center[0] + unit_x * half_width_px,
             center[1] + unit_y * half_width_px),
        )

    def _draw_direction_arrow(
            self, color, origin, yaw_degrees, width=2,
            length_px=ROUNDABOUT_ARROW_LENGTH_PX,
            head_length_px=ROUNDABOUT_ARROW_HEAD_LENGTH_PX):
        """Draw an unmistakable screen-space arrow instead of a plain dash."""
        yaw = math.radians(float(yaw_degrees))
        end = (
            origin[0] + math.cos(yaw) * float(length_px),
            origin[1] - math.sin(yaw) * float(length_px),
        )
        back = yaw + math.pi
        wing_angle = math.radians(32.0)
        left = (
            end[0] + math.cos(back - wing_angle) * float(head_length_px),
            end[1] - math.sin(back - wing_angle) * float(head_length_px),
        )
        right = (
            end[0] + math.cos(back + wing_angle) * float(head_length_px),
            end[1] - math.sin(back + wing_angle) * float(head_length_px),
        )
        pygame.draw.line(self.screen, color, origin, end, width)
        pygame.draw.line(self.screen, color, end, left, width)
        pygame.draw.line(self.screen, color, end, right, width)

    def _zoom_at(self, screen_x, screen_y, wheel_steps):
        """Zoom around the mouse while keeping coordinates inside safe bounds."""
        if not wheel_steps:
            return
        world_x, world_y = self.screen_to_world(screen_x, screen_y)
        requested = self.zoom * (ZOOM_SPEED ** wheel_steps)
        self.zoom = min(MAX_ZOOM, max(MIN_ZOOM, requested))
        self.offset_x = screen_x - world_x * self.zoom
        self.offset_y = screen_y + world_y * self.zoom

    def get_nearest_waypoint(self, wx, wy):
        min_dist = 999999
        best_wp = None
        for wp in self.waypoints:
            dx = wp.transform.location.x - wx
            dy = wp.transform.location.y - wy
            dist = dx * dx + dy * dy
            if dist < min_dist:
                min_dist = dist
                best_wp = wp
        return best_wp

    def _roundabout_click_waypoint_candidates(self, wx, wy):
        """Return nearby lane candidates instead of committing to one snap.

        At compact junctions, an inbound lane, an outbound lane and one or
        more connector lanes may all lie within a few screen pixels.  Keep a
        small, distance-ordered set and let the lane-graph search decide which
        candidate is actually reachable from the route built so far.
        """
        radius_m = max(
            ROUNDABOUT_CLICK_CANDIDATE_MIN_RADIUS_M,
            min(ROUNDABOUT_CLICK_CANDIDATE_MAX_RADIUS_M,
                ROUNDABOUT_CLICK_CANDIDATE_SCREEN_RADIUS_PX
                / max(float(getattr(self, "zoom", ZOOM)), MIN_ZOOM)))
        records = {}

        def remember(waypoint):
            if waypoint is None:
                return
            if (getattr(waypoint, "lane_type", None) is not None
                    and not self._lane_type_is_driving(waypoint)):
                return
            location = waypoint.transform.location
            distance_m = math.hypot(location.x - wx, location.y - wy)
            if distance_m > radius_m:
                return
            token = self._waypoint_token(waypoint)
            current = records.get(token)
            if current is None or distance_m < current[1]:
                records[token] = (waypoint, distance_m)

        nearest = self.get_nearest_waypoint(wx, wy)
        remember(nearest)
        for waypoint in getattr(self, "waypoints", []):
            remember(waypoint)

        # Generated waypoints are normally spaced at 2 m.  Retaining at most
        # two samples per lane keeps both sides of a nearby hairpin/connector
        # without allowing one long lane to crowd all other candidates out.
        output = []
        counts_by_lane = {}
        for waypoint, distance_m in sorted(
                records.values(), key=lambda item: item[1]):
            lane = self._lane_key(waypoint)
            if counts_by_lane.get(lane, 0) >= 2:
                continue
            output.append((waypoint, distance_m))
            counts_by_lane[lane] = counts_by_lane.get(lane, 0) + 1
            if len(output) >= ROUNDABOUT_CLICK_CANDIDATE_LIMIT:
                break
        return output

    def get_ground_z(self, x, y):
        location = carla.Location(x=x, y=y, z=100)
        waypoint = self.map.get_waypoint(location, project_to_road=False, lane_type=carla.LaneType.Any)
        if waypoint:
            return waypoint.transform.location.z + 0.2
        return 0.5

    def draw_waypoints(self):
        # The map is sampled at 2 m in world space.  At a small screen zoom,
        # fixed-radius waypoint circles overlap into opaque grey bands.  For
        # the 2.b editor, retain lane direction arrows but adapt their spacing
        # to at least ~10 screen pixels and omit the decorative grey circles.
        roundabout_bins = set()
        roundabout_spacing_m = max(
            2.0, ROUNDABOUT_ARROW_SCREEN_SPACING_PX / max(self.zoom, MIN_ZOOM))
        for wp in self.waypoints:
            x, y = self.world_to_screen(wp.transform.location.x, wp.transform.location.y)
            if not self._screen_point_visible((x, y), margin=16):
                continue
            if self.is_roundabout_2b:
                lane_key = (
                    int(getattr(wp, "road_id", 0)),
                    int(getattr(wp, "section_id", 0)),
                    int(getattr(wp, "lane_id", 0)),
                )
                longitudinal = float(getattr(
                    wp, "s", wp.transform.location.x + wp.transform.location.y))
                sample_key = lane_key + (
                    int(math.floor(longitudinal / roundabout_spacing_m)),)
                if sample_key in roundabout_bins:
                    continue
                roundabout_bins.add(sample_key)
            signal_route = self._waypoint_is_signal_route(wp)
            point_color = (255, 180, 0) if signal_route else (220, 220, 220)
            arrow_color = (255, 90, 0) if signal_route else (0, 255, 0)
            if not self.is_roundabout_2b:
                pygame.draw.circle(
                    self.screen, point_color, (x, y), 5 if signal_route else 3)
            if self.is_roundabout_2b:
                self._draw_direction_arrow(
                    arrow_color, (x, y), wp.transform.rotation.yaw, width=2)
            else:
                yaw_rad = math.radians(wp.transform.rotation.yaw)
                dx = math.cos(yaw_rad) * 10.0
                dy = -math.sin(yaw_rad) * 10.0
                pygame.draw.line(
                    self.screen, arrow_color, (x, y), (x + dx, y + dy), 3)

    def draw_traffic_lights(self):
        for (x, y) in self.traffic_light_positions:
            sx, sy = self.world_to_screen(x, y)
            if not self._screen_point_visible((sx, sy), margin=160):
                continue
            pygame.draw.circle(self.screen, (255, 0, 0), (sx, sy), 10)
            if self._roundabout_map_labels_visible():
                self.screen.blit(
                    self.font.render("TRAFFIC LIGHT", True, (255, 0, 0)),
                    (sx + 12, sy))

    def _draw_route(self, points, color, prefix, width=3, actor_start=None,
                    hollow=False, draw_polyline=True):
        screen_points = []
        if actor_start is not None:
            screen_points.append(self.world_to_screen(
                actor_start["x"], actor_start["y"]))
        for index, point in enumerate(points):
            sx, sy = self.world_to_screen(point["x"], point["y"])
            screen_points.append((sx, sy))
            if self._screen_point_visible((sx, sy), margin=160):
                endpoint = index in (0, len(points) - 1)
                pygame.draw.circle(
                    self.screen, color, (sx, sy), 7 if endpoint else 4,
                    2 if hollow else 0)
                # Intermediate P/T numbers created most of the map clutter.
                # The polyline and map arrows already convey ordering, so
                # label only the first and final user anchors by default.
                if endpoint and self._roundabout_map_labels_visible():
                    self.screen.blit(
                        self.font.render(
                            "{}{}".format(prefix, index + 1), True, color),
                        (sx + 8, sy - 18))
        if draw_polyline and len(screen_points) > 1:
            self._draw_clipped_polyline(screen_points, color, width)

    def _draw_roundabout_draft_route(self, cache):
        payload = self._decimated_roundabout_payload(cache.get("payload", []))
        if len(payload) > 1:
            self._draw_clipped_polyline(
                [self.world_to_screen(point["x"], point["y"])
                 for point in payload],
                ROUNDABOUT_COLOR_UNCONFIRMED, 2)

    def _decimated_roundabout_payload(self, payload):
        if len(payload) <= 2:
            return list(payload)
        # Keep screen-space segments short enough that zooming cannot create
        # the historical full-width SDL bars, while avoiding thousands of
        # circles at map overview scale.
        spacing_m = min(
            ROUNDABOUT_CONFIRMED_ROUTE_SAMPLE_M,
            max(0.2, 80.0 / max(float(self.zoom), 0.01)))
        output = [payload[0]]
        accumulated = 0.0
        previous = payload[0]
        for point in payload[1:-1]:
            accumulated += math.hypot(
                float(point["x"]) - float(previous["x"]),
                float(point["y"]) - float(previous["y"]))
            previous = point
            if accumulated >= spacing_m:
                output.append(point)
                accumulated = 0.0
        output.append(payload[-1])
        return output

    def _draw_confirmed_roundabout_route(self, cache, color):
        payload = self._decimated_roundabout_payload(cache.get("payload", []))
        if not payload:
            return
        screen_points = [self.world_to_screen(point["x"], point["y"])
                         for point in payload]
        self._draw_clipped_polyline(screen_points, color, 5)
        for index in (0, len(screen_points) - 1):
            point = screen_points[index]
            if self._screen_point_visible(point, margin=0):
                pygame.draw.circle(self.screen, color, point, 7)
        since_arrow_px = 0.0
        for start, end in zip(screen_points, screen_points[1:]):
            length_px = math.dist(start, end)
            since_arrow_px += length_px
            if since_arrow_px < ROUNDABOUT_ARROW_SCREEN_SPACING_PX:
                continue
            since_arrow_px = 0.0
            midpoint = (
                int(round((start[0] + end[0]) * 0.5)),
                int(round((start[1] + end[1]) * 0.5)),
            )
            if not self._screen_point_visible(midpoint, margin=8):
                continue
            yaw = math.degrees(math.atan2(
                -(end[1] - start[1]), end[0] - start[0]))
            self._draw_direction_arrow(
                color, midpoint, yaw, width=2, length_px=10, head_length_px=4)

    def _draw_failed_roundabout_route(self, failure):
        if not failure:
            return
        payload = self._decimated_roundabout_payload(failure.get("payload", []))
        if len(payload) > 1:
            self._draw_clipped_polyline(
                [self.world_to_screen(point["x"], point["y"])
                 for point in payload],
                ROUNDABOUT_COLOR_FAILED_ROUTE, 3)
        start = failure.get("segment_start")
        target = failure.get("segment_target")
        if start and target:
            self._draw_clipped_polyline(
                [self.world_to_screen(start["x"], start["y"]),
                 self.world_to_screen(target["x"], target["y"])],
                ROUNDABOUT_COLOR_FAILED_SEGMENT, 3)

    def _draw_gate(self, gate, color, label):
        if not gate:
            return
        center = self.world_to_screen(gate["center"]["x"], gate["center"]["y"])
        # Gate overlays are operator markers, not map-scale road geometry.
        # Cull by their centre and keep the transverse bar fixed at 48 px so
        # an off-screen gate cannot turn into a viewport-wide coloured band.
        if not self._screen_point_visible(center, margin=80):
            return
        left, right = self._fixed_screen_gate_segment(gate, center)
        self._draw_clipped_line(color, left, right, 4)
        if self._screen_point_visible(center, margin=0):
            pygame.draw.circle(self.screen, color, center, 5)
            if self._roundabout_map_labels_visible():
                self.screen.blit(
                    self.font.render(label, True, color),
                    (center[0] + 8, center[1] - 20))

    def _draw_roundabout_topology_preview(self, preview):
        """Render only route-relevant derived gates after STEP 1."""
        if not preview:
            return
        # At VUT-confirmation time VT1 may not exist yet.  This is the
        # operator-reviewed inbound gate, not the later VUT/VT1 merge point.
        self._draw_gate(
            preview["entry_gate"], ROUNDABOUT_COLOR_VUT, "VUT IN GATE")
        exit_colors = {
            1: ROUNDABOUT_COLOR_EXIT_1,
            2: ROUNDABOUT_COLOR_EXIT_2,
            3: ROUNDABOUT_COLOR_EXIT_3,
        }
        for arm in preview["arm_gates"]:
            short_id = arm["arm_id"].replace("arm_", "A")
            if arm["is_downstream_entry_arm"]:
                self._draw_gate(
                    arm["inbound_gate"], ROUNDABOUT_COLOR_VT2,
                    "VT2 IN / {}".format(short_id))
            if 1 <= int(arm["exit_number"]) <= 3:
                exit_number = int(arm["exit_number"])
                self._draw_gate(
                    arm["outbound_gate"], exit_colors[exit_number],
                    "EXIT {} / {}".format(exit_number, short_id))

    def _active_roundabout_role(self):
        if self.selected_mode == "ego":
            return "vut"
        if (self.selected_mode == "agent"
                and 0 <= self.selected_agent_idx < len(self.actor_points)):
            return self.actor_points[self.selected_agent_idx].get("role")
        return None

    def _draw_roundabout_panel(self):
        expanded = getattr(self, "roundabout_help_expanded", False)
        panel_font = getattr(self, "panel_font", self.font)
        vt1 = self._roundabout_actor("vt1")
        vt2 = self._roundabout_actor("vt2")
        active = (self._active_roundabout_role() or "none").upper()
        branch = "CAPABLE" if self.roundabout_capable else "INCAPABLE / ODD"
        cjk = getattr(self, "font_supports_cjk", False)
        branch_label = (("具备环岛能力" if self.roundabout_capable else "不具备/ODD")
                        if cjk else branch)
        vt1_route_count = len(vt1.get("route_points", [])) if vt1 else 0
        vut_route_confirmed = bool(getattr(
            self, "roundabout_vut_route_confirmed", False))
        vt1_route_confirmed = bool(getattr(
            self, "roundabout_vt1_route_confirmed", False))
        branch_confirmed = bool(getattr(
            self, "roundabout_branch_confirmed", False))
        topology_confirmed = bool(getattr(
            self, "roundabout_topology_confirmed", False))
        topology_review_ready = bool(getattr(
            self, "roundabout_topology_review_ready", False))
        validation_ready = bool(
            branch_confirmed and topology_confirmed
            and vut_route_confirmed and vt1_route_confirmed and vt2)
        show_validation_errors = bool(
            getattr(self, "roundabout_validation_attempted", False)
            and validation_ready)
        if vut_route_confirmed:
            self._ensure_roundabout_topology_preview()
        topology_seeds = getattr(self, "roundabout_topology_seeds", [])
        complete_arm_count = sum(
            1 for arm in topology_seeds if arm.get("inbound") and arm.get("outbound"))
        step_done = {
            0: branch_confirmed,
            1: topology_confirmed,
            2: bool(self.ego_point),
            3: vut_route_confirmed,
            4: bool(vt1),
            5: vt1_route_confirmed,
            6: bool(vt2),
            7: bool(self.roundabout_analysis),
        }
        if not branch_confirmed:
            current_step = 0
        elif not topology_confirmed:
            current_step = 1
        elif not self.ego_point:
            current_step = 2
        elif not vut_route_confirmed:
            current_step = 3
        elif not vt1:
            current_step = 4
        elif not vt1_route_confirmed:
            current_step = 5
        elif not vt2:
            current_step = 6
        else:
            current_step = 7

        def step(number, text_value):
            done = step_done[number]
            marker = ">>" if number == current_step else "  "
            partial = number == 1 and topology_review_ready and not topology_confirmed
            state = (("部分" if partial else ("完成" if done else "待做")) if cjk
                     else ("PART" if partial else ("OK" if done else "DO")))
            color = ((255, 225, 100) if number == current_step
                     else ((120, 255, 150) if done else (215, 215, 215)))
            return ("{} STEP {} [{}] {}".format(marker, number, state, text_value), color)

        rows = [
            (("GB/T 41798-2022 2.b 环形路口 | 分步操作向导" if cjk else
              "GB/T 41798-2022 2.b ROUNDABOUT | GUIDED WORKFLOW"), (255, 255, 255)),
            (("先完成STEP 0与STEP 1；后续对象可灵活放置，黄色 >> 表示当前建议步骤。" if cjk else
              "Complete STEP 0 and STEP 1 first; later placement may be flexible."),
             (170, 200, 255)),
            (("地图颜色：IN绿｜OUT蓝｜VUT青｜出口1黄｜出口2橙｜出口3橙红｜VT2紫"
              if cjk else
              "MAP: IN green | OUT blue | VUT cyan | E1 yellow | E2 orange | E3 red | VT2 purple"),
             (130, 235, 255)),
            step(0, (("确认测试分支：{}（Enter确认；R切换）" if cjk else
                      "Confirm test branch: {} (Enter confirms; R switches)").format(
                          branch_label))),
            step(1, (("标定并审核拓扑：{}个arm（P=检查；左键选自动门线+Delete=排除；Enter=确认；X=重做）"
                      if cjk else
                      "Review topology: {} arms (P=audit; select auto gate + Delete=exclude; Enter=accept; X=reset)").format(
                          complete_arm_count))),
            step(2, "按V选VUT；进口车道Ctrl+左键放置" if cjk else
                 "V selects VUT; Ctrl+Left places it inbound"),
            step(3, ("选中VUT；Alt+右键画穿蓝色出口2/3门线（{}点）；Enter确认" if cjk else
                     "Draw VUT across blue exit 2/3 gate: Alt+Right ({}); Enter confirms").format(
                         len(self.ego_route_points))),
            step(4, "按T选VT1；入口上游Ctrl+左键放置" if cjk else
                 "T selects VT1; Ctrl+Left places it upstream"),
            step(5, ("选中VT1；Alt+右键画至出口1（{}点）；Enter确认" if cjk else
                     "Draw VT1 to exit 1: Alt+Right ({}); Enter confirms").format(
                         vt1_route_count)),
            step(6, "按Y选VT2；下游第1入口Ctrl+左键放置" if cjk else
                 "Y selects VT2; Ctrl+Left places it downstream"),
            step(7, "校验并按天气×车型生成场景：S" if cjk else
                 "Validate and expand weather x vehicle scenes: S"),
            (("VT1位置：环岛环形行车道上，沿环流方向位于VUT汇入口之前；不要放在接入道路"
              if cjk else
              "VT1: on the circulating lane before the VUT merge; not on an access road"),
             (255, 190, 80)),
            (("VT2位置：沿环流方向的下一个接入口，放在箭头朝向环岛的进口车道、入口线前"
              if cjk else
              "VT2: next downstream arm, inbound lane pointing toward the ring, before entry"),
             (255, 150, 255)),
            (("注意：VT2所在下游第1入口不等同于VT1驶出的出口1"
              if cjk else
              "The downstream-first entry for VT2 is not the same concept as VT1 exit 1"),
             (255, 205, 255)),
            ((("路线锚点：普通{:.0f}-{:.0f}m | 直线至{:.0f}m | 分叉/入口/出口5-{:.0f}m"
               if cjk else
               "ANCHOR SPACING: {:.0f}-{:.0f}m normal | to {:.0f}m straight | 5-{:.0f}m at branches/gates").format(
                   ROUTE_ANCHOR_RECOMMENDED_MIN_M, ROUTE_ANCHOR_RECOMMENDED_MAX_M,
                   ROUTE_ANCHOR_STRAIGHT_MAX_M, ROUTE_ANCHOR_BRANCH_MAX_M)),
             (130, 235, 255)),
            (("Alt+右键立即验路：失败点不加入，红线提示后直接重选；Enter做完整复核"
              if cjk else
              "Alt+Right validates now: rejected points are not added; Enter runs formal checks"),
             (255, 220, 100)),
            ((("当前路线={} | 放置角色={} | V/T/Y选择 | L=完整拓扑" if cjk else
               "ACTIVE={} | PLACE={} | V/T/Y select | L=full topology").format(
                   active, self.roundabout_place_role.upper())), (190, 255, 205)),
        ]
        if validation_ready and self.roundabout_analysis:
            save_status_row = (("[可保存] 出口={} 接入方向={} 拓扑={}" if cjk else
                                "[READY] exit={} arms={} topology={}").format(
                self.roundabout_analysis["planned_exit"],
                self.roundabout_analysis["topology"]["arm_count"],
                self.roundabout_analysis["topology_hash"][:10]), (80, 255, 120))
            rows.append(save_status_row)
        else:
            if show_validation_errors:
                save_status_row = (("[校验未通过] {}个问题；按H查看详情" if cjk else
                                    "[VALIDATION FAILED] {} issue(s); H shows details").format(
                                        len(self.roundabout_errors)), (255, 90, 90))
            else:
                save_status_row = (("[待完成] 按当前步骤继续；完成后按S校验" if cjk else
                                    "[IN PROGRESS] Follow the current step; S validates when complete"),
                                   (170, 200, 255))
            rows.append(save_status_row)
            if not branch_confirmed:
                rows.append((
                    ("- STEP 0：按Enter接受当前测试分支，或先按R切换" if cjk else
                     "- STEP 0: Press Enter to accept branch, or R to switch"),
                    (220, 220, 220)))
            if branch_confirmed and not topology_confirmed:
                if topology_review_ready:
                    rows.append((
                        ("- STEP 1待用户审核：检查全部IN/OUT和双车道标记；无误按Enter"
                         if cjk else
                         "- STEP 1 review: inspect all IN/OUT lanes; Enter accepts"),
                        (255, 225, 100)))
                else:
                    rows.append((
                        ("- STEP 1下一点：{}；逐臂标完后按P进行程序拓扑检查" if cjk else
                         "- STEP 1 next: {}; mark every arm, then P runs audit").format(
                              "进口" if getattr(
                                  self, "roundabout_topology_mark_kind", "inbound") == "inbound"
                              else "出口"),
                        (220, 220, 220)))
            if not vut_route_confirmed:
                rows.append((("- 最后一点须在蓝色OUT门线下游；画完后按Enter确认" if cjk else
                              "- End beyond the blue OUT gate, then press Enter"),
                             (220, 220, 220)))
            if vt1 is not None and not vt1_route_confirmed:
                rows.append((("- 画完VT1路线后按Enter确认" if cjk else
                              "- Press Enter after finishing the VT1 route"),
                             (220, 220, 220)))
            for error in (self.roundabout_errors[:2]
                          if show_validation_errors else []):
                rows.append(("- " + error[:104], (255, 130, 130)))
        if show_validation_errors and self.roundabout_warnings:
            rows.append(("WARN: " + self.roundabout_warnings[0][:100], (255, 190, 80)))
        feedback_color = (255, 100, 100) if self.last_feedback_error else (255, 245, 150)
        screen_feedback = self.last_feedback
        if not cjk and not screen_feedback.isascii():
            screen_feedback = (
                "Action rejected; see console and SAVE BLOCKED items."
                if self.last_feedback_error else
                "Action accepted; see console details and updated checklist.")

        def wrap_feedback(value, first_prefix, continuation_prefix, maximum_lines=8):
            maximum_width = max(
                160, min(900, self.screen.get_width() - 16) - 34)
            output = []
            remaining = str(value)
            prefix = first_prefix
            while remaining and len(output) < maximum_lines:
                line = ""
                for character in remaining:
                    candidate = prefix + line + character
                    if line and panel_font.size(candidate)[0] > maximum_width:
                        break
                    line += character
                if not line:
                    line = remaining[0]
                output.append(prefix + line)
                remaining = remaining[len(line):]
                prefix = continuation_prefix
            if remaining and output:
                while output[-1] and panel_font.size(output[-1] + "…")[0] > maximum_width:
                    output[-1] = output[-1][:-1]
                output[-1] += "…"
            return output

        if self.last_feedback_error:
            feedback_text_lines = wrap_feedback(
                screen_feedback,
                "❌ 操作/校验失败：" if cjk else "ERROR: ",
                "  ")
            feedback_rows = [
                (line, feedback_color) for line in feedback_text_lines]
        else:
            feedback_rows = [
                (("最近操作：" if cjk else "Last: ") + screen_feedback[:105],
                 feedback_color)]
        rows.extend(feedback_rows)
        undo_label = (self.roundabout_history[-1]["action"]
                      if getattr(self, "roundabout_history", []) else "none")
        help_action = (("折叠" if expanded else "展开") if cjk else
                       ("collapse" if expanded else "expand"))
        rows.append((("撤销=Backspace/Ctrl+Z（下一项：{}）| H={}帮助/地图文字 | Q=退出" if cjk else
                      "UNDO=Backspace/Ctrl+Z (next: {}) | H={} help/map labels | Q=quit").format(
                          undo_label[:48], help_action), (200, 200, 200)))

        if not expanded:
            vut_route_progress = "{}{}".format(
                len(self.ego_route_points), "✓" if vut_route_confirmed else "?")
            vt1_route_progress = "{}{}".format(
                vt1_route_count, "✓" if vt1_route_confirmed else "?")
            progress = (("进度 拓扑={} | VUT={}/路线={} | VT1={}/路线={} | VT2={}"
                         if cjk else
                         "PROGRESS topology={} | VUT={}/route={} | VT1={}/route={} | VT2={}").format(
                             "OK" if topology_confirmed else "--",
                             "OK" if self.ego_point else "--",
                             vut_route_progress,
                             "OK" if vt1 else "--", vt1_route_progress,
                             "OK" if vt2 else "--"))
            compact_header = (("GB/T 41798 2.b 环岛 | {} | H显示帮助/地图文字" if cjk else
                               "GB/T 41798 2.b | {} | H shows help/map labels").format(
                                   branch_label), (255, 255, 255))
            current_step_row = rows[current_step + 3]
            vut_proposal = getattr(self, "roundabout_vut_recommendation", None)
            if vut_proposal:
                recommended_exits = vut_proposal["exit_gates"]
                vut_proposal_text = (
                    "初步建议：从{}汇入；VUT驶向EXIT 2={}或EXIT 3={}（Enter后正式复核）"
                    if cjk else
                    "PREVIEW: merge {}; VUT EXIT 2={} or EXIT 3={} (Enter verifies)")
                vut_proposal_text = vut_proposal_text.format(
                    vut_proposal["entry_arm_id"],
                    recommended_exits["2"]["arm_id"],
                    recommended_exits["3"]["arm_id"])
            else:
                proposal_errors = getattr(
                    self, "roundabout_vut_recommendation_errors", [])
                vut_proposal_text = (
                    "初步建议不可用：{}".format(proposal_errors[0])
                    if cjk and proposal_errors else
                    ("Preliminary merge/exit suggestion unavailable"
                     if proposal_errors else
                     ("设置VUT后自动显示建议汇入口和EXIT 2/3" if cjk else
                      "Place VUT to show suggested merge and EXIT 2/3")))
            placement_hints = {
                3: (vut_proposal_text,
                    (90, 235, 255) if vut_proposal else (255, 180, 100)),
                4: (("位置提示：VT1放环形行车道，沿箭头方向在VUT汇入口之前；不要放接入道路"
                     if cjk else
                     "VT1 position: circulating lane before VUT merge; not an access road"),
                    (255, 190, 80)),
                6: (("位置提示：VT2放下游第1接入口的进口车道，沿箭头朝环岛，且在入口线前"
                     if cjk else
                     "VT2 position: next downstream inbound lane, pointing into the ring"),
                    (255, 150, 255)),
            }
            placement_recommendations = \
                self._ensure_roundabout_placement_recommendations()
            if (placement_recommendations
                    and placement_recommendations.get("vt1")):
                placement_hints[4] = ((
                    "推荐放置：橙色路点/箭头是满足距离且可到达冲突点的VT1环流区间"
                    if cjk else
                    "Recommended: orange lane points can reach the conflict with enough VT1 distance"),
                    ROUNDABOUT_COLOR_VT1_RECOMMENDED)
            elif (current_step == 4
                    and getattr(self, "roundabout_placement_recommendation_errors", [])):
                placement_hints[4] = ((
                    "VT1推荐区间暂不可用：{}".format(
                        self.roundabout_placement_recommendation_errors[0])
                    if cjk else
                    "VT1 recommendation unavailable; inspect CARLA predecessor topology"),
                    (255, 150, 110))
            if (placement_recommendations
                    and placement_recommendations.get("vt2")):
                placement_hints[6] = ((
                    "推荐放置：紫色路点/箭头是下游第1入口门线上游的VT2进口区间"
                    if cjk else
                    "Recommended: purple lane points are upstream of the downstream-first IN gate"),
                    ROUNDABOUT_COLOR_VT2_RECOMMENDED)
            elif (current_step == 6
                    and getattr(self, "roundabout_placement_recommendation_errors", [])):
                placement_hints[6] = ((
                    "VT2推荐区间暂不可用：{}".format(
                        self.roundabout_placement_recommendation_errors[-1])
                    if cjk else
                    "VT2 recommendation unavailable; inspect downstream IN topology"),
                    (255, 150, 180))
            feedback_row = feedback_rows[0]
            undo_row = rows[-1]
            if current_step in (0, 1):
                if current_step == 0:
                    primary = ((
                        ">> STEP 0 [待做] 按Enter确认测试分支：{}（R=切换）"
                        if cjk else
                        ">> STEP 0 [DO] Press Enter to confirm {}; R switches").format(
                            branch_label), (255, 225, 100))
                    secondary = ((
                        "STEP 1 [锁定] STEP 0完成后标定并审核IN/OUT拓扑"
                        if cjk else
                        "STEP 1 [LOCKED] Complete STEP 0 before IN/OUT topology"),
                        (185, 185, 185))
                elif topology_review_ready:
                    review = getattr(self, "roundabout_topology_review", None) or {}
                    primary = ((
                        "STEP 0 [完成] 测试分支：{}"
                        if cjk else
                        "STEP 0 [OK] Test branch: {}").format(branch_label),
                        (120, 255, 150))
                    secondary = ((
                        ">> STEP 1 [待审核] 程序已展开{}个arm、{}条车道；检查后按Enter"
                        if cjk else
                        ">> STEP 1 [REVIEW] {} arms / {} lanes shown; Enter accepts").format(
                            review.get("arm_count", 0), review.get("lane_count", 0)),
                        (255, 225, 100))
                else:
                    incomplete = bool(
                        topology_seeds and not topology_seeds[-1].get("outbound"))
                    arm_number = len(topology_seeds) if incomplete else len(topology_seeds) + 1
                    next_kind = "出口 OUT（蓝色）" if incomplete else "进口 IN（绿色）"
                    primary = ((
                        "STEP 0 [完成] 测试分支：{}"
                        if cjk else
                        "STEP 0 [OK] Test branch: {}").format(branch_label),
                        (120, 255, 150))
                    secondary = ((
                        ">> STEP 1 [待做] A{:02d} {}：移动鼠标预览，右键确认"
                        if cjk else
                        ">> STEP 1 [DO] A{:02d} {}: hover to preview; Right-click").format(
                            arm_number, next_kind if cjk else (
                                "OUT (blue)" if incomplete else "IN (green)")),
                        (255, 225, 100))
                topology_controls = ((
                    "已完成{}个arm | P=程序检查并展开全部车道 | Enter=确认审核 | X=重做"
                    if cjk else
                    "{} arms | P=program audit/all lanes | Enter=accept review | X=reset").format(
                        complete_arm_count), (120, 230, 255))
                topology_legend = ((
                    "图例：绿色箭头=驶向环岛(IN) | 蓝色箭头=驶离环岛(OUT) | 横线=位置"
                    if cjk else
                    "Legend: green=toward ring (IN) | blue=away (OUT) | bar=position"),
                    (130, 235, 255))
                rows = [
                    compact_header,
                    primary,
                    secondary,
                    topology_controls,
                    topology_legend,
                    undo_row,
                ]
            else:
                rows = [
                    compact_header,
                    current_step_row,
                    (progress, (170, 220, 255)),
                    save_status_row,
                    placement_hints.get(current_step, feedback_row),
                    undo_row,
                ]
            if self.last_feedback_error:
                # An explicit operation failure is more important than normal
                # placement/recommendation hints.  Keep the full wrapped
                # diagnosis visible until the next accepted action.
                rows = [
                    compact_header,
                    current_step_row,
                    (progress, (170, 220, 255)),
                    save_status_row,
                ] + feedback_rows + [undo_row]

        row_height = max(18, panel_font.get_linesize() + 1)
        panel_height = 8 + len(rows) * row_height
        panel_width = min(950 if expanded else 900, self.screen.get_width() - 16)
        panel = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
        panel.fill((0, 0, 0, 205))
        self.screen.blit(panel, (8, 8))
        for index, (text_value, color) in enumerate(rows):
            while text_value and panel_font.size(text_value)[0] > panel_width - 20:
                text_value = text_value[:-2]
            if text_value != rows[index][0]:
                text_value = text_value.rstrip() + "…"
            self.screen.blit(panel_font.render(text_value, True, color),
                             (18, 12 + index * row_height))

    def draw_points(self):
        self.draw_traffic_lights()
        if len(self.ego_route_points) > 0:
            if self.is_roundabout_2b:
                vut_cache = (self._confirmed_roundabout_route(
                    "vut", required=False)
                    if getattr(self, "roundabout_vut_route_confirmed", False)
                    else None)
                if vut_cache:
                    self._draw_confirmed_roundabout_route(
                        vut_cache, ROUNDABOUT_COLOR_VUT)
                else:
                    vut_draft = self._confirmed_roundabout_route(
                        "vut", required=False)
                    if vut_draft:
                        self._draw_roundabout_draft_route(vut_draft)
                    self._draw_route(
                        self.ego_route_points, ROUNDABOUT_COLOR_UNCONFIRMED,
                        "V", width=2, actor_start=self.ego_point, hollow=True,
                        draw_polyline=not bool(vut_draft))
                self._draw_failed_roundabout_route(getattr(
                    self, "roundabout_vut_route_failure", None))
            else:
                point_list = []
                contains_signal = any(self._point_is_signal_route(p) for p in self.ego_route_points)
                route_color = (255, 80, 220) if contains_signal else (0, 255, 128)
                for idx, p in enumerate(self.ego_route_points):
                    sx, sy = self.world_to_screen(p['x'], p['y'])
                    point_list.append((sx, sy))
                    if self._screen_point_visible((sx, sy), margin=160):
                        pygame.draw.circle(
                            self.screen, route_color, (sx, sy),
                            7 if contains_signal else 6)
                        self.screen.blit(
                            self.font.render(f"P{idx+1}", True, route_color),
                            (sx + 8, sy - 18))
                if len(point_list) > 1:
                    self._draw_clipped_polyline(
                        point_list, route_color, 5 if contains_signal else 3)
                if contains_signal:
                    self.screen.blit(
                        self.font.render("SIGNAL-CONTROLLED ROUTE", True, route_color),
                        (20, 20))

        if self.is_roundabout_2b:
            # STEP 1 identities never disappear.  Before acceptance (or with
            # L enabled), draw every expanded parallel lane.  Afterwards keep
            # one coloured IN/OUT gate and arrow per physical arm while H
            # independently controls only the text labels.
            self._draw_roundabout_topology_seeds()
            self._draw_roundabout_topology_hover()
            self._draw_roundabout_placement_recommendations()
            if not self.ego_route_points:
                self._draw_failed_roundabout_route(getattr(
                    self, "roundabout_vut_route_failure", None))
            if (self.ego_point is not None
                    and not getattr(self, "roundabout_vut_route_confirmed", False)):
                self._draw_roundabout_vut_recommendation()
                self._draw_roundabout_trace_failure()
            vt1 = self._roundabout_actor("vt1")
            if vt1 and vt1.get("route_points"):
                vt1_cache = (self._confirmed_roundabout_route(
                    "vt1", required=False)
                    if getattr(self, "roundabout_vt1_route_confirmed", False)
                    else None)
                if vt1_cache:
                    self._draw_confirmed_roundabout_route(
                        vt1_cache, ROUNDABOUT_COLOR_VT1)
                else:
                    vt1_draft = self._confirmed_roundabout_route(
                        "vt1", required=False)
                    if vt1_draft:
                        self._draw_roundabout_draft_route(vt1_draft)
                    self._draw_route(
                        vt1["route_points"], ROUNDABOUT_COLOR_UNCONFIRMED,
                        "T", width=2, actor_start=vt1, hollow=True,
                        draw_polyline=not bool(vt1_draft))
                self._draw_failed_roundabout_route(getattr(
                    self, "roundabout_vt1_route_failure", None))
            elif vt1:
                self._draw_failed_roundabout_route(getattr(
                    self, "roundabout_vt1_route_failure", None))
            preview = None
            if getattr(self, "roundabout_vut_route_confirmed", False):
                preview = self._ensure_roundabout_topology_preview()
            if preview:
                self._draw_roundabout_topology_preview(preview)
            analysis = (self._ensure_roundabout_analysis()
                        if getattr(self, "roundabout_vut_route_confirmed", False)
                        else None)
            if analysis and not preview:
                self._draw_gate(
                    analysis["entry_gate"], ROUNDABOUT_COLOR_VUT, "VUT ENTRY")
                exit_colors = {
                    "1": ROUNDABOUT_COLOR_EXIT_1,
                    "2": ROUNDABOUT_COLOR_EXIT_2,
                    "3": ROUNDABOUT_COLOR_EXIT_3,
                }
                for number, gate in analysis["exit_gates"].items():
                    self._draw_gate(
                        gate, exit_colors[str(number)],
                        "EXIT {}".format(number))
                self._draw_gate(
                    analysis["downstream_entry"], ROUNDABOUT_COLOR_VT2,
                    "DOWNSTREAM ENTRY")

        if self.is_signal_1d:
            label = "1.d  maneuver={}  seed={}  J/K/L=left/straight/right".format(
                self.maneuver, self.base_seed)
            self.screen.blit(self.font.render(label, True, (255, 255, 255)), (20, 50))

        if self.trigger_point:
            x, y = self.world_to_screen(self.trigger_point['x'], self.trigger_point['y'])
            color = (255, 0, 0) if self.selected_mode == 'trigger' else (255, 100, 100)
            if self._screen_point_visible((x, y), margin=160):
                pygame.draw.circle(self.screen, color, (x, y), 10)
                if self._roundabout_map_labels_visible():
                    self.screen.blit(
                        self.font.render("TRIGGER", True, color), (x + 10, y))

        if self.ego_point:
            x, y = self.world_to_screen(self.ego_point['x'], self.ego_point['y'])
            color = (0, 255, 255) if self.selected_mode == 'ego' else (0, 200, 255)
            if self._screen_point_visible((x, y), margin=160):
                pygame.draw.circle(self.screen, color, (x, y), 11)
                ego_label = "VUT" if self.is_roundabout_2b else "EGO"
                if self._roundabout_map_labels_visible():
                    self.screen.blit(
                        self.font.render(ego_label, True, color), (x + 10, y))

        for i, p in enumerate(self.actor_points):
            x, y = self.world_to_screen(p['x'], p['y'])
            if not self._screen_point_visible((x, y), margin=200):
                continue
            is_selected = (i == self.selected_agent_idx)
            atype = p.get("type", "person")

            if self.is_roundabout_2b and p.get("role") == "vt1":
                color = (255, 225, 80) if is_selected else (255, 165, 0)
                label = "VT1 (15 km/h)"
            elif self.is_roundabout_2b and p.get("role") == "vt2":
                color = (255, 140, 255) if is_selected else (180, 60, 230)
                label = "VT2 (STATIONARY)"
            elif atype == "obstacle":
                color = (255, 165, 0) if is_selected else (255, 140, 0)
                label = "OBSTACLE"
            else:
                color = (255, 255, 0) if is_selected else (0, 0, 255)
                label = {"person": "PERSON", "bike": "BIKE", "car": "CAR"}[atype]

            pygame.draw.circle(self.screen, color, (x, y), 9 if atype == "car" else 7)
            if self._roundabout_map_labels_visible():
                self.screen.blit(self.font.render(label, True, color), (x + 10, y))

            yaw = math.radians(p['yaw'])
            dx = math.cos(yaw) * 18
            dy = -math.sin(yaw) * 18
            pygame.draw.line(self.screen, color, (x, y), (x + dx, y + dy), 2)

        # 对向车道直行车辆标记（橙色渲染，用于区分）
        for i, p in enumerate(self.opposite_points):
            x, y = self.world_to_screen(p['x'], p['y'])
            if not self._screen_point_visible((x, y), margin=200):
                continue
            atype = p.get("type", "car")
            color = (255, 165, 0)  # 橙色
            prefix = "OPPO-CAR"
            pygame.draw.circle(self.screen, color, (x, y), 10)
            if self._roundabout_map_labels_visible():
                self.screen.blit(self.font.render(prefix, True, color), (x + 12, y))
            yaw = math.radians(p['yaw'])
            dx = math.cos(yaw) * 18
            dy = -math.sin(yaw) * 18
            pygame.draw.line(self.screen, color, (x, y), (x + dx, y + dy), 2)

        # ego自动驾驶目标点标记（绿色渲染）
        if self.ego_steer_target_point is not None:
            p = self.ego_steer_target_point
            x, y = self.world_to_screen(p['x'], p['y'])
            if self._screen_point_visible((x, y), margin=200):
                color = (0, 255, 0)  # 绿色
                pygame.draw.circle(self.screen, color, (x, y), 12, 2)
                pygame.draw.circle(self.screen, color, (x, y), 4)
                if self._roundabout_map_labels_visible():
                    self.screen.blit(self.font.render("TARGET", True, color), (x + 14, y - 6))
                yaw = math.radians(p['yaw'])
                dx = math.cos(yaw) * 22
                dy = -math.sin(yaw) * 22
                pygame.draw.line(self.screen, color, (x, y), (x + dx, y + dy), 2)

        if self.is_roundabout_2b:
            self._draw_roundabout_panel()

    def set_agent_type(self, atype):
        if self.selected_agent_idx < 0 or self.selected_agent_idx >= len(self.actor_points):
            print("❌ 请先选中一个 AGENT")
            return
        if self.is_roundabout_2b and atype != "car":
            self._feedback("2.b 的 VT1/VT2 必须使用机动车模型；请使用 T/Y 设置角色。", True)
            return
        self.actor_points[self.selected_agent_idx]["type"] = atype
        print(f"✅ AGENT 已切换为：{atype.upper()}")

    def set_roundabout_role(self, role, assign_selected=False):
        if role not in ("vut", "vt1", "vt2"):
            return
        if role == "vut":
            self.roundabout_place_role = "vut"
            self._feedback(
                "放置角色已切换为VUT：请在进口车道Ctrl+左键放置；"
                "放置后用Alt+右键绘制路线。")
            return
        if (assign_selected and self.selected_mode == "agent"
                and 0 <= self.selected_agent_idx < len(self.actor_points)):
            point = self.actor_points[self.selected_agent_idx]
            old_role = point.get("role")
            duplicate = [index for index, actor in enumerate(self.actor_points)
                         if index != self.selected_agent_idx and actor.get("role") == role]
            if duplicate:
                self._feedback(
                    "{} 已存在；请删除现有车辆后再修改角色。".format(role.upper()), True)
                return
            if role == "vt2" and point.get("route_points"):
                self._feedback("VT2必须静止；请先按C清空当前VT1路线，再改为VT2。", True)
                return
            self._push_roundabout_history("将选中车辆角色改为{}".format(role.upper()))
            self.roundabout_place_role = role
            point.update({
                "type": "car",
                "role": role,
                "motion": "route_follow" if role == "vt1" else "stationary",
                "autopilot": False,
            })
            point.setdefault("route_points", [])
            if old_role == "vt1" or role == "vt1":
                self._invalidate_roundabout_route("vt1")
            self._mark_roundabout_dirty()
            self._feedback("已将选中车辆设为 {}。".format(role.upper()))
        else:
            self.roundabout_place_role = role
            if role == "vt1":
                self._feedback(
                    "放置角色已切换为VT1：请在环岛环形行车道上，沿绿色箭头方向位于"
                    "VUT汇入口之前的位置Ctrl+左键；不要放在VUT接入道路。")
            else:
                self._feedback(
                    "放置角色已切换为VT2：沿环流方向找到VUT入口后的下一个接入口，"
                    "在绿色箭头朝向环岛的进口车道Ctrl+左键。"
                    "该入口不等同于VT1的出口1。")

    def add_roundabout_vut(self, wx, wy):
        """Place VUT through the same Ctrl+Left role workflow as VT1/VT2."""
        if not getattr(self, "roundabout_topology_confirmed", False):
            self._feedback(
                "请先完成STEP 0和STEP 1拓扑审核确认，再放置VUT。", True)
            return False
        if self.ego_point is not None:
            self._feedback(
                "VUT已经存在；如需重放请先选中VUT并按Delete。", True)
            return False
        waypoint = self.get_nearest_waypoint(wx, wy)
        if waypoint is None:
            self._feedback("VUT位置没有可用Driving Lane。", True)
            return False
        self._push_roundabout_history("放置VUT")
        self.ego_point = {
            "x": waypoint.transform.location.x,
            "y": waypoint.transform.location.y,
            "z": waypoint.transform.location.z + 0.3,
            "yaw": waypoint.transform.rotation.yaw,
        }
        self.selected_mode = "ego"
        self.selected_agent_idx = -1
        self._invalidate_roundabout_route("vut")
        self._mark_roundabout_dirty()
        recommendation, errors = self._recommend_roundabout_from_vut_start(waypoint)
        self.roundabout_vut_recommendation = recommendation
        self.roundabout_vut_recommendation_errors = errors
        if recommendation:
            exits = recommendation["exit_gates"]
            self._feedback(
                "VUT已放置并关联{}；EXIT 1={}供VT1，VUT可选EXIT 2={}或"
                "EXIT 3={}。请用Alt+右键绘制并穿过目标蓝色OUT门线。".format(
                    recommendation["entry_arm_id"], exits["1"]["arm_id"],
                    exits["2"]["arm_id"], exits["3"]["arm_id"]))
            return True
        self._feedback(
            "VUT已放置，但无法生成可靠出口推荐：{}。请撤销或删除后重放。".format(
                "；".join(errors[:2])), True)
        return False

    def place_roundabout_selected_role(self, wx, wy):
        """Dispatch Ctrl+Left placement for the role selected by V/T/Y."""
        if self.roundabout_place_role == "vut":
            return self.add_roundabout_vut(wx, wy)
        return self.add_roundabout_actor(wx, wy)

    def add_roundabout_actor(self, wx, wy):
        role = self.roundabout_place_role
        if role not in ("vt1", "vt2"):
            self._feedback("请先按T选择VT1或按Y选择VT2。", True)
            return False
        if any(point.get("role") == role for point in self.actor_points):
            other = "VT2" if role == "vt1" else "VT1"
            self._feedback(
                "{} 已存在。点击它可编辑，或按{}选择{}后再放置。".format(
                    role.upper(), "Y" if role == "vt1" else "T", other), True)
            return False
        if len(self.actor_points) >= 2:
            self._feedback("2.b仅允许一个VT1和一个VT2；请先删除不需要的目标车。", True)
            return False
        waypoint = self.get_nearest_waypoint(wx, wy)
        if waypoint is None:
            self._feedback("目标位置没有可用Driving Lane。", True)
            return False
        vt2_placement = None
        if role == "vt2":
            preview = self._ensure_roundabout_topology_preview()
            downstream_arm = (
                preview["entry_order"][0]
                if preview and preview.get("entry_order") else None)
            try:
                waypoint, vt2_placement = self._prepare_vt2_waypoint(
                    waypoint, downstream_arm)
            except RuntimeError as exc:
                self._feedback("VT2不能放在这里：{}".format(exc), True)
                return False
        location = waypoint.transform.location
        point = {
            "x": float(location.x),
            "y": float(location.y),
            "z": float(location.z + 0.3),
            "yaw": float(waypoint.transform.rotation.yaw),
            "type": "car",
            "role": role,
            "motion": "route_follow" if role == "vt1" else "stationary",
            "autopilot": False,
            "route_points": [],
        }
        self._push_roundabout_history("放置{}".format(role.upper()))
        self.actor_points.append(point)
        if role == "vt1":
            self._invalidate_roundabout_route("vt1")
        self.selected_agent_idx = len(self.actor_points) - 1
        self.selected_mode = "agent"
        self._mark_roundabout_dirty()
        if role == "vt1":
            self._feedback(
                "VT1已吸附到环形车道并选中。若位置正确，请用Alt+右键沿环流方向"
                "绘制到出口1下游；目标速度自动设为15 km/h。程序会在路线首次到达"
                "VUT汇入冲突点时提示上游距离，并在STEP 5按Enter时正式校验，不会等到保存。")
        else:
            if vt2_placement and vt2_placement["adjusted"]:
                self._feedback(
                    "VT2点击车道已识别为{}进口方向；点击点在门线之后或过近，"
                    "程序已自动放到同一进口车道门线上游{:.1f}m，lane={}。"
                    "VT2保持静止且不需要路线；按Enter确认STEP 6后按S校验。".format(
                        vt2_placement["arm_id"],
                        vt2_placement["upstream_distance_m"],
                        vt2_placement["lane_key"]))
            else:
                self._feedback(
                    "VT2已验证并吸附到{}下游第1入口的进口车道，lane={}，"
                    "位于门线上游{:.1f}m。VT2保持静止且不需要路线；"
                    "按Enter确认STEP 6后按S校验。".format(
                        vt2_placement["arm_id"],
                        vt2_placement["lane_key"],
                        abs(vt2_placement["gate_signed_distance_m"])))
        return True

    def add_active_route_point(self, wx, wy):
        role = self._active_roundabout_role()
        if role is None:
            self._feedback("请先左键选中VUT或VT1，再用Alt+右键添加该车路线点。", True)
            return False
        if role == "vt2":
            self._feedback("VT2按国标要求保持静止，不能绘制路线；请选择VUT或VT1。", True)
            return False
        if role == "vut":
            route = self.ego_route_points
            label = "VUT"
            actor_start = self.ego_point
        else:
            actor = self.actor_points[self.selected_agent_idx]
            route = actor.setdefault("route_points", [])
            label = "VT1"
            actor_start = actor
        previous = route[-1] if route else (
            self.ego_point if role == "vut" else self.actor_points[self.selected_agent_idx])
        candidates = self._roundabout_click_waypoint_candidates(wx, wy)
        if not candidates:
            self._feedback("该位置附近无法找到Driving Lane候选。", True)
            return False

        existing_cache = getattr(
            self, "roundabout_vut_route_cache" if role == "vut"
            else "roundabout_vt1_route_cache", None)
        existing_length_m = (
            float(existing_cache.get("length_m", 0.0))
            if isinstance(existing_cache, dict) else 0.0)
        existing_lane_changes = (
            sum(self._road_option_value(item) in (5, 6)
                for item in existing_cache.get("options", []))
            if isinstance(existing_cache, dict) else 0)
        accepted = []
        rejected = []
        duplicate_count = 0
        for waypoint, click_distance_m in candidates:
            payload = self._route_anchor_payload(waypoint)
            if route and math.hypot(
                    route[-1]["x"] - payload["x"],
                    route[-1]["y"] - payload["y"]) < 0.2:
                duplicate_count += 1
                continue
            try:
                payload, draft_cache = self._build_roundabout_candidate_extension(
                    role, route, actor_start, waypoint)
                options = draft_cache["options"]
                if (role == "vut"
                        and getattr(
                            self, "roundabout_topology_confirmed", False)
                        and isinstance(getattr(
                            self, "roundabout_vut_recommendation", None), dict)):
                    # Once a draft has actually reached an OUT gate it is
                    # mature enough for the same entry/exit topology check
                    # used by Enter.  Reject the offending click immediately
                    # instead of displaying a plausible route and failing one
                    # workflow step later.
                    draft_arms, _center, _direction = \
                        self._reviewed_roundabout_topology()
                    try:
                        self._route_exit_arm(
                            draft_cache["dense"], draft_arms,
                            route_label="VUT草稿")
                    except RuntimeError:
                        if self._route_touches_directed_out_gate(
                                draft_cache["dense"], draft_arms):
                            # The draft is no longer merely unfinished. It
                            # reached an OUT cross-section but failed its
                            # lane/topology evidence, so report it on this
                            # click instead of delaying it until Enter.
                            raise
                        pass  # Still upstream/in-ring: more points are expected.
                    else:
                        self._topology_preview_from_vut_route(
                            draft_cache["dense"])
            except RuntimeError as exc:
                rejected.append((click_distance_m, waypoint, payload, exc))
                continue

            spacing_m = (math.hypot(
                float(previous["x"]) - float(payload["x"]),
                float(previous["y"]) - float(payload["y"])
            ) if previous else 0.0)
            added_length_m = max(
                spacing_m, draft_cache["length_m"] - existing_length_m)
            detour_m = max(0.0, added_length_m - spacing_m)
            total_lane_changes = sum(
                self._road_option_value(item) in (5, 6)
                for item in options)
            added_lane_changes = max(
                0, total_lane_changes - existing_lane_changes)
            # Reachability is mandatory.  Among reachable candidates, prefer
            # the actual click, then a short continuation with fewer lane
            # changes.  A slightly farther connected lane beats a geometrically
            # nearest but disconnected/opposite connector.
            score = (
                click_distance_m
                + added_lane_changes * ROUNDABOUT_LANE_CHANGE_PENALTY_M
                + detour_m * 0.15)
            accepted.append((
                score, click_distance_m, waypoint, payload,
                draft_cache, spacing_m, added_lane_changes))

        if not accepted:
            if duplicate_count and not rejected:
                self._feedback(
                    "{}路线点与上一点重复，请选择前方车道位置。".format(label), True)
                return False
            click_distance_m, _waypoint, payload, exc = min(
                rejected, key=lambda item: item[0])
            self._set_roundabout_route_failure(
                role, exc,
                segment_start=self._point_payload(previous),
                segment_target=self._point_payload(payload))
            tried = [
                "({},{},{})".format(
                    item[1].road_id, item[1].section_id, item[1].lane_id)
                for item in sorted(rejected, key=lambda item: item[0])[:4]
            ]
            self._feedback(
                "{}候选点P{}已拒绝，路线未改变：已检查附近{}条车道候选{}，"
                "均无法从当前路线连通；最近候选距离鼠标{:.1f}m，原因：{}。"
                "请直接在目标箭头附近重新Alt+右键；无需先撤销。".format(
                    label, len(route) + 1, len(rejected),
                    "/".join(tried), click_distance_m, exc), True)
            return False

        (_score, click_distance_m, waypoint, payload,
         draft_cache, spacing_m, _added_lane_changes) = min(
            accepted, key=lambda item: (item[0], item[1]))
        topology_status = ""
        if role == "vut":
            try:
                draft_arms, _center, _direction = \
                    self._reviewed_roundabout_topology()
                entry_index, entry_arm, _entry_record = self._route_entry_event(
                    draft_cache["dense"], draft_arms)
                topology_status = "；入口状态={}，稠密点{}已穿过IN门线".format(
                    entry_arm["arm_id"], entry_index)
            except RuntimeError:
                topology_status = "；入口状态=尚未穿过推荐IN门线（继续绘制）"
        nearest_token = self._waypoint_token(candidates[0][0])
        selected_token = self._waypoint_token(waypoint)
        auto_resolved = selected_token != nearest_token
        self._push_roundabout_history("添加{}路线点P{}".format(label, len(route) + 1))
        route.append(payload)
        if role == "vut":
            self.roundabout_vut_route_confirmed = False
            self.roundabout_vut_route_cache = draft_cache
            self.roundabout_vut_route_failure = None
            self._invalidate_roundabout_topology_confirmation()
        else:
            self.roundabout_vt1_route_confirmed = False
            self.roundabout_vt1_route_cache = draft_cache
            self.roundabout_vt1_route_failure = None
        self._mark_roundabout_dirty()

        vt1_timing_warning = None
        if role == "vt1":
            # Once the draft reaches the VUT merge, the start-to-conflict
            # distance is final. Report it on that click, before Enter/save.
            try:
                vut_cache = self._confirmed_roundabout_route("vut")
                preview = self._ensure_roundabout_topology_preview(force=True)
                if preview is not None:
                    merge = self._vut_vt1_merge_geometry(
                        vut_cache["dense"], draft_cache["dense"],
                        preview["entry_index"])
                    vt1_timing_warning = self._vt1_upstream_timing_error(
                        self._vt1_upstream_timing(draft_cache["dense"], merge))
            except RuntimeError:
                # Still upstream of the merge: the VT1 draft is incomplete.
                pass
        if spacing_m is not None and spacing_m > ROUTE_ANCHOR_STRAIGHT_MAX_M:
            guidance = "本段直线距离{:.1f}m偏大；若经过分叉/入口/出口，请在其前后补点".format(
                spacing_m)
        elif spacing_m is not None and spacing_m < 3.0:
            guidance = "本段仅{:.1f}m，无需如此密集；通常保持{:.0f}-{:.0f}m".format(
                spacing_m, ROUTE_ANCHOR_RECOMMENDED_MIN_M,
                ROUTE_ANCHOR_RECOMMENDED_MAX_M)
        else:
            guidance = ("本段{:.1f}m；普通路段建议{:.0f}-{:.0f}m，分叉/门线附近5-{:.0f}m".format(
                spacing_m, ROUTE_ANCHOR_RECOMMENDED_MIN_M,
                ROUTE_ANCHOR_RECOMMENDED_MAX_M, ROUTE_ANCHOR_BRANCH_MAX_M)
                if spacing_m is not None else
                "普通路段建议{:.0f}-{:.0f}m，分叉/门线附近5-{:.0f}m".format(
                    ROUTE_ANCHOR_RECOMMENDED_MIN_M,
                    ROUTE_ANCHOR_RECOMMENDED_MAX_M, ROUTE_ANCHOR_BRANCH_MAX_M))
        message = (
            "已添加{}路线点P{}；已检查附近{}条车道候选，选择"
            "road/section/lane=({},{},{})、s={:.1f}，距鼠标{:.1f}m{}；"
            "当前真实草稿{:.1f}m；{}{}。".format(
                label, len(route), len(candidates),
                waypoint.road_id, waypoint.section_id, waypoint.lane_id,
                waypoint.s, click_distance_m,
                "（几何最近车道不可达，已自动改选连通车道）"
                if auto_resolved else "",
                draft_cache["length_m"], guidance, topology_status))
        if vt1_timing_warning:
            message += (
                " 已在当前点击识别到VT1起点问题：{} 延长路线不能修复起点；"
                "请删除并沿环流反方向重新放置VT1。".format(vt1_timing_warning))
        self._feedback(message, bool(vt1_timing_warning))
        return True

    def confirm_roundabout_branch(self):
        """Explicitly accept the currently displayed STEP 0 test branch."""
        if not self.is_roundabout_2b:
            return False
        if getattr(self, "roundabout_branch_confirmed", False):
            self._feedback("测试分支已经确认；请继续当前未完成步骤。")
            return True
        self._push_roundabout_history("确认环岛测试分支")
        self.roundabout_branch_confirmed = True
        branch = ("具备环形路口行驶功能" if self.roundabout_capable
                  else "不具备环形路口行驶功能/ODD")
        self._feedback(
            "STEP 0分支已确认：{}。进入STEP 1：从任一物理arm开始，先在进口车道右键，"
            "再在同一接入方向的出口车道右键；逐臂重复，全部标定后按P。".format(
                branch))
        return True

    def confirm_roundabout_topology(self):
        """Run the P-stage topology audit and expose every detected lane."""
        if not self.is_roundabout_2b:
            return False
        if not getattr(self, "roundabout_branch_confirmed", False):
            self._feedback("请先按Enter确认STEP 0中的测试能力分支。", True)
            return False
        if getattr(self, "roundabout_topology_confirmed", False):
            self._feedback("STEP 1物理arm拓扑已经确认；请继续STEP 2。")
            return True
        if getattr(self, "roundabout_topology_review_ready", False):
            self._feedback(
                "程序拓扑审核已完成；请检查地图上所有绿色IN和蓝色OUT。"
                "确认正确后按Enter进入STEP 2。")
            return True
        seeds = getattr(self, "roundabout_topology_seeds", [])
        complete = [
            arm for arm in seeds if arm.get("inbound") and arm.get("outbound")]
        if len(complete) < 3 or len(complete) != len(seeds):
            self._feedback(
                "STEP 1拓扑无法确认：至少标定3个完整物理arm；每个arm必须先右键进口、"
                "再右键同一接入方向的出口。当前完整={}/总数={}。".format(
                    len(complete), len(seeds)), True)
            return False
        review, errors = self._build_roundabout_topology_review()
        # Keep the failed review as well: its diagnostics identify the exact
        # cross-section that must be highlighted on the map.
        self.roundabout_topology_review = review
        self.roundabout_topology_review_errors = errors
        if errors:
            self._feedback(
                "STEP 1程序拓扑审核未通过：{}。请按Backspace撤销最后标记，"
                "或按X重新标定。".format("；".join(errors[:3])), True)
            return False
        self._push_roundabout_history("生成环岛拓扑审核")
        self.roundabout_topology_review = review
        self.roundabout_topology_review_ready = True
        warning_suffix = ("；另有{}项方向提示，请重点核对".format(
            len(review["warnings"])) if review["warnings"] else "")
        lane_summary = "，".join(
            "{}:IN{} / OUT{}".format(
                arm["arm_id"], len(arm["inbound"]), len(arm["outbound"]))
            for arm in review["arms"])
        self._feedback(
            "STEP 1程序拓扑审核已完成：{}个物理接入方向、{}条Driving Lane。"
            "各方向识别结果={}。地图已展开同一横断面内连续的同向车道并用相同"
            "Axx IN/OUT标记。请逐项检查；错误的程序展开门线可左键选中后按Delete排除；"
            "确认没有问题后按Enter进入STEP 2{}。".format(
                review["arm_count"], review["lane_count"], lane_summary,
                warning_suffix))
        return True

    def accept_roundabout_topology_review(self):
        """Record the operator's Enter acceptance of the visible P-stage audit."""
        if not getattr(self, "roundabout_topology_review_ready", False):
            self._feedback("请先完成STEP 1标定并按P运行程序拓扑审核。", True)
            return False
        self._push_roundabout_history("确认程序拓扑审核")
        self.roundabout_topology_confirmed = True
        self.roundabout_topology_edit_target = None
        self.roundabout_topology_lane_target = None
        review = self.roundabout_topology_review or {}
        try:
            cache_path = self._save_roundabout_topology_cache()
        except (OSError, RuntimeError, ValueError) as exc:
            self.roundabout_topology_confirmed = False
            self._feedback("STEP 1拓扑缓存写入失败：{}".format(exc), True)
            return False
        self.roundabout_place_role = "vut"
        self._feedback(
            "STEP 1审核已确认：{}个arm、{}条Driving Lane；地图绑定缓存已写入{}。"
            "进入STEP 2：按V选择VUT，再Ctrl+左键放置。".format(
                review.get("arm_count", 0), review.get("lane_count", 0),
                cache_path))
        return True

    def confirm_vut_route(self):
        """Explicitly finish STEP 3 after a basic Driving Lane check."""
        if not self.is_roundabout_2b:
            return False
        if self.ego_point is None:
            self._feedback("请先完成STEP 2：在进口车道放置VUT。", True)
            return False
        if len(self.ego_route_points) < 2:
            self._feedback(
                "VUT路线至少需要两个锚点；最后一个锚点必须沿箭头方向穿过"
                "出口2或出口3的蓝色OUT门线并位于其下游。", True)
            return False
        if getattr(self, "roundabout_vut_route_confirmed", False):
            try:
                self._confirmed_roundabout_route("vut")
                self._feedback("VUT路线已经确认；请继续STEP 4放置VT1。")
                return True
            except RuntimeError:
                self._invalidate_roundabout_route("vut")
        try:
            cache = self._confirmed_roundabout_route("vut", required=False)
            if cache is None:
                dense, options = self._densify_route(
                    self.ego_route_points, actor_start=self.ego_point)
                cache = self._build_roundabout_route_cache(
                    "vut", self.ego_route_points, self.ego_point,
                    dense, options)
        except RuntimeError as exc:
            self.roundabout_vut_route_cache = None
            self._set_roundabout_route_failure("vut", exc)
            self._feedback("VUT路线不能确认：{}".format(exc), True)
            return False
        self.roundabout_vut_route_cache = cache
        self.roundabout_vut_route_confirmed = True
        preview = self._ensure_roundabout_topology_preview(force=True)
        if preview is None:
            self.roundabout_vut_route_confirmed = False
            self.roundabout_vut_route_cache = None
            message = "；".join(self.roundabout_topology_preview_errors)
            self._set_roundabout_route_failure("vut", message, cache["payload"])
            self._feedback(
                "VUT路线无法确认：程序不能用STEP 1标定的物理arm推导汇入口和"
                "出口编号：{}。请修正路线或按X重新标定拓扑后再按Enter。".format(
                    "；".join(self.roundabout_topology_preview_errors)), True)
            return False
        preliminary_signed_m = gate_signed_distance(
            self.ego_point, preview["entry_gate"])
        if preliminary_signed_m > -VUT_START_ENTRY_MARGIN_M:
            self.roundabout_vut_route_confirmed = False
            self.roundabout_vut_route_cache = None
            clearance_m = max(0.0, -preliminary_signed_m)
            self._set_roundabout_route_failure(
                "vut", "VUT起点距入口门线余量不足", cache["payload"])
            self._feedback(
                "VUT路线不能确认：起点距离STEP 1审核进口门线只有{:.1f}m，"
                "要求至少{:.1f}m。请把VUT沿进口车道向上游移动至少{:.1f}m后"
                "重新绘制路线。".format(
                    clearance_m, VUT_START_ENTRY_MARGIN_M,
                    VUT_START_ENTRY_MARGIN_M - clearance_m), True)
            return False
        # The flag was temporarily enabled only because topology preview
        # generation requires a confirmed VUT route.  Capture undo state
        # before committing the confirmation.
        self.roundabout_vut_route_confirmed = False
        self.roundabout_vut_route_cache = None
        self._push_roundabout_history("确认VUT路线")
        self.roundabout_vut_route_cache = cache
        self.roundabout_vut_route_failure = None
        self.roundabout_vut_route_confirmed = True
        self.roundabout_place_role = "vt1"
        self._feedback(
            "VUT真实路线已冻结并改为青色显示（{:.1f}m，指纹{}）；保存时将"
            "直接复用，不会重新规划。程序已用STEP 1的{}个arm推导入口和"
            "出口编号。现在进入STEP 4放置VT1。".format(
                cache["length_m"], cache["route_fingerprint"][:8],
                preview["arm_count"]))
        return True

    def confirm_vt1_route(self):
        """Explicitly finish STEP 5 after checking the VT1 route connection."""
        if not self.is_roundabout_2b:
            return False
        vt1 = self._roundabout_actor("vt1")
        if vt1 is None:
            self._feedback("请先完成STEP 4：在环形行车道放置唯一的VT1。", True)
            return False
        route = vt1.get("route_points", [])
        if len(route) < 2:
            self._feedback("VT1路线至少需要两个锚点；请沿环流方向画到出口1下游。", True)
            return False
        if getattr(self, "roundabout_vt1_route_confirmed", False):
            try:
                self._confirmed_roundabout_route("vt1")
                self._feedback("VT1路线已经确认；请继续STEP 6放置VT2。")
                return True
            except RuntimeError:
                self._invalidate_roundabout_route("vt1")
        try:
            cache = self._confirmed_roundabout_route("vt1", required=False)
            if cache is None:
                dense, options = self._densify_route(route, actor_start=vt1)
                cache = self._build_roundabout_route_cache(
                    "vt1", route, vt1, dense, options)
        except RuntimeError as exc:
            self.roundabout_vt1_route_cache = None
            self._set_roundabout_route_failure("vt1", exc)
            self._feedback("VT1路线不能确认：{}".format(exc), True)
            return False
        self.roundabout_vt1_route_cache = cache
        preflight_error = self._vut_vt1_preflight_error()
        if preflight_error:
            self.roundabout_vt1_route_cache = None
            self._set_roundabout_route_failure(
                "vt1", preflight_error, cache["payload"])
            self._feedback(
                "VT1路线不能确认，STEP 5几何预检未通过：{}".format(
                    preflight_error), True)
            return False
        self.roundabout_vt1_route_cache = None
        self._push_roundabout_history("确认VT1路线")
        self.roundabout_vt1_route_cache = cache
        self.roundabout_vt1_route_failure = None
        self.roundabout_vt1_route_confirmed = True
        self.roundabout_place_role = "vt2"
        self._feedback(
            "VT1真实路线已冻结并改为橙色显示（{:.1f}m，指纹{}）；保存时不会"
            "重新规划。VUT最终入口余量检查通过；进入STEP 6：按Y，在沿环流方向的"
            "下游第1接入口进口车道Ctrl+左键放置静止VT2。"
            "其余完整场景条件将在确认VT2时统一预检。".format(
                cache["length_m"], cache["route_fingerprint"][:8]))
        return True

    def confirm_active_roundabout_route(self):
        """Use Enter for the selected route, with workflow-aware fallback."""
        if not getattr(self, "roundabout_branch_confirmed", False):
            return self.confirm_roundabout_branch()
        if not getattr(self, "roundabout_topology_confirmed", False):
            if getattr(self, "roundabout_topology_review_ready", False):
                return self.accept_roundabout_topology_review()
            self._feedback(
                "请先完成STEP 1的IN/OUT标定并按P运行程序拓扑审核。", True)
            return False
        role = self._active_roundabout_role()
        if role == "vut":
            return self.confirm_vut_route()
        if role == "vt1":
            return self.confirm_vt1_route()
        if role == "vt2":
            # A newly placed actor remains selected.  Pressing Enter at the
            # end of STEP 6 is therefore a natural acknowledgement of VT2,
            # not an attempt to draw or validate a route for the stationary
            # target.  Keep this informational and direct the operator to the
            # next real action instead of reporting a false validation error.
            if (getattr(self, "roundabout_vut_route_confirmed", False)
                    and getattr(self, "roundabout_vt1_route_confirmed", False)):
                analysis = self._ensure_roundabout_analysis(force=True)
                if analysis is None:
                    self._feedback(
                        "STEP 6完成后综合预检未通过：{}。请先修正，再进入STEP 7；"
                        "按S不应作为首次发现该问题的步骤。".format(
                            "；".join(self.roundabout_errors[:3])), True)
                    return False
                self._feedback(
                    "STEP 6已完成：VT2保持静止且不需要路线；完整场景预检通过。"
                    "请按S生成YAML配置的天气/车型场景。")
                return True
            self._feedback(
                "VT2保持静止且不需要路线；请先完成尚未确认的VUT或VT1路线。")
            return True
        if not getattr(self, "roundabout_vut_route_confirmed", False):
            return self.confirm_vut_route()
        if not getattr(self, "roundabout_vt1_route_confirmed", False):
            return self.confirm_vt1_route()
        self._feedback("VUT和VT1路线均已确认。")
        return True

    def add_ego_route_point(self, wx, wy):
        if self.is_roundabout_2b:
            self.add_active_route_point(wx, wy)
            return
        wp = self.get_nearest_waypoint(wx, wy)
        if not wp: return
        self.ego_route_points.append({
            'x': round(wp.transform.location.x, 2),
            'y': round(wp.transform.location.y, 2),
            'z': round(wp.transform.location.z + 0.2, 2),
            'yaw': round(wp.transform.rotation.yaw, 2)
        })
        print(f"✅ 已添加路线点 {len(self.ego_route_points)}")

    def clear_ego_route(self):
        if self.is_roundabout_2b:
            role = self._active_roundabout_role()
            if role == "vut":
                if not self.ego_route_points:
                    self._feedback("当前VUT路线已经为空。", True)
                    return
                self._push_roundabout_history("清空VUT路线")
                self.ego_route_points.clear()
                self._invalidate_roundabout_route("vut")
                self._mark_roundabout_dirty()
                self._feedback("已清空当前VUT路线；VUT起点保留。")
            elif role == "vt1":
                route = self.actor_points[self.selected_agent_idx].setdefault(
                    "route_points", [])
                if not route:
                    self._feedback("当前VT1路线已经为空。", True)
                    return
                self._push_roundabout_history("清空VT1路线")
                route.clear()
                self._invalidate_roundabout_route("vt1")
                self._mark_roundabout_dirty()
                self._feedback("已清空当前VT1路线；VT1位置保留。")
            elif role == "vt2":
                self._feedback("VT2没有运动路线可清除。", True)
            else:
                self._feedback("请先选择VUT或VT1，再按C清空其路线。", True)
            return
        self.ego_route_points.clear()
        print("🗑️ 已清空路线")

    def set_maneuver(self, maneuver):
        if maneuver not in MANEUVERS:
            return
        self.maneuver = maneuver
        pygame.display.set_caption(self._window_caption())
        print("✅ 1.d 路线类型已切换为：{}".format(maneuver))

    def _serialized_actors(self):
        actors = []
        model_map = {
            "person": "walker.pedestrian.0001",
            "bike": "vehicle.diamondback.century",
            "car": "vehicle.tesla.model3",
            "obstacle": "static.prop.container",
        }
        for point in self.actor_points:
            actor_type = point.get("type", "person")
            actors.append({
                "type": actor_type,
                "model": model_map[actor_type],
                "transform": {
                    "pitch": "0.00",
                    "x": "{:.2f}".format(round(point['x'], 2)),
                    "y": "{:.2f}".format(round(point['y'], 2)),
                    "yaw": "{:.2f}".format(round(point['yaw'], 2)),
                    "z": "{:.2f}".format(round(point['z'], 2)),
                },
                "rolename": actor_type,
                "autopilot": True,
            })
        return actors

    @staticmethod
    def _location_dict(location):
        return {
            "x": round(float(location.x), 4),
            "y": round(float(location.y), 4),
            "z": round(float(location.z), 4),
        }

    def save_1d_matrix(self):
        """Generate the reproducible three-trial matrix for one drawn route."""
        if not self.ego_point or not self.trigger_point:
            print("❌ 1.d 必须设置 EGO 和交通灯提示点（Shift+左键）")
            return
        if len(self.ego_route_points) < 2:
            print("❌ 1.d 至少需要两个 Ego 路线点")
            return

        try:
            signal = self._select_route_traffic_light()
        except RuntimeError as exc:
            print("❌ {}".format(exc))
            return

        route_geometry = {
            "town": self.town_name,
            "maneuver": self.maneuver,
            "ego_start": self.ego_point,
            "ego_route": self.ego_route_points,
            "stop_line": {
                "left": self._location_dict(signal['left']),
                "right": self._location_dict(signal['right']),
                "center": self._location_dict(signal['stop_center']),
                "approach_yaw": round(float(signal['approach_yaw']), 4),
            },
        }
        route_fingerprint = canonical_hash(route_geometry)
        matrix_id = route_fingerprint[:12]
        route_key = "{}:{}:{}".format(
            self.town_name, self.maneuver, route_fingerprint)
        cases, matrix_seed = generate_signal_case_matrix(self.base_seed, route_key, 3)
        fixed_weather = {
            "cloudiness": 0.0,
            "dust_storm": 0.0,
            "fog_density": 0.0,
            "fog_distance": 100.0,
            "fog_falloff": 1.0,
            "mie_scattering_scale": 0.03,
            "precipitation": 0.0,
            "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 1.0,
            "sun_altitude_angle": 60.0,
            "sun_azimuth_angle": 120.0,
            "wetness": 0.0,
            "wind_intensity": 5.0,
        }
        route_payload = [
            {"x": p['x'], "y": p['y'], "z": p['z'], "yaw": p['yaw']}
            for p in self.ego_route_points
        ]
        entries = []
        for trial_index, signal_case in enumerate(cases, start=1):
            trial_seed = int(canonical_hash({
                "matrix_seed": matrix_seed,
                "trial_index": trial_index,
                "signal_case": signal_case,
            })[:16], 16) % (2 ** 32)
            scenario_name = "scenario_1d_{:04d}_{}_trial_{:02d}_{}".format(
                self.route_index, self.maneuver, trial_index, signal_case)
            signal_test = {
                "schema_version": SCHEMA_VERSION,
                "matrix_id": matrix_id,
                "maneuver": self.maneuver,
                "signal_case": signal_case,
                "trigger_distance_m": 50.0,
                "valid_trigger_range_m": [40.0, 60.0],
                "yellow_duration_s": 3.0,
                "red_duration_s": 30.0,
                "speed_limit_kmh": 40.0,
                "vehicle_category": "passenger",
                "passenger_stop_gap_max_m": 2.0,
                "passenger_restart_max_s": 3.0,
                "stop_speed_threshold_mps": 0.1,
                "start_speed_threshold_mps": 0.5,
                "unjustified_stop_duration_s": 1.0,
                "stop_line": route_geometry["stop_line"],
                "traffic_light_selector": {
                    "actor_location": self._location_dict(signal['actor_location']),
                    "trigger_location": self._location_dict(signal['trigger_location']),
                    "road_id": int(signal['road_id']),
                    "lane_id": int(signal['lane_id']),
                },
                "reproducibility": {
                    "base_seed": self.base_seed,
                    "matrix_seed": matrix_seed,
                    "trial_seed": trial_seed,
                    "matrix_sequence": cases,
                    "trial_index": trial_index,
                    "route_fingerprint": route_fingerprint,
                    "generator_version": SCHEMA_VERSION,
                },
            }
            scenario_config = {
                "name": scenario_name,
                "ego_start": {
                    "x": "{:.2f}".format(round(self.ego_point['x'], 2)),
                    "y": "{:.2f}".format(round(self.ego_point['y'], 2)),
                    "z": "{:.2f}".format(round(self.ego_point['z'], 2)),
                    "yaw": "{:.2f}".format(round(self.ego_point['yaw'], 2)),
                },
                "ego_route": route_payload,
                "weather": fixed_weather,
                "trigger_position": {
                    "pitch": "0.0",
                    "x": "{:.2f}".format(round(self.trigger_point['x'], 2)),
                    "y": "{:.2f}".format(round(self.trigger_point['y'], 2)),
                    "yaw": "{:.2f}".format(round(self.trigger_point['yaw'], 2)),
                    "z": "{:.2f}".format(round(self.trigger_point['z'], 2)),
                },
                "trigger_radius": 2.0,
                "other_actors": {"center": self._serialized_actors()},
                "signal_test": signal_test,
                "timeout": 90.0,
                "active": True,
            }
            document = {self.town_name: {ROUTE_ID: [scenario_config]}}
            filename = scenario_name + ".json"
            path = os.path.join(self.save_dir, filename)
            with open(path, "w", encoding="utf-8") as stream:
                json.dump(document, stream, indent=2, ensure_ascii=False, sort_keys=True)
            entries.append({
                "file": filename,
                "config_sha256": canonical_hash(document),
                "matrix_id": matrix_id,
                "matrix_seed": matrix_seed,
                "trial_seed": trial_seed,
                "route_fingerprint": route_fingerprint,
                "maneuver": self.maneuver,
                "trial_index": trial_index,
                "signal_case": signal_case,
            })
            print("✅ 1.d {}/3：{}".format(trial_index, filename))

        manifest = update_matrix_manifest(self.save_dir, entries, self.base_seed)
        print("🎉 同路线矩阵已生成：{}；清单：{}".format(cases, manifest))
        self.clear_all_points()
        self.ego_route_points.clear()
        self.route_index += 1

    def _roundabout_variant_conditions(self):
        """Expand configured weather and vehicle profiles for one drawn route."""
        configured = getattr(self, "roundabout_variants", None) or {}
        weather_profiles = configured.get("weather") or [
            {"id": "clear_day", "preset": "sunny", "level": 7}]
        vehicle_profiles = configured.get("vehicles") or [{
            "id": "tesla_model3",
            "vt1_model": "vehicle.tesla.model3",
            "vt2_model": "vehicle.tesla.model3",
        }]
        conditions = []
        condition_ids = set()
        for weather_profile in weather_profiles:
            weather_id = str(weather_profile["id"])
            if isinstance(weather_profile.get("parameters"), dict):
                weather = copy.deepcopy(weather_profile["parameters"])
            else:
                preset = str(weather_profile["preset"])
                level = int(weather_profile.get("level", 1))
                choices = self.weather_groups.get(preset)
                if choices is None:
                    raise ValueError("unknown weather preset: {}".format(preset))
                if level < 1 or level > len(choices):
                    raise ValueError(
                        "weather profile {} level must be 1..{}".format(
                            weather_id, len(choices)))
                weather = copy.deepcopy(choices[level - 1])
            for vehicle_profile in vehicle_profiles:
                condition_id = "{}--{}".format(
                    weather_id, str(vehicle_profile["id"]))
                if condition_id in condition_ids:
                    raise ValueError(
                        "duplicate weather/vehicle condition: {}".format(
                            condition_id))
                condition_ids.add(condition_id)
                conditions.append({
                    "condition_id": condition_id,
                    "weather_id": weather_id,
                    "weather": weather,
                    "vehicle_id": str(vehicle_profile["id"]),
                    "vt1_model": str(vehicle_profile["vt1_model"]),
                    "vt2_model": str(vehicle_profile["vt2_model"]),
                })
        return conditions

    def _validate_roundabout_vehicle_blueprints(self):
        """Fail before editing if a configured target model is unavailable."""
        available = {
            blueprint.id for blueprint in
            self.world.get_blueprint_library().filter("vehicle.*")
        }
        configured = getattr(self, "roundabout_variants", None) or {}
        requested = set()
        for profile in configured.get("vehicles") or []:
            if isinstance(profile, dict):
                requested.update((
                    str(profile.get("vt1_model", "")),
                    str(profile.get("vt2_model", "")),
                ))
        requested.discard("")
        missing = sorted(requested - available)
        if missing:
            raise ValueError(
                "2.b车型配置不属于当前CARLA车辆蓝图库：{}。"
                "请修改config/roundabout_2b.yaml中的vehicle_profiles。".format(
                    ", ".join(missing)))

    @staticmethod
    def _actor_transform(point):
        return {
            "pitch": 0.0,
            "x": round(float(point["x"]), 4),
            "y": round(float(point["y"]), 4),
            "yaw": round(float(point["yaw"]), 4),
            "z": round(float(point["z"]), 4),
        }

    def _serialized_roundabout_actors(
            self, analysis, vt1_route,
            vt1_model="vehicle.tesla.model3",
            vt2_model="vehicle.tesla.model3"):
        vt1 = analysis["vt1"]
        vt2 = analysis["vt2"]
        return [
            {
                "type": "car",
                "model": vt1_model,
                "transform": self._actor_transform(vt1),
                "role": "vt1",
                "rolename": "vt1",
                "route": vt1_route,
                "motion": "route_follow",
                "target_speed_kmh": 15.0,
                "autopilot": False,
            },
            {
                "type": "car",
                "model": vt2_model,
                "transform": self._actor_transform(vt2),
                "role": "vt2",
                "rolename": "vt2",
                "route": [],
                "motion": "stationary",
                "autopilot": False,
            },
        ]

    @staticmethod
    def _roundabout_lane_text(value):
        if not isinstance(value, dict):
            return "未知"
        try:
            return "(road={}, section={}, lane={})".format(
                int(value["road_id"]), int(value["section_id"]),
                int(value["lane_id"]))
        except (KeyError, TypeError, ValueError):
            return "未知"

    def _roundabout_save_validation_feedback(self, error, scenario_config):
        """Translate schema failures into operator-facing placement evidence."""
        raw = str(error)
        mismatch = (
            "roundabout_test.vt2 road_id/section_id/lane_id must match "
            "a downstream_entry inbound lane")
        if mismatch in raw:
            section = scenario_config.get("roundabout_test", {})
            vt2 = section.get("vt2", {})
            downstream = section.get("downstream_entry", {})
            actual = self._roundabout_lane_text(vt2)
            allowed = ", ".join(
                self._roundabout_lane_text(item)
                for item in downstream.get("inbound_lanes", [])) or "无"
            return (
                "VT2车道记录不一致：实际吸附车道={}；下游第1入口允许车道={}。"
                "这是放置阶段的CARLA拓扑结果与保存字段未同步，不表示你给VT2画了路线。"
                "请重新按Y、Ctrl+左键放置；若仍出现请保留此车道编号。".format(
                    actual, allowed))
        return "生成配置未通过一致性校验：{}".format(raw)

    def save_roundabout_trials(self):
        """Save one drawn route as configured weather/vehicle scenario variants.

        This editor boundary creates scenario definitions, not repeated test
        executions.  The formal launcher expands each definition into the
        three independent GB/T repetitions when an experiment is started.
        """
        # S is the explicit submission boundary: only from here should the
        # panel turn missing/invalid configuration into validation feedback.
        self.roundabout_validation_attempted = True
        if not getattr(self, "roundabout_branch_confirmed", False):
            self._feedback(
                "2.b无法保存：请先完成STEP 0，按Enter确认当前测试分支，或按R切换。", True)
            return
        if not getattr(self, "roundabout_topology_confirmed", False):
            self._feedback(
                "2.b无法保存：请先在STEP 1逐臂右键标定进口和对应出口，"
                "至少完成3个物理arm，按P运行程序审核，再检查地图并按Enter确认。", True)
            return
        if not getattr(self, "roundabout_vut_route_confirmed", False):
            self._feedback("2.b无法保存：请画完VUT出口路线后按Enter确认并进入STEP 4。", True)
            return
        if not getattr(self, "roundabout_vt1_route_confirmed", False):
            self._feedback("2.b无法保存：请画完VT1出口1路线后按Enter确认并进入STEP 6。", True)
            return
        analysis = self._ensure_roundabout_analysis(force=True)
        if analysis is None:
            self._feedback(
                "2.b无法保存：{}".format(
                    "；".join(self.roundabout_errors[:4]) or "拓扑校验未通过"), True)
            return

        try:
            vut_cache = self._confirmed_roundabout_route("vut")
            vt1_cache = self._confirmed_roundabout_route("vt1")
        except RuntimeError as exc:
            self._feedback("2.b无法保存：{}".format(exc), True)
            return
        vut_route = copy.deepcopy(vut_cache["payload"])
        vt1_route = copy.deepcopy(vt1_cache["payload"])
        analysed_vut = [
            self._route_item_payload(waypoint, road_option)
            for waypoint, road_option in zip(
                analysis["vut_dense"], analysis["vut_route_options"])]
        analysed_vt1 = [
            self._route_item_payload(waypoint, road_option)
            for waypoint, road_option in zip(
                analysis["vt1_dense"], analysis["vt1_route_options"])]
        if (roundabout_hash(analysed_vut) != vut_cache["route_fingerprint"]
                or roundabout_hash(analysed_vt1)
                != vt1_cache["route_fingerprint"]):
            self._feedback(
                "2.b无法保存：保存前分析路线与Enter确认时显示的路线不一致；"
                "请重新确认VUT和VT1路线。", True)
            return
        vt2_wp = analysis["vt2_waypoint"]
        downstream_arm = analysis["downstream_arm"]
        vt2_lane = self._waypoint_lane_descriptor(vt2_wp)
        conflict_headway_s = analysis.get(
            "vt1_conflict_headway_target_s",
            getattr(self, "roundabout_conflict_headway_s",
                    VT1_CONFLICT_HEADWAY_TARGET_S))
        conflict_tolerance_s = analysis.get(
            "vt1_conflict_headway_tolerance_s",
            getattr(self, "roundabout_conflict_tolerance_s",
                    VT1_CONFLICT_HEADWAY_TOLERANCE_S))
        conflict_sync = analysis.get("vt1_conflict_sync") or \
            roundabout_conflict_sync_distances(
                analysis.get(
                    "vut_sync_approach_distance_m",
                    max(0.0, analysis["vut_approach_distance_m"])),
                target_speed_kmh=15.0,
                vut_approach_speed_mps=
                analysis["vut_approach_speed_assumption_kmh"] / 3.6,
                target_headway_s=conflict_headway_s,
                headway_tolerance_s=conflict_tolerance_s,
                entry_upstream_min_distance_m=
                analysis["vt1_entry_upstream_minimum_m"])
        engineering = {
            "parameter_source": "engineering_fixed_not_gbt41798_original",
            "vt1_speed_tolerance_kmh": 1.0,
            "vt1_stable_duration_s": 1.0,
            "vt1_stabilization_timeout_s": 30.0,
            "vt1_initial_upstream_min_distance_m": round(
                analysis["vt1_initial_upstream_minimum_m"], 4),
            "vt1_entry_upstream_min_distance_m": round(
                analysis["vt1_entry_upstream_minimum_m"], 4),
            "vt1_setup_distance_budget_s": round(
                analysis["vt1_setup_budget_s"], 4),
            "vt1_timing_buffer_m": round(
                analysis["vt1_timing_buffer_m"], 4),
            "vut_approach_speed_assumption_kmh": round(
                analysis["vut_approach_speed_assumption_kmh"], 4),
            "vut_approach_time_budget_s": round(
                analysis["vut_approach_time_budget_s"], 4),
            "vut_approach_distance_m": round(
                analysis["vut_approach_distance_m"], 4),
            "vut_sync_approach_distance_m": round(
                analysis.get(
                    "vut_sync_approach_distance_m",
                    max(0.0, analysis["vut_approach_distance_m"])), 4),
            "vt1_conflict_headway_target_s": round(
                conflict_headway_s, 4),
            "vt1_conflict_headway_tolerance_s": round(
                conflict_tolerance_s, 4),
            "vt1_release_remaining_m": round(
                conflict_sync["vt1_release_remaining_m"], 4),
            "vt1_conflict_entry_gap_target_m": round(
                conflict_sync["target_entry_gap_m"], 4),
            "vt1_conflict_entry_gap_min_m": round(
                conflict_sync["minimum_entry_gap_m"], 4),
            "vt1_conflict_entry_gap_max_m": round(
                conflict_sync["maximum_entry_gap_m"], 4),
            "vut_start_entry_gate_margin_m": round(
                analysis["vut_start_entry_margin_m"], 4),
            "vt1_upstream_max_distance_m": round(
                max(60.0, analysis.get(
                    "vt1_route_length_m",
                    sum(self._distance_locations(
                        first.transform.location, second.transform.location)
                        for first, second in zip(
                            analysis["vt1_dense"], analysis["vt1_dense"][1:])))
                    + 10.0), 4),
            "vt1_conflict_max_offset_m": 12.0,
            "vt1_speed_continuity_duration_s": 0.5,
            "vt1_exit_observation_timeout_s": 20.0,
            "vt1_post_exit_clearance_distance_m": round(
                getattr(
                    self, "roundabout_vt1_post_exit_clearance_distance_m", 25.0),
                4),
            "vt2_stationary_speed_threshold_mps": 0.1,
            "vt2_initial_upstream_distance_m": VT2_DEFAULT_UPSTREAM_DISTANCE_M,
            "vt2_minimum_gate_clearance_m": VT2_MINIMUM_GATE_CLEARANCE_M,
            "vt2_downstream_entry_max_distance_m": 100.0,
            "stop_speed_threshold_mps": 0.1,
            "stop_duration_s": 1.0,
            "emergency_brake_deceleration_mps2": 4.0,
            "emergency_brake_threshold": 0.8,
            "emergency_brake_duration_s": 0.2,
            "speed_limit_tolerance_kmh": 0.5,
            "speed_limit_unobservable_duration_s": 0.5,
            "speed_limit_by_road_id": analysis["speed_limit_by_road_id"],
            "speed_limit_source": analysis["speed_limit_source"],
            "route_completion_distance_m": 3.0,
            "exit_completion_distance_m": 2.0,
            "entry_arrival_distance_m": round(max(
                VUT_ENTRY_ARRIVAL_DISTANCE_M,
                conflict_sync["entry_arrival_distance_m"]), 4),
            "lane_guidance_corridor_extra_m": 0.75,
            "lane_guidance_corridor_min_half_width_m": 1.75,
            "lane_guidance_corridor_max_half_width_m": 4.0,
            "gate_margin_m": 1.0,
            "indicator_lookback_s": 3.0,
            "lane_guidance_violation_duration_s": 0.5,
            "off_road_violation_duration_s": 0.2,
            "incapable_observation_s": 10.0,
            "scenario_timeout_s": 120.0,
            "timeline_sample_interval_s": 0.05,
            "camera_frame_timeout_s": 0.5,
        }
        entry_center = analysis["entry_gate"]["center"]
        entry_yaw = analysis["entry_gate"]["approach_yaw"]
        roundabout_test = {
            "schema_version": ROUNDABOUT_SCHEMA_VERSION,
            "standard_clause": "GB/T 41798-2022 6.2.2",
            "roundabout_capable": bool(self.roundabout_capable),
            "planned_exit": int(analysis["planned_exit"]),
            "entry_gate": analysis["entry_gate"],
            "exit_gates": analysis["exit_gates"],
            "downstream_entry": analysis["downstream_entry"],
            "topology": analysis["topology"],
            "topology_hash": analysis["topology_hash"],
            "vut": {
                "role": "vut",
                "route": vut_route,
                "route_fingerprint": vut_cache["route_fingerprint"],
            },
            "vt1": {
                "role": "vt1",
                "route": vt1_route,
                "route_fingerprint": vt1_cache["route_fingerprint"],
                "target_speed_kmh": 15.0,
                "exit_number": 1,
                "initial_upstream_path_m": round(
                    analysis.get(
                        "vt1_upstream_path_m",
                        analysis["vt1_upstream_arc_m"]), 4),
                # Compatibility alias for older readers.  The value is now
                # measured along VT1's open route, never along a fitted arc.
                "initial_upstream_arc_m": round(analysis["vt1_upstream_arc_m"], 4),
            },
            "vt2": {
                "role": "vt2",
                "motion": "stationary",
                "transform": self._actor_transform(analysis["vt2"]),
                "arm_id": downstream_arm["arm_id"],
                "road_id": int(vt2_wp.road_id),
                "section_id": int(vt2_wp.section_id),
                "lane_id": int(vt2_wp.lane_id),
                "lane": vt2_lane,
            },
            "engineering": engineering,
        }
        base_config = {
            "name": "scenario_2b_{:04d}".format(self.route_index),
            "ego_start": self._actor_transform(self.ego_point),
            "ego_role": "vut",
            "ego_route": vut_route,
            "weather": {},
            # Compatibility alias for legacy readers. It is automatically
            # derived from entry_gate and never manually required for 2.b.
            "trigger_position": {
                "pitch": 0.0,
                "x": entry_center["x"],
                "y": entry_center["y"],
                "yaw": entry_yaw,
                "z": entry_center["z"],
            },
            "trigger_radius": 0.0,
            "other_actors": {"center": []},
            "roundabout_test": roundabout_test,
            "timeout": engineering["scenario_timeout_s"],
            "active": True,
        }
        try:
            variants = self._roundabout_variant_conditions()
        except (KeyError, TypeError, ValueError) as exc:
            self._feedback("2.b场景变体配置无效：{}".format(exc), True)
            return

        def build_variant_config(variant):
            scenario_config = copy.deepcopy(base_config)
            scenario_name = "scenario_2b_{:04d}_{}".format(
                self.route_index, variant["condition_id"])
            scenario_config["name"] = scenario_name
            scenario_config["weather"] = copy.deepcopy(variant["weather"])
            scenario_config["other_actors"]["center"] = \
                self._serialized_roundabout_actors(
                    analysis, vt1_route,
                    variant["vt1_model"], variant["vt2_model"])
            condition_fingerprint = roundabout_hash(
                roundabout_condition_payload(scenario_config))
            route_fingerprint = roundabout_route_fingerprint(scenario_config)
            scenario_config["roundabout_test"]["generation"] = {
                "mode": "scenario_definition",
                "base_seed": self.base_seed,
                "route_id": "route_{:04d}".format(self.route_index),
                "route_index": self.route_index,
                "route_fingerprint": route_fingerprint,
                "condition_id": variant["condition_id"],
                "condition_fingerprint": condition_fingerprint,
                "weather_profile": variant["weather_id"],
                "vehicle_profile": variant["vehicle_id"],
            }
            return scenario_name, scenario_config, condition_fingerprint

        # Validate every combination first, but do not retain hundreds of
        # duplicate dense route documents in memory.  The second pass rebuilds
        # and writes each already-proven deterministic configuration.
        prepared = []
        for variant in variants:
            scenario_name, scenario_config, condition_fingerprint = \
                build_variant_config(variant)
            try:
                validate_roundabout_config(
                    scenario_config, require_reproducibility=False)
            except ValueError as exc:
                self._feedback(
                    "2.b保存前校验失败：{}".format(
                        self._roundabout_save_validation_feedback(
                            exc, scenario_config)), True)
                return
            prepared.append((scenario_name + ".json", variant,
                             condition_fingerprint))

        entries = []
        total_variants = len(prepared)
        for generated_index, (filename, variant, expected_fingerprint) in enumerate(
                prepared, start=1):
            _, scenario_config, condition_fingerprint = build_variant_config(variant)
            if condition_fingerprint != expected_fingerprint:
                self._feedback(
                    "2.b场景生成不稳定：{}的条件指纹发生变化。".format(filename), True)
                return
            document = {self.town_name: {ROUTE_ID: [scenario_config]}}
            path = os.path.join(self.save_dir, filename)
            with open(path, "w", encoding="utf-8") as stream:
                json.dump(document, stream, indent=2, ensure_ascii=False, sort_keys=True)
            entries.append({
                "file": filename,
                "base_seed": self.base_seed,
                "config_sha256": roundabout_hash(document),
                "route_id": scenario_config["roundabout_test"]["generation"][
                    "route_id"],
                "route_fingerprint": scenario_config["roundabout_test"][
                    "generation"]["route_fingerprint"],
                "condition_id": variant["condition_id"],
                "condition_fingerprint": condition_fingerprint,
                "weather_profile": variant["weather_id"],
                "vehicle_profile": variant["vehicle_id"],
                "topology_hash": analysis["topology_hash"],
                "planned_exit": analysis["planned_exit"],
                "roundabout_capable": bool(self.roundabout_capable),
            })
            if (total_variants <= 20 or generated_index == 1
                    or generated_index == total_variants
                    or generated_index % 100 == 0):
                self._feedback("2.b场景生成进度 {}/{}：{}".format(
                    generated_index, total_variants, filename))
        manifest_path = os.path.join(self.save_dir, "2b_scenario_manifest.json")
        merged_entries = {}
        if os.path.isfile(manifest_path):
            try:
                with open(manifest_path, "r", encoding="utf-8") as stream:
                    previous_manifest = json.load(stream)
                for entry in previous_manifest.get("entries", []):
                    if isinstance(entry, dict) and entry.get("file"):
                        merged_entries[entry["file"]] = entry
            except (OSError, TypeError, ValueError):
                merged_entries = {}
        for entry in entries:
            merged_entries[entry["file"]] = entry
        manifest_document = {
            "schema_version": ROUNDABOUT_SCHEMA_VERSION,
            "generation_mode": "scenario_definitions",
            "route_index": self.route_index,
            "scenario_count": len(merged_entries),
            "last_variant_count": len(entries),
            "entries": [merged_entries[name] for name in sorted(merged_entries)],
        }
        with open(manifest_path, "w", encoding="utf-8") as stream:
            json.dump(
                manifest_document, stream, indent=2,
                ensure_ascii=False, sort_keys=True)
        output_dir = os.path.abspath(self.save_dir)
        manifest_path = os.path.abspath(manifest_path)
        generated_paths = [
            os.path.join(output_dir, entry["file"]) for entry in entries]
        self.last_roundabout_output_dir = output_dir
        print("\n✅ 2.b场景定义已写入（正式执行时再展开重复次数）：")
        if len(generated_paths) <= 20:
            for generated_path in generated_paths:
                print("  - {}".format(generated_path))
        else:
            for generated_path in generated_paths[:5]:
                print("  - {}".format(generated_path))
            print("  ... 共{}个场景；其余见manifest".format(len(generated_paths)))
            for generated_path in generated_paths[-2:]:
                print("  - {}".format(generated_path))
        print("  - manifest: {}".format(manifest_path))
        self.clear_all_points()
        self.ego_route_points.clear()
        self.roundabout_dirty = True
        self.route_index += 1
        self._feedback(
            "2.b已从同一路线生成{}个天气/车型场景：{}；manifest={}。"
            "正式实验的3次重复不在编辑器中生成。".format(
                len(entries), output_dir, manifest_path))

    # ======================
    # ✅ 保存单个天气文件
    # ======================
    def save_single(self, weather_name, level_idx, weather_params):
        actors = []
        model_map = {
            "person": "walker.pedestrian.0001",
            "bike": "vehicle.diamondback.century",
            "car": "vehicle.tesla.model3",
            "obstacle": "static.prop.container"
        }
        for p in self.actor_points:
            atype = p.get("type", "person")
            model = model_map[atype]
            actors.append({
                "type": atype, "model": model,
                "transform": {
                    "pitch": "0.00",
                    "x": f"{round(p['x'],2):.2f}",
                    "y": f"{round(p['y'],2):.2f}",
                    "yaw": f"{round(p['yaw'],2):.2f}",
                    "z": f"{round(p['z'],2):.2f}"
                },
                "rolename": atype, "autopilot": True
            })

        # 对向车道直行车辆
        opposite_actors = []
        for p in self.opposite_points:
            atype = p.get("type", "car")
            model = model_map.get(atype, "vehicle.tesla.model3")
            opposite_actors.append({
                "type": atype, "model": model,
                "transform": {
                    "pitch": "0.00",
                    "x": f"{round(p['x'],2):.2f}",
                    "y": f"{round(p['y'],2):.2f}",
                    "yaw": f"{round(p['yaw'],2):.2f}",
                    "z": f"{round(p['z'],2):.2f}"
                },
                "rolename": atype, "autopilot": True
            })

        ego_route_save = [{"x": p['x'], "y": p['y'], "z": p['z'], "yaw": p['yaw']} for p in self.ego_route_points]

        fname = f"scenario_{self.scenario}_{self.route_index:04d}_{weather_name}_{level_idx+1:02d}.json"
        path = os.path.join(self.save_dir, fname)

        data = {
            self.town_name: {
                ROUTE_ID: [{
                    "name": f"scenario_{self.scenario}_{self.route_index:04d}",
                    "ego_start": {
                        "x": f"{round(self.ego_point['x'],2):.2f}",
                        "y": f"{round(self.ego_point['y'],2):.2f}",
                        "z": f"{round(self.ego_point['z'],2):.2f}",
                        "yaw": f"{round(self.ego_point['yaw'],2):.2f}"
                    },
                    "ego_route": ego_route_save,
                    "weather": weather_params,
                    "trigger_position": {
                        "pitch": "0.0",
                        "x": f"{round(self.trigger_point['x'],2):.2f}",
                        "y": f"{round(self.trigger_point['y'],2):.2f}",
                        "yaw": f"{round(self.trigger_point['yaw'],2):.2f}",
                        "z": f"{round(self.trigger_point['z'],2):.2f}"
                    },
                    "trigger_radius": 2.0,
                    "other_actors": {"center": actors, "opposite": opposite_actors},
                    "timeout": 60.0,
                    "active": True,
                    "ego_steer_target": {
                        "x": f"{round(self.ego_steer_target_point['x'],2):.2f}",
                        "y": f"{round(self.ego_steer_target_point['y'],2):.2f}",
                        "z": f"{round(self.ego_steer_target_point['z'],2):.2f}",
                        "yaw": f"{round(self.ego_steer_target_point['yaw'],2):.2f}"
                    } if self.ego_steer_target_point is not None else None,
                }]
            }
        }

        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return fname

    # ======================
    # ✅ 按 S → 自动保存 64 个天气
    # ======================
    def save_all_weather_combinations(self):
        if self.is_signal_1d:
            self.save_1d_matrix()
            return
        if self.is_roundabout_2b:
            self.save_roundabout_trials()
            return
        if not self.ego_point or not self.trigger_point:
            print("❌ 必须设置 EGO 和 Trigger")
            return

        print("\n===== 开始生成 64 种天气场景 =====")
        total = 0
        for weather_name, params_list in self.weather_groups.items():
            for idx, param in enumerate(params_list):
                self.save_single(weather_name, idx, param)
                total += 1
                print(f"✅ {total}/64 生成完成")

        self.clear_all_points()
        self.route_index += 1
        print("\n🎉 全部 64 个天气场景已生成完成！")

    def clear_all_points(self):
        self.trigger_point = None
        self.ego_point = None
        self.actor_points.clear()
        self.opposite_points.clear()
        self.ego_steer_target_point = None
        self.agent_placement_mode = "center"
        self.selected_agent_idx = -1
        self.selected_mode = None
        if self.is_roundabout_2b:
            self.ego_route_points.clear()
            self.roundabout_history.clear()
            self.roundabout_branch_confirmed = False
            self.roundabout_topology_confirmed = False
            # STEP 1 is map infrastructure, not scenario-instance state.
            # Keep the accepted map-bound cache loaded for the next scenario;
            # the operator still reviews and presses Enter before reuse.
            self.roundabout_topology_review_ready = bool(
                self.roundabout_topology_seeds and self.roundabout_topology_review)
            self.roundabout_vut_recommendation = None
            self.roundabout_vut_recommendation_errors = []
            self.roundabout_last_trace_failure = None
            self.roundabout_topology_mark_kind = "inbound"
            self.roundabout_topology_edit_target = None
            self.roundabout_topology_lane_target = None
            self.roundabout_topology_preview = None
            self.roundabout_topology_preview_errors = []
            self.roundabout_vut_route_confirmed = False
            self.roundabout_vt1_route_confirmed = False
            self.roundabout_vut_route_cache = None
            self.roundabout_vt1_route_cache = None
            self.roundabout_vut_route_failure = None
            self.roundabout_vt1_route_failure = None
            self.roundabout_place_role = "vut"
        self._mark_roundabout_dirty()

    def select_click(self, sx, sy):
        wx, wy = self.screen_to_world(sx, sy)
        self.selected_agent_idx = -1
        self.selected_mode = None
        min_dist = 999
        sel_idx = -1
        for i, p in enumerate(self.actor_points):
            d = (p['x']-wx)**2 + (p['y']-wy)**2
            selection_limit = (max(0.8, 16.0 / (self.zoom + 0.01)) ** 2
                               if self.is_roundabout_2b else 10/(self.zoom+0.01))
            if d < min_dist and d < selection_limit:
                min_dist = d
                sel_idx = i
        if sel_idx >= 0:
            self.selected_agent_idx = sel_idx
            self.selected_mode = 'agent'
            if self.is_roundabout_2b:
                role = self.actor_points[sel_idx].get("role", "target").upper()
                suffix = ("Alt+右键绘制VT1路线，画完按Enter确认。" if role == "VT1"
                          else "VT2保持静止，不能绘制路线。")
                self._feedback("已选中{}；{}".format(role, suffix))
            return
        # 检查是否选中了opposite_points中的对向车辆
        sel_idx = -1
        min_dist = 999
        for i, p in enumerate(self.opposite_points):
            d = (p['x']-wx)**2 + (p['y']-wy)**2
            selection_limit = 10/(self.zoom+0.01)
            if d < min_dist and d < selection_limit:
                min_dist = d
                sel_idx = i
        if sel_idx >= 0:
            self.selected_agent_idx = sel_idx
            self.selected_mode = 'opposite_agent'
            self._feedback("已选中对向车道直行车辆 (OPPO-CAR)。")
            return
        # 检查是否选中了ego目标点
        if self.ego_steer_target_point is not None:
            d = (self.ego_steer_target_point['x'] - wx)**2 + (self.ego_steer_target_point['y'] - wy)**2
            if d < 15 / (self.zoom + 0.01):
                self.selected_mode = 'steer_target'
                self._feedback("已选中ego自动驾驶目标点 (TARGET)。")
                return
        if self.trigger_point and not self.is_roundabout_2b:
            d = (self.trigger_point['x']-wx)**2 + (self.trigger_point['y']-wy)**2
            if d < 15/(self.zoom+0.01):
                self.selected_mode = 'trigger'
                return
        if self.ego_point:
            d = (self.ego_point['x']-wx)**2 + (self.ego_point['y']-wy)**2
            if d < 15/(self.zoom+0.01):
                self.selected_mode = 'ego'
                if self.is_roundabout_2b:
                    self._feedback(
                        "已选中VUT；Alt+右键添加VUT路线点，画完按Enter确认，C清空路线。")
                return
        if self.is_roundabout_2b:
            self._feedback("未选中车辆；点击VUT或VT1后才能绘制路线。", True)

    def exclude_selected_roundabout_topology_lane(self):
        """Persistently exclude one auto-expanded STEP 1 cross-section."""
        target = getattr(self, "roundabout_topology_lane_target", None)
        if not target:
            return False
        if target.get("is_operator_seed"):
            self._feedback(
                "选中的是人工代表点，不能作为自动车道排除。请右键移动该点；"
                "需要重新标定整个拓扑时按X。", True)
            return False
        arm_index = int(target.get("arm_index", -1))
        direction = target.get("direction")
        if (not (0 <= arm_index < len(self.roundabout_topology_seeds))
                or direction not in ("inbound", "outbound")):
            self.roundabout_topology_lane_target = None
            self._feedback("所选拓扑车道已经失效，请重新左键选择。", True)
            return False
        arm = self.roundabout_topology_seeds[arm_index]
        seed = arm.get(direction)
        if not seed:
            self.roundabout_topology_lane_target = None
            self._feedback("所选arm方向已经失效，请重新执行STEP 1审核。", True)
            return False
        lane = copy.deepcopy(target.get("lane", {}))
        gate = copy.deepcopy(target.get("gate", {}))
        center = copy.deepcopy(gate.get("center", {}))
        if not lane or "x" not in center or "y" not in center:
            self._feedback("所选自动车道缺少可持久化的位置数据，无法排除。", True)
            return False

        self._push_roundabout_history(
            "排除{} {}自动车道".format(
                arm["arm_id"], "IN" if direction == "inbound" else "OUT"))
        exclusions = seed.setdefault("excluded_cross_sections", [])
        exclusions.append({
            "lane": lane,
            "center": {
                "x": float(center["x"]),
                "y": float(center["y"]),
                "z": float(center.get("z", 0.0)),
            },
            "s": gate.get("s"),
            "reason": "operator_excluded_auto_expansion",
        })
        review, errors = self._build_roundabout_topology_review()
        self.roundabout_topology_review = review
        self.roundabout_topology_review_errors = errors
        self.roundabout_topology_review_ready = not errors
        self.roundabout_topology_confirmed = False
        self.roundabout_topology_cache_loaded = False
        self.roundabout_topology_edit_target = None
        self.roundabout_topology_lane_target = None
        self._mark_roundabout_dirty()
        lane_key = (
            lane.get("road_id"), lane.get("section_id"), lane.get("lane_id"))
        if errors:
            self._feedback(
                "已排除{} {}的自动车道{}，但重新审核仍有问题：{}。"
                "可按Backspace撤销本次排除。".format(
                    arm["arm_id"], direction.upper(), lane_key,
                    "；".join(errors[:2])), True)
            return False
        self._feedback(
            "已排除{} {}的自动车道{}；地图已重新审核。"
            "确认结果正确后按Enter，排除记录会写入该地图的STEP 1缓存。".format(
                arm["arm_id"], direction.upper(), lane_key))
        return True

    def delete_selected(self):
        deleted = None
        if self.is_roundabout_2b:
            if getattr(self, "roundabout_topology_lane_target", None) is not None:
                self.exclude_selected_roundabout_topology_lane()
                return
            has_selection = (
                self.selected_mode == "ego"
                or (self.selected_mode == "agent"
                    and 0 <= self.selected_agent_idx < len(self.actor_points)))
            if not has_selection:
                self._feedback("没有选中可删除的对象。", True)
                return
            selected_label = ("VUT" if self.selected_mode == "ego" else
                              self.actor_points[self.selected_agent_idx].get(
                                  "role", "target").upper())
            self._push_roundabout_history("删除{}".format(selected_label))
        if self.selected_mode == 'trigger':
            self.trigger_point = None
            deleted = "Trigger"
        elif self.selected_mode == 'ego':
            self.ego_point = None
            deleted = "VUT" if self.is_roundabout_2b else "EGO"
            if self.is_roundabout_2b:
                self.ego_route_points.clear()
                self._invalidate_roundabout_route("vut")
                self.roundabout_vut_recommendation = None
                self.roundabout_vut_recommendation_errors = []
        elif self.selected_mode == 'agent' and 0 <= self.selected_agent_idx < len(self.actor_points):
            deleted_role = self.actor_points[self.selected_agent_idx].get("role")
            deleted = self.actor_points[self.selected_agent_idx].get(
                "role", self.actor_points[self.selected_agent_idx].get("type", "agent")).upper()
            del self.actor_points[self.selected_agent_idx]
            if self.is_roundabout_2b and deleted_role == "vt1":
                self._invalidate_roundabout_route("vt1")
        elif self.selected_mode == 'opposite_agent' and 0 <= self.selected_agent_idx < len(self.opposite_points):
            deleted = "OPPO-CAR"
            del self.opposite_points[self.selected_agent_idx]
        elif self.selected_mode == 'steer_target':
            self.ego_steer_target_point = None
            deleted = "TARGET"
        self.selected_agent_idx = -1
        self.selected_mode = None
        self._mark_roundabout_dirty()
        if self.is_roundabout_2b:
            if deleted:
                self._feedback("已删除{}；请按面板补齐必需对象。".format(deleted))

    def set_agent_yaw(self, yaw_type):
        if self.is_roundabout_2b:
            self._feedback(
                "2.b车辆朝向由所在Driving Lane自动确定，无需使用1..4手动设置。", True)
            return
        if self.selected_agent_idx < 0 or self.selected_agent_idx >= len(self.actor_points):
            print("❌ 请先选中AGENT")
            return
        p = self.actor_points[self.selected_agent_idx]
        wp = self.get_nearest_waypoint(p['x'], p['y'])
        y = wp.transform.rotation.yaw
        if yaw_type == 1: ny = y
        elif yaw_type == 2: ny = y + 180
        elif yaw_type == 3: ny = y - 90
        elif yaw_type == 4: ny = y + 90
        else: ny = 0
        p['yaw'] = ny
        self._mark_roundabout_dirty()

    def run(self):
        running = True
        print("\n========== 使用说明 ==========")
        if self.is_signal_1d:
            print("✅ 橙色=受信号灯控制车道；紫色=当前路线包含信号灯")
            print("✅ J/K/L 选择左转/直行/右转；按 S 生成同路线3次可复现矩阵")
        elif self.is_roundabout_2b:
            print("✅ 必须先完成STEP 0和STEP 1；之后按STEP 2～7操作")
            print("✅ STEP 0：Enter/R确认能力分支")
            print("✅ STEP 1：逐臂右键标IN/OUT；P程序审核；检查全部车道后Enter确认")
            print("✅ STEP 2起：V/T/Y选择VUT/VT1/VT2，三者都用Ctrl+左键放置")
            print("✅ VUT和VT1都在选中后用Alt+右键画路线，并分别按Enter确认绘制结束")
            print("✅ L显示/隐藏完整拓扑；默认仅显示当前任务相关门线，路线仅标首尾锚点")
            print("✅ VUT路线确认后自动显示VUT汇入口、出口编号和下游第1入口")
            print("✅ 路线推荐10-20m一点，分叉/入口/出口5-10m")
            print("✅ STEP 1左键选自动IN/OUT后Delete可排除；其他步骤Delete删选中车辆")
            print("✅ Backspace或Ctrl+Z撤销上一步；C清当前路线；H折叠帮助")
            print("✅ R切换能力分支；S严格校验并生成恰好3个确定性试验")
            print("✅ Trigger、15 km/h、VT2静止、三车时序及门线均由程序处理")
        else:
            print("✅ 画完路线 → 按 S 自动生成天气 JSON")
        print("====================================\n")

        while running:
            self.screen.fill((0, 0, 0))
            self.draw_waypoints()
            self.draw_points()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        running = False
                    # ======================
                    # ✅ 只按 S → 生成 64 个
                    # ======================
                    if event.key == pygame.K_s:
                        self.save_all_weather_combinations()

                    if event.key == pygame.K_c:
                        self.clear_ego_route()
                    if self.is_roundabout_2b and (
                            event.key == pygame.K_BACKSPACE
                            or (event.key == pygame.K_z and event.mod & pygame.KMOD_CTRL)):
                        self.undo_roundabout_action()
                    if self.is_roundabout_2b and event.key == pygame.K_h:
                        self.roundabout_help_expanded = not self.roundabout_help_expanded
                    if self.is_roundabout_2b and event.key == pygame.K_p:
                        self.confirm_roundabout_topology()
                    if self.is_roundabout_2b and event.key == pygame.K_x:
                        self.clear_roundabout_topology_seeds()
                    if self.is_roundabout_2b and event.key in (
                            pygame.K_RETURN, pygame.K_KP_ENTER):
                        self.confirm_active_roundabout_route()
                    if self.is_signal_1d and event.key == pygame.K_j: self.set_maneuver("left")
                    if self.is_signal_1d and event.key == pygame.K_k: self.set_maneuver("straight")
                    if self.is_signal_1d and event.key == pygame.K_l: self.set_maneuver("right")
                    if self.is_roundabout_2b and event.key == pygame.K_t:
                        self.set_roundabout_role(
                            "vt1", assign_selected=bool(event.mod & pygame.KMOD_SHIFT))
                    if self.is_roundabout_2b and event.key == pygame.K_y:
                        self.set_roundabout_role(
                            "vt2", assign_selected=bool(event.mod & pygame.KMOD_SHIFT))
                    if self.is_roundabout_2b and event.key == pygame.K_v:
                        self.set_roundabout_role("vut")
                    if self.is_roundabout_2b and event.key == pygame.K_l:
                        self.roundabout_show_all_overlays = not getattr(
                            self, "roundabout_show_all_overlays", False)
                        self._feedback(
                            "完整拓扑叠加已{}。".format(
                                "展开" if self.roundabout_show_all_overlays else "隐藏"))
                    if self.is_roundabout_2b and event.key == pygame.K_r:
                        self._push_roundabout_history("切换环岛能力分支")
                        self.roundabout_capable = not self.roundabout_capable
                        # Pressing R is an explicit operator choice, so it both
                        # changes and confirms STEP 0.
                        self.roundabout_branch_confirmed = True
                        self._mark_roundabout_dirty()
                        pygame.display.set_caption(self._window_caption())
                        branch = ("具备环形路口行驶功能：检查正确出口、碰撞、急刹和停车"
                                  if self.roundabout_capable else
                                  "不具备环形路口行驶功能：检查入口前ODD提示且不得进入")
                        self._feedback(
                            "STEP 0分支已确认并切换为：{}。进入STEP 1，请右键依次标定每个arm的"
                            "进口和出口，全部完成后按P。".format(branch))
                    if event.key == pygame.K_1: self.set_agent_yaw(1)
                    if event.key == pygame.K_2: self.set_agent_yaw(2)
                    if event.key == pygame.K_3: self.set_agent_yaw(3)
                    if event.key == pygame.K_4: self.set_agent_yaw(4)
                    if not self.is_roundabout_2b:
                        if event.key == pygame.K_5: self.set_agent_type("person")
                        if event.key == pygame.K_6: self.set_agent_type("bike")
                        if event.key == pygame.K_7: self.set_agent_type("car")
                        if event.key == pygame.K_8: self.set_agent_type("obstacle")
                    if not self.is_roundabout_2b and event.key == pygame.K_o:
                        if self.agent_placement_mode == "center":
                            self.agent_placement_mode = "opposite"
                            self._feedback("已切换到对向车道模式：右键放置对向直行车辆。")
                        elif self.agent_placement_mode == "opposite":
                            self.agent_placement_mode = "target"
                            self._feedback("已切换到目标点模式：右键放置ego自动驾驶目标点（仅一个）。")
                        else:
                            self.agent_placement_mode = "center"
                            self._feedback("已切换到center模式：右键放置横穿车辆。")
                    if event.key == pygame.K_DELETE: self.delete_selected()

                if event.type == pygame.MOUSEWHEEL:
                    mx, my = pygame.mouse.get_pos()
                    self._zoom_at(mx, my, event.y)

                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 2:
                        self.dragging = True
                        self.last_mouse_pos = pygame.mouse.get_pos()
                    if event.button == 1:
                        mods = pygame.key.get_mods()
                        sx, sy = event.pos
                        wx, wy = self.screen_to_world(sx, sy)
                        wp = self.get_nearest_waypoint(wx, wy)
                        if mods & pygame.KMOD_SHIFT:
                            if self.is_roundabout_2b:
                                self._feedback(
                                    "2.b不需要手动Trigger；入口门线会从VUT路线自动生成。", True)
                            elif wp is not None:
                                self.trigger_point = {
                                    'x': wp.transform.location.x,
                                    'y': wp.transform.location.y,
                                    'z': wp.transform.location.z,
                                    'yaw': wp.transform.rotation.yaw
                                }
                        elif mods & pygame.KMOD_CTRL:
                            if (self.is_roundabout_2b
                                    and not getattr(
                                        self, "roundabout_topology_confirmed", False)):
                                self._feedback(
                                    "请先完成STEP 0和STEP 1：Enter确认分支，右键逐臂标记"
                                    "IN/OUT，按P程序审核，检查后再按Enter确认。", True)
                            elif wp is not None:
                                if self.is_roundabout_2b:
                                    self.place_roundabout_selected_role(wx, wy)
                                else:
                                    self.ego_point = {
                                        'x': wp.transform.location.x,
                                        'y': wp.transform.location.y,
                                        'z': wp.transform.location.z + 0.3,
                                        'yaw': wp.transform.rotation.yaw
                                    }
                        else:
                            if (self.is_roundabout_2b
                                    and getattr(
                                        self, "roundabout_branch_confirmed", False)
                                    and not getattr(
                                        self, "roundabout_topology_confirmed", False)
                                    and getattr(
                                        self, "roundabout_topology_review", None)
                                    and self.select_roundabout_topology_seed(sx, sy)):
                                pass
                            else:
                                self.select_click(sx, sy)
                    if event.button == 3:
                        mods = pygame.key.get_mods()
                        sx, sy = event.pos
                        wx, wy = self.screen_to_world(sx, sy)
                        if (self.is_roundabout_2b
                                and not getattr(
                                    self, "roundabout_topology_confirmed", False)):
                            if mods & pygame.KMOD_ALT:
                                self._feedback(
                                    "当前仍是STEP 1；请直接右键标定进口/出口，"
                                    "完成全部物理arm并按P后再绘制路线。", True)
                            elif (getattr(
                                    self, "roundabout_topology_review_ready", False)
                                  or getattr(
                                      self, "roundabout_topology_edit_target", None)
                                  is not None):
                                self.replace_selected_roundabout_topology_seed(wx, wy)
                            else:
                                self.add_roundabout_topology_seed(wx, wy)
                        elif mods & pygame.KMOD_ALT:
                            self.add_ego_route_point(wx, wy)
                        else:
                            if self.is_roundabout_2b:
                                self._feedback(
                                    "车辆统一使用V/T/Y选择角色后Ctrl+左键放置；"
                                    "Alt+右键仅用于VUT/VT1路线点。", True)
                            else:
                                cx, cy = self.screen_to_world(sx, sy)
                                z = self.ego_point['z'] if self.ego_point else self.get_ground_z(cx, cy)
                                wp = self.get_nearest_waypoint(cx, cy)
                                point = {
                                    'x': cx, 'y': cy, 'z': z,
                                    'yaw': wp.transform.rotation.yaw,
                                    'type': 'car' if self.agent_placement_mode == 'opposite' else 'person'
                                }
                                if self.agent_placement_mode == 'opposite':
                                    self.opposite_points.append(point)
                                    self._feedback("已在对向车道添加直行车辆标记。")
                                elif self.agent_placement_mode == 'target':
                                    self.ego_steer_target_point = point
                                    self._feedback("已放置ego自动驾驶目标点（仅保留一个，再次右键可移动）。")
                                else:
                                    self.actor_points.append(point)

                if event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 2:
                        self.dragging = False
                if event.type == pygame.MOUSEMOTION and self.dragging:
                    cx, cy = pygame.mouse.get_pos()
                    dx = cx - self.last_mouse_pos[0]
                    dy = cy - self.last_mouse_pos[1]
                    self.offset_x += dx
                    self.offset_y += dy
                    self.last_mouse_pos = (cx, cy)

            pygame.display.flip()
            self.clock.tick(30)
        pygame.quit()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--name', default=None, help='Town name (default: auto-detect from CARLA)')
    parser.add_argument('--scenario', default='1')
    parser.add_argument('--save_dir', default='output')
    parser.add_argument('--maneuver', choices=MANEUVERS, default='straight')
    parser.add_argument('--seed', type=int, default=41798)
    parser.add_argument(
        '--roundabout_speed_limit_kmh', type=float, default=None,
        help='Audited engineering fallback when the target OpenDRIVE omits route speed limits')
    parser.add_argument(
        '--roundabout_variants_json', default=None,
        help='JSON weather/vehicle profiles used to expand one drawn 2.b route')
    parser.add_argument(
        '--roundabout_conflict_headway_s', type=float,
        default=VT1_CONFLICT_HEADWAY_TARGET_S,
        help='Engineering target time gap from VUT entry to VT1 at the merge')
    parser.add_argument(
        '--roundabout_conflict_tolerance_s', type=float,
        default=VT1_CONFLICT_HEADWAY_TOLERANCE_S,
        help='Allowed engineering tolerance around the target merge time gap')
    parser.add_argument(
        '--roundabout_vt1_post_exit_clearance_distance_m', type=float,
        default=25.0,
        help='Distance VT1 follows the connected exit lane after crossing exit 1')
    args = parser.parse_args()
    roundabout_variants = None
    if args.roundabout_variants_json:
        roundabout_variants = json.loads(args.roundabout_variants_json)
    editor = CarlaMapEditor0916(
        host=args.host, port=args.port,
        town_name=args.name, scenario=args.scenario, save_dir=args.save_dir,
        maneuver=args.maneuver, seed=args.seed,
        roundabout_speed_limit_kmh=args.roundabout_speed_limit_kmh,
        roundabout_variants=roundabout_variants,
        roundabout_conflict_headway_s=args.roundabout_conflict_headway_s,
        roundabout_conflict_tolerance_s=args.roundabout_conflict_tolerance_s,
        roundabout_vt1_post_exit_clearance_distance_m=
        args.roundabout_vt1_post_exit_clearance_distance_m)
    editor.run()

if __name__ == '__main__':
    main()
