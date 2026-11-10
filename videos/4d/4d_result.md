# GB/T 41798—2022 机动车信号灯（1.d）测试结果

- 生成时间（UTC）：2026-11-10T10:54:05.554025+00:00
- 已记录试验：134
- 通过：0；失败：21
- 完整同路线矩阵：0 / 0
- batch_id：—
- carla_client_version：0.9.16
- carla_server_version：294096eb1-dirty
- evidence_profile：stf_carla_simulation
- execution_mode：formal
- fixed_delta_seconds：0.05
- host：127.0.0.1
- latest_run_id：20261110T105340.241478Z
- max_invalid_retries：—
- model：behavior
- model_path：—
- model_sha256：—
- port：2000
- pygame_window_mode：headless
- route_id：route_01
- runtime_environment：{'repo': {'commit': '3718de45f843edbe7bf7be34ba6905ce979f048e', 'dirty': True}, 'launcher_config_path': None, 'launcher_config_sha256': None, 'batch_id': None, 'python': '3.10.0 \| packaged by conda-forge \| (default, Nov 20 2021, 02:24:10) [GCC 9.4.0]', 'executable': '/home/djhuai/anaconda3/envs/TCP/bin/python', 'packages': {'numpy': '1.24.2', 'pandas': '2.3.3', 'pygame': '2.6.1', 'imageio': '2.37.2', 'torch': '2.10.0', 'torchvision': '0.25.0'}, 'platform': 'Linux-6.8.0-138-generic-x86_64-with-glibc2.35', 'processor': 'x86_64', 'cuda': {'available': True, 'version': '12.8', 'device': 'NVIDIA GeForce RTX 3090'}, 'carla_client_version': '0.9.16', 'carla_server_version': '294096eb1-dirty', 'map_name': 'Carla/Maps/Town10HD_Opt', 'opendrive_sha256': '5d883b799f634030af92be1e9d79d107845540ba04338e8c60e095be1aef7be7', 'synchronous_mode': True, 'fixed_delta_seconds': 0.05, 'telemetry_frequency_hz': 20, 'behavior_implementation': 'scene.EgoRouteFollowScene.follow_route; repository reference controller, not CARLA BehaviorAgent', 'tcp_implementation': 'model.tcp.TCPAgent.get_action', 'pygame_window_mode': 'headless', 'pygame_video_driver': 'dummy', 'ads': {'name': 'behavior', 'implementation': 'scene.EgoRouteFollowScene.follow_route', 'checkpoint_path': None, 'checkpoint_sha256': None}, 'collision_configuration': {'path': None, 'sha256': None, 'resolved': {'global': {'enabled': True, 'ga': {'enabled': True, 'population_size': 30, 'generations': 30, 'mutation_rate': 0.13, 'crossover_rate': 0.7}}, 'npc_speed': {'throttle_multiplier': 1.0, 'overrides': {'car_cross': {'throttle': 0.9, 'trigger_distance': 12}, 'pedestrian': {'speed': 3.5, 'trigger_distance': 12}, 'car_cut_out': {'initial_throttle': 0.5, 'merge_throttle': 0.6, 'cut_out_delay': 0.8, 'trigger_distance': 14}, 'car_cut_in': {'throttle': 0.8, 'trigger_distance': 14}, 'oncoming_pass': {'throttle': 0.8, 'trigger_distance': 14}, 'stop_and_go': {'brake_force': 0.6, 'trigger_distance': 14}, 'cut_out_static': {'initial_throttle': 0.5, 'merge_throttle': 0.7, 'static_brake': 0.8, 'cut_out_delay': 0.8, 'trigger_distance': 14}, 'bicycle': {'speed': 5.0, 'trigger_distance': 12}, 'static_car_cross': {'throttle': 0.8, 'trigger_distance': 12}, 'static_obstacle': {'obstacle_distance': 8, 'trigger_distance': 12}, 'ego_route_follow': {'throttle': 0.8, 'trigger_distance': 14}}}, 'ga_bounds': {'car_cross': {'bounds': {'actor_speed': [6.0, 14.0], 'scenario_trigger': [20.0, 50.0], 'deceleration_trigger': [5.0, 10.0], 'x_offset': [-1.0, 1.0], 'y_offset': [-10.0, 10.0], 'yaw_offset': [-15.0, 15.0]}}, 'car_cut_out': {'bounds': {'actor_speed': [5.0, 12.0], 'scenario_trigger': [15.0, 40.0], 'deceleration_trigger': [5.0, 10.0], 'x_offset': [-1.0, 1.0], 'y_offset': [-10.0, 10.0], 'yaw_offset': [-15.0, 15.0]}}, 'pedestrian': {'bounds': {'walker_speed': [2.5, 5.0], 'trigger_distance': [8.0, 15.0], 'x_offset': [-2.0, 2.0], 'y_offset': [-5.0, 5.0], 'yaw_offset': [-30.0, 30.0]}}}, 'collision_profile': {'trigger_distance_scale': 0.7, 'deceleration_scale': 0.6, 'min_trigger_distance': 8.0, 'min_deceleration_distance': 5.0, 'speed_boost': 1.3, 'timeout_scale': 1.2}, 'scenario_overrides': {'3': {'trigger_distance_scale': 0.6, 'deceleration_scale': 0.58, 'min_deceleration_distance': 5.0, 'max_deceleration_distance': 9.5, 'speed_boost': 1.3}, '5': {'target_ttc': 2.6, 'trigger_distance_scale': 0.58, 'min_trigger_distance': 13.0, 'max_trigger_distance': 29.0, 'speed_boost': 1.2}, '8': {'min_trigger_speed': 2.6, 'target_ttc': 3.0, 'trigger_distance_scale': 0.55, 'min_trigger_distance': 12.0, 'max_trigger_distance': 26.0, 'ego_max_driven_distance_scale': 1.55, 'min_ego_max_driven_distance': 110.0, 'speed_boost': 1.2}}}}}
- telemetry_frequency_hz：20
- town：Town10HD_Opt
- video_purpose：simulation_visualization

## 逐次结果

| run_id | scenario | matrix_id | maneuver | trial_index | signal_case | matrix_seed | trial_seed | trigger_distance_m | stop_gap_m | restart_delay_s | crossed_stop_line_on_red | unjustified_stop | collision | route_complete | pass | failure_reasons |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20261110T091059.793518Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cold_fog_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cold_fog_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cold_fog_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cold_fog_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091059.793518Z | scenario_1_0000_cold_fog_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: name 'random' is not defined |
| 20261110T091128.445951Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: 自车生成失败！ |
| 20261110T091128.445951Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: 自车生成失败！ |
| 20261110T091128.445951Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: 自车生成失败！ |
| 20261110T091128.445951Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: 自车生成失败！ |
| 20261110T091541.148984Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: blueprint 'static.prop.warningbarrier' not found |
| 20261110T091541.148984Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: blueprint 'static.prop.warningbarrier' not found |
| 20261110T091849.705414Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cold_fog_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cold_fog_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cold_fog_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cold_fog_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cold_fog_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cold_fog_06 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cold_fog_07 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T091849.705414Z | scenario_1_0000_cold_fog_08 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T092442.681874Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T092532.314527Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T092532.314527Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T092532.314527Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T092532.314527Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T092532.314527Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T092532.314527Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T092532.314527Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T092532.314527Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T093255.473176Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T093255.473176Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T093255.473176Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T093905.979280Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cold_fog_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095044.674011Z | scenario_1_0000_cold_fog_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cold_fog_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cold_fog_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cold_fog_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cold_fog_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095357.564214Z | scenario_1_0000_cold_fog_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095951.870995Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T095951.870995Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cold_fog_01 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cold_fog_02 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cold_fog_03 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cold_fog_04 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cold_fog_05 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100238.006131Z | scenario_1_0000_cold_fog_06 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T100652.523611Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: 自车生成失败！ |
| 20261110T100652.523611Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: 自车生成失败！ |
| 20261110T100840.589417Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100840.589417Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100840.589417Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100840.589417Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100840.589417Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100840.589417Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100840.589417Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100840.589417Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100840.589417Z | scenario_1_0000_cold_fog_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T100840.589417Z | scenario_1_0000_cold_fog_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101417.896979Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101417.896979Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101417.896979Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101417.896979Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101417.896979Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101555.411499Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101626.258688Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cold_fog_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T101756.584060Z | scenario_1_0000_cold_fog_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T102232.159489Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T102232.159489Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T102232.159489Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T102711.251947Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T102711.251947Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T102711.251947Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T102711.251947Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T102711.251947Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T102711.251947Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T102711.251947Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T103402.544944Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T103402.544944Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T103402.544944Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | no running | False | — | — |
| 20261110T103402.544944Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T103657.708336Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T103807.175340Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T103854.233534Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20261110T105340.241478Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |

## 可复现性说明

每次试验保留基础种子、派生矩阵种子、路线指纹和输入配置哈希。使用相同地图、CARLA版本、被测模型及输入配置即可重放同一矩阵。
