# GB/T 41798—2022 机动车信号灯（1.d）测试结果

- 生成时间（UTC）：2026-09-03T11:16:21.909846+00:00
- 已记录试验：117
- 通过：0；失败：90
- 完整同路线矩阵：0 / 0
- batch_id：—
- carla_client_version：0.9.16
- carla_server_version：294096eb1-dirty
- evidence_profile：stf_carla_simulation
- execution_mode：formal
- fixed_delta_seconds：0.05
- host：127.0.0.1
- latest_run_id：20260903T111559.524254Z
- max_invalid_retries：—
- model：behavior
- model_path：—
- model_sha256：—
- port：2000
- pygame_window_mode：headless
- route_id：route_01
- runtime_environment：{'repo': {'commit': '4a1b8a5acb60331f173fea197cbf2da4129259b1', 'dirty': True}, 'launcher_config_path': None, 'launcher_config_sha256': None, 'batch_id': None, 'python': '3.10.0 \| packaged by conda-forge \| (default, Nov 20 2021, 02:24:10) [GCC 9.4.0]', 'executable': '/home/djhuai/anaconda3/envs/TCP/bin/python', 'packages': {'numpy': '1.24.2', 'pandas': '2.3.3', 'pygame': '2.6.1', 'imageio': '2.37.2', 'torch': '2.10.0', 'torchvision': '0.25.0'}, 'platform': 'Linux-6.8.0-136-generic-x86_64-with-glibc2.35', 'processor': 'x86_64', 'cuda': {'available': True, 'version': '12.8', 'device': 'NVIDIA GeForce RTX 3090'}, 'carla_client_version': '0.9.16', 'carla_server_version': '294096eb1-dirty', 'map_name': '5c_4/Maps/5c_4/5c_4', 'opendrive_sha256': '2936962a4605a3de2d9fa483392808d623ab871eb290e49fa9364bf777b72dce', 'synchronous_mode': True, 'fixed_delta_seconds': 0.05, 'telemetry_frequency_hz': 20, 'behavior_implementation': 'scene.EgoRouteFollowScene.follow_route; repository reference controller, not CARLA BehaviorAgent', 'tcp_implementation': 'model.tcp.TCPAgent.get_action', 'pygame_window_mode': 'headless', 'pygame_video_driver': 'dummy', 'ads': {'name': 'behavior', 'implementation': 'scene.EgoRouteFollowScene.follow_route', 'checkpoint_path': None, 'checkpoint_sha256': None}, 'collision_configuration': {'path': None, 'sha256': None, 'resolved': {'global': {'enabled': True, 'ga': {'enabled': True, 'population_size': 30, 'generations': 30, 'mutation_rate': 0.13, 'crossover_rate': 0.7}}, 'npc_speed': {'throttle_multiplier': 1.0, 'overrides': {'car_cross': {'throttle': 0.9, 'trigger_distance': 12}, 'pedestrian': {'speed': 3.5, 'trigger_distance': 12}, 'car_cut_out': {'initial_throttle': 0.5, 'merge_throttle': 0.6, 'cut_out_delay': 0.8, 'trigger_distance': 14}, 'car_cut_in': {'throttle': 0.8, 'trigger_distance': 14}, 'oncoming_pass': {'throttle': 0.8, 'trigger_distance': 14}, 'stop_and_go': {'brake_force': 0.6, 'trigger_distance': 14}, 'cut_out_static': {'initial_throttle': 0.5, 'merge_throttle': 0.7, 'static_brake': 0.8, 'cut_out_delay': 0.8, 'trigger_distance': 14}, 'bicycle': {'speed': 5.0, 'trigger_distance': 12}, 'static_car_cross': {'throttle': 0.8, 'trigger_distance': 12}, 'static_obstacle': {'obstacle_distance': 8, 'trigger_distance': 12}, 'ego_route_follow': {'throttle': 0.8, 'trigger_distance': 14}}}, 'ga_bounds': {'car_cross': {'bounds': {'actor_speed': [6.0, 14.0], 'scenario_trigger': [20.0, 50.0], 'deceleration_trigger': [5.0, 10.0], 'x_offset': [-1.0, 1.0], 'y_offset': [-10.0, 10.0], 'yaw_offset': [-15.0, 15.0]}}, 'car_cut_out': {'bounds': {'actor_speed': [5.0, 12.0], 'scenario_trigger': [15.0, 40.0], 'deceleration_trigger': [5.0, 10.0], 'x_offset': [-1.0, 1.0], 'y_offset': [-10.0, 10.0], 'yaw_offset': [-15.0, 15.0]}}, 'pedestrian': {'bounds': {'walker_speed': [2.5, 5.0], 'trigger_distance': [8.0, 15.0], 'x_offset': [-2.0, 2.0], 'y_offset': [-5.0, 5.0], 'yaw_offset': [-30.0, 30.0]}}}, 'collision_profile': {'trigger_distance_scale': 0.7, 'deceleration_scale': 0.6, 'min_trigger_distance': 8.0, 'min_deceleration_distance': 5.0, 'speed_boost': 1.3, 'timeout_scale': 1.2}, 'scenario_overrides': {'3': {'trigger_distance_scale': 0.6, 'deceleration_scale': 0.58, 'min_deceleration_distance': 5.0, 'max_deceleration_distance': 9.5, 'speed_boost': 1.3}, '5': {'target_ttc': 2.6, 'trigger_distance_scale': 0.58, 'min_trigger_distance': 13.0, 'max_trigger_distance': 29.0, 'speed_boost': 1.2}, '8': {'min_trigger_speed': 2.6, 'target_ttc': 3.0, 'trigger_distance_scale': 0.55, 'min_trigger_distance': 12.0, 'max_trigger_distance': 26.0, 'ego_max_driven_distance_scale': 1.55, 'min_ego_max_driven_distance': 110.0, 'speed_boost': 1.2}}}}}
- telemetry_frequency_hz：20
- town：5c_4
- video_purpose：simulation_visualization

## 逐次结果

| run_id | scenario | matrix_id | maneuver | trial_index | signal_case | matrix_seed | trial_seed | trigger_distance_m | stop_gap_m | restart_delay_s | crossed_stop_line_on_red | unjustified_stop | collision | route_complete | pass | failure_reasons |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20260903T110230.222272Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cold_fog_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cold_fog_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110230.222272Z | scenario_1_0000_cold_fog_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110412.379959Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110412.379959Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110412.379959Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110515.300571Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cold_fog_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cold_fog_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cold_fog_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cold_fog_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cold_fog_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cold_fog_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cold_fog_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_cold_fog_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dawn_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dawn_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dawn_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dawn_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dawn_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dawn_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dawn_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dawn_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dusk_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dusk_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dusk_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dusk_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dusk_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dusk_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dusk_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_dusk_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_fog_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_fog_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_fog_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_fog_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_fog_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_fog_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_fog_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_fog_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_heavy_rain_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_heavy_rain_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_heavy_rain_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_heavy_rain_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_heavy_rain_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_heavy_rain_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_heavy_rain_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_heavy_rain_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_light_rain_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_light_rain_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_light_rain_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_light_rain_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_light_rain_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_light_rain_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_light_rain_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_light_rain_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_night_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_night_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_night_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_night_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_night_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_night_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_night_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_night_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_overcast_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_overcast_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_overcast_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_overcast_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_overcast_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_overcast_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_overcast_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_overcast_08 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_rainy_dusk_01 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_rainy_dusk_02 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_rainy_dusk_03 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_rainy_dusk_04 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_rainy_dusk_05 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_rainy_dusk_06 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110515.300571Z | scenario_1_0000_rainy_dusk_07 | — | — | — | — | — | — | — | — | — | — | — | — | False | False | spawn_failed: Spawn failed because of collision at spawn position |
| 20260903T110755.647824Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110755.647824Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110755.647824Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110856.620289Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110856.620289Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110856.620289Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110856.620289Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T110856.620289Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111040.413366Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111040.413366Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111126.558424Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111126.558424Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111126.558424Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111232.661161Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111232.661161Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111318.254884Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111318.254884Z | scenario_1_0000_cloudy_02 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111318.254884Z | scenario_1_0000_cloudy_03 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111318.254884Z | scenario_1_0000_cloudy_04 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111318.254884Z | scenario_1_0000_cloudy_05 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111318.254884Z | scenario_1_0000_cloudy_06 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111318.254884Z | scenario_1_0000_cloudy_07 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111318.254884Z | scenario_1_0000_cloudy_08 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |
| 20260903T111559.524254Z | scenario_1_0000_cloudy_01 | — | — | — | — | — | — | — | — | — | — | — | running | False | — | — |

## 可复现性说明

每次试验保留基础种子、派生矩阵种子、路线指纹和输入配置哈希。使用相同地图、CARLA版本、被测模型及输入配置即可重放同一矩阵。
