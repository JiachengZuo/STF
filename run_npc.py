import carla
import json
import pygame
import argparse
import os
import imageio
import numpy as np
import pickle
import pandas as pd
import random
from scene import PedestrianCrossScene, BicycleCrossScene,StaticPedestrianCrossScene, CarCrossScene, StaticCarCrossScene, StaticObstacleScene, OccludedPedestrianScene, CarCutOutScene, CarCutInScene, CarOncomingPassScene, CarStopandGoScene, CarCutOutandStaticScene, CarGoandStopScene, EgoRouteFollowScene
from render import Visualizer
from model.tcp import TCPAgent
from model.tcp import TCPRoutePlanner
from carla import VehicleControl
from collision_enhancer import load_collision_config
# 待开发功能
# 1. 33个场景的专用类（完成）
# 2. 24种天气（完成）
# 3. 摆放agent（完成）
# 4. 衔接TCP
# ====================== 全局配置 ======================
HOST = '127.0.0.1'
PORT = 2000
FPS = 20

def get_sorted_scenario_files(input_dir):
    files = []
    for f in os.listdir(input_dir):
        if f.startswith('scenario_') and f.endswith('.json'):
            files.append(f)
    files.sort()
    return [os.path.join(input_dir, f) for f in files]

# ======================
# ✅ 新增：从JSON加载天气并设置CARLA
# ======================
def load_weather_from_json(world, json_path, town_name, route_id="route_01"):
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        scenario = data[town_name][route_id][0]
        if "weather" not in scenario:
            print("ℹ️ No weather in JSON, use default")
            return

        w = scenario["weather"]
        weather = carla.WeatherParameters(
            cloudiness=w.get("cloudiness", 0.0),
            dust_storm=w.get("dust_storm", 0.0),
            fog_density=w.get("fog_density", 0.0),
            fog_distance=w.get("fog_distance", 100.0),
            fog_falloff=w.get("fog_falloff", 1.0),
            mie_scattering_scale=w.get("mie_scattering_scale", 0.03),
            precipitation=w.get("precipitation", 0.0),
            precipitation_deposits=w.get("precipitation_deposits", 0.0),
            rayleigh_scattering_scale=w.get("rayleigh_scattering_scale", 0.0331),
            scattering_intensity=w.get("scattering_intensity", 1.0),
            sun_altitude_angle=w.get("sun_altitude_angle", 60.0),
            sun_azimuth_angle=w.get("sun_azimuth_angle", 120.0),
            wetness=w.get("wetness", 0.0),
            wind_intensity=w.get("wind_intensity", 5.0)
        )
        world.set_weather(weather)
        print(f"✅ Weather loaded from JSON: {os.path.basename(json_path)}")
    except Exception as e:
        print(f"❌ Weather load failed: {e}")

def spawn_traffic_npcs(world, client, total_npcs=20, car_ratio=0.5, cyclist_ratio=0.3):
    """
    在场景周围随机生成NPC（车辆、自行车、行人）并让它们自动驾驶
    使用CARLA自带的Traffic Manager实现车辆自动驾驶，Walker AI控制行人
    """
    if total_npcs <= 0:
        return []

    bp_lib = world.get_blueprint_library()
    spawned_actors = []

    # 计算各类NPC数量
    n_cars = int(total_npcs * car_ratio)
    n_cyclists = int(total_npcs * cyclist_ratio)
    n_pedestrians = total_npcs - n_cars - n_cyclists
    print(f"\n🏙️  Spawning {total_npcs} NPCs: {n_cars} cars, {n_cyclists} cyclists, {n_pedestrians} pedestrians")

    # ========== 生成车辆 ==========
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    if not spawn_points:
        print("⚠️ No spawn points available!")
        return []

    # 普通车辆蓝图（排除自行车/摩托车/特殊车辆）
    car_bps = [bp for bp in bp_lib.filter('vehicle.*')
               if 'bicycle' not in bp.id and 'motorcycle' not in bp.id
               and 'ambulance' not in bp.id and 'firetruck' not in bp.id]

    # 自行车/摩托车蓝图
    cyclist_bps = [bp for bp in bp_lib.filter('vehicle.*')
                   if 'bicycle' in bp.id or 'motorcycle' in bp.id]

    # 批量生成车辆
    vehicle_actors = []
    if n_cars > 0 and car_bps:
        batch = []
        for i in range(min(n_cars, len(spawn_points))):
            bp = random.choice(car_bps)
            if bp.has_attribute('color'):
                color = random.choice(bp.get_attribute('color').recommended_values)
                bp.set_attribute('color', color)
            batch.append(carla.command.SpawnActor(bp, spawn_points[i]))
        results = client.apply_batch_sync(batch, False)
        for result in results:
            if not result.error:
                actor = world.get_actor(result.actor_id)
                if actor:
                    vehicle_actors.append(actor)
        print(f"  ✅ Spawned {len(vehicle_actors)} cars")

    # 批量生成自行车
    offset = min(n_cars, len(spawn_points))
    avail_pts = spawn_points[offset:]
    if n_cyclists > 0 and cyclist_bps and avail_pts:
        batch = []
        for i in range(min(n_cyclists, len(avail_pts))):
            bp = random.choice(cyclist_bps)
            batch.append(carla.command.SpawnActor(bp, avail_pts[i]))
        results = client.apply_batch_sync(batch, False)
        for result in results:
            if not result.error:
                actor = world.get_actor(result.actor_id)
                if actor:
                    vehicle_actors.append(actor)
        print(f"  ✅ Spawned {n_cyclists} cyclists")

    # 设置Traffic Manager（自动驾驶）
    if vehicle_actors:
        tm = client.get_trafficmanager()
        tm.set_synchronous_mode(True)
        tm.set_global_distance_to_leading_vehicle(2.0)
        for v in vehicle_actors:
            v.set_autopilot(True, tm.get_port())
            # 随机设置速度偏移，让行驶更自然
            tm.vehicle_percentage_speed_difference(v, random.uniform(-20, 20))
        spawned_actors.extend(vehicle_actors)

    # ========== 生成行人 ==========
    if n_pedestrians > 0:
        ped_bps = list(bp_lib.filter('walker.pedestrian.*'))
        if ped_bps:
            # 获取行人可用的导航点
            ped_spawn_points = []
            for _ in range(n_pedestrians * 3):
                loc = world.get_random_location_from_navigation()
                if loc is not None:
                    ped_spawn_points.append(carla.Transform(loc))
            ped_spawn_points = ped_spawn_points[:n_pedestrians]

            # 批量生成行人
            batch = []
            chosen_bps = [random.choice(ped_bps) for _ in range(len(ped_spawn_points))]
            for i, sp in enumerate(ped_spawn_points):
                batch.append(carla.command.SpawnActor(chosen_bps[i], sp))
            results = client.apply_batch_sync(batch, False)
            walker_ids = []
            for result in results:
                if not result.error and result.actor_id > 0:
                    walker_ids.append(result.actor_id)

            # 生成行人控制器
            if walker_ids:
                ai_bp = bp_lib.find('controller.ai.walker')
                batch = []
                for wid in walker_ids:
                    batch.append(carla.command.SpawnActor(ai_bp, carla.Transform(), wid))
                results = client.apply_batch_sync(batch, True)
                controller_ids = []
                for result in results:
                    if not result.error:
                        controller_ids.append(result.actor_id)

                # 启动行人AI
                world.tick()
                for cid in controller_ids:
                    controller = world.get_actor(cid)
                    if controller:
                        controller.start()
                        dest = world.get_random_location_from_navigation()
                        if dest:
                            controller.go_to_location(dest)
                        controller.set_max_speed(1.4 + random.random() * 0.6)

                # 记录actors以便清理
                for wid in walker_ids:
                    actor = world.get_actor(wid)
                    if actor:
                        spawned_actors.append(actor)
                for cid in controller_ids:
                    actor = world.get_actor(cid)
                    if actor:
                        spawned_actors.append(actor)
                print(f"  ✅ Spawned {len(walker_ids)} pedestrians")

    print(f"🎯 Total NPCs spawned: {len(spawned_actors)}\n")
    return spawned_actors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--town', type=str, required=True)
    parser.add_argument('--route_id', type=str, default='route_01')
    parser.add_argument('--input_dir', type=str, required=True)
    parser.add_argument('--scenario', type=str, default='3a')
    parser.add_argument('--video_dir', type=str, default='videos')
    parser.add_argument('--model', type=str, default='behavior')
    parser.add_argument('--model_path', type=str, default='./tcp/best_model.ckpt')
    parser.add_argument('--collision_config', type=str, default=None,
                        help='Path to collision enhancement config YAML (default: collision_config.yaml in tools dir)')
    parser.add_argument('--npc_total', type=int, default=0,
                        help='Total number of NPCs to spawn (cars + cyclists + pedestrians)')
    parser.add_argument('--npc_car_ratio', type=float, default=0.50,
                        help='Ratio of cars among NPCs (default: 0.50)')
    parser.add_argument('--npc_cyclist_ratio', type=float, default=0.30,
                        help='Ratio of cyclists among NPCs (default: 0.30, remaining = pedestrians)')

    args = parser.parse_args()

    # ======================
    # Load collision enhancement config
    # ======================
    _collision_cfg = load_collision_config(args.collision_config)
    is_enabled = _collision_cfg.get('global', {}).get('enabled', True)
    print(f"[COLLISION] Enhancement {'ENABLED' if is_enabled else 'DISABLED'}")

    VIDEO_DIR = os.path.join(args.video_dir, args.scenario)
    RESULT_PKL = os.path.join(VIDEO_DIR, f"{args.scenario}_result.pkl")
    os.makedirs(VIDEO_DIR, exist_ok=True)
    scenario_files = get_sorted_scenario_files(args.input_dir)
    if not scenario_files:
        print("❌ No scenarios found")
        return

    test_records = []

    pygame.init()
    client = carla.Client(HOST, PORT)
    client.set_timeout(12.0)
    world = client.get_world()
    world.apply_settings(carla.WorldSettings(synchronous_mode=True, fixed_delta_seconds=0.05))

    for idx, cfg_path in enumerate(scenario_files):
        scenario_name = os.path.basename(cfg_path).replace(".json", "")
        print(f"\n======= Running {scenario_name} ({idx+1}/{len(scenario_files)}) =======")

        # ======================
        # ✅ 自动加载天气（关键行）
        # ======================
        load_weather_from_json(world, cfg_path, args.town, args.route_id)

        video_path = os.path.join(VIDEO_DIR, f"{scenario_name}.mp4")
        writer = imageio.get_writer(video_path, fps=FPS, codec='libx264')
        if args.scenario == '3a':
            scene = PedestrianCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '2b':
            scene = EgoRouteFollowScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '2c':
            scene = CarCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '2d':
            scene = CarCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '2e':
            scene = CarCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '2g':
            scene = StaticCarCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '2f':
            scene = StaticObstacleScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '3b_1':
            scene = PedestrianCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '3b_2':
            scene = PedestrianCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '3c':
            scene = BicycleCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '3d':
            scene = OccludedPedestrianScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '4a':
            scene = CarCutInScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '4b':
            scene = CarCutOutScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '4c':
            scene = CarOncomingPassScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '4d':
            scene = CarStopandGoScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '5a':
            scene = BicycleCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '5b':
            scene = BicycleCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '5c':
            scene = CarCutOutandStaticScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '5d':
            scene = CarGoandStopScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '5e':
            scene = CarCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '5f':
            scene = StaticPedestrianCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '5g':
            scene = PedestrianCrossScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '6a':
            scene = EgoRouteFollowScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '6b':
            scene = EgoRouteFollowScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        elif args.scenario == '6c':
            scene = EgoRouteFollowScene(client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
        try:
            scene.spawn()
        except Exception as e:
            print(f"❌ Spawn failed: {e}")
            writer.close()
            continue

        # ======================
        # 生成环境NPC（自动驾驶流量）
        # ======================
        npc_actors = spawn_traffic_npcs(
            world, client,
            total_npcs=args.npc_total,
            car_ratio=args.npc_car_ratio,
            cyclist_ratio=args.npc_cyclist_ratio
        )

        # ======================
        # 碰撞传感器
        # ======================
        collision_occurred = False

        def on_collision(event):
            nonlocal collision_occurred
            collision_occurred = True

        bp_lib = world.get_blueprint_library()
        collision_bp = bp_lib.find('sensor.other.collision')
        collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=scene.ego)
        collision_sensor.listen(on_collision)

        # ======================
        # 距离统计
        # ======================
        previous_location = None
        total_distance = 0.0

        viz = Visualizer(world, scene.ego)
        running = True

        ego_velocity = 0.0
        ego_acc_x = ego_acc_y = ego_acc_z = 0.0
        ego_x = ego_y = ego_z = 0.0
        ego_roll = ego_pitch = ego_yaw = 0.0
        current_game_time = 0.0

        while running:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False

            if not scene.tick():
                running = False

            trans = scene.ego.get_transform()
            vel = scene.ego.get_velocity()
            acc = scene.ego.get_acceleration()

            if previous_location is not None:
                dx = trans.location.x - previous_location.x
                dy = trans.location.y - previous_location.y
                dist = np.sqrt(dx**2 + dy**2)
                total_distance += dist
            previous_location = trans.location

            ego_velocity = np.sqrt(vel.x**2 + vel.y**2)
            ego_acc_x, ego_acc_y, ego_acc_z = acc.x, acc.y, acc.z
            ego_x, ego_y, ego_z = trans.location.x, trans.location.y, trans.location.z
            ego_roll, ego_pitch, ego_yaw = trans.rotation.roll, trans.rotation.pitch, trans.rotation.yaw

            waypoints = scene.get_future_waypoints(12)
            viz.render(waypoints)
            frame = pygame.surfarray.array3d(viz.screen).swapaxes(0, 1)
            writer.append_data(frame)
            world.tick()

        running_status = "no running" if collision_occurred else "running"

        record = {
            "scenario": scenario_name,
            "ego_velocity": round(ego_velocity, 2),
            "ego_acceleration_x": round(ego_acc_x, 2),
            "ego_acceleration_y": round(ego_acc_y, 2),
            "ego_acceleration_z": round(ego_acc_z, 2),
            "ego_x": round(ego_x, 2),
            "ego_y": round(ego_y, 2),
            "ego_z": round(ego_z, 2),
            "ego_roll": round(ego_roll, 2),
            "ego_pitch": round(ego_pitch, 2),
            "ego_yaw": round(ego_yaw, 2),
            "current_game_time": round(world.get_snapshot().timestamp.elapsed_seconds, 1),
            "driven_distance": round(total_distance, 2),
            "average_velocity": round(ego_velocity, 2),
            "lane_invasion": 0,
            "off_road": 0,
            "collision": running_status,
            "run_red_light": 0,
            "run_stop": 0,
            "distance_to_route": 0.0,
            "route_complete": False
        }
        test_records.append(record)
        print(f"✅ {scenario_name} | collision: {running_status}")

        collision_sensor.stop()
        collision_sensor.destroy()

        # 清理环境NPC
        for actor in npc_actors:
            if actor.is_alive:
                actor.destroy()
        print(f"  🧹 Cleaned up {len(npc_actors)} NPCs")

        writer.close()
        viz.destroy()
        scene.destroy()
        world.tick()
        pygame.time.wait(1000)

    df = pd.DataFrame(test_records)
    with open(RESULT_PKL, "wb") as f:
        pickle.dump(df, f)

    print(f"\n🎉 ALL DONE! Result saved to: {RESULT_PKL}")
    print("\nPreview:")
    print(df[["scenario", "ego_velocity", "collision"]])
    pygame.quit()

if __name__ == '__main__':
    main()

    # python tools/run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 3a
    # input_dir 读取json文件，town和scenario决定使用哪个场景类，video_dir决定视频输出目录
    # python tools/run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 3a --model tcp
    # python tools/run.py --input_dir ./save_scenarios/ --town roadside_1 --scenario 3a --model behavior
    #
    # 生成环境NPC示例（20个NPC：50%汽车 + 30%自行车 + 20%行人）：
    # python tools/run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 2b --npc_total 20 --npc_car_ratio 0.50 --npc_cyclist_ratio 0.30