import carla
import json
import pygame
import argparse
import os
import imageio
import numpy as np
import pickle
import pandas as pd
from scene import PedestrianCrossScene, BicycleCrossScene,StaticPedestrianCrossScene, CarCrossScene, StaticCarCrossScene, StaticObstacleScene, OccludedPedestrianScene, CarCutOutScene, CarCutInScene, CarOncomingPassScene, CarStopandGoScene, CarCutOutandStaticScene, CarGoandStopScene, EgoRouteFollowScene
from render import Visualizer
# 待开发功能
# 1. 33个场景的专用类
# 2. 24种天气
# 3. 摆放agent
# 4. 衔接TCP
# ====================== 全局配置 ======================
HOST = '127.0.0.1'
PORT = 2000
FPS = 20
VIDEO_DIR = "videos"
RESULT_PKL = "./log/result.pkl"

def get_sorted_scenario_files(input_dir):
    files = []
    for f in os.listdir(input_dir):
        if f.startswith('scenario_') and f.endswith('.json'):
            files.append(f)
    files.sort()
    return [os.path.join(input_dir, f) for f in files]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--town', type=str, required=True)
    parser.add_argument('--route_id', type=str, default='route_01')
    parser.add_argument('--input_dir', type=str, required=True)
    parser.add_argument('--scenario', type=str, default='3a')
    args = parser.parse_args()

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

        video_path = os.path.join(VIDEO_DIR, f"{scenario_name}.mp4")
        writer = imageio.get_writer(video_path, fps=FPS, codec='libx264')
        if args.scenario == '3a':
            scene = PedestrianCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '2b':
            scene = EgoRouteFollowScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '2c':
            scene = CarCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '2d':
            scene = CarCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '2e':
            scene = CarCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '2g':
            scene = StaticCarCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '2f':
            scene = StaticObstacleScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '3b_1':
            scene = PedestrianCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '3b_2':
            scene = PedestrianCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '3c':
            scene = BicycleCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '3d':
            scene = OccludedPedestrianScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '4a':
            scene = CarCutInScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '4b':
            scene = CarCutOutScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '4c':
            scene = CarOncomingPassScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '4d':
            scene = CarStopandGoScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '5a':
            scene = BicycleCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '5b':
            scene = BicycleCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '5c':
            scene = CarCutOutandStaticScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '5d':
            scene = CarGoandStopScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '5e':
            scene = CarCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '5f':
            scene = StaticPedestrianCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '5g':
            scene = PedestrianCrossScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '6a':
            scene = EgoRouteFollowScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '6b':
            scene = EgoRouteFollowScene(client, world, cfg_path, args.town, args.route_id)
        elif args.scenario == '6c':
            scene = EgoRouteFollowScene(client, world, cfg_path, args.town, args.route_id)
        try:
            scene.spawn()
        except Exception as e:
            print(f"❌ Spawn failed: {e}")
            writer.close()
            continue

        # ======================
        # 碰撞传感器（修复版）
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
        # 距离统计（修复版）
        # ======================
        previous_location = None
        total_distance = 0.0

        # ----------------------
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

            # 获取车辆数据
            trans = scene.ego.get_transform()
            vel = scene.ego.get_velocity()
            acc = scene.ego.get_acceleration()

            # 计算行驶距离
            if previous_location is not None:
                dx = trans.location.x - previous_location.x
                dy = trans.location.y - previous_location.y
                dist = np.sqrt(dx**2 + dy**2)
                total_distance += dist
            previous_location = trans.location

            # 数据采集
            ego_velocity = np.sqrt(vel.x**2 + vel.y**2)
            ego_acc_x, ego_acc_y, ego_acc_z = acc.x, acc.y, acc.z
            ego_x, ego_y, ego_z = trans.location.x, trans.location.y, trans.location.z
            ego_roll, ego_pitch, ego_yaw = trans.rotation.roll, trans.rotation.pitch, trans.rotation.yaw

            # 渲染 + 视频
            waypoints = scene.get_future_waypoints(12)
            viz.render(waypoints)
            frame = pygame.surfarray.array3d(viz.screen).swapaxes(0, 1)
            writer.append_data(frame)
            world.tick()

        # 状态
        running_status = "no running" if collision_occurred else "running"

        # 生成记录
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

        # 销毁（彻底修复传感器泄漏）
        collision_sensor.stop()
        collision_sensor.destroy()
        writer.close()
        viz.destroy()
        scene.destroy()
        world.tick()
        pygame.time.wait(500)

    # 保存最终结果
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
# python tools/waypoints.py --name TOWN10HD_Opt --scenario 1 --save_dir ./save_scenarios