#!/usr/bin/env python3
import carla
import json
import time
import pygame

# 配置
JSON_FILE = "trigger_person_output.json"
HOST = '127.0.0.1'
PORT = 2000

def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    spectator = world.get_spectator()

    # 初始化 pygame 用于监听键盘
    pygame.init()
    screen = pygame.display.set_mode((200, 100))
    pygame.display.set_caption("视角控制")
    font = pygame.font.SysFont(None, 20)

    # 读取 JSON
    with open(JSON_FILE, 'r', encoding='utf-8') as f:
        data = json.load(f)

    town_name = list(data.keys())[0]
    route_key = list(data[town_name])[0]
    scenario = data[town_name][route_key][0]

    # Trigger 位置
    trig = scenario['trigger_position']
    trig_loc = carla.Location(
        x=float(trig['x']),
        y=float(trig['y']),
        z=float(trig['z']) + 0.5
    )
    trig_rot = carla.Rotation(yaw=float(trig['yaw']))

    # 生成车辆
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    vehicle = world.spawn_actor(vehicle_bp, carla.Transform(trig_loc, trig_rot))
    print("✅ 车辆已生成在 Trigger 点")

    # 生成所有行人
    spawned_walkers = []
    actors = scenario['other_actors']['center']
    for idx, act in enumerate(actors):
        tra = act['transform']
        loc = carla.Location(
            x=float(tra['x']),
            y=float(tra['y']),
            z=float(tra['z']) + 0.2
        )
        rot = carla.Rotation(yaw=float(tra['yaw']))
        walker_bp = blueprint_library.filter('walker.pedestrian.0001')[0]
        walker = world.try_spawn_actor(walker_bp, carla.Transform(loc, rot))
        if walker:
            spawned_walkers.append(walker)
            print(f"✅ 行人 {idx+1} 生成成功")
        else:
            print(f"❌ 行人 {idx+1} 生成失败")

    if not spawned_walkers:
        print("⚠ 没有成功生成的行人")
        vehicle.destroy()
        return

    target_walker = spawned_walkers[0]
    print("\n========== 视角控制 ==========")
    print("F1：上帝俯视视角（看整体）")
    print("F2：车辆斜后方视角（看场景）")
    print("F3：行人正面视角（看朝向）")
    print("Q ：退出")
    print("==============================\n")

    running = True
    while running:
        # 小窗口显示提示
        screen.fill((30, 30, 30))
        screen.blit(font.render("F1:俯视  F2:跟车  F3:看行人", True, (255,255,255)), (10, 20))
        screen.blit(font.render("Q:退出", True, (255,100,100)), (10, 60))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                # F1 上帝俯视
                if event.key == pygame.K_F1:
                    spectator.set_transform(
                        carla.Transform(
                            trig_loc + carla.Location(z=30),
                            carla.Rotation(pitch=-90, yaw=0)
                        )
                    )
                    print("视角：上帝俯视")

                # F2 车辆后上方
                elif event.key == pygame.K_F2:
                    spectator.set_transform(
                        carla.Transform(
                            trig_loc + carla.Location(x=-8, z=5),
                            carla.Rotation(pitch=-15, yaw=trig_rot.yaw)
                        )
                    )
                    print("视角：车辆斜后方")

                # F3 看行人朝向（最关键）
                elif event.key == pygame.K_F3:
                    w_trans = target_walker.get_transform()
                    fwd = w_trans.get_forward_vector()
                    view_loc = w_trans.location - fwd * 3 + carla.Location(z=1.2)
                    spectator.set_transform(
                        carla.Transform(
                            view_loc,
                            carla.Rotation(pitch=0, yaw=w_trans.rotation.yaw)
                        )
                    )
                    print("视角：行人正面（看朝向）")

                # 退出
                elif event.key == pygame.K_q:
                    running = False

        time.sleep(0.05)

    # 清理
    vehicle.destroy()
    for w in spawned_walkers:
        w.destroy()
    pygame.quit()
    print("\n✅ 已清理所有Actor，验证结束")

if __name__ == '__main__':
    main()