#!/usr/bin/env python3
import carla
import json
import time
import pygame
import math
import numpy as np

# ====================== 配置 ======================
JSON_PATH = "./trigger_person_output.json"
HOST = '127.0.0.1'
PORT = 2000
ACTOR_SPEED = 2.5
EGO_SPEED = 12.0
TOWN = "TOWN10HD_Opt"
ROUTE_ID = "route_01"
# ==================================================

def load_trigger_config(json_path, town, route):
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return data[town][route][0]

def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 800))
    pygame.display.set_caption("CARLA 车辆俯视图 + 行人横穿")
    clock = pygame.time.Clock()

    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)
    world = client.get_world()  # 🔴 保留你原本的写法，不 reload 地图
    bp_lib = world.get_blueprint_library()
    scenario = load_trigger_config(JSON_PATH, TOWN, ROUTE_ID)

    trig = scenario['trigger_position']
    trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
    trig_yaw = float(trig['yaw'])

    # 车辆生成在JSON触发点后方
    ego_bp = bp_lib.find('vehicle.tesla.model3')
    spawn_offset = carla.Location(
        x=-35 * math.cos(math.radians(trig_yaw)),
        y=-35 * math.sin(math.radians(trig_yaw)),
        z=0.3
    )
    spawn_loc = trig_loc + spawn_offset
    ego_transform = carla.Transform(spawn_loc, carla.Rotation(yaw=trig_yaw))

    ego = None
    for retry in range(10):
        ego_transform.location.z = 0.2 + retry * 0.1
        ego = world.try_spawn_actor(ego_bp, ego_transform)
        if ego:
            print("✅ 车辆已生成在 JSON 触发点后方！")
            break
        time.sleep(0.1)

    if not ego:
        print("❌ 车辆生成失败")
        return

    ego.enable_constant_velocity(carla.Vector3D(EGO_SPEED, 0, 0))

    # 俯视相机
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '800')
    cam_bp.set_attribute('image_size_y', '800')
    cam_bp.set_attribute('fov', '110')

    camera = world.spawn_actor(
        cam_bp,
        carla.Transform(carla.Location(z=30), carla.Rotation(pitch=-90)),
        attach_to=ego
    )

    image_data = None
    def callback(image):
        nonlocal image_data
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        image_data = arr[:, :, :3][:, :, ::-1]

    camera.listen(callback)

    # ==================== 行人配置 ====================
    walkers = []
    walker_targets = []
    for cfg in scenario['other_actors']['center']:
        x = float(cfg['transform']['x'])
        y = float(cfg['transform']['y'])
        z = float(cfg['transform']['z'])
        yaw = float(cfg['transform']['yaw'])

        w_bp = bp_lib.find('walker.pedestrian.0001')
        tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
        walker = world.try_spawn_actor(w_bp, tf)
        if walker:
            walkers.append(walker)

            # ==============================================
            # ✅ 这里我完全学你的代码：行人 **垂直横穿马路**
            # ==============================================
            cross_yaw = yaw  
            dx = math.cos(math.radians(cross_yaw))
            dy = math.sin(math.radians(cross_yaw))
            walker_targets.append((dx, dy, ACTOR_SPEED))

    # ==================== 主循环 ====================
    triggered = False
    trigger_time = 0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # 触发行人横穿
        if not triggered and ego.get_location().distance(trig_loc) < 10:
            triggered = True
            trigger_time = time.time()
            print("🔥 触发！行人开始横穿")

        # 触发10秒后退出
        if triggered and time.time() - trigger_time > 10:
            print("⏹️ 10秒到，自动结束")
            break

        # ==============================================
        # ✅ 强制行人移动（100% 会动，和你逻辑一样）
        # ==============================================
        if triggered:
            for i, w in enumerate(walkers):
                dx, dy, speed = walker_targets[i]
                control = carla.WalkerControl()
                control.direction = carla.Vector3D(dx, dy, 0)
                control.speed = speed
                w.apply_control(control)

        # 显示俯视图
        if image_data is not None:
            surf = pygame.surfarray.make_surface(image_data.swapaxes(0, 1))
            screen.blit(surf, (0, 0))

        pygame.display.flip()
        world.tick()
        clock.tick(30)

    # 清理
    camera.stop()
    camera.destroy()
    ego.destroy()
    for w in walkers:
        w.destroy()
    pygame.quit()

if __name__ == '__main__':
    main()