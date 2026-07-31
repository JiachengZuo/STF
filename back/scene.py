import math
import json
import carla
import weakref
import time
from model.tcp import TCPRoutePlanner
from model.tcp import TCPAgent
import numpy as np

def get_available_waypoints(world, start_location, num_waypoints=50, step_distance=10.0):
    """
    从指定起点获取连续的可行waypoints
    参数：
        world: CARLA的world对象（已连接服务器）
        start_location: 起点位置（carla.Location对象）
        num_waypoints: 要生成的路点数量
        step_distance: 相邻路点的距离（米）
    返回：
        list[carla.Waypoint]: 可行的路点列表
    """
    # 1. 获取地图对象
    map = world.get_map()
    
    # 2. 获取起点对应的waypoint（确保在道路上，project_to_road=True强制投影到道路）
    start_waypoint = map.get_waypoint(
        start_location,
        project_to_road=True,  # 关键：将位置投影到最近的可行道路上
        lane_type=carla.LaneType.Driving  # 只获取行车道的路点（排除人行道/非机动车道）
    )
    
    if not start_waypoint:
        raise ValueError("起点位置无可行的行车道路点！")
    # Location(x=-41.831966, y=-16.555155, z=-0.001584)
#     (Pdb) print(available_waypoints[0])
# Waypoint(Transform(Location(x=-41.844612, y=-16.507988, z=0.000000), Rotation(pitch=0.000000, yaw=270.352692, roll=0.000000)))
# (Pdb) print(available_waypoints[7])
# Waypoint(Transform(Location(x=-23.784306, y=-57.742027, z=0.000000), Rotation(pitch=0.000000, yaw=0.596735, roll=0.000000)))
    # 3. 沿道路生成连续的可行路点（避免死胡同/非行车道）
    waypoints = []
    current_waypoint = start_waypoint
    for _ in range(num_waypoints):
        waypoints.append(current_waypoint)
        # 获取下一个路点（沿道路前进，step_distance米）
        next_waypoints = current_waypoint.next(step_distance)
        if not next_waypoints:
            break  # 无后续路点则停止
        # 优先选择主路（避免拐入小巷）
        current_waypoint = next_waypoints[0]
    
    return waypoints

# ============================
# 基础场景类（所有场景继承这个）
# ============================
class BaseScene:
    def __init__(self, client, world, config_path, town, route_id):
        self.client = client
        self.world = world
        self.town = town
        self.route_id = route_id
        self.config = self._load_config(config_path)
        self.ego = None
        self.actors = []

    def _load_config(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return data[self.town][self.route_id][0]

    # ======================
    # ✅ 已修改：从 JSON ego_start 读取坐标
    # ======================
    def spawn_ego(self, blueprint='vehicle.tesla.model3'):
        bp_lib = self.world.get_blueprint_library()
        ego_bp = bp_lib.find(blueprint)

        # 直接读取你编辑器生成的 ego_start 位置
        ego_cfg = self.config['ego_start']
        x = float(ego_cfg['x'])
        y = float(ego_cfg['y'])
        z = float(ego_cfg['z'])
        yaw = float(ego_cfg['yaw'])
        # print(ego_cfg)
        transform = carla.Transform(
            carla.Location(x, y, z),
            carla.Rotation(yaw=yaw)
        )

        ego = None
        for i in range(10):
            # 轻微调整 z 高度防止卡地面
            transform.location.z = z + i * 0.05
            ego = self.world.try_spawn_actor(ego_bp, transform)
            if ego:
                break
            time.sleep(0.1)

        self.ego = ego
        self.actors.append(ego)
        return ego

    def get_future_waypoints(self, length=12):
        waypoints = []
        wp = self.world.get_map().get_waypoint(self.ego.get_location())
        dist = 0
        while wp and dist < length:
            waypoints.append((wp.transform.location.x, wp.transform.location.y))
            nexts = wp.next(2.0)
            if nexts:
                wp = nexts[0]
                dist += 2
            else:
                break
        return waypoints

    def tick(self):
        pass

    def destroy(self):
        for actor in self.actors:
            if actor.is_alive:
                actor.destroy()


class CarCutOutandStaticScene(BaseScene):
    def __init__(self, client, world, config, town, route_id):
        super().__init__(client, world, config, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.cars = []
        self.directions = []
        self.cut_out_finish = []
        self.static_vehicle = None
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()

        # ----------------------
        # 生成动态切出车
        # ----------------------
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            if cfg['type'] == 'car':
                car_bp = bp_lib.find('vehicle.tesla.model3')
                tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
                car = self.world.try_spawn_actor(car_bp, tf)

                if car:
                    self.cars.append(car)
                    self.actors.append(car)
                    self.directions.append(0)
                    self.cut_out_finish.append(False)

        # ----------------------
        # 生成静态车（在 obstacle 位置）
        # ----------------------
        for cfg in self.config['other_actors']['center']:
            if cfg['type'] == 'obstacle':
                x = float(cfg['transform']['x'])
                y = float(cfg['transform']['y'])
                z = 0.3  # 固定安全高度，必生成
                yaw = float(cfg['transform']['yaw'])

                static_bp = bp_lib.find('vehicle.tesla.model3')
                static_tf = carla.Transform(
                    carla.Location(x, y, z),
                    carla.Rotation(yaw=yaw)
                )

                # 【修复】强制生成，无视碰撞（临时）
                self.static_vehicle = self.world.spawn_actor(
                    static_bp, static_tf
                )

                if self.static_vehicle:
                    self.actors.append(self.static_vehicle)
                    ctrl = carla.VehicleControl()
                    ctrl.brake = 1.0
                    ctrl.hand_brake = True
                    ctrl.throttle = 0.0
                    ctrl.steer = 0.0
                    self.static_vehicle.apply_control(ctrl)
                break

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

                for i, car in enumerate(self.cars):
                    self.directions[i] = self._get_safe_lane_direction(car)

        if self.triggered:
            for i, car in enumerate(self.cars):
                if not car.is_alive:
                    continue

                control = carla.VehicleControl()
                dir = self.directions[i]
                elapsed = time.time() - self.trigger_time

                control.throttle = 0.5
                control.brake = 0.0

                # ----------------------
                # 【修复】正常切出逻辑
                # ----------------------
                if elapsed < 0.5:
                    control.steer = 0.25 * dir
                elif elapsed < 1.0:
                    control.steer = -0.25 * dir
                else:
                    control.steer = 0.0
                    self.cut_out_finish[i] = True

                car.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

    def _get_safe_lane_direction(self, vehicle):
        loc = vehicle.get_location()
        wp = self.map.get_waypoint(loc, project_to_road=True)

        if not wp:
            return 0

        left_wp = wp.get_left_lane()
        right_wp = wp.get_right_lane()

        # 【修复】你的道路是反向车道，向右切出
        if right_wp and right_wp.lane_type == carla.LaneType.Driving:
            return -1
        if left_wp and left_wp.lane_type == carla.LaneType.Driving:
            return 1

        return 0

class CarOncomingPassScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.cars = []
        self.state = []           # 0:等待 1:切入ego车道 2:借道直行 3:切回原车道 4:回正
        self.cut_direction = []   # 转向方向
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            car_bp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(car_bp, tf)

            if car:
                self.cars.append(car)
                self.actors.append(car)
                self.state.append(0)
                self.cut_direction.append(0)

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < 18:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for idx, car in enumerate(self.cars):
                if not car.is_alive:
                    continue

                car_trans = car.get_transform()
                car_loc = car_trans.location
                ego_loc = self.ego.get_location()

                control = carla.VehicleControl()
                control.throttle = 0.5
                control.brake = 0.0

                # 第一步：判断朝向 Ego 车道的方向
                if self.state[idx] == 0:
                    dx = ego_loc.x - car_loc.x
                    dy = ego_loc.y - car_loc.y
                    yaw_car = math.radians(car_trans.rotation.yaw)
                    cross = dx * math.sin(yaw_car) - dy * math.cos(yaw_car)
                    self.cut_direction[idx] = -1 if cross > 0 else 1
                    self.state[idx] = 1

                # 第二步：小幅切入 Ego 车道
                elif self.state[idx] == 1:
                    # 方向只打一点点，非常柔和
                    control.steer = 0.10 * self.cut_direction[idx]
                    if time.time() - self.trigger_time > 1.2:
                        self.state[idx] = 2

                # 第三步：借道直行
                elif self.state[idx] == 2:
                    control.steer = 0.0
                    if time.time() - self.trigger_time > 2:
                        self.state[idx] = 3

                # 第四步：小幅切回原车道
                elif self.state[idx] == 3:
                    control.steer = -0.10 * self.cut_direction[idx]
                    if time.time() - self.trigger_time > 2.8:
                        self.state[idx] = 4

                # 第五步：回正直行
                elif self.state[idx] == 4:
                    control.steer = 0.0

                car.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > 12:
            return False
        return True


class CarCutOutScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.cars = []
        self.directions = []       # 1左 -1右 0不变道
        self.cut_out_finish = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            car_bp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(car_bp, tf)

            if car:
                self.cars.append(car)
                self.actors.append(car)
                self.directions.append(0)
                self.cut_out_finish.append(False)

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 12:
                self.triggered = True
                self.trigger_time = time.time()

                # 为每辆车判断安全变道方向
                for i, car in enumerate(self.cars):
                    self.directions[i] = self._get_safe_lane_direction(car)

        if self.triggered:
            for i, car in enumerate(self.cars):
                if not car.is_alive:
                    continue

                control = carla.VehicleControl()
                dir = self.directions[i]

                # 不变道 → 减速让行
                if dir == 0:
                    control.throttle = 0.0
                    control.brake = 0.5
                    control.steer = 0.0
                    car.apply_control(control)
                    continue

                # 已经完成变道 → 直行
                if self.cut_out_finish[i]:
                    control.throttle = 0.5
                    control.steer = 0.0
                    control.brake = 0.0
                    car.apply_control(control)
                    continue

                # 变道过程：短时间打方向
                control.throttle = 0.45
                if time.time() - self.trigger_time < 1.3:
                    control.steer = 0.20 * dir
                else:
                    control.steer = 0.0
                    self.cut_out_finish[i] = True
                control.brake = 0.0

                car.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

    def _get_safe_lane_direction(self, vehicle):
        loc = vehicle.get_location()
        wp = self.map.get_waypoint(loc, project_to_road=True)

        if not wp:
            return 0

        left_wp = wp.get_left_lane()
        right_wp = wp.get_right_lane()

        # 优先向左变道
        if left_wp and left_wp.lane_type == carla.LaneType.Driving:
            return 1

        # 其次向右变道
        if right_wp and right_wp.lane_type == carla.LaneType.Driving:
            return -1

        # 左右都没车道 → 不变道，直接刹车让行
        return 0

class CarCutInScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.cars = []
        self.cut_in_finish = []
        self.original_yaw = []  # 保存初始朝向，保证回正
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            car_bp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(car_bp, tf)

            if car:
                self.cars.append(car)
                self.actors.append(car)
                self.cut_in_finish.append(False)
                self.original_yaw.append(yaw)  # 保存初始角度

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for idx, car in enumerate(self.cars):
                if not car.is_alive or self.cut_in_finish[idx]:
                    continue

                trans = car.get_transform()
                car_loc = trans.location
                ego_loc = self.ego.get_location()

                # 目标：ego 前方 6 米
                fwd = self.ego.get_transform().get_forward_vector()
                target_x = ego_loc.x + fwd.x * 6.0
                target_y = ego_loc.y + fwd.y * 6.0

                # 计算方向
                dx = target_x - car_loc.x
                dy = target_y - car_loc.y
                dist = math.hypot(dx, dy)

                control = carla.VehicleControl()
                control.throttle = 0.5
                control.brake = 0.0

                # 切入完成条件
                if dist < 1.8 or time.time() - self.trigger_time > 2.2:
                    # ✅ 自动回正到车道方向
                    ego_yaw = math.radians(self.ego.get_transform().rotation.yaw)
                    car_yaw = math.radians(trans.rotation.yaw)
                    yaw_err = ego_yaw - car_yaw
                    yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

                    control.steer = max(min(1.8 * yaw_err, 0.15), -0.15)
                    
                    if abs(yaw_err) < 0.08:
                        self.cut_in_finish[idx] = True
                        control.steer = 0.0
                else:
                    # ✅ 平滑切入，限制最大转角，不会横车
                    desired_yaw = math.atan2(dy, dx)
                    car_yaw = math.radians(trans.rotation.yaw)
                    yaw_err = desired_yaw - car_yaw
                    yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
                    control.steer = max(min(1.6 * yaw_err, 0.22), -0.22)

                car.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

# ============================
# 【场景 1：行人横穿马路】TCP 集成完整版
# ============================
class PedestrianCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.walkers = []
        self.walker_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.walker_speed = 2.5
        self.ego = self.spawn_ego()
        self.model = model
        self.model_path = model_path

        # ======================
        # 修复：初始化控制变量
        # ======================
        self.control = carla.VehicleControl()  # <-- 必须加
        self.camera_data = None                # <-- 必须加
        self.tcp_flag = False                  # <-- 先默认关闭

        if self.model == 'tcp':
            self.tcp_flag = True
            self.planner = TCPRoutePlanner(self.world, self.ego)
            self.tcp = TCPAgent(self.model_path, self.planner)
            available_waypoints = get_available_waypoints(self.world, self.ego.get_location(), num_waypoints=1, step_distance=18.0)
            self.planner.set_route(available_waypoints)

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()

        # ======================
        # 修复：tcp_flag 一定存在
        # ======================
        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
            self.ego.set_autopilot(True)
            self.world.tick()

        # 生成行人
        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])
            wbp = bp_lib.find('walker.pedestrian.0001')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            walker = self.world.try_spawn_actor(wbp, tf)
            if walker:
                self.walkers.append(walker)
                self.actors.append(walker)
                angle = math.radians(yaw)
                self.walker_ctrls.append((math.cos(angle), math.sin(angle)))

    def spawn_camera(self):
        """ 挂载相机获取图像给 TCP 模型 """
        bp_lib = self.world.get_blueprint_library()
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800')
        cam_bp.set_attribute('image_size_y', '600')
        cam_bp.set_attribute('fov', '100')

        transform = carla.Transform(carla.Location(x=1.5, z=2.0))
        self.camera_sensor = self.world.spawn_actor(cam_bp, transform, attach_to=self.ego)

        def callback(image):
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape((image.height, image.width, 4))
            self.camera_data = array[:, :, :3]  # RGB

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        # ======================
        # TCP 主控制逻辑（完全正确）
        # ======================
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)

            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False

            self.ego.apply_control(self.control)

            current = self.ego.get_control()
            print(f"[TCP] throttle={current.throttle:.2f} steer={current.steer:.2f}")

        # 行人触发逻辑
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for w, (dx, dy) in zip(self.walkers, self.walker_ctrls):
                ctrl = carla.WalkerControl()
                ctrl.direction = carla.Vector3D(dx, dy, 0)
                ctrl.speed = self.walker_speed
                w.apply_control(ctrl)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

# ============================
# 【场景 3d：静态障碍物】
class OccludedPedestrianScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.walkers = []       # 行人
        self.walker_ctrls = []
        self.obstacles = []     # 遮挡用障碍物
        self.triggered = False
        self.trigger_time = 0
        self.walker_speed = 1.0  # 行人静止
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()

        # ======================
        # 遍历所有配置的物体
        # ======================
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])
            actor_type = cfg.get("type", "")

            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))

            # ======================
            # 生成行人
            # ======================
            if actor_type == "person":
                wbp = bp_lib.find('walker.pedestrian.0001')
                walker = self.world.try_spawn_actor(wbp, tf)
                if walker:
                    self.walkers.append(walker)
                    self.actors.append(walker)
                    angle = math.radians(yaw)
                    self.walker_ctrls.append((math.cos(angle), math.sin(angle)))

            # ======================
            # 生成障碍物（1个，从JSON读取位置）
            # ======================
            elif actor_type == "obstacle":
                # 用你配置里的 static.prop.container
                cone_bp = bp_lib.find('static.prop.container')
                obs = self.world.try_spawn_actor(cone_bp, tf)
                if obs:
                    self.obstacles.append(obs)
                    self.actors.append(obs)

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        # 行人保持静止
        if self.triggered:
            for w, (dx, dy) in zip(self.walkers, self.walker_ctrls):
                ctrl = carla.WalkerControl()
                ctrl.direction = carla.Vector3D(dx, dy, 0)
                ctrl.speed = self.walker_speed
                w.apply_control(ctrl)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

# ============================
# 【场景 1：行人横穿马路】
# ============================
class StaticPedestrianCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.walkers = []
        self.walker_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.walker_speed = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        # 生成行人
        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])
            wbp = bp_lib.find('walker.pedestrian.0001')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            walker = self.world.try_spawn_actor(wbp, tf)
            if walker:
                self.walkers.append(walker)
                self.actors.append(walker)
                angle = math.radians(yaw)
                self.walker_ctrls.append((math.cos(angle), math.sin(angle)))

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for w, (dx, dy) in zip(self.walkers, self.walker_ctrls):
                ctrl = carla.WalkerControl()
                ctrl.direction = carla.Vector3D(dx, dy, 0)
                ctrl.speed = self.walker_speed
                w.apply_control(ctrl)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

class StaticObstacleScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.obstacles = []       
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            obstacle_bp = bp_lib.find('static.prop.constructioncone')

            # ==========================
            # ✔ 在当前点周围 生成一圈路锥（施工效果）
            # ==========================
            offsets = [
                (0, 0), (1.5, 0), (-1.5, 0),
                (0, 1.5), (0, -1.5),
                (1.2, 1.2), (1.2, -1.2),
                (-1.2, 1.2), (-1.2, -1.2)
            ]

            for dx, dy in offsets:
                nx = x + dx
                ny = y + dy
                tf = carla.Transform(
                    carla.Location(nx, ny, z),
                    carla.Rotation(yaw=yaw)
                )
                obs = self.world.try_spawn_actor(obstacle_bp, tf)
                if obs:
                    self.obstacles.append(obs)
                    self.actors.append(obs)

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

class BicycleCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.bikes = []          # 对应 walkers
        self.bike_ctrls = []     # 对应 walker_ctrls
        self.triggered = False
        self.trigger_time = 0
        self.bike_speed = 2.5    # 自行车速度
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        # 生成自行车
        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            # 自行车蓝图
            wbp = bp_lib.find('vehicle.diamondback.century')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            bike = self.world.try_spawn_actor(wbp, tf)

            if bike:
                self.bikes.append(bike)
                self.actors.append(bike)
                angle = math.radians(yaw)
                self.bike_ctrls.append((math.cos(angle), math.sin(angle)))

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        # 自行车控制（使用 VehicleControl）
        if self.triggered:
            for w, (dx, dy) in zip(self.bikes, self.bike_ctrls):
                control = carla.VehicleControl()
                control.throttle = 1.0
                control.steer = 0.0
                control.brake = 0.0
                w.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

class CarStopandGoScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.cars = []
        self.car_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            # 汽车蓝图
            wbp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(wbp, tf)

            if car:
                self.cars.append(car)
                self.actors.append(car)
                angle = math.radians(yaw)
                self.car_ctrls.append((math.cos(angle), math.sin(angle)))

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            elapsed = time.time() - self.trigger_time  # 触发后经过的时间
            for w, (dx, dy) in zip(self.cars, self.car_ctrls):
                control = carla.VehicleControl()
                
                # ======================
                # 触发后 停1秒
                # ======================
                if elapsed < 2.0:
                    control.throttle = 0.0
                    control.brake = 1.0  # 刹车停住
                else:
                    # 1秒后起步走
                    control.throttle = 0.7
                    control.steer = 0.0
                    control.brake = 0.0
                
                w.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

class CarGoandStopScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.cars = []
        self.car_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            # 汽车蓝图
            wbp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(wbp, tf)

            if car:
                self.cars.append(car)
                self.actors.append(car)
                angle = math.radians(yaw)
                self.car_ctrls.append((math.cos(angle), math.sin(angle)))

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            elapsed = time.time() - self.trigger_time  # 触发后经过的时间
            for w, (dx, dy) in zip(self.cars, self.car_ctrls):
                control = carla.VehicleControl()
                
                # ======================
                # 前 4 秒：正常行驶
                # ======================
                if elapsed < 2.0:
                    control.throttle = 0.5
                    control.steer = 0.0
                    control.brake = 0.0
                
                # ======================
                # 4 秒后：紧急制动 急停
                # ======================
                else:
                    control.throttle = 0.0
                    control.brake = 1.0       # 急刹车
                    control.steer = 0.0
                
                w.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > 15:
            return False
        return True



class EgoRouteFollowScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.ego = None

        # 路线点
        self.route_points = []
        self.current_target_idx = 0
        self.finished = False

        # 循迹控制
        self.speed_limit = 8.0
        self.stop_distance = 2.0

        # 存储其他智能体
        self.agents = []

    def load_ego_route(self):
        """从你的JSON加载ego_route路线点"""
        try:
            route_data = self.config.get("ego_route", [])
            for p in route_data:
                loc = carla.Location(
                    float(p["x"]),
                    float(p["y"]),
                    float(p["z"])
                )
                self.route_points.append(loc)
            print(f"✅ EGO路线加载完成：{len(self.route_points)} 个点")
        except:
            self.route_points = []

    def spawn_ego(self):
        """生成自车"""
        ego_cfg = self.config["ego_start"]
        x = float(ego_cfg["x"])
        y = float(ego_cfg["y"])
        z = float(ego_cfg["z"])
        yaw = float(ego_cfg["yaw"])

        bp_lib = self.world.get_blueprint_library()
        ego_bp = bp_lib.find("vehicle.tesla.model3")
        transform = carla.Transform(
            carla.Location(x, y, z),
            carla.Rotation(yaw=yaw)
        )
        self.ego = self.world.spawn_actor(ego_bp, transform)
        self.actors.append(self.ego)
        return self.ego

    def spawn_agents(self):
        """生成 JSON 里的所有车辆 / 行人，自动行驶"""
        bp_lib = self.world.get_blueprint_library()

        for cfg in self.config['other_actors'].get('center', []):
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])
            model = cfg['model']
            atype = cfg.get('type', '')

            bp = bp_lib.find(model)
            if not bp:
                continue

            tf = carla.Transform(
                carla.Location(x, y, z),
                carla.Rotation(yaw=yaw)
            )

            actor = self.world.try_spawn_actor(bp, tf)
            if not actor:
                continue

            self.actors.append(actor)
            self.agents.append(actor)

            # 车辆 → 自动巡航
            if 'vehicle' in model:
                actor.set_autopilot(True)

            # 行人 → 自动行走（修复版，100%会动）
            if 'walker' in model or 'pedestrian' in model:
                try:
                    # 正确获取AI控制器
                    ai_bp = bp_lib.find('controller.ai.walker')
                    walker_controller = self.world.spawn_actor(ai_bp, carla.Transform(), attach_to=actor)
                    
                    # 必须等一帧再启动AI
                    self.world.tick()
                    
                    walker_controller.start()
                    # 设置目标点：向前走20米
                    target_loc = actor.get_location() + actor.get_transform().get_forward_vector() * 20.0
                    walker_controller.go_to_location(target_loc)
                    walker_controller.set_max_speed(1.5)  # 设置行人速度
                    
                    # 把控制器也加入销毁列表，防止内存泄漏
                    self.actors.append(walker_controller)
                except Exception as e:
                    print(f"行人AI启动失败: {e}")

    def spawn(self):
        """统一生成入口"""
        self.ego = self.spawn_ego()
        if not self.ego:
            raise RuntimeError("EGO生成失败！")

        # 生成其他车辆/行人
        self.spawn_agents()

        # 加载路线
        self.load_ego_route()

        # EGO 手动循迹
        self.ego.set_autopilot(False)

        time.sleep(0.2)
        self.world.tick()

    def has_obstacle_ahead(self):
        """前方障碍检测"""
        ego_tf = self.ego.get_transform()
        forward = ego_tf.get_forward_vector()
        start = ego_tf.location

        for actor in self.world.get_actors().filter("vehicle*"):
            if actor.id == self.ego.id:
                continue
            loc = actor.get_location()
            dist = start.distance(loc)
            if dist < 9.0:
                return True
        return False

    def follow_route(self):
        """EGO 路线循迹"""
        if not self.route_points or self.finished:
            return

        if self.current_target_idx >= len(self.route_points):
            control = carla.VehicleControl()
            control.brake = 1.0
            self.ego.apply_control(control)
            self.finished = True
            return

        target = self.route_points[self.current_target_idx]
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location

        dx = target.x - ego_loc.x
        dy = target.y - ego_loc.y
        dist = math.hypot(dx, dy)

        if dist < self.stop_distance:
            self.current_target_idx += 1
            return

        target_yaw = math.degrees(math.atan2(dy, dx))
        ego_yaw = ego_tf.rotation.yaw
        error = target_yaw - ego_yaw
        error = (error + 180) % 360 - 180

        vel = self.ego.get_velocity()
        speed = math.hypot(vel.x, vel.y)
        obstacle = False

        control = carla.VehicleControl()
        control.steer = max(min(error * 0.08, 1.0), -1.0)

        if obstacle or speed > self.speed_limit:
            control.throttle = 0.0
            control.brake = 0.3
        else:
            control.throttle = 0.45
            control.brake = 0.0

        self.ego.apply_control(control)

    def tick(self):
        if self.ego:
            self.follow_route()
        return not self.finished

class CarCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.cars = []
        self.car_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            # 汽车蓝图
            wbp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(wbp, tf)

            if car:
                self.cars.append(car)
                self.actors.append(car)
                angle = math.radians(yaw)
                self.car_ctrls.append((math.cos(angle), math.sin(angle)))

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for w, (dx, dy) in zip(self.cars, self.car_ctrls):
                control = carla.VehicleControl()
                control.throttle = 0.7
                control.steer = 0.0
                control.brake = 0.0
                w.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

class StaticCarCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.cars = []
        self.car_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

    def spawn(self):
        if not self.ego:
            raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        self.ego.set_autopilot(True)
        self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            # 汽车蓝图
            wbp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(wbp, tf)

            if car:
                self.cars.append(car)
                self.actors.append(car)
                angle = math.radians(yaw)
                self.car_ctrls.append((math.cos(angle), math.sin(angle)))

    def tick(self):
        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < 10:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for w, (dx, dy) in zip(self.cars, self.car_ctrls):
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.steer = 0.0
                control.brake = 0.0
                w.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > 10:
            return False
        return True

