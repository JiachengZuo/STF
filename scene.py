import math
import json
import carla
import weakref
import time
from model.tcp import TCPRoutePlanner
from model.tcp import TCPAgent
import numpy as np
from collision_enhancer import load_collision_config, get_collision_config, get_adjusted_trigger_distance, get_adjusted_throttle, apply_ga_params_to_npcs, get_optimized_npc_control, SimpleGeneticOptimizer, get_npc_speed_boost

# Global collision enhancement config (loaded once, shared by all scenes)
_collision_enhance_cfg = None

def get_collision_enhance_config():
    global _collision_enhance_cfg
    if _collision_enhance_cfg is None:
        _collision_enhance_cfg = load_collision_config()
    return _collision_enhance_cfg

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
        print(ego_cfg)
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

# ============================
# 车辆切出 + 静态障碍车（标准化 + TCP兼容）
# ============================
class CarCutOutandStaticScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.cars = []
        self.directions = []
        self.cut_out_finish = []
        self.static_vehicle = None
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

        # ======================
        # 标准 TCP 初始化（和你给的完全一致）
        # ======================
        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

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

        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
            self.ego.set_autopilot(True)
            self.world.tick()

        bp_lib = self.world.get_blueprint_library()

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

        for cfg in self.config['other_actors']['center']:
            if cfg['type'] == 'obstacle':
                x = float(cfg['transform']['x'])
                y = float(cfg['transform']['y'])
                z = 0.3
                yaw = float(cfg['transform']['yaw'])
                static_bp = bp_lib.find('vehicle.tesla.model3')
                static_tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
                self.static_vehicle = self.world.spawn_actor(static_bp, static_tf)
                if self.static_vehicle:
                    self.actors.append(self.static_vehicle)
                    ctrl = carla.VehicleControl()
                    ctrl.brake = 1.0
                    ctrl.hand_brake = True
                    ctrl.throttle = 0.0
                    ctrl.steer = 0.0
                    self.static_vehicle.apply_control(ctrl)
                break

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        # ======================
        # TCP 控制（标准格式）
        # ======================
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'cut_out_static', 5, config)
        adj_throttle = get_adjusted_throttle(0.5, 'cut_out_static', config)
        boost = get_npc_speed_boost('cut_out_static', config)
        timeout = 10.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 10.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()
                for i, car in enumerate(self.cars):
                    self.directions[i] = self._get_safe_lane_direction(car)

        if self.triggered:
            for i, car in enumerate(self.cars):
                if not car.is_alive: continue
                control = carla.VehicleControl()
                dir = self.directions[i]
                elapsed = time.time() - self.trigger_time
                control.throttle = min(1.0, adj_throttle * boost)
                control.brake = 0.0
                if elapsed < 0.5:
                    control.steer = 0.25 * dir
                elif elapsed < 1.0:
                    control.steer = -0.25 * dir
                else:
                    control.steer = 0.0
                    self.cut_out_finish[i] = True
                car.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True

    def _get_safe_lane_direction(self, vehicle):
        loc = vehicle.get_location()
        wp = self.map.get_waypoint(loc, project_to_road=True)
        if not wp: return 0
        left_wp = wp.get_left_lane()
        right_wp = wp.get_right_lane()
        if right_wp and right_wp.lane_type == carla.LaneType.Driving:
            return -1
        if left_wp and left_wp.lane_type == carla.LaneType.Driving:
            return 1
        return 0

# ============================
# 对向借道（标准化 + TCP兼容）
# ============================
class CarOncomingPassScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.cars = []
        self.state = []
        self.cut_direction = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

        if self.model == 'tcp':
            self.tcp_flag = True
            self.planner = TCPRoutePlanner(self.world, self.ego)
            self.tcp = TCPAgent(self.model_path, self.planner)
            available_waypoints = get_available_waypoints(self.world, self.ego.get_location(), num_waypoints=1, step_distance=18.0)
            self.planner.set_route(available_waypoints)

    def spawn(self):
        if not self.ego: raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
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

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]
        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(18, 'oncoming_pass', 4, config)
        adj_throttle = get_adjusted_throttle(0.5, 'oncoming_pass', config)
        boost = get_npc_speed_boost('oncoming_pass', config)
        timeout = 12.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 12.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for idx, car in enumerate(self.cars):
                if not car.is_alive: continue
                control = carla.VehicleControl()
                control.throttle = min(1.0, adj_throttle * boost)
                control.brake = 0.0
                if self.state[idx] == 0:
                    trans = car.get_transform()
                    dx = self.ego.get_location().x - car.get_location().x
                    dy = self.ego.get_location().y - car.get_location().y
                    yaw_car = math.radians(trans.rotation.yaw)
                    cross = dx * math.sin(yaw_car) - dy * math.cos(yaw_car)
                    self.cut_direction[idx] = -1 if cross > 0 else 1
                    self.state[idx] = 1
                elif self.state[idx] == 1:
                    control.steer = 0.10 * self.cut_direction[idx]
                    if time.time() - self.trigger_time > 1.2:
                        self.state[idx] = 2
                elif self.state[idx] == 2:
                    control.steer = 0.0
                    if time.time() - self.trigger_time > 2:
                        self.state[idx] = 3
                elif self.state[idx] == 3:
                    control.steer = -0.10 * self.cut_direction[idx]
                    if time.time() - self.trigger_time > 2.8:
                        self.state[idx] = 4
                elif self.state[idx] == 4:
                    control.steer = 0.0
                car.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True

# ============================
# 车辆切出（标准化 + TCP兼容）
# ============================
class CarCutOutScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.cars = []
        self.directions = []
        self.cut_out_finish = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

        if self.model == 'tcp':
            self.tcp_flag = True
            self.planner = TCPRoutePlanner(self.world, self.ego)
            self.tcp = TCPAgent(self.model_path, self.planner)
            available_waypoints = get_available_waypoints(self.world, self.ego.get_location(), num_waypoints=1, step_distance=18.0)
            self.planner.set_route(available_waypoints)

    def spawn(self):
        if not self.ego: raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
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

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]
        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(12, 'car_cut_out', 4, config)
        boost = get_npc_speed_boost('car_cut_out', config)
        cut_out_delay = 1.3
        merge_throttle = 0.5
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            type_overrides = config.get('npc_speed', {}).get('overrides', {}).get('car_cut_out', {})
            if 'cut_out_delay' in type_overrides:
                cut_out_delay = type_overrides['cut_out_delay']
            if 'merge_throttle' in type_overrides:
                merge_throttle = type_overrides['merge_throttle']
            timeout = 10.0 * t_scale
        else:
            timeout = 10.0

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()
                for i, car in enumerate(self.cars):
                    self.directions[i] = self._get_safe_lane_direction(car)

        if self.triggered:
            for i, car in enumerate(self.cars):
                if not car.is_alive: continue
                control = carla.VehicleControl()
                dir = self.directions[i]
                if dir == 0:
                    control.throttle = 0.0
                    control.brake = 0.5
                    control.steer = 0.0
                    car.apply_control(control)
                    continue
                if self.cut_out_finish[i]:
                    control.throttle = min(1.0, merge_throttle * boost)
                    control.steer = 0.0
                    control.brake = 0.0
                    car.apply_control(control)
                    continue
                control.throttle = min(1.0, 0.45 * boost)
                if time.time() - self.trigger_time < cut_out_delay:
                    control.steer = 0.20 * dir
                else:
                    control.steer = 0.0
                    self.cut_out_finish[i] = True
                control.brake = 0.0
                car.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True

    def _get_safe_lane_direction(self, vehicle):
        loc = vehicle.get_location()
        wp = self.map.get_waypoint(loc, project_to_road=True)
        if not wp: return 0
        left_wp = wp.get_left_lane()
        right_wp = wp.get_right_lane()
        if left_wp and left_wp.lane_type == carla.LaneType.Driving: return 1
        if right_wp and right_wp.lane_type == carla.LaneType.Driving: return -1
        return 0

# ============================
# 车辆切入（标准化 + TCP兼容）
# ============================
class CarCutInScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.cars = []
        self.cut_in_finish = []
        self.original_yaw = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

        if self.model == 'tcp':
            self.tcp_flag = True
            self.planner = TCPRoutePlanner(self.world, self.ego)
            self.tcp = TCPAgent(self.model_path, self.planner)
            available_waypoints = get_available_waypoints(self.world, self.ego.get_location(), num_waypoints=1, step_distance=18.0)
            self.planner.set_route(available_waypoints)

    def spawn(self):
        if not self.ego: raise RuntimeError("自车生成失败！")
        time.sleep(0.2)
        self.world.tick()
        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
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
                self.original_yaw.append(yaw)

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]
        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'car_cut_in', 4, config)
        adj_throttle = get_adjusted_throttle(0.5, 'car_cut_in', config)
        boost = get_npc_speed_boost('car_cut_in', config)
        timeout = 10.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 10.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for idx, car in enumerate(self.cars):
                if not car.is_alive or self.cut_in_finish[idx]: continue
                trans = car.get_transform()
                car_loc = trans.location
                ego_loc = self.ego.get_location()
                fwd = self.ego.get_transform().get_forward_vector()
                target_x = ego_loc.x + fwd.x * 6.0
                target_y = ego_loc.y + fwd.y * 6.0
                dx = target_x - car_loc.x
                dy = target_y - car_loc.y
                dist = math.hypot(dx, dy)
                control = carla.VehicleControl()
                control.throttle = min(1.0, adj_throttle * boost)
                control.brake = 0.0

                if dist < 1.8 or time.time() - self.trigger_time > 2.2:
                    ego_yaw = math.radians(self.ego.get_transform().rotation.yaw)
                    car_yaw = math.radians(trans.rotation.yaw)
                    yaw_err = ego_yaw - car_yaw
                    yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
                    control.steer = max(min(1.8 * yaw_err, 0.15), -0.15)
                    if abs(yaw_err) < 0.08:
                        self.cut_in_finish[idx] = True
                        control.steer = 0.0
                else:
                    desired_yaw = math.atan2(dy, dx)
                    car_yaw = math.radians(trans.rotation.yaw)
                    yaw_err = desired_yaw - car_yaw
                    yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
                    control.steer = max(min(1.6 * yaw_err, 0.22), -0.22)
                car.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > timeout:
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

        # 行人触发逻辑 (collision-enhanced)
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'pedestrian', 3, config)
        walker_speed = self.walker_speed
        if config.get('global', {}).get('enabled', True):
            speed_cfg = config.get('npc_speed', {}).get('overrides', {}).get('pedestrian', {})
            base_speed = speed_cfg.get('speed', self.walker_speed)
            boost = get_npc_speed_boost('pedestrian', config)
            walker_speed = base_speed * boost
        timeout = 10.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 10.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(
                float(trig['x']),
                float(trig['y']),
                float(trig['z'])
            )
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for w, (dx, dy) in zip(self.walkers, self.walker_ctrls):
                ctrl = carla.WalkerControl()
                ctrl.direction = carla.Vector3D(dx, dy, 0)
                ctrl.speed = walker_speed
                w.apply_control(ctrl)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True


class OccludedPedestrianScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.walkers = []
        self.walker_ctrls = []
        self.obstacles = []
        self.triggered = False
        self.trigger_time = 0
        self.walker_speed = 1.0
        self.ego = self.spawn_ego()

        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

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

        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
            self.ego.set_autopilot(True)
            self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])
            actor_type = cfg.get("type", "")
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))

            if actor_type == "person":
                wbp = bp_lib.find('walker.pedestrian.0001')
                walker = self.world.try_spawn_actor(wbp, tf)
                if walker:
                    self.walkers.append(walker)
                    self.actors.append(walker)
                    angle = math.radians(yaw)
                    self.walker_ctrls.append((math.cos(angle), math.sin(angle)))

            elif actor_type == "obstacle":
                cone_bp = bp_lib.find('static.prop.container')
                obs = self.world.try_spawn_actor(cone_bp, tf)
                if obs:
                    self.obstacles.append(obs)
                    self.actors.append(obs)

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'pedestrian', 3, config)
        walker_speed = self.walker_speed
        if config.get('global', {}).get('enabled', True):
            speed_cfg = config.get('npc_speed', {}).get('overrides', {}).get('pedestrian', {})
            base_speed = speed_cfg.get('speed', self.walker_speed)
            boost = get_npc_speed_boost('pedestrian', config)
            walker_speed = base_speed * boost
        timeout = 8.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 8.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for w, (dx, dy) in zip(self.walkers, self.walker_ctrls):
                ctrl = carla.WalkerControl()
                ctrl.direction = carla.Vector3D(dx, dy, 0)
                ctrl.speed = walker_speed
                w.apply_control(ctrl)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True

# ============================
# 静态行人横穿（标准化 + TCP）
# ============================
class StaticPedestrianCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.walkers = []
        self.walker_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.walker_speed = 0
        self.ego = self.spawn_ego()

        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

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

        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
            self.ego.set_autopilot(True)
            self.world.tick()

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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'pedestrian', 3, config)
        timeout = 10.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 10.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for w, (dx, dy) in zip(self.walkers, self.walker_ctrls):
                ctrl = carla.WalkerControl()
                ctrl.direction = carla.Vector3D(dx, dy, 0)
                ctrl.speed = self.walker_speed
                w.apply_control(ctrl)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True

# ============================
# 静态障碍物场景（标准化 + TCP）
# ============================
class StaticObstacleScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.obstacles = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

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

        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
            self.ego.set_autopilot(True)
            self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])
            obstacle_bp = bp_lib.find('static.prop.constructioncone')

            offsets = [
                (0, 0), (1.5, 0), (-1.5, 0),
                (0, 1.5), (0, -1.5),
                (1.2, 1.2), (1.2, -1.2),
                (-1.2, 1.2), (-1.2, -1.2)
            ]

            for dx, dy in offsets:
                nx = x + dx
                ny = y + dy
                tf = carla.Transform(carla.Location(nx, ny, z), carla.Rotation(yaw=yaw))
                obs = self.world.try_spawn_actor(obstacle_bp, tf)
                if obs:
                    self.obstacles.append(obs)
                    self.actors.append(obs)

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'static_obstacle', 2, config)
        timeout = 10.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 10.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True

# ============================
# 自行车横穿场景（标准化 + TCP）
# ============================
class BicycleCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.bikes = []
        self.bike_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.bike_speed = 2.5
        self.ego = self.spawn_ego()

        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

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

        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
            self.ego.set_autopilot(True)
            self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])
            wbp = bp_lib.find('vehicle.diamondback.century')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            bike = self.world.try_spawn_actor(wbp, tf)
            if bike:
                self.bikes.append(bike)
                self.actors.append(bike)
                angle = math.radians(yaw)
                self.bike_ctrls.append((math.cos(angle), math.sin(angle)))

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'bicycle', 3, config)
        boost = get_npc_speed_boost('bicycle', config)
        timeout = 10.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 10.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            for w, (dx, dy) in zip(self.bikes, self.bike_ctrls):
                control = carla.VehicleControl()
                control.throttle = min(1.0, self.bike_speed * boost / 5.0)
                control.steer = 0.0
                control.brake = 0.0
                w.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True

# ============================
# 停车起步场景（标准化 + TCP）
# ============================
class CarStopandGoScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.cars = []
        self.car_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

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

        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
            self.ego.set_autopilot(True)
            self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])
            wbp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(wbp, tf)
            if car:
                self.cars.append(car)
                self.actors.append(car)
                angle = math.radians(yaw)
                self.car_ctrls.append((math.cos(angle), math.sin(angle)))

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'stop_and_go', 4, config)
        adj_throttle = get_adjusted_throttle(0.7, 'stop_and_go', config)
        boost = get_npc_speed_boost('stop_and_go', config)
        timeout = 15.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 15.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            elapsed = time.time() - self.trigger_time
            for w, (dx, dy) in zip(self.cars, self.car_ctrls):
                control = carla.VehicleControl()
                if elapsed < 2.0:
                    control.throttle = 0.0
                    control.brake = 1.0
                else:
                    control.throttle = min(1.0, adj_throttle * boost)
                    control.steer = 0.0
                    control.brake = 0.0
                w.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True

# ============================
# 行驶急停场景（标准化 + TCP）
# ============================
class CarGoandStopScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.cars = []
        self.car_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

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

        if self.tcp_flag:
            print("[TCP] 模型已加载，启用TCP控制")
            self.spawn_camera()
            self.world.tick()
        else:
            self.ego.set_autopilot(True)
            self.world.tick()

        bp_lib = self.world.get_blueprint_library()
        for cfg in self.config['other_actors']['center']:
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])
            wbp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(wbp, tf)
            if car:
                self.cars.append(car)
                self.actors.append(car)
                angle = math.radians(yaw)
                self.car_ctrls.append((math.cos(angle), math.sin(angle)))

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'car_cross', 4, config)
        adj_throttle = get_adjusted_throttle(0.5, 'car_cross', config)
        boost = get_npc_speed_boost('car_cross', config)
        timeout = 15.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 15.0 * t_scale

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            elapsed = time.time() - self.trigger_time
            for car in self.cars:
                control = carla.VehicleControl()
                if elapsed < 2.0:
                    control.throttle = min(1.0, adj_throttle * boost)
                    control.steer = 0.0
                    control.brake = 0.0
                else:
                    control.throttle = 0.0
                    control.brake = 1.0
                    control.steer = 0.0
                car.apply_control(control)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True
    


class EgoRouteFollowScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
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

        # ======================
        # TCP 集成
        # ======================
        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False
        self.planner = None
        self.tcp = None

        # ✅【新增】统一超时退出（和所有场景一样）
        self.triggered = False
        self.trigger_time = 0
        self.timeout = 10  # 10秒超时

        if self.model == 'tcp' and self.model_path is not None:
            self.tcp_flag = True

    def load_ego_route(self):
        """从 JSON 加载路线点"""
        try:
            route_data = self.config.get("ego_route", [])
            for p in route_data:
                loc = carla.Location(
                    float(p["x"]),
                    float(p["y"]),
                    float(p["z"])
                )
                self.route_points.append(loc)
            print(f"✅ EGO 路线加载完成：{len(self.route_points)} 个点")
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
        """生成其他智能体"""
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

            if 'vehicle' in model:
                actor.set_autopilot(True)

            if 'walker' in model or 'pedestrian' in model:
                try:
                    ai_bp = bp_lib.find('controller.ai.walker')
                    walker_controller = self.world.spawn_actor(ai_bp, carla.Transform(), attach_to=actor)
                    self.world.tick()
                    walker_controller.start()
                    target_loc = actor.get_location() + actor.get_transform().get_forward_vector() * 20.0
                    walker_controller.go_to_location(target_loc)
                    walker_controller.set_max_speed(1.5)
                    self.actors.append(walker_controller)
                except Exception as e:
                    print(f"行人AI启动失败: {e}")

    def spawn(self):
        """生成入口"""
        self.ego = self.spawn_ego()
        if not self.ego:
            raise RuntimeError("EGO生成失败！")

        self.load_ego_route()
        self.ego.set_autopilot(False)
        self.spawn_agents()

        # ======================
        # ✅ 修复：强制正确创建 planner，不传入错误参数
        # ======================
        if self.tcp_flag:
            # 正确初始化：只传 world 和 ego
            self.planner = TCPRoutePlanner()
            self.tcp = TCPAgent(self.model_path, self.planner)

            # 把 Location 转成 waypoint
            waypoints = []
            for loc in self.route_points:
                wp = self.map.get_waypoint(loc, project_to_road=True)
                if wp:
                    waypoints.append(wp)

            if waypoints:
                self.planner.set_route(waypoints)
                print(f"[TCP] 加载路线成功：{len(waypoints)} 个路径点")

            self.spawn_camera()

        time.sleep(0.2)
        self.world.tick()

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def has_obstacle_ahead(self):
        ego_tf = self.ego.get_transform()
        start = ego_tf.location
        for actor in self.world.get_actors().filter("vehicle*"):
            if actor.id == self.ego.id:
                continue
            loc = actor.get_location()
            if start.distance(loc) < 9.0:
                return True
        return False

    def follow_route(self):
        if not self.route_points or self.finished:
            return

        if self.current_target_idx >= len(self.route_points):
            control = carla.VehicleControl()
            control.brake = 1.0
            self.ego.apply_control(control)
            self.finished = True
            return

        target = self.route_points[self.current_target_idx]
        ego_loc = self.ego.get_location()
        dist = ego_loc.distance(target)

        if dist < self.stop_distance:
            self.current_target_idx += 1
            return

        dx = target.x - ego_loc.x
        dy = target.y - ego_loc.y
        target_yaw = math.degrees(math.atan2(dy, dx))
        ego_yaw = self.ego.get_transform().rotation.yaw
        error = target_yaw - ego_yaw
        error = (error + 180) % 360 - 180

        vel = self.ego.get_velocity()
        speed = math.hypot(vel.x, vel.y)
        obstacle = self.has_obstacle_ahead()

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
        if not self.ego:
            return True

        # ✅【新增】触发计时（一启动就开始算10秒）
        if not self.triggered:
            self.triggered = True
            self.trigger_time = time.time()

        # ✅【新增】10秒超时自动退出（和所有场景完全一样）
        if self.triggered and time.time() - self.trigger_time > self.timeout:
            print(f"⏹️  EGO循迹场景 {self.timeout}秒超时，自动退出")
            return False

        if self.tcp_flag and self.camera_data is not None:
            try:
                img_np = self.camera_data
                action = self.tcp.get_action(img_np, self.ego)
                self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
                self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
                self.control.brake = 0.0
                self.control.hand_brake = False
                self.ego.apply_control(self.control)
                print(f"[TCP] throttle={self.control.throttle:.2f} steer={self.control.steer:.2f}")
            except Exception as e:
                print(f"[TCP] 控制异常，切换为手动循迹: {e}")
                self.tcp_flag = False
                self.follow_route()
        else:
            self.follow_route()

        return not self.finished
# ============================
# 车辆横穿场景（标准化 + TCP兼容）
# ============================
class CarCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.cars = []
        self.car_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

        # 统一 TCP 初始化
        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

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

        # ---- Collision enhancement: run GA optimization for NPC states ----
        config = get_collision_enhance_config()
        ego_loc = self.ego.get_location()
        ego_vel = self.ego.get_velocity()
        ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2)
        ego_yaw = self.ego.get_transform().rotation.yaw

        self.ga_params = None
        if config.get('global', {}).get('enabled', True) and config.get('global', {}).get('ga', {}).get('enabled', True):
            npc_base_loc = carla.Location(0, 0, 0)
            npc_base_yaw = 0.0
            if self.config.get('other_actors', {}).get('center'):
                first_npc = self.config['other_actors']['center'][0]
                npc_base_loc = carla.Location(
                    float(first_npc['transform']['x']),
                    float(first_npc['transform']['y']),
                    float(first_npc['transform']['z'])
                )
                npc_base_yaw = float(first_npc['transform']['yaw'])

            try:
                optimizer = SimpleGeneticOptimizer(
                    scenario_type='car_cross',
                    ego_location=ego_loc,
                    ego_speed=max(ego_speed, 5.0),
                    ego_heading=ego_yaw,
                    npc_base_location=npc_base_loc,
                    npc_base_yaw=npc_base_yaw,
                    config=config
                )
                self.ga_params = optimizer.optimize()
                print(f"[GA] Optimized NPC params: speed={self.ga_params['actor_speed']:.1f}m/s, "
                      f"decel_dist={self.ga_params['deceleration_trigger_distance']:.1f}m")
            except Exception as e:
                print(f"[GA] Optimization failed: {e}, using defaults")

        bp_lib = self.world.get_blueprint_library()
        for i, cfg in enumerate(self.config['other_actors']['center']):
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            wbp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(wbp, tf)

            if car:
                self.cars.append(car)
                self.actors.append(car)
                angle = math.radians(yaw)
                self.car_ctrls.append((math.cos(angle), math.sin(angle)))

                # Apply GA-optimized parameters to this NPC
                if self.ga_params:
                    car._optimized_actor_speed = self.ga_params['actor_speed']
                    car._optimized_decel_dist = self.ga_params['deceleration_trigger_distance']
                    car._optimized_scenario_trigger = self.ga_params['scenario_trigger_distance']
                    offset = self.ga_params['position_offset']
                    fwd = carla.Vector3D(math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0)
                    right = carla.Vector3D(-math.sin(math.radians(yaw)), math.cos(math.radians(yaw)), 0)
                    offset_vec = right * offset['x'] + fwd * offset['y']
                    car.set_location(carla.Location(x + offset_vec.x, y + offset_vec.y, z))

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        # TCP 控制逻辑
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement with GA integration ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'car_cross', 3, config)
        adj_throttle = get_adjusted_throttle(0.7, 'car_cross', config)
        boost = get_npc_speed_boost('car_cross', config)
        timeout = 10.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 10.0 * t_scale
        # Use GA-optimized trigger distance if available
        if self.ga_params:
            adj_trigger = self.ga_params.get('scenario_trigger_distance', adj_trigger)
            if self.ga_params.get('deceleration_trigger_distance'):
                adj_trigger = self.ga_params['deceleration_trigger_distance']

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            ego_loc = self.ego.get_location()
            ego_vel = self.ego.get_velocity()
            ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2)
            for w in self.cars:
                if not w.is_alive:
                    continue
                # Use GA-optimized NPC control if available
                if self.ga_params:
                    ctrl = get_optimized_npc_control(
                        w, ego_loc, ego_speed, self.triggered, self.trigger_time,
                        config=config
                    )
                    if not ctrl.throttle and not ctrl.brake:
                        ctrl.throttle = min(1.0, adj_throttle * boost)
                        ctrl.brake = 0.0
                else:
                    ctrl = carla.VehicleControl()
                    ctrl.throttle = min(1.0, adj_throttle * boost)
                    ctrl.steer = 0.0
                    ctrl.brake = 0.0
                w.apply_control(ctrl)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True

# ============================
# 静态车辆横穿场景（标准化 + TCP兼容）
# ============================
class StaticCarCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.cars = []
        self.car_ctrls = []
        self.triggered = False
        self.trigger_time = 0
        self.ego = self.spawn_ego()

        # 统一 TCP 初始化
        self.model = model
        self.model_path = model_path
        self.control = carla.VehicleControl()
        self.camera_data = None
        self.tcp_flag = False

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

        # ---- Collision enhancement: run GA optimization for NPC states ----
        config = get_collision_enhance_config()
        ego_loc = self.ego.get_location()
        ego_vel = self.ego.get_velocity()
        ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2)
        ego_yaw = self.ego.get_transform().rotation.yaw

        self.ga_params = None
        if config.get('global', {}).get('enabled', True) and config.get('global', {}).get('ga', {}).get('enabled', True):
            npc_base_loc = carla.Location(0, 0, 0)
            npc_base_yaw = 0.0
            if self.config.get('other_actors', {}).get('center'):
                first_npc = self.config['other_actors']['center'][0]
                npc_base_loc = carla.Location(
                    float(first_npc['transform']['x']),
                    float(first_npc['transform']['y']),
                    float(first_npc['transform']['z'])
                )
                npc_base_yaw = float(first_npc['transform']['yaw'])

            try:
                optimizer = SimpleGeneticOptimizer(
                    scenario_type='car_cross',
                    ego_location=ego_loc,
                    ego_speed=max(ego_speed, 5.0),
                    ego_heading=ego_yaw,
                    npc_base_location=npc_base_loc,
                    npc_base_yaw=npc_base_yaw,
                    config=config
                )
                self.ga_params = optimizer.optimize()
                print(f"[GA] Optimized NPC params: speed={self.ga_params['actor_speed']:.1f}m/s, "
                      f"decel_dist={self.ga_params['deceleration_trigger_distance']:.1f}m")
            except Exception as e:
                print(f"[GA] Optimization failed: {e}, using defaults")

        bp_lib = self.world.get_blueprint_library()
        for i, cfg in enumerate(self.config['other_actors']['center']):
            x = float(cfg['transform']['x'])
            y = float(cfg['transform']['y'])
            z = float(cfg['transform']['z'])
            yaw = float(cfg['transform']['yaw'])

            wbp = bp_lib.find('vehicle.tesla.model3')
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw))
            car = self.world.try_spawn_actor(wbp, tf)

            if car:
                self.cars.append(car)
                self.actors.append(car)
                angle = math.radians(yaw)
                self.car_ctrls.append((math.cos(angle), math.sin(angle)))

                # Apply GA-optimized parameters to this NPC
                if self.ga_params:
                    car._optimized_actor_speed = self.ga_params['actor_speed']
                    car._optimized_decel_dist = self.ga_params['deceleration_trigger_distance']
                    car._optimized_scenario_trigger = self.ga_params['scenario_trigger_distance']
                    offset = self.ga_params['position_offset']
                    fwd = carla.Vector3D(math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0)
                    right = carla.Vector3D(-math.sin(math.radians(yaw)), math.cos(math.radians(yaw)), 0)
                    offset_vec = right * offset['x'] + fwd * offset['y']
                    car.set_location(carla.Location(x + offset_vec.x, y + offset_vec.y, z))

    def spawn_camera(self):
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
            self.camera_data = array[:, :, :3]

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    def tick(self):
        # TCP 控制逻辑
        if self.tcp_flag and self.camera_data is not None:
            img_np = self.camera_data
            action = self.tcp.get_action(img_np, self.ego)
            self.control.throttle = max(0.0, min(1.0, float(action[0][0])))
            self.control.steer = max(-1.0, min(1.0, float(action[0][1])))
            self.control.brake = 0.0
            self.control.hand_brake = False
            self.ego.apply_control(self.control)

        # ---- Collision enhancement with GA integration ----
        config = get_collision_enhance_config()
        adj_trigger = get_adjusted_trigger_distance(10, 'car_cross', 3, config)
        timeout = 10.0
        if config.get('global', {}).get('enabled', True):
            t_scale = config.get('collision_profile', {}).get('timeout_scale', 1.2)
            timeout = 10.0 * t_scale
        # Use GA-optimized trigger distance if available
        if self.ga_params:
            adj_trigger = self.ga_params.get('scenario_trigger_distance', adj_trigger)
            if self.ga_params.get('deceleration_trigger_distance'):
                adj_trigger = self.ga_params['deceleration_trigger_distance']

        if not self.triggered:
            trig = self.config['trigger_position']
            trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
            if self.ego.get_location().distance(trig_loc) < adj_trigger:
                self.triggered = True
                self.trigger_time = time.time()

        if self.triggered:
            ego_loc = self.ego.get_location()
            ego_vel = self.ego.get_velocity()
            ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2)
            for w in self.cars:
                if not w.is_alive:
                    continue
                if self.ga_params:
                    ctrl = get_optimized_npc_control(
                        w, ego_loc, ego_speed, self.triggered, self.trigger_time,
                        config=config
                    )
                else:
                    ctrl = carla.VehicleControl()
                    ctrl.throttle = 0.0
                    ctrl.steer = 0.0
                    ctrl.brake = 0.0
                w.apply_control(ctrl)

        if self.triggered and time.time() - self.trigger_time > timeout:
            return False
        return True