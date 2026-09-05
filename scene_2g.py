import math
import json
import copy
import carla
import weakref
import time
import threading
from model.tcp import TCPRoutePlanner
from model.tcp import TCPAgent
from model.tcp import (
    TCP_CAMERA_FOV,
    TCP_CAMERA_HEIGHT,
    TCP_CAMERA_WIDTH,
    TCP_CAMERA_X,
    TCP_CAMERA_Y,
    TCP_CAMERA_Z,
)
import numpy as np
from roundabout_2b import decode_vehicle_action, roundabout_conflict_sync_distances
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
        self.npc_start_times = []  # 每辆NPC实际开始切出的时间（可能因前方车辆而延迟）
        self.target_lane_ids = []  # 每辆NPC的目标车道lane_id（触发时固定，避免变道中途漂移）
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
                    self.npc_start_times.append(0.0)
                    self.target_lane_ids.append(0)

        for cfg in self.config['other_actors']['center']:
            if cfg['type'] == 'obstacle':
                x = float(cfg['transform']['x'])
                y = float(cfg['transform']['y'])
                z = float(cfg['transform']['z'])
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

    def _check_front_vehicle(self, vehicle, check_distance=12.0):
        """检查当前车道前方是否有NPC车辆（不含静态障碍车），返回 (has_vehicle, distance)"""
        loc = vehicle.get_location()
        wp = self.map.get_waypoint(loc, project_to_road=True)
        if not wp:
            return False, float('inf')

        fwd = vehicle.get_transform().get_forward_vector()
        for other in self.cars:
            if other.id == vehicle.id or not other.is_alive:
                continue
            other_loc = other.get_location()
            dx = other_loc.x - loc.x
            dy = other_loc.y - loc.y
            # 前向向量点积 > 0 说明在前方
            dot = dx * fwd.x + dy * fwd.y
            if dot <= 0:
                continue
            dist = math.hypot(dx, dy)
            if dist < check_distance:
                other_wp = self.map.get_waypoint(other_loc, project_to_road=True)
                if other_wp and other_wp.lane_id == wp.lane_id:
                    return True, dist
        return False, float('inf')

    def _lane_change_control(self, vehicle, direction, throttle, boost, target_lane_id):
        """基于目标车道waypoint的两阶段变道控制，返回 (control, done)

        target_lane_id: 触发时固定的目标车道ID，不随车辆移动而变化

        Phase 1 — 横向移动：引导车头指向目标车道前方waypoint，斜向切入
        Phase 2 — 车头摆正：进入目标车道后，对齐车头方向与目标车道方向，
                   对齐后才标记 done，避免 autopilot 又把车拉回原车道
        """
        loc = vehicle.get_location()
        trans = vehicle.get_transform()

        # 当前投影车道
        current_wp = self.map.get_waypoint(loc, project_to_road=True)
        if not current_wp:
            return None, True

        # 用固定的 target_lane_id 判断是否已在目标车道
        on_target = (target_lane_id != 0 and current_wp.lane_id == target_lane_id)

        if on_target:
            # ===== Phase 2：沿目标车道waypoints行驶，自然摆正车头 =====
            # 取目标车道前方waypoints，持续转向跟随，车头自然对齐车道方向
            target_next = current_wp.next(6.0)
            if not target_next:
                target_next = current_wp.next(3.0)

            # 检查车头是否已对齐目标车道方向（用当前waypoint的forward判断）
            lane_forward = current_wp.transform.get_forward_vector()
            vehicle_forward = trans.get_forward_vector()
            heading_dot = (vehicle_forward.x * lane_forward.x +
                           vehicle_forward.y * lane_forward.y)

            if heading_dot > 0.97 and target_next:  # 对齐 + 前方有waypoints可跟随
                # 已完成，交还autopilot沿车道线行驶
                control = carla.VehicleControl()
                control.steer = 0.0
                control.throttle = min(1.0, throttle * boost)
                control.brake = 0.0
                return control, True

            if target_next:
                # 跟随目标车道前方waypoints，自然摆正车头
                target_loc = target_next[0].transform.location
                dx = target_loc.x - loc.x
                dy = target_loc.y - loc.y
                desired_yaw = math.atan2(dy, dx)
                current_yaw = math.radians(trans.rotation.yaw)
                yaw_err = desired_yaw - current_yaw
                yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
                steer = max(min(2.0 * yaw_err, 0.35), -0.35)
            else:
                # 兜底：无前方waypoints时仅用方向对齐
                lane_yaw = math.atan2(lane_forward.y, lane_forward.x)
                current_yaw = math.radians(trans.rotation.yaw)
                yaw_err = lane_yaw - current_yaw
                yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
                steer = max(min(2.0 * yaw_err, 0.4), -0.4)
                if abs(yaw_err) < 0.08:
                    control = carla.VehicleControl()
                    control.steer = 0.0
                    control.throttle = min(1.0, throttle * boost)
                    control.brake = 0.0
                    return control, True

            control = carla.VehicleControl()
            control.steer = steer
            control.throttle = min(1.0, throttle * boost)
            control.brake = 0.0
            return control, False

        # ===== Phase 1：横向移动，斜向切入目标车道 =====
        # 从当前车道获取目标车道的 waypoint（仅用于导航）
        if direction == 1:
            target_wp = current_wp.get_right_lane()
        else:
            target_wp = current_wp.get_left_lane()

        if not target_wp:
            return None, True

        target_next = target_wp.next(8.0)
        if not target_next:
            target_next = target_wp.next(4.0)
        if not target_next:
            return None, True

        target_loc = target_next[0].transform.location

        dx = target_loc.x - loc.x
        dy = target_loc.y - loc.y
        desired_yaw = math.atan2(dy, dx)

        current_yaw = math.radians(trans.rotation.yaw)
        yaw_err = desired_yaw - current_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        steer = max(min(2.0 * yaw_err, 0.45), -0.45)

        control = carla.VehicleControl()
        control.steer = steer
        control.throttle = min(1.0, throttle * boost)
        control.brake = 0.0

        return control, False

    def _lane_follow_control(self, vehicle, throttle, boost):
        """沿当前车道waypoints持续行驶（替代autopilot，避免切回原车道）"""
        loc = vehicle.get_location()
        trans = vehicle.get_transform()

        current_wp = self.map.get_waypoint(loc, project_to_road=True)
        if not current_wp:
            return None

        next_wps = current_wp.next(6.0)
        if not next_wps:
            next_wps = current_wp.next(3.0)
        if not next_wps:
            return None

        target_loc = next_wps[0].transform.location

        dx = target_loc.x - loc.x
        dy = target_loc.y - loc.y
        desired_yaw = math.atan2(dy, dx)

        current_yaw = math.radians(trans.rotation.yaw)
        yaw_err = desired_yaw - current_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        steer = max(min(1.5 * yaw_err, 0.3), -0.3)

        control = carla.VehicleControl()
        control.steer = steer
        control.throttle = min(1.0, throttle * boost)
        control.brake = 0.0

        return control

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
                # ==========触发瞬间关闭NPC autopilot，准备手动控制==========
                for car in self.cars:
                    if car.is_alive:
                        car.set_autopilot(False)

                for i, car in enumerate(self.cars):
                    self.directions[i] = self._get_safe_lane_direction(car)
                    # 存储目标车道lane_id（固定参考，变道中途不重新计算）
                    if self.directions[i] != 0:
                        wp = self.map.get_waypoint(car.get_location(), project_to_road=True)
                        if wp:
                            if self.directions[i] == 1:
                                target_wp = wp.get_right_lane()
                            else:
                                target_wp = wp.get_left_lane()
                            self.target_lane_ids[i] = target_wp.lane_id if target_wp else 0

        if self.triggered:
            for i, car in enumerate(self.cars):
                if not car.is_alive:
                    continue
                # =========已完成cut_out：手动沿目标车道waypoints行驶，不使用autopilot=========
                if self.cut_out_finish[i]:
                    follow_control = self._lane_follow_control(car, adj_throttle, boost)
                    if follow_control is not None:
                        car.apply_control(follow_control)
                    continue

                dir = self.directions[i]

                # === 前方碰撞检测（仅检查NPC车辆，不包含静态障碍车）===
                has_front, front_dist = self._check_front_vehicle(car, check_distance=10.0)

                # 记录实际开始切出的时间（可能因前方车辆而延迟）
                if self.npc_start_times[i] == 0.0:
                    if has_front and front_dist < 5.0:
                        # 前方NPC车辆太近 → 减速等待，暂不开始切出
                        control = carla.VehicleControl()
                        control.throttle = 0.0
                        control.brake = 0.4
                        control.steer = 0.0
                        car.apply_control(control)
                        continue
                    else:
                        self.npc_start_times[i] = time.time()

                # === waypoint引导变道：持续转向目标车道，直到实际到达 ===
                control, done = self._lane_change_control(car, dir, adj_throttle, boost, self.target_lane_ids[i])
                if done:
                    # 变道完成，后续由 _lane_follow_control 沿车道行驶
                    self.cut_out_finish[i] = True
                elif control is not None:
                    car.apply_control(control)
                else:
                    # 兜底：无法获取目标车道 → 标记完成
                    self.cut_out_finish[i] = True

            # 超时兜底：全部NPC恢复autopilot，退出场景
            if self.triggered and time.time() - self.trigger_time > timeout:
                for car in self.cars:
                    if car.is_alive:
                        car.set_autopilot(True)
                return False

        return True

    def _get_safe_lane_direction(self, vehicle):
        loc = vehicle.get_location()
        wp = self.map.get_waypoint(loc, project_to_road=True)
        if not wp: return 0
        left_wp = wp.get_left_lane()
        right_wp = wp.get_right_lane()
        # 优先右侧车道（CARLA中 get_left_lane/get_right_lane 已按行驶方向返回）
        if right_wp and right_wp.lane_type == carla.LaneType.Driving:
            return 1
        if left_wp and left_wp.lane_type == carla.LaneType.Driving:
            return -1
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
        if not wp:
            return 0

        left_wp = wp.get_left_lane()
        right_wp = wp.get_right_lane()

        # 必须：相邻车道存在 + 是行驶车道 + 和当前车道方向相同
        if left_wp and left_wp.lane_type == carla.LaneType.Driving and left_wp.lane_id * wp.lane_id > 0:
            return 1
        if right_wp and right_wp.lane_type == carla.LaneType.Driving and right_wp.lane_id * wp.lane_id > 0:
            return -1
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
        self.route_commands = []
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
        self.camera_frame = None
        self.camera_sim_time = None
        self.camera_condition = threading.Condition()
        self.tcp_flag = False
        self.planner = None
        self.tcp = None

        # ✅【新增】统一超时退出（和所有场景一样）
        self.triggered = False
        self.trigger_time = 0
        self.timeout = 10  # 10秒超时

        if self.model == 'tcp' and self.model_path is not None:
            self.tcp_flag = True

        # 2.b keeps the generic route-following scene, but opts into a
        # deterministic roundabout runtime when the editor emits this block.
        # Other scenarios mapped to EgoRouteFollowScene deliberately keep the
        # legacy behaviour below.
        self.roundabout_cfg = self.config.get("roundabout_test")
        self.is_roundabout_2b = isinstance(self.roundabout_cfg, dict)
        if self.is_roundabout_2b:
            self._init_roundabout_runtime()

    def load_ego_route(self):
        """从 JSON 加载路线点"""
        try:
            route_data = self.config.get("ego_route", [])
            if self.is_roundabout_2b:
                vut_cfg = self.roundabout_cfg.get("vut", {})
                if isinstance(vut_cfg, dict) and vut_cfg.get("route"):
                    route_data = vut_cfg["route"]
            for p in route_data:
                if self.is_roundabout_2b:
                    loc = self._roundabout_location(p)
                else:
                    loc = carla.Location(
                        float(p["x"]),
                        float(p["y"]),
                        float(p["z"])
                    )
                self.route_points.append(loc)
                try:
                    command = int(p.get("road_option", 4)) if isinstance(p, dict) else 4
                except (TypeError, ValueError):
                    command = 4
                self.route_commands.append(command)
            print(f"✅ EGO 路线加载完成：{len(self.route_points)} 个点")
        except:
            self.route_points = []
            self.route_commands = []

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

        for cfg in self.config.get('other_actors', {}).get('center', []):
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
                if self.is_roundabout_2b:
                    role = self._roundabout_actor_role(cfg)
                    if role == "vt1":
                        if self.vt1_actor is not None:
                            raise RuntimeError("2.b requires exactly one VT1")
                        self.vt1_actor = actor
                        self.vt1_actor_cfg = cfg
                        actor.set_autopilot(False)
                        actor_route = cfg.get("route", cfg.get("route_points", []))
                        if actor_route:
                            self.vt1_route = self._roundabout_locations(actor_route)
                    elif role == "vt2":
                        if self.vt2_actor is not None:
                            raise RuntimeError("2.b requires exactly one VT2")
                        self.vt2_actor = actor
                        self.vt2_actor_cfg = cfg
                        actor.set_autopilot(False)
                        self._roundabout_hold_vehicle(actor, hand_brake=True)
                    else:
                        actor.set_autopilot(bool(cfg.get('autopilot', True)))
                else:
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
            for index, loc in enumerate(self.route_points):
                wp = self.map.get_waypoint(loc, project_to_road=True)
                if wp:
                    command = self.route_commands[index] \
                        if index < len(self.route_commands) else 4
                    waypoints.append((wp, command))

            if waypoints:
                self.planner.set_route(waypoints)
                print(f"[TCP] 加载路线成功：{len(waypoints)} 个路径点")

            self.spawn_camera()

        if self.is_roundabout_2b:
            self._roundabout_finish_spawn()
            return

        time.sleep(0.2)
        self.world.tick()

    def spawn_camera(self):
        bp_lib = self.world.get_blueprint_library()
        cam_bp = bp_lib.find('sensor.camera.rgb')
        # Match TCP/leaderboard/team_code/tcp_agent.py exactly.  The released
        # checkpoint expects this native aspect ratio and rearward mounting;
        # capturing a front-mounted 800x600 image and stretching it changes
        # obstacle scale and can leave the policy braking after traffic clears.
        cam_bp.set_attribute('image_size_x', str(TCP_CAMERA_WIDTH))
        cam_bp.set_attribute('image_size_y', str(TCP_CAMERA_HEIGHT))
        cam_bp.set_attribute('fov', str(TCP_CAMERA_FOV))
        transform = carla.Transform(carla.Location(
            x=TCP_CAMERA_X, y=TCP_CAMERA_Y, z=TCP_CAMERA_Z))
        self.camera_sensor = self.world.spawn_actor(cam_bp, transform, attach_to=self.ego)

        def callback(image):
            rgb = self._roundabout_decode_camera_rgb(image)
            with self.camera_condition:
                self.camera_data = rgb
                self.camera_frame = getattr(image, "frame", None)
                timestamp = getattr(image, "timestamp", None)
                self.camera_sim_time = float(timestamp) if timestamp is not None else None
                self.camera_condition.notify_all()

        self.camera_sensor.listen(callback)
        self.actors.append(self.camera_sensor)

    @staticmethod
    def _roundabout_decode_camera_rgb(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        return array[:, :, :3][:, :, ::-1].copy()

    # ============================
    # GB/T 41798—2022 6.2.2 环形路口（2.b）
    # ============================
    def _init_roundabout_runtime(self):
        """Initialise 2.b state without changing generic route-scene semantics."""
        cfg = self.roundabout_cfg
        try:
            self.roundabout_planned_exit = int(cfg.get("planned_exit"))
        except (TypeError, ValueError):
            raise ValueError("2.b roundabout_test.planned_exit must be 2 or 3")
        if self.roundabout_planned_exit not in (2, 3):
            raise ValueError("2.b planned_exit must be 2 or 3")

        capable_value = cfg.get("roundabout_capable", True)
        if not isinstance(capable_value, bool):
            raise ValueError("2.b roundabout_capable must be a JSON boolean")
        self.roundabout_capable = capable_value
        self.roundabout_entry_gate = self._roundabout_parse_gate(
            cfg.get("entry_gate"), "entry_gate")
        self.roundabout_exit_gates = {}
        raw_exit_gates = cfg.get("exit_gates", [])
        if isinstance(raw_exit_gates, dict):
            raw_exit_gates = [dict(value, number=key)
                              for key, value in raw_exit_gates.items()
                              if isinstance(value, dict)]
        for gate_cfg in raw_exit_gates:
            if not isinstance(gate_cfg, dict):
                continue
            try:
                number = int(gate_cfg.get("number", gate_cfg.get("exit_number")))
            except (TypeError, ValueError):
                continue
            self.roundabout_exit_gates[number] = self._roundabout_parse_gate(
                gate_cfg, "exit_gate_{}".format(number))
        if self.roundabout_planned_exit not in self.roundabout_exit_gates:
            raise ValueError(
                "2.b exit_gates does not contain planned exit {}".format(
                    self.roundabout_planned_exit))

        self.vt1_cfg = cfg.get("vt1", {}) or {}
        self.vt2_cfg = cfg.get("vt2", {}) or {}
        self.vt1_route = self._roundabout_locations(
            self.vt1_cfg.get("route", self.vt1_cfg.get("route_points", [])))
        self.vt1_target_speed_mps = float(
            self.vt1_cfg.get("target_speed_kmh", 15.0)) / 3.6

        engineering = cfg.get("engineering", cfg.get("engineering_parameters", {})) or {}
        self.roundabout_engineering = engineering
        self.timeout = float(self.config.get("timeout", engineering.get(
            "scenario_timeout_s", engineering.get("timeout_s", 90.0))))
        self.rb_stabilization_timeout = float(
            engineering.get("vt1_stabilization_timeout_s", 30.0))
        self.rb_vt1_speed_tolerance = float(
            engineering.get("vt1_speed_tolerance_kmh", 1.0)) / 3.6
        self.rb_vt1_stable_required = float(
            engineering.get("vt1_stable_duration_s",
                            engineering.get("vt1_speed_stable_duration_s", 1.0)))
        # The editor separately records the initial placement budget needed for
        # acceleration/stabilisation.  At the instant VUT reaches the entry we
        # only need the target vehicle to remain geometrically upstream.
        self.rb_vt1_upstream_min = float(engineering.get(
            "vt1_entry_upstream_min_distance_m",
            engineering.get("vt1_upstream_min_distance_m", 3.0)))
        self.rb_vt1_upstream_max = float(
            engineering.get("vt1_upstream_max_distance_m", 60.0))
        self.rb_vt2_stationary_speed = float(
            engineering.get("vt2_stationary_speed_threshold_mps", 0.1))
        self.rb_stop_speed = float(engineering.get("stop_speed_threshold_mps", 0.1))
        self.rb_stop_duration_limit = float(
            engineering.get("stop_duration_s", 1.0))
        self.rb_emergency_decel = float(
            engineering.get("emergency_brake_deceleration_mps2",
                            engineering.get("emergency_deceleration_mps2", 4.0)))
        self.rb_emergency_brake = float(
            engineering.get("emergency_brake_threshold", 0.8))
        self.rb_emergency_brake_duration_limit = float(
            engineering.get("emergency_brake_duration_s", 0.2))
        self.rb_speed_tolerance_kmh = float(
            engineering.get("speed_limit_tolerance_kmh", 0.5))
        self.rb_speed_limit_unobservable_duration = float(
            engineering.get("speed_limit_unobservable_duration_s", 0.5))
        self.rb_speed_limit_by_road_id = {
            int(key): float(value)
            for key, value in (engineering.get("speed_limit_by_road_id", {}) or {}).items()
        }
        self.rb_entry_arrival_distance = float(
            engineering.get("entry_arrival_distance_m", 3.0))
        self.rb_vut_approach_time_budget = float(
            engineering.get("vut_approach_time_budget_s", 15.0))
        self.rb_camera_frame_timeout = float(
            engineering.get("camera_frame_timeout_s", 0.5))
        self.rb_route_completion_distance = float(
            engineering.get("route_completion_distance_m", 3.0))
        self.rb_exit_lane_check_distance = float(
            engineering.get("exit_completion_distance_m",
                            engineering.get("exit_lane_check_distance_m", 2.0)))
        self.rb_incapable_observation = float(
            engineering.get("incapable_observation_s", 10.0))
        self.rb_max_timeline_samples = max(1, int(engineering.get(
            "max_timeline_samples", max(1, int(math.ceil(self.timeout / 0.05)) + 100))))
        self.rb_timeline_interval = max(
            0.0, float(engineering.get("timeline_sample_interval_s", 0.05)))
        self.rb_indicator_lookback = float(
            engineering.get("indicator_lookback_s", 3.0))
        self.rb_lane_direction_duration_limit = float(
            engineering.get("lane_direction_violation_duration_s", 0.5))
        self.rb_sut_readiness_timeout = float(
            engineering.get("sut_readiness_timeout_s", 3.0))
        self.rb_vt1_speed_continuity_duration = float(
            engineering.get("vt1_speed_continuity_duration_s", 0.5))
        self.rb_vt1_speed_control_mode = str(
            engineering.get("vt1_speed_control_mode", "constant_velocity"))
        self.rb_vt1_route_lookahead = float(
            engineering.get("vt1_route_lookahead_m", 6.0))
        self.rb_vt1_corner_speed_compensation = float(
            engineering.get("vt1_corner_speed_compensation", 0.12))
        self.rb_vt1_exit_observation_timeout = float(
            engineering.get("vt1_exit_observation_timeout_s", 20.0))
        self.rb_vt1_post_exit_clearance_distance = float(
            engineering.get("vt1_post_exit_clearance_distance_m", 25.0))
        self.rb_lane_guidance_duration_limit = float(
            engineering.get("lane_guidance_violation_duration_s", 0.5))
        self.rb_lane_corridor_extra_m = float(
            engineering.get("lane_guidance_corridor_extra_m", 0.75))
        self.rb_lane_corridor_min_half_width_m = float(
            engineering.get("lane_guidance_corridor_min_half_width_m", 1.75))
        self.rb_lane_corridor_max_half_width_m = float(
            engineering.get("lane_guidance_corridor_max_half_width_m", 4.0))
        self.rb_conflict_headway_target_s = float(
            engineering.get("vt1_conflict_headway_target_s", 1.0))
        self.rb_conflict_headway_tolerance_s = float(
            engineering.get("vt1_conflict_headway_tolerance_s", 0.5))
        self.rb_vut_approach_speed_assumption_mps = float(
            engineering.get("vut_approach_speed_assumption_kmh", 15.0)) / 3.6
        self.rb_off_road_duration_limit = float(
            engineering.get("off_road_violation_duration_s", 0.2))

        self.vt1_actor = None
        self.vt2_actor = None
        self.vt1_actor_cfg = None
        self.vt2_actor_cfg = None
        self.vt1_target_idx = 0
        self.vt1_route_finished = False
        self.rb_vt1_route_s = []
        self.rb_vt1_conflict_s = None
        self.rb_vut_conflict_s = None
        self.rb_vt1_release_remaining_m = None
        self.rb_vt1_entry_gap_target_m = None
        self.rb_vt1_entry_gap_min_m = None
        self.rb_vt1_entry_gap_max_m = None
        self.rb_vt1_conflict_ttc_at_entry_s = None
        self.rb_vt1_conflict_crossed = False
        self.rb_vt1_conflict_crossing_time = None
        self.rb_vut_entry_crossing_time = None
        self.rb_vt1_lead_time_at_vut_entry_s = None
        self.rb_vut_entered_before_vt1 = False
        self.rb_vt1_stable_observed = False
        self.rb_vt1_progress_s = 0.0
        self.rb_vt1_speed_integral = 0.0
        self.rb_vt1_constant_velocity_enabled = False
        self.rb_vt1_stable_time = 0.0
        self.rb_vt1_ready = False
        self.rb_vt1_speed_at_entry = None
        self.rb_vt1_upstream_at_entry = None
        self.rb_vt1_remaining_at_entry = None
        self.rb_vt1_speed_out_of_tolerance_time = 0.0
        self.rb_vt1_speed_maintained = True
        self.rb_vt1_exit1_crossed = False
        self.rb_vt1_exit_clearance_travel_m = 0.0
        self.rb_vt1_exit_clearance_last_location = None
        self.rb_vt1_departed = False
        self.rb_vt1_departure_time = None
        self.rb_vt1_drawn_route_finished = False
        self.rb_vt1_topology_fallback_reported = False
        self.rb_vut_finished_waiting_for_vt1_since = None
        self.rb_vt2_max_speed = 0.0
        self.rb_vt2_moved = False

        self.rb_phase = "SETUP"
        self.rb_phase_history = []
        self.rb_events = []
        self.timeline_samples = []
        self.rb_start_sim_time = None
        self.rb_last_sim_time = None
        self.rb_trial_start_sim_time = None
        self.rb_timed_out = False
        self.rb_invalid_reasons = []
        self.rb_invalid_events = []
        self.rb_entry_sync_missed = False
        self.rb_approach_time_budget_exceeded = False
        self.rb_entry_crossed = False
        self.rb_entry_arrived = False
        self.rb_entry_arrival_time = None
        self.rb_correct_exit_crossed = False
        self.rb_wrong_exit = None
        self.rb_exit_lane_correct = None
        self.rb_exit_location = None
        self.rb_exit_lane_checked = False
        self.rb_gate_progress = {}
        self.rb_stop_duration = 0.0
        self.rb_stopped_in_roundabout = False
        self.rb_emergency_brake_duration = 0.0
        self.rb_emergency_braking = False
        self.rb_max_deceleration = 0.0
        self.rb_max_speed_mps = 0.0
        self.rb_speed_limit_exceeded = False
        self.rb_observed_speed_limit_kmh = None
        self.rb_speed_limit_observed = False
        self.rb_speed_limit_unobservable_time = 0.0
        self.rb_speed_limit_unobservable_gap = False
        self.rb_collision_recorded = False
        self.rb_valid_collision_time = None
        self.rb_solid_line_invasion = False
        self.rb_lane_invasion_events = []
        self.rb_indicator_observable = False
        self.rb_exit_indicator_last_seen = None
        self.rb_exit_indicator_observed = None
        self.rb_indicator_active = False
        self.rb_indicator_evidence_source = None
        self.rb_last_timeline_sample_time = None
        self.rb_tcp_required = self.model == "tcp"
        self.rb_sut_ready_wait_started = None
        self.rb_sut_error = None
        self.rb_sut_failure = None
        self.rb_collision_vt1 = False
        self.rb_collision_vt2 = False
        self.rb_infrastructure_collision = False
        self.rb_other_collision = False
        self.rb_lane_direction_violation = False
        self.rb_lane_direction_violation_time = 0.0
        self.rb_lane_guidance_violation = False
        self.rb_lane_guidance_violation_time = 0.0
        self.rb_lane_guidance_violation_detail = None
        self.rb_off_road = False
        self.rb_off_road_time = 0.0
        self.rb_vut_route_lane_tokens = set()
        self.rb_vut_route_s = []
        self.rb_vut_route_lane_half_widths = []
        self.rb_vut_route_progress_s = 0.0
        self.rb_vut_route_segment_idx = 0
        self.rb_last_requested_control = None
        self.rb_ads_control_source = (
            "tcp_model" if self.rb_tcp_required
            else "ego_route_follow_scene_reference_controller")

        alert_cfg = cfg.get("odd_alert", {})
        config_alert = alert_cfg.get("observed", False) \
            if isinstance(alert_cfg, dict) else alert_cfg
        config_alert = bool(cfg.get("odd_alert_observed", config_alert))
        self.rb_allow_debug_odd_alert = bool(
            engineering.get("allow_config_odd_alert_debug", False))
        self.rb_formal_hmi_evidence = bool(
            engineering.get("formal_hmi_evidence_required", True))
        self.rb_odd_alert_debug_observed = bool(
            config_alert and self.rb_allow_debug_odd_alert)
        # Formal GB/T results require runtime evidence from the SUT/HMI hook.
        # A config flag is retained only for explicitly non-formal debugging.
        self.rb_odd_alert_observed = bool(
            self.rb_odd_alert_debug_observed and not self.rb_formal_hmi_evidence)
        self.rb_odd_alert_time = None
        self.rb_odd_alert_source = "config_debug" if self.rb_odd_alert_observed else None

    @staticmethod
    def _roundabout_location(data):
        if data is None:
            return None
        if hasattr(data, "x") and hasattr(data, "y"):
            return carla.Location(float(data.x), float(data.y), float(getattr(data, "z", 0.0)))
        if isinstance(data, dict):
            if "location" in data and not ("x" in data and "y" in data):
                return EgoRouteFollowScene._roundabout_location(data["location"])
            if "transform" in data and not ("x" in data and "y" in data):
                return EgoRouteFollowScene._roundabout_location(data["transform"])
        if isinstance(data, (list, tuple)) and len(data) >= 2:
            return carla.Location(
                float(data[0]), float(data[1]),
                float(data[2]) if len(data) >= 3 else 0.0)
        return carla.Location(
            float(data["x"]), float(data["y"]), float(data.get("z", 0.0)))

    @classmethod
    def _roundabout_locations(cls, route):
        locations = []
        for point in route or []:
            try:
                locations.append(cls._roundabout_location(point))
            except (KeyError, TypeError, ValueError):
                raise ValueError("2.b route contains an invalid waypoint")
        return locations

    @classmethod
    def _roundabout_parse_gate(cls, data, name):
        if isinstance(data, (list, tuple)) and len(data) == 2:
            data = {"left": data[0], "right": data[1]}
        if not isinstance(data, dict):
            raise ValueError("2.b {} is required".format(name))
        outer_data = data
        if isinstance(data.get("gate"), dict):
            data = dict(data["gate"])
            for key in ("road_id", "section_id", "lane_id", "lane_ids",
                        "number", "exit_number"):
                if key in outer_data and key not in data:
                    data[key] = outer_data[key]

        endpoint_pair = (data.get("left"), data.get("right"))
        if endpoint_pair[0] is None or endpoint_pair[1] is None:
            for first_key, second_key in (("start", "end"), ("p1", "p2")):
                if first_key in data and second_key in data:
                    endpoint_pair = (data[first_key], data[second_key])
                    break
        if ((endpoint_pair[0] is None or endpoint_pair[1] is None)
                and isinstance(data.get("points"), (list, tuple))
                and len(data["points"]) == 2):
            endpoint_pair = tuple(data["points"])
        left = cls._roundabout_location(endpoint_pair[0])
        right = cls._roundabout_location(endpoint_pair[1])
        center_data = data.get("center")
        if center_data is None and "x" in data and "y" in data:
            center_data = data
        if center_data is not None:
            center = cls._roundabout_location(center_data)
            if left is not None and right is not None:
                midpoint_x = (left.x + right.x) * 0.5
                midpoint_y = (left.y + right.y) * 0.5
                if math.hypot(center.x - midpoint_x, center.y - midpoint_y) > 1e-3:
                    raise ValueError(
                        "2.b {} center must match its endpoint midpoint".format(name))
        elif left is not None and right is not None:
            center = carla.Location(
                x=(left.x + right.x) * 0.5,
                y=(left.y + right.y) * 0.5,
                z=(left.z + right.z) * 0.5)
        else:
            raise ValueError("2.b {} requires center or left/right".format(name))

        forward_cfg = data.get("forward", data.get("normal"))
        if isinstance(forward_cfg, (dict, list, tuple)):
            forward_location = cls._roundabout_location(forward_cfg)
            forward = (forward_location.x, forward_location.y)
        elif "approach_yaw" in data or "yaw" in data:
            yaw = math.radians(float(data.get("approach_yaw", data.get("yaw"))))
            forward = (math.cos(yaw), math.sin(yaw))
        elif left is not None and right is not None:
            # The editor normally emits approach_yaw.  This fallback provides
            # a deterministic perpendicular for older generated data.
            tangent = (right.x - left.x, right.y - left.y)
            forward = (-tangent[1], tangent[0])
        else:
            raise ValueError("2.b {} requires forward or approach_yaw".format(name))
        norm = math.hypot(forward[0], forward[1])
        if norm < 1e-6:
            raise ValueError("2.b {} has a zero forward vector".format(name))
        forward = (forward[0] / norm, forward[1] / norm)
        if left is None and right is None and data.get("width_m") is not None:
            half_width = float(data["width_m"]) * 0.5
            if half_width <= 0.0:
                raise ValueError("2.b {} width_m must be positive".format(name))
            tangent = (-forward[1], forward[0])
            left = carla.Location(
                center.x + tangent[0] * half_width,
                center.y + tangent[1] * half_width, center.z)
            right = carla.Location(
                center.x - tangent[0] * half_width,
                center.y - tangent[1] * half_width, center.z)
        return {
            "name": name,
            "center": center,
            "forward": forward,
            "left": left,
            "right": right,
            "road_id": data.get("road_id"),
            "section_id": data.get("section_id"),
            "lane_id": data.get("lane_id"),
            "lane_ids": data.get("lane_ids"),
            "raw": dict(outer_data, **data),
        }

    @staticmethod
    def _roundabout_actor_role(cfg):
        role = str(cfg.get("role", cfg.get("rolename", ""))).strip().lower()
        role = role.replace("_", "").replace("-", "")
        aliases = {
            "vt1": "vt1", "target1": "vt1", "targetvehicle1": "vt1",
            "vt2": "vt2", "target2": "vt2", "targetvehicle2": "vt2",
        }
        return aliases.get(role, role)

    @staticmethod
    def _roundabout_vehicle_speed(actor):
        velocity = actor.get_velocity()
        # Road-vehicle speed is planar.  Counting the short vertical settling
        # motion after spawn incorrectly marks a hand-braked VT2 as moving.
        return math.hypot(velocity.x, velocity.y)

    @staticmethod
    def _roundabout_route_distances(route):
        cumulative = [0.0]
        for first, second in zip(route, route[1:]):
            cumulative.append(cumulative[-1] + math.hypot(
                second.x - first.x, second.y - first.y))
        return cumulative

    @staticmethod
    def _roundabout_project_route_detail(
            location, route, cumulative, start_index=0, end_index=None):
        """Return a bounded polyline projection with progress and tangent.

        A long roundabout route can revisit the same physical location.  The
        optional segment range prevents a nearest-point lookup from jumping
        hundreds of metres ahead to a later visit of that location.
        """
        if not route:
            return {
                "progress_m": 0.0, "distance_m": float("inf"),
                "segment_index": 0, "tangent_yaw_deg": None,
            }
        if len(route) == 1:
            return {
                "progress_m": 0.0,
                "distance_m": math.hypot(
                    location.x - route[0].x, location.y - route[0].y),
                "segment_index": 0, "tangent_yaw_deg": None,
            }
        segment_count = len(route) - 1
        start_index = max(0, min(int(start_index), segment_count - 1))
        if end_index is None:
            end_index = segment_count
        end_index = max(start_index + 1, min(int(end_index), segment_count))
        best_s = cumulative[start_index]
        best_distance = float("inf")
        best_index = start_index
        best_yaw = None
        for index in range(start_index, end_index):
            first, second = route[index], route[index + 1]
            dx = second.x - first.x
            dy = second.y - first.y
            length_squared = dx * dx + dy * dy
            if length_squared <= 1e-9:
                continue
            fraction = ((location.x - first.x) * dx + (location.y - first.y) * dy)
            fraction = max(0.0, min(1.0, fraction / length_squared))
            projected_x = first.x + fraction * dx
            projected_y = first.y + fraction * dy
            distance = math.hypot(location.x - projected_x, location.y - projected_y)
            if distance < best_distance:
                best_distance = distance
                best_s = cumulative[index] + fraction * math.sqrt(length_squared)
                best_index = index
                best_yaw = math.degrees(math.atan2(dy, dx))
        return {
            "progress_m": best_s,
            "distance_m": best_distance,
            "segment_index": best_index,
            "tangent_yaw_deg": best_yaw,
        }

    @staticmethod
    def _roundabout_project_route(
            location, route, cumulative, start_index=0, end_index=None):
        detail = EgoRouteFollowScene._roundabout_project_route_detail(
            location, route, cumulative, start_index=start_index,
            end_index=end_index)
        return detail["progress_m"], detail["distance_m"]

    def _roundabout_vut_route_corridor(self, location):
        """Project VUT onto the confirmed route without trusting road IDs.

        Custom OpenDRIVE maps may expose a different connector ``road_id`` at
        runtime than the one returned while the editor densifies the same
        centreline.  Spatial containment in a local, forward-moving route
        corridor is the authoritative lane-guidance observation; lane tokens
        remain diagnostic metadata only.
        """
        segment_count = max(1, len(self.route_points) - 1)
        cursor = max(0, min(self.rb_vut_route_segment_idx, segment_count - 1))
        detail = self._roundabout_project_route_detail(
            location, self.route_points, self.rb_vut_route_s,
            start_index=max(0, cursor - 25),
            end_index=min(segment_count, cursor + 101))
        if detail["progress_m"] + 1.0 >= self.rb_vut_route_progress_s:
            self.rb_vut_route_progress_s = max(
                self.rb_vut_route_progress_s, detail["progress_m"])
            self.rb_vut_route_segment_idx = max(
                self.rb_vut_route_segment_idx, detail["segment_index"])
        index = min(
            detail["segment_index"],
            max(0, len(self.rb_vut_route_lane_half_widths) - 1))
        lane_half_width = (
            self.rb_vut_route_lane_half_widths[index]
            if self.rb_vut_route_lane_half_widths
            else self.rb_lane_corridor_min_half_width_m)
        allowed_m = max(
            self.rb_lane_corridor_min_half_width_m,
            min(self.rb_lane_corridor_max_half_width_m,
                lane_half_width + self.rb_lane_corridor_extra_m))
        return detail, allowed_m

    def _roundabout_finish_spawn(self):
        if len(self.route_points) < 2:
            raise RuntimeError("2.b requires at least two VUT route points")
        if self.vt1_actor is None or self.vt2_actor is None:
            raise RuntimeError("2.b requires one role-tagged VT1 and one role-tagged VT2")
        if len(self.vt1_route) < 2:
            raise RuntimeError("2.b requires at least two VT1 route points")
        if self.rb_tcp_required and (self.tcp is None or self.planner is None):
            raise RuntimeError("2.b TCP controller was requested but is not initialized")
        motion = str(self.vt2_cfg.get(
            "motion", (self.vt2_actor_cfg or {}).get("motion", "stationary"))).lower()
        if motion not in ("stationary", "static", "stopped"):
            raise RuntimeError("2.b VT2 motion must be stationary")

        self.rb_vt1_route_s = self._roundabout_route_distances(self.vt1_route)
        if self.rb_vt1_speed_control_mode == "constant_velocity":
            self._roundabout_enable_constant_speed(
                self.vt1_actor, self.vt1_target_speed_mps)
            self.rb_vt1_constant_velocity_enabled = True
        self.rb_vt1_conflict_s, conflict_distance = self._roundabout_project_route(
            self.roundabout_entry_gate["center"], self.vt1_route, self.rb_vt1_route_s)
        maximum_conflict_offset = float(
            self.roundabout_engineering.get("vt1_conflict_max_offset_m", 12.0))
        if conflict_distance > maximum_conflict_offset:
            raise RuntimeError(
                "2.b VT1 route does not pass the VUT entry (offset {:.2f} m)".format(
                    conflict_distance))

        self.rb_vut_route_s = self._roundabout_route_distances(self.route_points)
        self.rb_vut_conflict_s, _ = self._roundabout_project_route(
            self.roundabout_entry_gate["center"], self.route_points,
            self.rb_vut_route_s)
        configured_approach_distance = self.roundabout_engineering.get(
            "vut_approach_distance_m")
        # Synchronise against the shared physical conflict point, not the
        # earlier upstream observation plane.
        sync_approach_distance = (
            float(configured_approach_distance)
            if configured_approach_distance is not None
            else max(0.0, self.rb_vut_conflict_s))
        sync = roundabout_conflict_sync_distances(
            sync_approach_distance,
            target_speed_kmh=self.vt1_target_speed_mps * 3.6,
            vut_approach_speed_mps=self.rb_vut_approach_speed_assumption_mps,
            target_headway_s=self.rb_conflict_headway_target_s,
            headway_tolerance_s=self.rb_conflict_headway_tolerance_s,
            entry_upstream_min_distance_m=self.rb_vt1_upstream_min)
        self.rb_vt1_release_remaining_m = sync["vt1_release_remaining_m"]
        self.rb_entry_arrival_distance = max(
            self.rb_entry_arrival_distance,
            sync["entry_arrival_distance_m"])
        self.rb_vt1_entry_gap_target_m = sync["target_entry_gap_m"]
        self.rb_vt1_entry_gap_min_m = sync["minimum_entry_gap_m"]
        self.rb_vt1_entry_gap_max_m = min(
            self.rb_vt1_upstream_max, sync["maximum_entry_gap_m"])

        for location in self.route_points:
            waypoint = self.map.get_waypoint(
                location, project_to_road=True, lane_type=carla.LaneType.Driving)
            if waypoint is not None:
                self.rb_vut_route_lane_tokens.add((
                    int(waypoint.road_id), int(getattr(waypoint, "section_id", 0)),
                    int(waypoint.lane_id)))
                self.rb_vut_route_lane_half_widths.append(
                    max(0.0, float(getattr(waypoint, "lane_width", 3.5))) / 2.0)
            else:
                self.rb_vut_route_lane_half_widths.append(
                    self.rb_lane_corridor_min_half_width_m)
        if not self.rb_vut_route_lane_tokens:
            raise RuntimeError("2.b VUT route has no verifiable Driving Lane")

        self._roundabout_hold_vehicle(self.ego, hand_brake=True)
        self._roundabout_hold_vehicle(self.vt2_actor, hand_brake=True)
        now = self.world.get_snapshot().timestamp.elapsed_seconds
        self.rb_start_sim_time = now
        self.rb_last_sim_time = now
        self._roundabout_transition("SETUP", now, force=True)
        self.rb_gate_progress["entry"] = self._roundabout_gate_progress(
            self.ego, self.roundabout_entry_gate)[0]
        for number, gate in self.roundabout_exit_gates.items():
            self.rb_gate_progress["exit_{}".format(number)] = \
                self._roundabout_gate_progress(self.ego, gate)[0]
        exit_one = self.roundabout_exit_gates.get(1)
        if exit_one is not None:
            self.rb_gate_progress["vt1_exit_1"] = self._roundabout_gate_progress(
                self.vt1_actor, exit_one)[0]
        if self.rb_odd_alert_observed:
            self.rb_odd_alert_time = now
            self._roundabout_record_event(now, "ODD_ALERT", source="config_debug")
        elif self.rb_odd_alert_debug_observed:
            self._roundabout_record_event(
                now, "ODD_ALERT_DEBUG_IGNORED_IN_FORMAL_MODE")
        print(
            "[2.b] actors ready; planned exit={}, capable={}, VT1 target={:.1f} km/h".format(
                self.roundabout_planned_exit, self.roundabout_capable,
                self.vt1_target_speed_mps * 3.6))

    def _roundabout_record_event(self, now, event, event_frame=None, **details):
        snapshot = self.world.get_snapshot()
        trial_time = None
        if self.rb_trial_start_sim_time is not None:
            trial_time = max(0.0, now - self.rb_trial_start_sim_time)
        item = {
            "event": event,
            "frame": (event_frame if event_frame is not None
                      else getattr(snapshot, "frame", None)),
            "sim_time": round(float(now), 4),
            "trial_time": round(float(trial_time), 4) if trial_time is not None else None,
        }
        item.update(details)
        self.rb_events.append(item)

    def _roundabout_sensor_time(self, event, fallback=None):
        timestamp = getattr(event, "timestamp", None)
        if timestamp is not None:
            if hasattr(timestamp, "elapsed_seconds"):
                return float(timestamp.elapsed_seconds)
            try:
                return float(timestamp)
            except (TypeError, ValueError):
                pass
        if fallback is not None:
            return float(fallback)
        return float(self.world.get_snapshot().timestamp.elapsed_seconds)

    def _roundabout_event_is_pretrial(self, event_time):
        """Classify delayed sensor callbacks by their frame time, not arrival time."""
        return (self.rb_trial_start_sim_time is None
                or float(event_time) <= float(self.rb_trial_start_sim_time) + 1e-7)

    def _roundabout_transition(self, phase, now, reason=None, force=False):
        if not force and self.rb_phase == phase:
            return
        self.rb_phase = phase
        entry = {"phase": phase, "sim_time": round(float(now), 4)}
        if reason:
            entry["reason"] = reason
        self.rb_phase_history.append(entry)
        self._roundabout_record_event(now, "PHASE_{}".format(phase), reason=reason)

    def _roundabout_invalidate(self, now, reason):
        if reason not in self.rb_invalid_reasons:
            self.rb_invalid_reasons.append(reason)
            if hasattr(self, "rb_invalid_events"):
                self.rb_invalid_events.append({
                    "reason": reason, "sim_time": float(now),
                    "terminal": True})
        self._roundabout_transition("INVALID", now, reason=reason)
        self._roundabout_hold_vehicle(self.ego, hand_brake=False)

    def _roundabout_experiment_elapsed(self, now):
        """Return elapsed time on the formal trial clock.

        Fixture construction and VT1 stabilisation have their own setup
        timeout.  The scenario timeout starts when the VUT is released so
        telemetry, video and timeout decisions share one time origin.
        """
        origin = (self.rb_trial_start_sim_time
                  if self.rb_trial_start_sim_time is not None
                  else self.rb_start_sim_time)
        return max(0.0, float(now) - float(origin))

    def set_odd_alert(self, active=True, now=None, source="external"):
        """External SUT/HMI adapter hook for the no-roundabout-capability branch."""
        if not self.is_roundabout_2b or not active:
            return
        if now is None:
            now = self.world.get_snapshot().timestamp.elapsed_seconds
        if not self.rb_odd_alert_observed:
            self.rb_odd_alert_observed = True
            self.rb_odd_alert_time = float(now)
            self.rb_odd_alert_source = str(source or "external")
            self._roundabout_record_event(
                float(now), "ODD_ALERT", source=self.rb_odd_alert_source)

    def record_turn_signal_observation(
            self, active=True, now=None, source="external_observer"):
        """Record independent runtime evidence of the VUT's right indicator.

        This hook is for a SUT/HMI adapter or an authorised test observer.  It
        never changes the simulated vehicle's lights and therefore cannot make
        a controller appear to signal when no evidence was observed.
        """
        if not self.is_roundabout_2b or not active:
            return
        if now is None:
            now = self.world.get_snapshot().timestamp.elapsed_seconds
        self.rb_indicator_observable = True
        self.rb_exit_indicator_last_seen = float(now)
        self.rb_indicator_evidence_source = str(source or "external_observer")
        self._roundabout_record_event(
            float(now), "RIGHT_INDICATOR_OBSERVED",
            source=self.rb_indicator_evidence_source)

    def abort_roundabout(self, reason="operator_abort", now=None):
        """Mark an interrupted 2.b execution invalid instead of failing the SUT."""
        if not self.is_roundabout_2b:
            return
        if now is None:
            now = self.world.get_snapshot().timestamp.elapsed_seconds
        self._roundabout_invalidate(float(now), str(reason))

    def record_collision(self, event=None, now=None):
        """Sensor callback hook; ``run.py`` may also pass its aggregate flag."""
        if not self.is_roundabout_2b:
            return
        now = self._roundabout_sensor_time(event, fallback=now)
        pretrial = self._roundabout_event_is_pretrial(now)
        if pretrial:
            self._roundabout_invalidate(now, "collision_before_trial_start")
        else:
            previous_time = getattr(self, "rb_valid_collision_time", None)
            self.rb_valid_collision_time = (
                float(now) if previous_time is None else min(float(previous_time), float(now)))
        self.rb_collision_recorded = True
        details = {}
        other_actor = getattr(event, "other_actor", None)
        if other_actor is not None:
            details["other_actor_id"] = getattr(other_actor, "id", None)
            details["other_actor_type"] = getattr(other_actor, "type_id", None)
            if self.vt1_actor is not None and other_actor.id == self.vt1_actor.id:
                self.rb_collision_vt1 = True
            elif self.vt2_actor is not None and other_actor.id == self.vt2_actor.id:
                self.rb_collision_vt2 = True
            else:
                type_id = str(getattr(other_actor, "type_id", ""))
                if type_id.startswith(("vehicle.", "walker.")):
                    self.rb_other_collision = True
                else:
                    self.rb_infrastructure_collision = True
        else:
            self.rb_infrastructure_collision = True
        self._roundabout_record_event(
            float(now), "COLLISION", event_frame=getattr(event, "frame", None),
            **details)
        if not pretrial and self.rb_phase not in ("COMPLETE", "TIMEOUT"):
            self._roundabout_hold_vehicle(self.ego, hand_brake=False)
            if self.rb_phase != "INVALID":
                self._roundabout_transition(
                    "COMPLETE", float(now), reason="post_release_collision")

    def record_fixture_collision(self, role, event=None, now=None):
        """Treat target/fixture collisions as setup failure unless VUT caused them."""
        if not self.is_roundabout_2b:
            return
        role = str(role).strip().lower()
        if role not in ("vt1", "vt2"):
            return
        now = self._roundabout_sensor_time(event, fallback=now)
        other_actor = getattr(event, "other_actor", None)
        other_id = getattr(other_actor, "id", None)
        ego_id = getattr(self.ego, "id", None)
        self._roundabout_record_event(
            float(now), "{}_FIXTURE_COLLISION".format(role.upper()),
            event_frame=getattr(event, "frame", None),
            other_actor_id=other_id,
            other_actor_type=getattr(other_actor, "type_id", None))
        if ego_id is not None and other_id == ego_id:
            return
        self._roundabout_invalidate(
            float(now), "{}_fixture_collision".format(role))

    def record_lane_invasion(self, event=None, now=None):
        """Sensor hook for markings with a solid component (including mixed types)."""
        if not self.is_roundabout_2b:
            return
        now = self._roundabout_sensor_time(event, fallback=now)
        if self._roundabout_event_is_pretrial(now):
            self._roundabout_invalidate(now, "lane_invasion_before_trial_start")
        marking_types = []
        for marking in getattr(event, "crossed_lane_markings", []) or []:
            marking_type = str(getattr(marking, "type", marking)).split(".")[-1]
            marking_types.append(marking_type)
            if "Solid" in marking_type:
                self.rb_solid_line_invasion = True
        item = {
            "frame": getattr(event, "frame", None),
            "sim_time": round(float(now), 4),
            "marking_types": marking_types,
            "solid": any("Solid" in value for value in marking_types),
        }
        self.rb_lane_invasion_events.append(item)
        self._roundabout_record_event(
            float(now), "LANE_INVASION", marking_types=marking_types,
            solid=item["solid"], event_frame=getattr(event, "frame", None))

    @staticmethod
    def _roundabout_hold_vehicle(actor, hand_brake=False):
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = 1.0
        control.steer = 0.0
        control.hand_brake = bool(hand_brake)
        actor.apply_control(control)

    @staticmethod
    def _roundabout_front_center(actor):
        transform = actor.get_transform()
        forward = transform.get_forward_vector()
        extent = actor.bounding_box.extent
        center = transform.location
        return carla.Location(
            x=center.x + forward.x * extent.x,
            y=center.y + forward.y * extent.x,
            z=center.z + forward.z * extent.x)

    @staticmethod
    def _roundabout_set_planar_target_speed(actor, speed_mps):
        """Apply the prescribed fixture speed along the vehicle heading."""
        forward = actor.get_transform().get_forward_vector()
        actor.set_target_velocity(carla.Vector3D(
            x=float(forward.x) * float(speed_mps),
            y=float(forward.y) * float(speed_mps),
            z=0.0))

    @staticmethod
    def _roundabout_enable_constant_speed(actor, speed_mps):
        """Lock a fixture vehicle to its prescribed local forward speed."""
        actor.enable_constant_velocity(carla.Vector3D(
            x=float(speed_mps), y=0.0, z=0.0))

    def _roundabout_gate_progress(self, actor, gate):
        front = self._roundabout_front_center(actor)
        center = gate["center"]
        forward_x, forward_y = gate["forward"]
        signed_progress = ((front.x - center.x) * forward_x
                           + (front.y - center.y) * forward_y)
        inside_segment = True
        left, right = gate.get("left"), gate.get("right")
        if left is not None and right is not None:
            segment_x = right.x - left.x
            segment_y = right.y - left.y
            length_squared = segment_x ** 2 + segment_y ** 2
            if length_squared > 1e-9:
                fraction = ((front.x - left.x) * segment_x
                            + (front.y - left.y) * segment_y) / length_squared
                margin = float(self.roundabout_engineering.get(
                    "gate_margin_m",
                    self.roundabout_engineering.get("gate_lateral_margin_m", 1.0)))
                margin_fraction = margin / math.sqrt(length_squared)
                inside_segment = -margin_fraction <= fraction <= 1.0 + margin_fraction
        return signed_progress, inside_segment

    def _roundabout_gate_crossed(self, actor, key, gate):
        progress, inside_segment = self._roundabout_gate_progress(actor, gate)
        previous = self.rb_gate_progress.get(key)
        self.rb_gate_progress[key] = progress
        return previous is not None and previous < 0.0 <= progress and inside_segment

    def _roundabout_vt1_upstream_remaining(self):
        if (self.vt1_actor is None or not self.vt1_actor.is_alive
                or self.rb_vt1_departed):
            return float("-inf")
        # Track near the controller's current route cursor.  A global nearest
        # projection is ambiguous when the route loops back near its start.
        segment_count = max(1, len(self.vt1_route) - 1)
        cursor = max(0, min(int(self.vt1_target_idx), segment_count - 1))
        window = 25
        progress, _ = self._roundabout_project_route(
            self.vt1_actor.get_location(), self.vt1_route, self.rb_vt1_route_s,
            start_index=max(0, cursor - window),
            end_index=min(segment_count, cursor + window + 1))
        self.rb_vt1_progress_s = max(self.rb_vt1_progress_s, progress)
        return self.rb_vt1_conflict_s - self.rb_vt1_progress_s

    def _roundabout_vt1_is_upstream(self):
        remaining = self._roundabout_vt1_upstream_remaining()
        return (self.rb_vt1_upstream_min <= remaining
                <= self.rb_vt1_upstream_max), remaining

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

    def _roundabout_vt1_lane_continuation_target(self, location):
        """Return a connected outgoing-lane target after the drawn VT1 route.

        The hand-drawn route remains authoritative through exit 1.  Once its
        last point is consumed, CARLA topology is used only to clear the target
        vehicle from the experiment area; it does not change the tested route
        or the exit verdict.
        """
        waypoint = self.map.get_waypoint(
            location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if waypoint is None:
            return None
        lookahead = max(4.0, float(self.rb_vt1_route_lookahead))
        try:
            candidates = list(waypoint.next(lookahead))
        except (AttributeError, RuntimeError):
            candidates = []
        if not candidates:
            return None

        actor_yaw = self.vt1_actor.get_transform().rotation.yaw

        def heading_error(candidate):
            target = candidate.transform.location
            target_yaw = math.degrees(math.atan2(
                target.y - location.y, target.x - location.x))
            return abs((target_yaw - actor_yaw + 180.0) % 360.0 - 180.0)

        return min(candidates, key=heading_error).transform.location

    def _roundabout_remove_departed_vt1(self, now):
        """Remove VT1 only after exit-1 evidence and downstream clearance."""
        if self.rb_vt1_departed:
            return
        actor = self.vt1_actor
        if actor is not None and actor.is_alive:
            if self.rb_vt1_constant_velocity_enabled:
                try:
                    actor.disable_constant_velocity()
                except RuntimeError:
                    pass
                self.rb_vt1_constant_velocity_enabled = False
            try:
                destroyed = actor.destroy()
            except RuntimeError:
                # Retry on the next synchronous tick instead of claiming that
                # a still-visible target has departed.
                return
            if destroyed is False and actor.is_alive:
                return
        self.rb_vt1_departed = True
        self.rb_vt1_departure_time = float(now)
        self.vt1_route_finished = True
        # Do not retain a CARLA proxy after its actor has been destroyed.
        # Accessing get_transform()/get_velocity() through that proxy raises
        # ``trying to operate on a destroyed actor`` while the runner is
        # assembling the terminal telemetry/result record.
        self.vt1_actor = None
        self._roundabout_record_event(
            now, "VT1_CLEARED_EXIT_AND_REMOVED",
            clearance_distance_m=round(self.rb_vt1_exit_clearance_travel_m, 3),
            required_clearance_distance_m=round(
                self.rb_vt1_post_exit_clearance_distance, 3))

    def _roundabout_follow_vt1(self):
        if self.vt1_actor is None or not self.vt1_actor.is_alive:
            return
        location = self.vt1_actor.get_location()
        while self.vt1_target_idx < len(self.vt1_route):
            if location.distance(self.vt1_route[self.vt1_target_idx]) \
                    > self.rb_vt1_route_lookahead:
                break
            self.vt1_target_idx += 1
        if self.vt1_target_idx >= len(self.vt1_route):
            self.rb_vt1_drawn_route_finished = True
            if not self.rb_vt1_exit1_crossed:
                self.vt1_route_finished = True
                if self.rb_vt1_constant_velocity_enabled:
                    self.vt1_actor.disable_constant_velocity()
                    self.rb_vt1_constant_velocity_enabled = False
                self._roundabout_hold_vehicle(self.vt1_actor, hand_brake=False)
                return
            target = self._roundabout_vt1_lane_continuation_target(location)
            if target is None:
                # A malformed/dead-end topology must not leave VT1 parked on
                # the exit.  Continue along its current heading as a bounded
                # cleanup fallback; any fixture collision remains INVALID.
                transform = self.vt1_actor.get_transform()
                forward = transform.get_forward_vector()
                distance = max(4.0, float(self.rb_vt1_route_lookahead))
                target = carla.Location(
                    x=location.x + forward.x * distance,
                    y=location.y + forward.y * distance,
                    z=location.z + forward.z * distance)
                if not self.rb_vt1_topology_fallback_reported:
                    self.rb_vt1_topology_fallback_reported = True
                    now = self.world.get_snapshot().timestamp.elapsed_seconds
                    self._roundabout_record_event(
                        now, "VT1_POST_EXIT_TOPOLOGY_FALLBACK")
        else:
            target = self.vt1_route[self.vt1_target_idx]
        target_yaw = math.degrees(math.atan2(target.y - location.y, target.x - location.x))
        transform = self.vt1_actor.get_transform()
        heading_error = (target_yaw - transform.rotation.yaw + 180.0) % 360.0 - 180.0
        speed = self._roundabout_vehicle_speed(self.vt1_actor)
        speed_error = self.vt1_target_speed_mps - speed
        # Different CARLA vehicle blueprints need substantially different
        # steady-state throttle.  A feed-forward + PI term reaches and holds
        # 15 km/h across the configured target fleet; the previous weak
        # proportional law plateaued near 8.5 km/h for several vehicles.
        self.rb_vt1_speed_integral = max(-2.0, min(
            3.0, self.rb_vt1_speed_integral + speed_error * 0.05))
        control = carla.VehicleControl()
        control.steer = max(-0.85, min(0.85, heading_error * 0.035))
        control.hand_brake = False
        if speed_error > -0.20:
            control.throttle = max(0.0, min(
                0.85,
                0.32 + speed_error * 0.20
                + self.rb_vt1_speed_integral * 0.04))
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(0.45, 0.08 + (-speed_error) * 0.10)
        self.vt1_actor.apply_control(control)
        if self.rb_vt1_speed_control_mode == "constant_velocity":
            # CARLA's constant-velocity controller loses planar speed under
            # steering. Compensate the simulator artefact using the applied
            # steering magnitude so measured fixture speed remains 15 km/h.
            compensated_speed = self.vt1_target_speed_mps * (
                1.0 + self.rb_vt1_corner_speed_compensation
                * abs(control.steer))
            self._roundabout_enable_constant_speed(
                self.vt1_actor, compensated_speed)
        elif self.rb_vt1_speed_control_mode == "target_velocity":
            # VT1 is a prescribed test fixture, not the ADS under test.  Set
            # its planar velocity explicitly so different blueprint engines
            # and gearboxes do not change the required 15 km/h condition.
            self._roundabout_set_planar_target_speed(
                self.vt1_actor, self.vt1_target_speed_mps)

    def _roundabout_update_vt1_fixture(self, now, dt):
        """Verify that the controlled target continues to realize the test fixture."""
        if (self.rb_vt1_departed or self.vt1_actor is None
                or not self.vt1_actor.is_alive):
            return
        remaining_to_conflict = self._roundabout_vt1_upstream_remaining()
        if (not self.rb_vt1_conflict_crossed
                and remaining_to_conflict <= 0.0):
            self.rb_vt1_conflict_crossed = True
            self.rb_vt1_conflict_crossing_time = float(now)
            self._roundabout_record_event(
                now, "VT1_CROSSED_VUT_CONFLICT_POINT")
        exit_one = self.roundabout_exit_gates.get(1)
        if (not self.rb_vt1_exit1_crossed and exit_one is not None
                and self._roundabout_gate_crossed(
                    self.vt1_actor, "vt1_exit_1", exit_one)):
            self.rb_vt1_exit1_crossed = True
            self.rb_vt1_exit_clearance_last_location = \
                self.vt1_actor.get_location()
            self._roundabout_record_event(now, "VT1_CROSSED_EXIT", exit_number=1)

        if self.rb_vt1_exit1_crossed:
            location = self.vt1_actor.get_location()
            previous = self.rb_vt1_exit_clearance_last_location
            if previous is not None:
                step_distance = previous.distance(location)
                # Ignore simulator teleports while retaining ordinary 15 km/h
                # motion.  This prevents a reset from falsely satisfying the
                # downstream-clearance requirement.
                maximum_step = max(2.0, self.vt1_target_speed_mps * max(dt, 0.05) * 4.0)
                if step_distance <= maximum_step:
                    self.rb_vt1_exit_clearance_travel_m += step_distance
            self.rb_vt1_exit_clearance_last_location = location
            if (self.rb_vt1_exit_clearance_travel_m
                    >= self.rb_vt1_post_exit_clearance_distance):
                self._roundabout_remove_departed_vt1(now)
            return

        if (self.rb_trial_start_sim_time is None
                or self.rb_phase in ("COMPLETE", "INVALID", "TIMEOUT")):
            return
        speed = self._roundabout_vehicle_speed(self.vt1_actor)
        if abs(speed - self.vt1_target_speed_mps) > self.rb_vt1_speed_tolerance:
            self.rb_vt1_speed_out_of_tolerance_time += dt
        else:
            self.rb_vt1_speed_out_of_tolerance_time = 0.0
        if (self.rb_vt1_speed_out_of_tolerance_time
                >= self.rb_vt1_speed_continuity_duration):
            self.rb_vt1_speed_maintained = False
            if not self.rb_collision_recorded:
                self._roundabout_invalidate(now, "vt1_speed_not_maintained")
        if (self.vt1_route_finished and not self.rb_vt1_exit1_crossed
                and not self.rb_collision_recorded):
            self._roundabout_invalidate(now, "vt1_did_not_cross_exit_1")

    def _roundabout_wait_for_camera_frame(self, expected_frame):
        if expected_frame is None:
            return self.camera_data is not None
        deadline = time.monotonic() + self.rb_camera_frame_timeout
        with self.camera_condition:
            while (self.camera_frame is None
                   or int(self.camera_frame) < int(expected_frame)):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self.camera_condition.wait(remaining)
            return int(self.camera_frame) == int(expected_frame)

    def _roundabout_apply_ego_control(self, expected_frame=None, now=None):
        if self.rb_trial_start_sim_time is None:
            self._roundabout_hold_vehicle(self.ego, hand_brake=True)
            return
        if self.finished:
            # The VUT has completed its planned route outside the ring.  Keep
            # it stationary while the fixture confirms VT1 actually exits at
            # exit 1; this waiting period is not an in-roundabout stop.
            self._roundabout_hold_vehicle(self.ego, hand_brake=False)
            return
        if self.rb_tcp_required:
            if not self._roundabout_wait_for_camera_frame(expected_frame):
                if now is None:
                    now = self.world.get_snapshot().timestamp.elapsed_seconds
                self.rb_sut_error = "camera_frame_unavailable:{}".format(expected_frame)
                self._roundabout_invalidate(now, self.rb_sut_error)
                return
            try:
                action = self.tcp.get_action(self.camera_data, self.ego)
                throttle, steer, brake = decode_vehicle_action(action)
                self.control.throttle = throttle
                self.control.steer = steer
                self.control.brake = brake
                self.control.hand_brake = False
                self.rb_last_requested_control = {
                    "throttle": float(throttle),
                    "steer": float(steer),
                    "brake": float(brake),
                    "hand_brake": False,
                }
                self.ego.apply_control(self.control)
                return
            except Exception as exc:
                now = self.world.get_snapshot().timestamp.elapsed_seconds
                self.rb_sut_failure = "sut_control_error:{}".format(exc)
                print("[2.b/TCP] {}".format(self.rb_sut_failure))
                self._roundabout_record_event(
                    now, "SUT_CONTROL_FAILURE", detail=str(exc))
                self._roundabout_hold_vehicle(self.ego, hand_brake=False)
                self._roundabout_transition(
                    "COMPLETE", now, reason="sut_control_failure")
                return
        self.follow_route()
        applied = self.ego.get_control()
        self.rb_last_requested_control = {
            "throttle": float(getattr(applied, "throttle", 0.0)),
            "steer": float(getattr(applied, "steer", 0.0)),
            "brake": float(getattr(applied, "brake", 0.0)),
            "hand_brake": bool(getattr(applied, "hand_brake", False)),
        }

    def _roundabout_update_reference_indicator(self):
        """Operate lights only for the non-SUT facility self-check controller."""
        if self.model != "behavior" or self.ego is None:
            return
        gate = self.roundabout_exit_gates.get(self.roundabout_planned_exit)
        if gate is None:
            return
        distance = self.ego.get_location().distance(gate["center"])
        active = self.rb_phase == "IN_ROUNDABOUT" and distance <= 20.0
        try:
            current = int(self.ego.get_light_state())
            right = int(carla.VehicleLightState.RightBlinker)
            desired = current | right if active else current & ~right
            self.ego.set_light_state(carla.VehicleLightState(desired))
            if active:
                self.rb_indicator_evidence_source = "behavior_reference_control"
        except (AttributeError, RuntimeError, TypeError, ValueError):
            return

    def _roundabout_update_ego_route_completion(self):
        if self.finished or not self.route_points:
            return
        if self.rb_tcp_required:
            location = self.ego.get_location()
            while self.current_target_idx < len(self.route_points):
                if location.distance(self.route_points[self.current_target_idx]) \
                        > self.rb_route_completion_distance:
                    break
                self.current_target_idx += 1
            cumulative = self._roundabout_route_distances(self.route_points)
            progress, offset = self._roundabout_project_route(
                location, self.route_points, cumulative)
            if (progress >= cumulative[-1] - self.rb_route_completion_distance
                    and offset <= self.rb_route_completion_distance * 2.0):
                self.current_target_idx = len(self.route_points)
                self.finished = True

    def _roundabout_lane_matches_gate(self, gate):
        waypoint = self.map.get_waypoint(
            self.ego.get_location(), project_to_road=False,
            lane_type=carla.LaneType.Driving)
        if waypoint is None:
            return False
        raw = gate.get("raw", {})
        allowed = raw.get("allowed_lanes", [])
        if allowed:
            return any(
                int(item.get("road_id")) == int(waypoint.road_id)
                and (item.get("section_id") is None
                     or int(item.get("section_id"))
                     == int(getattr(waypoint, "section_id", 0)))
                and int(item.get("lane_id")) == int(waypoint.lane_id)
                for item in allowed if isinstance(item, dict)
            )
        road_id = gate.get("road_id")
        section_id = gate.get("section_id")
        lane_ids = gate.get("lane_ids")
        if lane_ids is None and gate.get("lane_id") is not None:
            lane_ids = [gate.get("lane_id")]
        if road_id is None and section_id is None and not lane_ids:
            return None
        if road_id is not None and int(road_id) != int(waypoint.road_id):
            return False
        if (section_id is not None
                and int(section_id) != int(getattr(waypoint, "section_id", 0))):
            return False
        if lane_ids and int(waypoint.lane_id) not in {int(value) for value in lane_ids}:
            return False
        return True

    def _roundabout_capture_entry_conditions(self, now):
        if self.rb_entry_arrived:
            return True
        self.rb_entry_arrived = True
        self.rb_entry_arrival_time = float(now)
        self._roundabout_record_event(now, "VUT_REACHED_ROUNDABOUT_ENTRY")
        if not self.roundabout_capable:
            return True

        vt1_actor = self.vt1_actor
        vt1_available = (
            vt1_actor is not None
            and bool(getattr(vt1_actor, "is_alive", True)))
        if vt1_available:
            vt1_speed = self._roundabout_vehicle_speed(vt1_actor)
            vt1_upstream, remaining = self._roundabout_vt1_is_upstream()
        else:
            # A slow/stopped VUT can reach the entry after VT1 has already
            # cleared exit 1 and been deliberately destroyed.  That is an
            # invalid fixture timing condition, not a runner exception.
            vt1_speed = None
            vt1_upstream = False
            remaining = None
        self.rb_vt1_speed_at_entry = vt1_speed
        self.rb_vt1_upstream_at_entry = vt1_upstream
        self.rb_vt1_remaining_at_entry = remaining
        self.rb_vt1_conflict_ttc_at_entry_s = (
            remaining / vt1_speed
            if remaining is not None and vt1_speed is not None
            and vt1_speed > 1e-6 else None)
        speed_valid = (
            vt1_speed is not None
            and abs(vt1_speed - self.vt1_target_speed_mps)
            <= self.rb_vt1_speed_tolerance)
        vt2_valid = self._roundabout_vehicle_speed(self.vt2_actor) \
            <= self.rb_vt2_stationary_speed
        if not vt1_available:
            self.rb_entry_sync_missed = True
            reason = (
                "vt1_passed_entry_before_vut_entry"
                if self.rb_vt1_departed or self.vt1_route_finished
                else "vt1_unavailable_at_vut_entry")
            self._roundabout_record_event(
                now, "VT1_UNAVAILABLE_AT_VUT_ENTRY",
                departed=bool(self.rb_vt1_departed), reason=reason)
            if not vt2_valid:
                self._roundabout_invalidate(now, "vt2_not_stationary_at_vut_entry")
                return False
            return True
        if not speed_valid:
            self._roundabout_invalidate(now, "vt1_speed_invalid_at_vut_entry")
            return False
        if not vt1_upstream:
            self.rb_entry_sync_missed = True
            self._roundabout_record_event(
                now, "VT1_NOT_UPSTREAM_AT_VUT_ENTRY",
                remaining_m=round(remaining, 3))
        gap_valid = (
            self.rb_vt1_entry_gap_min_m
            <= remaining <= self.rb_vt1_entry_gap_max_m)
        if vt1_upstream and not gap_valid:
            self.rb_entry_sync_missed = True
            self._roundabout_record_event(
                now, "VT1_CONFLICT_GAP_OUT_OF_WINDOW",
                remaining_m=round(remaining, 3),
                minimum_m=round(self.rb_vt1_entry_gap_min_m, 3),
                maximum_m=round(self.rb_vt1_entry_gap_max_m, 3),
                target_m=round(self.rb_vt1_entry_gap_target_m, 3))
        if not vt2_valid:
            self._roundabout_invalidate(now, "vt2_not_stationary_at_vut_entry")
            return False
        if vt1_upstream and gap_valid:
            self._roundabout_record_event(
                now, "CONFLICT_SYNC_VERIFIED",
                vt1_remaining_m=round(remaining, 3),
                vt1_ttc_s=round(self.rb_vt1_conflict_ttc_at_entry_s, 3)
                if self.rb_vt1_conflict_ttc_at_entry_s is not None else None,
                target_gap_m=round(self.rb_vt1_entry_gap_target_m, 3),
                allowed_gap_m=[
                    round(self.rb_vt1_entry_gap_min_m, 3),
                    round(self.rb_vt1_entry_gap_max_m, 3),
                ])
        return True

    def _roundabout_update_entry_arrival(self, now):
        if self.rb_entry_arrived or self.rb_entry_crossed \
                or self.rb_phase in ("COMPLETE", "INVALID", "TIMEOUT"):
            return
        progress, inside_segment = self._roundabout_gate_progress(
            self.ego, self.roundabout_entry_gate)
        if inside_segment and progress >= -self.rb_entry_arrival_distance:
            self._roundabout_capture_entry_conditions(now)

    def _roundabout_update_gate_events(self, now):
        if self.rb_phase in ("COMPLETE", "INVALID", "TIMEOUT"):
            return
        if (not self.rb_entry_crossed
                and self._roundabout_gate_crossed(
                    self.ego, "entry", self.roundabout_entry_gate)):
            self.rb_entry_crossed = True
            self.rb_vut_entry_crossing_time = float(now)
            if self.rb_vt1_conflict_crossing_time is not None:
                self.rb_vt1_lead_time_at_vut_entry_s = (
                    float(now) - self.rb_vt1_conflict_crossing_time)
            else:
                self.rb_vut_entered_before_vt1 = True
            self._roundabout_record_event(now, "VUT_ENTERED_ROUNDABOUT")
            self._roundabout_record_event(
                now, "MERGE_ORDER_CHECK",
                vt1_crossed_first=not self.rb_vut_entered_before_vt1,
                vt1_lead_time_s=(
                    round(self.rb_vt1_lead_time_at_vut_entry_s, 3)
                    if self.rb_vt1_lead_time_at_vut_entry_s is not None
                    else None))
            if not self.roundabout_capable:
                self._roundabout_transition(
                    "COMPLETE", now, reason="entered_without_roundabout_capability")
                return
            if not self._roundabout_capture_entry_conditions(now):
                return
            self._roundabout_transition("IN_ROUNDABOUT", now)

        if not self.rb_entry_crossed or self.rb_phase in ("INVALID", "COMPLETE"):
            return
        for number, gate in sorted(self.roundabout_exit_gates.items()):
            if not self._roundabout_gate_crossed(
                    self.ego, "exit_{}".format(number), gate):
                continue
            self._roundabout_record_event(now, "VUT_CROSSED_EXIT", exit_number=number)
            if number != self.roundabout_planned_exit:
                # Custom OpenDRIVE roads may legitimately cross an auxiliary
                # reviewed OUT gate before the final planned gate.  Record it
                # for audit, but only the configured planned gate completes
                # the manoeuvre.
                continue
            self.rb_correct_exit_crossed = True
            if self.rb_indicator_observable:
                self.rb_exit_indicator_observed = (
                    self.rb_exit_indicator_last_seen is not None
                    and now - self.rb_exit_indicator_last_seen
                    <= self.rb_indicator_lookback)
            else:
                self.rb_exit_indicator_observed = None
            location = self.ego.get_location()
            self.rb_exit_location = carla.Location(location.x, location.y, location.z)
            self._roundabout_transition("EXITED", now)
            return

    def _roundabout_update_exit_lane(self, now):
        if not self.rb_correct_exit_crossed or self.rb_exit_lane_checked:
            return
        distance = self.ego.get_location().distance(self.rb_exit_location)
        if distance < self.rb_exit_lane_check_distance and not self.finished:
            return
        gate = self.roundabout_exit_gates[self.roundabout_planned_exit]
        self.rb_exit_lane_correct = self._roundabout_lane_matches_gate(gate)
        self.rb_exit_lane_checked = True
        self._roundabout_record_event(
            now, "EXIT_LANE_CHECK", correct=self.rb_exit_lane_correct)

    def _roundabout_update_indicator(self, now):
        """Track the right indicator in the engineering look-back window."""
        try:
            light_state = self.ego.get_light_state()
            right_blinker = carla.VehicleLightState.RightBlinker
            active = bool(light_state & right_blinker)
            self.rb_indicator_observable = True
        except (AttributeError, RuntimeError, TypeError):
            try:
                light_text = str(self.ego.get_light_state())
                active = "RightBlinker" in light_text
                self.rb_indicator_observable = True
            except (AttributeError, RuntimeError):
                return
        if active:
            self.rb_exit_indicator_last_seen = now
            self.rb_indicator_evidence_source = "carla_vehicle_light_state"
        if active and not self.rb_indicator_active:
            self._roundabout_record_event(now, "RIGHT_INDICATOR_ON")
        elif not active and self.rb_indicator_active:
            self._roundabout_record_event(now, "RIGHT_INDICATOR_OFF")
        self.rb_indicator_active = active

    def _roundabout_update_measurements(
            self, now, dt, phase_for_frame=None, brake_for_frame=None):
        phase_for_frame = phase_for_frame or self.rb_phase
        speed = self._roundabout_vehicle_speed(self.ego)
        self.rb_max_speed_mps = max(self.rb_max_speed_mps, speed)
        try:
            speed_limit_kmh = float(self.ego.get_speed_limit())
        except (AttributeError, RuntimeError, TypeError, ValueError):
            speed_limit_kmh = 0.0
        if speed_limit_kmh <= 0.0:
            waypoint = self.map.get_waypoint(
                self.ego.get_location(), project_to_road=True,
                lane_type=carla.LaneType.Driving)
            road_id = int(waypoint.road_id) if waypoint is not None else None
            speed_limit_kmh = float(self.rb_speed_limit_by_road_id.get(
                road_id, self.roundabout_engineering.get("speed_limit_kmh", 0.0)))
        if speed_limit_kmh > 0.0:
            self.rb_speed_limit_observed = True
            self.rb_observed_speed_limit_kmh = speed_limit_kmh
            self.rb_speed_limit_unobservable_time = 0.0
            if speed * 3.6 > speed_limit_kmh + self.rb_speed_tolerance_kmh:
                self.rb_speed_limit_exceeded = True
        elif (self.rb_trial_start_sim_time is not None
              and phase_for_frame in ("APPROACH", "IN_ROUNDABOUT", "EXITED")
              and not self.finished):
            self.rb_speed_limit_unobservable_time += dt
            if (self.rb_speed_limit_unobservable_time
                    >= self.rb_speed_limit_unobservable_duration):
                if not self.rb_speed_limit_unobservable_gap:
                    self.rb_speed_limit_unobservable_gap = True
                    self._roundabout_invalidate(
                        now, "speed_limit_not_observable_during_trial")

        vt2_speed = self._roundabout_vehicle_speed(self.vt2_actor)
        self.rb_vt2_max_speed = max(self.rb_vt2_max_speed, vt2_speed)
        if vt2_speed > self.rb_vt2_stationary_speed:
            self.rb_vt2_moved = True

        if phase_for_frame in ("APPROACH", "IN_ROUNDABOUT", "EXITED"):
            ego_location = self.ego.get_location()
            corridor_detail, corridor_allowed_m = \
                self._roundabout_vut_route_corridor(ego_location)
            waypoint = self.map.get_waypoint(
                ego_location, project_to_road=False,
                lane_type=carla.LaneType.Driving)
            if waypoint is None:
                self.rb_off_road_time += dt
                if self.rb_off_road_time >= self.rb_off_road_duration_limit:
                    self.rb_off_road = True
                self.rb_lane_direction_violation_time = 0.0
            else:
                self.rb_off_road_time = 0.0
                ego_yaw = self.ego.get_transform().rotation.yaw
                lane_yaw = waypoint.transform.rotation.yaw
                heading_error = abs((ego_yaw - lane_yaw + 180.0) % 360.0 - 180.0)
                if heading_error > 90.0:
                    self.rb_lane_direction_violation_time += dt
                    if (self.rb_lane_direction_violation_time
                            >= self.rb_lane_direction_duration_limit):
                        self.rb_lane_direction_violation = True
                else:
                    self.rb_lane_direction_violation_time = 0.0
                lane_token = (
                    int(waypoint.road_id), int(getattr(waypoint, "section_id", 0)),
                    int(waypoint.lane_id))
            if corridor_detail["distance_m"] > corridor_allowed_m:
                self.rb_lane_guidance_violation_time += dt
                if (self.rb_lane_guidance_violation_time
                        >= self.rb_lane_guidance_duration_limit):
                    self.rb_lane_guidance_violation = True
                    if self.rb_lane_guidance_violation_detail is None:
                        self.rb_lane_guidance_violation_detail = {
                            "sim_time": round(float(now), 4),
                            "route_offset_m": round(
                                corridor_detail["distance_m"], 4),
                            "allowed_offset_m": round(corridor_allowed_m, 4),
                            "route_progress_m": round(
                                corridor_detail["progress_m"], 4),
                            "route_segment_index": int(
                                corridor_detail["segment_index"]),
                            "runtime_lane": list(lane_token)
                            if waypoint is not None else None,
                            "runtime_lane_in_editor_token_set": bool(
                                waypoint is not None
                                and lane_token in self.rb_vut_route_lane_tokens),
                        }
            else:
                self.rb_lane_guidance_violation_time = 0.0

        if phase_for_frame != "IN_ROUNDABOUT":
            self.rb_stop_duration = 0.0
            self.rb_emergency_brake_duration = 0.0
            return
        if speed <= self.rb_stop_speed:
            self.rb_stop_duration += dt
            if self.rb_stop_duration >= self.rb_stop_duration_limit:
                self.rb_stopped_in_roundabout = True
        else:
            self.rb_stop_duration = 0.0

        transform = self.ego.get_transform()
        forward = transform.get_forward_vector()
        acceleration = self.ego.get_acceleration()
        longitudinal_acceleration = (
            acceleration.x * forward.x + acceleration.y * forward.y
            + acceleration.z * forward.z)
        self.rb_max_deceleration = max(
            self.rb_max_deceleration, max(0.0, -longitudinal_acceleration))
        if brake_for_frame is None:
            brake_for_frame = float(getattr(self.ego.get_control(), "brake", 0.0))
        emergency_now = (
            longitudinal_acceleration <= -self.rb_emergency_decel
            or float(brake_for_frame) >= self.rb_emergency_brake)
        if emergency_now:
            self.rb_emergency_brake_duration += dt
            if self.rb_emergency_brake_duration >= self.rb_emergency_brake_duration_limit:
                self.rb_emergency_braking = True
        else:
            self.rb_emergency_brake_duration = 0.0

    @staticmethod
    def _roundabout_vector_sample(vector):
        return {
            "x": round(float(getattr(vector, "x", 0.0)), 4),
            "y": round(float(getattr(vector, "y", 0.0)), 4),
            "z": round(float(getattr(vector, "z", 0.0)), 4),
        }

    def _roundabout_actor_sample(self, actor, role=None):
        if actor is None or not actor.is_alive:
            return None
        transform = actor.get_transform()
        location = transform.location
        control = actor.get_control()
        velocity = actor.get_velocity()
        try:
            acceleration = actor.get_acceleration()
        except (AttributeError, RuntimeError):
            acceleration = carla.Vector3D()
        try:
            angular_velocity = actor.get_angular_velocity()
        except (AttributeError, RuntimeError):
            angular_velocity = carla.Vector3D()
        forward = transform.get_forward_vector()
        right_x, right_y = -forward.y, forward.x
        longitudinal_velocity = velocity.x * forward.x + velocity.y * forward.y
        lateral_velocity = velocity.x * right_x + velocity.y * right_y
        longitudinal_acceleration = (
            acceleration.x * forward.x + acceleration.y * forward.y)
        lateral_acceleration = acceleration.x * right_x + acceleration.y * right_y
        waypoint = self.map.get_waypoint(
            location, project_to_road=True, lane_type=carla.LaneType.Driving)
        try:
            raw_light_state = int(actor.get_light_state())
            light_state = str(actor.get_light_state())
        except (AttributeError, RuntimeError):
            raw_light_state = None
            light_state = None
        applied_control = {
            "throttle": round(float(getattr(control, "throttle", 0.0)), 4),
            "brake": round(float(getattr(control, "brake", 0.0)), 4),
            "steer": round(float(getattr(control, "steer", 0.0)), 4),
            "hand_brake": bool(getattr(control, "hand_brake", False)),
            "reverse": bool(getattr(control, "reverse", False)),
            "gear": int(getattr(control, "gear", 0)),
        }
        sample = {
            "actor_id": getattr(actor, "id", None),
            "type_id": getattr(actor, "type_id", None),
            "location": {
                "x": round(float(location.x), 3),
                "y": round(float(location.y), 3),
                "z": round(float(location.z), 3),
            },
            "rotation": {
                "roll": round(float(transform.rotation.roll), 3),
                "pitch": round(float(transform.rotation.pitch), 3),
                "yaw": round(float(transform.rotation.yaw), 3),
            },
            "velocity": self._roundabout_vector_sample(velocity),
            "acceleration": self._roundabout_vector_sample(acceleration),
            "angular_velocity": self._roundabout_vector_sample(angular_velocity),
            "speed_mps": round(self._roundabout_vehicle_speed(actor), 4),
            "longitudinal_velocity_mps": round(float(longitudinal_velocity), 4),
            "lateral_velocity_mps": round(float(lateral_velocity), 4),
            "longitudinal_acceleration_mps2": round(
                float(longitudinal_acceleration), 4),
            "lateral_acceleration_mps2": round(float(lateral_acceleration), 4),
            "control": applied_control,
            "applied_control": applied_control,
            "road_id": int(waypoint.road_id) if waypoint is not None else None,
            "section_id": int(waypoint.section_id) if waypoint is not None else None,
            "lane_id": int(waypoint.lane_id) if waypoint is not None else None,
            "light_state_raw": raw_light_state,
            "light_state": light_state,
        }
        if role == "vut":
            sample["requested_control"] = copy.deepcopy(
                self.rb_last_requested_control)
        return sample

    @staticmethod
    def _roundabout_pair_metrics(ego_sample, target_sample, prefix):
        if not ego_sample or not target_sample:
            return {
                "{}_distance_m".format(prefix): None,
                "{}_ttc_s".format(prefix): None,
            }
        ego_location = ego_sample["location"]
        target_location = target_sample["location"]
        dx = target_location["x"] - ego_location["x"]
        dy = target_location["y"] - ego_location["y"]
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            ttc = 0.0
        else:
            ego_velocity = ego_sample["velocity"]
            target_velocity = target_sample["velocity"]
            relative_x = target_velocity["x"] - ego_velocity["x"]
            relative_y = target_velocity["y"] - ego_velocity["y"]
            closing_speed = -(dx * relative_x + dy * relative_y) / distance
            ttc = distance / closing_speed if closing_speed > 1e-3 else None
        return {
            "{}_distance_m".format(prefix): round(distance, 4),
            "{}_ttc_s".format(prefix): round(ttc, 4) if ttc is not None else None,
        }

    def _roundabout_sample_timeline(self, snapshot, now):
        trial_time = None
        if self.rb_trial_start_sim_time is not None:
            trial_time = max(0.0, now - self.rb_trial_start_sim_time)
        if (self.rb_last_timeline_sample_time is not None
                and now - self.rb_last_timeline_sample_time + 1e-9
                < self.rb_timeline_interval
                and self.rb_phase not in ("COMPLETE", "INVALID", "TIMEOUT")):
            return
        self.rb_last_timeline_sample_time = now
        vut_sample = self._roundabout_actor_sample(self.ego, "vut")
        vt1_sample = self._roundabout_actor_sample(self.vt1_actor, "vt1")
        vt2_sample = self._roundabout_actor_sample(self.vt2_actor, "vt2")
        relative_metrics = {}
        relative_metrics.update(self._roundabout_pair_metrics(
            vut_sample, vt1_sample, "vut_vt1"))
        relative_metrics.update(self._roundabout_pair_metrics(
            vut_sample, vt2_sample, "vut_vt2"))
        route_metrics = {}
        if self.route_points and self.ego is not None:
            cumulative = self._roundabout_route_distances(self.route_points)
            progress, lateral_offset = self._roundabout_project_route(
                self.ego.get_location(), self.route_points, cumulative)
            route_metrics = {
                "progress_m": round(float(progress), 4),
                "length_m": round(float(cumulative[-1]), 4),
                "completion_ratio": round(
                    float(progress / cumulative[-1]) if cumulative[-1] > 0 else 0.0,
                    6),
                "lateral_offset_m": round(float(lateral_offset), 4),
                "target_index": int(self.current_target_idx),
            }
        self.timeline_samples.append({
            "frame": getattr(snapshot, "frame", None),
            "sim_time": round(float(now), 4),
            "trial_time": round(float(trial_time), 4) if trial_time is not None else None,
            "phase": self.rb_phase,
            "ads_active": bool(self.rb_trial_start_sim_time is not None),
            "ads_control_source": self.rb_ads_control_source,
            "tcp_debug": copy.deepcopy(
                getattr(self.tcp, "last_debug", None))
            if self.rb_tcp_required and self.tcp is not None else None,
            "vut": vut_sample,
            "vt1": vt1_sample,
            "vt2": vt2_sample,
            "relative_metrics": relative_metrics,
            "route_metrics": route_metrics,
        })
        overflow = len(self.timeline_samples) - self.rb_max_timeline_samples
        if overflow > 0:
            del self.timeline_samples[:overflow]

    def _roundabout_tick(self):
        if not self.ego.is_alive:
            return False
        snapshot = self.world.get_snapshot()
        now = snapshot.timestamp.elapsed_seconds
        dt = max(0.0, now - self.rb_last_sim_time)
        self.rb_last_sim_time = now

        if self.rb_phase in ("COMPLETE", "INVALID", "TIMEOUT"):
            self._roundabout_sample_timeline(snapshot, now)
            return False
        if self.rb_phase == "SETUP":
            self._roundabout_transition("STABILIZING", now)

        self._roundabout_hold_vehicle(self.vt2_actor, hand_brake=True)
        self._roundabout_follow_vt1()
        self._roundabout_update_vt1_fixture(now, dt)
        if self.rb_phase in ("COMPLETE", "INVALID", "TIMEOUT"):
            self._roundabout_update_measurements(now, dt)
            self._roundabout_sample_timeline(snapshot, now)
            return False

        if self.rb_phase == "STABILIZING":
            self._roundabout_hold_vehicle(self.ego, hand_brake=True)
            vt1_speed = self._roundabout_vehicle_speed(self.vt1_actor)
            upstream, remaining = self._roundabout_vt1_is_upstream()
            speed_ready = abs(vt1_speed - self.vt1_target_speed_mps) \
                <= self.rb_vt1_speed_tolerance
            if speed_ready and upstream:
                self.rb_vt1_stable_time += dt
            else:
                self.rb_vt1_stable_time = 0.0
            if self.vt1_route_finished or remaining < self.rb_vt1_upstream_min:
                self._roundabout_invalidate(now, "vt1_passed_entry_before_ready")
            elif now - self.rb_start_sim_time >= self.rb_stabilization_timeout:
                self._roundabout_invalidate(now, "vt1_stabilization_timeout")
            elif self.rb_vt1_stable_time >= self.rb_vt1_stable_required:
                if not self.rb_vt1_stable_observed:
                    self.rb_vt1_stable_observed = True
                    self._roundabout_record_event(
                        now, "VT1_STABLE_AT_TARGET_SPEED",
                        speed_kmh=round(vt1_speed * 3.6, 3),
                        upstream_remaining_m=round(remaining, 3),
                        release_remaining_m=round(
                            self.rb_vt1_release_remaining_m, 3))
                release_due = (
                    not self.roundabout_capable
                    or remaining <= self.rb_vt1_release_remaining_m)
                if not release_due:
                    if not any(
                            event.get("event") == "WAITING_FOR_CONFLICT_RELEASE_LINE"
                            for event in self.rb_events):
                        self._roundabout_record_event(
                            now, "WAITING_FOR_CONFLICT_RELEASE_LINE",
                            current_remaining_m=round(remaining, 3),
                            release_remaining_m=round(
                                self.rb_vt1_release_remaining_m, 3))
                    self._roundabout_update_measurements(
                        now, dt, phase_for_frame="STABILIZING",
                        brake_for_frame=0.0)
                    self._roundabout_sample_timeline(snapshot, now)
                    return True
                camera_ready = (not self.rb_tcp_required
                                or self._roundabout_wait_for_camera_frame(
                                    getattr(snapshot, "frame", None)))
                if not camera_ready:
                    if self.rb_sut_ready_wait_started is None:
                        self.rb_sut_ready_wait_started = now
                        self._roundabout_record_event(now, "WAITING_FOR_SUT_CAMERA")
                    elif now - self.rb_sut_ready_wait_started \
                            >= self.rb_sut_readiness_timeout:
                        self.rb_sut_error = "sut_camera_readiness_timeout"
                        self._roundabout_invalidate(now, self.rb_sut_error)
                else:
                    self.rb_vt1_ready = True
                    self.rb_trial_start_sim_time = now
                    self._roundabout_record_event(
                        now, "VT1_READY", speed_kmh=round(vt1_speed * 3.6, 3),
                        upstream_remaining_m=round(remaining, 3),
                        target_entry_gap_m=round(
                            self.rb_vt1_entry_gap_target_m, 3))
                    self._roundabout_record_event(
                        now, "VUT_RELEASED",
                        vt1_remaining_m=round(remaining, 3),
                        synchronization="route_distance_time_headway")
                    self.ego.apply_control(carla.VehicleControl())
                    self._roundabout_transition("APPROACH", now)

        if self.rb_phase not in ("STABILIZING", "INVALID"):
            phase_for_frame = self.rb_phase
            try:
                brake_for_frame = float(getattr(self.ego.get_control(), "brake", 0.0))
            except (AttributeError, RuntimeError, TypeError, ValueError):
                brake_for_frame = 0.0
            self._roundabout_apply_ego_control(
                expected_frame=getattr(snapshot, "frame", None), now=now)
            if self.rb_phase in ("COMPLETE", "INVALID", "TIMEOUT"):
                self._roundabout_update_measurements(
                    now, dt, phase_for_frame=phase_for_frame,
                    brake_for_frame=brake_for_frame)
                self._roundabout_sample_timeline(snapshot, now)
                return False
            self._roundabout_update_ego_route_completion()
            self._roundabout_update_reference_indicator()
            self._roundabout_update_indicator(now)
            self._roundabout_update_entry_arrival(now)
            self._roundabout_update_gate_events(now)
            self._roundabout_update_exit_lane(now)
        else:
            phase_for_frame = self.rb_phase
            brake_for_frame = 0.0

        self._roundabout_update_measurements(
            now, dt, phase_for_frame=phase_for_frame,
            brake_for_frame=brake_for_frame)
        if self.rb_phase in ("COMPLETE", "INVALID", "TIMEOUT"):
            self._roundabout_sample_timeline(snapshot, now)
            return False

        if (self.roundabout_capable and self.rb_phase == "APPROACH"
                and not self.rb_entry_arrived):
            _, remaining = self._roundabout_vt1_is_upstream()
            approach_elapsed = now - self.rb_trial_start_sim_time
            if (approach_elapsed >= self.rb_vut_approach_time_budget
                    and not self.rb_approach_time_budget_exceeded):
                self.rb_approach_time_budget_exceeded = True
                self._roundabout_record_event(
                    now, "VUT_ENTRY_ARRIVAL_TIMEOUT",
                    approach_elapsed_s=round(approach_elapsed, 3),
                    time_budget_s=round(self.rb_vut_approach_time_budget, 3))
            if ((self.vt1_route_finished
                 or remaining < self.rb_vt1_upstream_min)
                    and not self.rb_entry_sync_missed):
                self.rb_entry_sync_missed = True
                self._roundabout_record_event(
                    now, "VT1_PASSED_ENTRY_BEFORE_VUT_ARRIVAL",
                    upstream_remaining_m=round(remaining, 3),
                    approach_elapsed_s=round(approach_elapsed, 3))

        if (self.rb_phase in ("APPROACH", "IN_ROUNDABOUT", "EXITED")
                and self.rb_vt2_moved and not self.rb_collision_vt2):
            self._roundabout_invalidate(now, "vt2_fixture_moved")

        if self.roundabout_capable:
            if self.rb_wrong_exit is not None:
                self._roundabout_transition("COMPLETE", now, reason="wrong_exit")
            elif self.finished:
                self._roundabout_hold_vehicle(self.ego, hand_brake=False)
                if self.rb_correct_exit_crossed:
                    self._roundabout_update_exit_lane(now)
                    if self.rb_vt1_departed or self.rb_collision_recorded:
                        self._roundabout_transition(
                            "COMPLETE", now,
                            reason=("vut_route_and_vt1_departure_finished"
                                    if self.rb_vt1_departed
                                    else "collision_verdict_complete"))
                    elif self.rb_vut_finished_waiting_for_vt1_since is None:
                        self.rb_vut_finished_waiting_for_vt1_since = now
                        self._roundabout_record_event(
                            now, ("WAITING_FOR_VT1_EXIT_CLEARANCE"
                                  if self.rb_vt1_exit1_crossed
                                  else "WAITING_FOR_VT1_EXIT_1"),
                            clearance_distance_m=round(
                                self.rb_vt1_exit_clearance_travel_m, 3),
                            required_clearance_distance_m=round(
                                self.rb_vt1_post_exit_clearance_distance, 3))
                    elif (now - self.rb_vut_finished_waiting_for_vt1_since
                          >= self.rb_vt1_exit_observation_timeout):
                        self._roundabout_invalidate(
                            now, ("vt1_exit_clearance_not_completed_before_fixture_timeout"
                                  if self.rb_vt1_exit1_crossed
                                  else "vt1_exit_1_not_observed_before_fixture_timeout"))
                else:
                    self._roundabout_transition(
                        "COMPLETE", now, reason="vut_route_finished_without_planned_exit")
        elif (self.rb_trial_start_sim_time is not None
              and self.rb_odd_alert_observed and not self.rb_entry_crossed
              and now - self.rb_trial_start_sim_time >= self.rb_incapable_observation
              and self._roundabout_vehicle_speed(self.ego) <= self.rb_stop_speed):
            self._roundabout_transition("COMPLETE", now, reason="odd_alert_and_no_entry")

        if (self.rb_phase not in ("COMPLETE", "INVALID", "TIMEOUT")
                and self.rb_trial_start_sim_time is not None
                and self._roundabout_experiment_elapsed(now) >= self.timeout):
            if (self.roundabout_capable and self.finished
                    and self.rb_correct_exit_crossed
                    and not self.rb_vt1_exit1_crossed
                    and not self.rb_collision_recorded):
                self._roundabout_invalidate(
                    now, "vt1_exit_1_not_observed_before_scenario_timeout")
            else:
                self.rb_timed_out = True
                self._roundabout_transition("TIMEOUT", now)

        self._roundabout_sample_timeline(snapshot, now)
        return self.rb_phase not in ("COMPLETE", "INVALID", "TIMEOUT")

    def tick(self):
        if not self.ego:
            return True

        if self.is_roundabout_2b:
            return self._roundabout_tick()

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

    def _roundabout_telemetry_summary(self):
        active = [sample for sample in self.timeline_samples
                  if sample.get("trial_time") is not None]
        if not active:
            return {
                "telemetry_sample_count": 0,
                "trial_duration_s": 0.0,
            }

        def finite_values(path):
            values = []
            for sample in active:
                value = sample
                for key in path:
                    value = value.get(key) if isinstance(value, dict) else None
                try:
                    value = float(value)
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    values.append(value)
            return values

        vut_speeds = finite_values(("vut", "speed_mps"))
        vt1_speeds = finite_values(("vt1", "speed_mps"))
        distances_vt1 = finite_values(("relative_metrics", "vut_vt1_distance_m"))
        distances_vt2 = finite_values(("relative_metrics", "vut_vt2_distance_m"))
        ttc_values = (
            finite_values(("relative_metrics", "vut_vt1_ttc_s"))
            + finite_values(("relative_metrics", "vut_vt2_ttc_s")))
        route_progress = finite_values(("route_metrics", "progress_m"))
        route_offsets = finite_values(("route_metrics", "lateral_offset_m"))
        trial_times = finite_values(("trial_time",))

        def mean(values):
            return sum(values) / len(values) if values else None

        vt1_mean = mean(vt1_speeds)
        vt1_variance = (
            sum((value - vt1_mean) ** 2 for value in vt1_speeds)
            / len(vt1_speeds)
            if vt1_speeds else None)
        summary = {
            "telemetry_sample_count": len(active),
            "telemetry_frequency_hz": round(
                1.0 / self.rb_timeline_interval, 3)
            if self.rb_timeline_interval > 0.0 else None,
            "trial_duration_s": round(max(trial_times), 4) if trial_times else 0.0,
            "average_speed_mps": round(mean(vut_speeds), 4)
            if vut_speeds else None,
            "minimum_vut_vt1_distance_m": round(min(distances_vt1), 4)
            if distances_vt1 else None,
            "minimum_vut_vt2_distance_m": round(min(distances_vt2), 4)
            if distances_vt2 else None,
            "minimum_ttc_s": round(min(ttc_values), 4) if ttc_values else None,
            "vt1_speed_mean_kmh": round(vt1_mean * 3.6, 4)
            if vt1_mean is not None else None,
            "vt1_speed_std_kmh": round(math.sqrt(vt1_variance) * 3.6, 4)
            if vt1_variance is not None else None,
            "route_progress_m": round(max(route_progress), 4)
            if route_progress else None,
            "maximum_route_lateral_offset_m": round(max(route_offsets), 4)
            if route_offsets else None,
        }
        if (self.rb_vt1_conflict_crossing_time is not None
                and self.rb_vut_entry_crossing_time is not None):
            summary["conflict_pet_s"] = round(abs(
                float(self.rb_vut_entry_crossing_time)
                - float(self.rb_vt1_conflict_crossing_time)), 4)
        else:
            summary["conflict_pet_s"] = None
        return summary

    def get_result(self, collision_occurred=False):
        """Return 2.b scoring; generic route scenes retain their old output."""
        if not self.is_roundabout_2b:
            return {}

        collision_occurred = bool(collision_occurred or self.rb_collision_recorded)
        if collision_occurred and not any(
                item.get("event") == "COLLISION" for item in self.rb_events):
            now = self.world.get_snapshot().timestamp.elapsed_seconds
            self._roundabout_record_event(now, "COLLISION")

        collision_time = getattr(self, "rb_valid_collision_time", None)
        if collision_occurred:
            # A collision is a valid tested-system failure.  If it prevents
            # VT1 from finishing its fixture route, do not relabel that same
            # attempt as a setup INVALID during asynchronous sensor drain.
            fixture_timeout_reasons = {
                "vt1_exit_1_not_observed_before_fixture_timeout",
                "vt1_exit_1_not_observed_before_scenario_timeout",
            }
            invalid_times = {
                item.get("reason"): item.get("sim_time")
                for item in getattr(self, "rb_invalid_events", [])
                if isinstance(item, dict)}
            retained = []
            for reason in self.rb_invalid_reasons:
                if reason in fixture_timeout_reasons:
                    continue
                invalid_time = invalid_times.get(reason)
                if (collision_time is not None and invalid_time is not None
                        and float(invalid_time) >= float(collision_time) - 1e-7):
                    continue
                retained.append(reason)
            self.rb_invalid_reasons = retained

        if (self.rb_speed_limit_unobservable_gap
                and not (collision_time is not None
                         and any(item.get("reason")
                                 == "speed_limit_not_observable_during_trial"
                                 and float(item.get("sim_time", float("-inf")))
                                 >= float(collision_time) - 1e-7
                                 for item in getattr(self, "rb_invalid_events", [])))):
            reason = "speed_limit_not_observable_during_trial"
            if reason not in self.rb_invalid_reasons:
                self.rb_invalid_reasons.append(reason)
        elif (not self.rb_speed_limit_observed
              and "speed_limit_not_observable" not in self.rb_invalid_reasons):
            self.rb_invalid_reasons.append("speed_limit_not_observable")

        failures = []
        for reason in self.rb_invalid_reasons:
            failures.append("invalid_precondition:{}".format(reason))
        if collision_occurred:
            failures.append("collision")
        if self.rb_timed_out:
            failures.append("timeout")
        if getattr(self, "rb_approach_time_budget_exceeded", False):
            failures.append("vut_entry_arrival_timeout")
        if self.rb_vt2_moved:
            failures.append("vt2_not_stationary")
        if self.rb_speed_limit_exceeded:
            failures.append("speed_limit_exceeded")
        if self.rb_solid_line_invasion:
            failures.append("solid_line_invasion")
        if self.rb_lane_direction_violation:
            failures.append("lane_direction_violation")
        if self.rb_lane_guidance_violation:
            failures.append("lane_guidance_violation")
        if self.rb_off_road:
            failures.append("off_road")
        if self.rb_sut_failure:
            failures.append("sut_control_failure")

        if self.roundabout_capable:
            if not self.rb_entry_crossed:
                failures.append("roundabout_entry_not_reached")
            if getattr(self, "rb_vut_entered_before_vt1", False):
                failures.append("vut_entered_before_vt1_at_merge")
            if self.rb_wrong_exit is not None:
                failures.append("wrong_exit_{}".format(self.rb_wrong_exit))
            if not self.rb_correct_exit_crossed:
                failures.append("planned_exit_not_reached")
            # Preserve the current OpenDRIVE lane check as diagnostic evidence.
            # The reviewed exit gate lies on a road-segment boundary, so exact
            # road_id equality is not reliable enough to affect PASS/FAIL.
            if (self.finished and self.rb_correct_exit_crossed
                    and not self.rb_vt1_exit1_crossed
                    and not collision_occurred):
                failures.append("vt1_exit_1_not_observed")
            if not self.finished:
                failures.append("route_incomplete")
            if self.rb_emergency_braking:
                failures.append("emergency_braking_in_roundabout")
            if self.rb_stopped_in_roundabout:
                failures.append("stopped_in_roundabout")
        else:
            if not self.rb_odd_alert_observed:
                failures.append("odd_alert_not_observed")
            if self.rb_entry_crossed:
                failures.append("entered_roundabout_without_capability")
            if (self.rb_entry_crossed and self.rb_odd_alert_time is not None
                    and any(item.get("event") == "VUT_ENTERED_ROUNDABOUT"
                            and self.rb_odd_alert_time >= item.get("sim_time", float("inf"))
                            for item in self.rb_events)):
                failures.append("odd_alert_not_before_entry")
            if self.rb_phase != "COMPLETE" and not self.rb_timed_out:
                failures.append("no_entry_observation_incomplete")

        if self.rb_invalid_reasons:
            # A failed setup/precondition is not a tested-system verdict.  Do
            # not mix secondary route/timeout symptoms into its rerun reason.
            failures = ["invalid_precondition:{}".format(reason)
                        for reason in self.rb_invalid_reasons]
        # Preserve order while avoiding duplicate reasons caused by combined
        # terminal conditions.
        failures = list(dict.fromkeys(failures))
        reproducibility = self.roundabout_cfg.get("reproducibility", {}) or {}
        topology = self.roundabout_cfg.get("topology", {}) or {}
        valid_trial = not self.rb_invalid_reasons
        objective_complete = (
            self.finished and self.rb_correct_exit_crossed
            if self.roundabout_capable
            else self.rb_phase == "COMPLETE" and not self.rb_entry_crossed)
        result = {
            "standard_clause": "GB/T 41798-2022 6.2.2",
            "matrix_id": (self.roundabout_cfg.get("matrix_id")
                          or reproducibility.get("matrix_id")),
            "trial_index": reproducibility.get("trial_index"),
            "base_seed": reproducibility.get("base_seed"),
            "matrix_seed": reproducibility.get("matrix_seed"),
            "trial_seed": reproducibility.get("trial_seed"),
            "route_fingerprint": reproducibility.get("route_fingerprint"),
            "condition_fingerprint": (
                (self.roundabout_cfg.get("generation") or {}).get(
                    "condition_fingerprint")
                or reproducibility.get("condition_fingerprint")
                or reproducibility.get("route_fingerprint")),
            "topology_hash": (reproducibility.get("topology_hash")
                              or self.roundabout_cfg.get("topology_hash")
                              or topology.get("topology_hash")),
            "planned_exit": self.roundabout_planned_exit,
            "roundabout_capable": self.roundabout_capable,
            "valid_trial": valid_trial,
            "trial_valid": valid_trial,
            "precondition_valid": valid_trial,
            "invalid_reason": self.rb_invalid_reasons[0]
            if self.rb_invalid_reasons else None,
            "invalid_reasons": self.rb_invalid_reasons,
            "phase": ("COMPLETE" if collision_occurred and valid_trial
                      else self.rb_phase),
            "route_complete": bool(objective_complete),
            "entry_crossed": self.rb_entry_crossed,
            "entry_arrived": getattr(self, "rb_entry_arrived", False),
            "entry_arrival_time": getattr(self, "rb_entry_arrival_time", None),
            "entry_sync_missed": getattr(self, "rb_entry_sync_missed", False),
            "approach_time_budget_exceeded": getattr(
                self, "rb_approach_time_budget_exceeded", False),
            "correct_exit_crossed": self.rb_correct_exit_crossed,
            "correct_exit": bool(
                self.rb_correct_exit_crossed and self.rb_wrong_exit is None),
            "wrong_exit": self.rb_wrong_exit,
            "correct_exit_lane": self.rb_exit_lane_correct,
            "correct_lane": self.rb_exit_lane_correct,
            "exit_lane_evaluated_as_failure": False,
            "vt1_ready": self.rb_vt1_ready,
            "vt1_target_speed_kmh": round(self.vt1_target_speed_mps * 3.6, 3),
            "vt1_speed_control_mode": getattr(
                self, "rb_vt1_speed_control_mode", "constant_velocity"),
            "vt1_speed_at_entry_kmh": round(self.rb_vt1_speed_at_entry * 3.6, 3)
            if self.rb_vt1_speed_at_entry is not None else None,
            "vt1_entry_speed_kmh": round(self.rb_vt1_speed_at_entry * 3.6, 3)
            if self.rb_vt1_speed_at_entry is not None else None,
            "vt1_upstream_at_entry": self.rb_vt1_upstream_at_entry,
            "vt1_upstream_remaining_at_entry_m": round(
                self.rb_vt1_remaining_at_entry, 3)
            if self.rb_vt1_remaining_at_entry is not None else None,
            "vt1_conflict_ttc_at_entry_s": round(
                self.rb_vt1_conflict_ttc_at_entry_s, 3)
            if getattr(self, "rb_vt1_conflict_ttc_at_entry_s", None) is not None
            else None,
            "vt1_conflict_crossed_before_vut": bool(
                getattr(self, "rb_vt1_conflict_crossing_time", None) is not None
                and getattr(self, "rb_vut_entry_crossing_time", None) is not None
                and self.rb_vt1_conflict_crossing_time
                <= self.rb_vut_entry_crossing_time),
            "vt1_conflict_crossing_time": getattr(
                self, "rb_vt1_conflict_crossing_time", None),
            "vut_entry_crossing_time": getattr(
                self, "rb_vut_entry_crossing_time", None),
            "vt1_lead_time_at_vut_entry_s": getattr(
                self, "rb_vt1_lead_time_at_vut_entry_s", None),
            "vt1_conflict_gap_target_m": getattr(
                self, "rb_vt1_entry_gap_target_m", None),
            "vt1_conflict_gap_window_m": [
                getattr(self, "rb_vt1_entry_gap_min_m", None),
                getattr(self, "rb_vt1_entry_gap_max_m", None),
            ],
            "vt1_release_remaining_m": getattr(
                self, "rb_vt1_release_remaining_m", None),
            "vt1_speed_maintained": self.rb_vt1_speed_maintained,
            "vt1_exit1_crossed": self.rb_vt1_exit1_crossed,
            "vt1_exit_clearance_target_m": round(
                getattr(self, "rb_vt1_post_exit_clearance_distance", 25.0), 3),
            "vt1_exit_clearance_travel_m": round(
                getattr(self, "rb_vt1_exit_clearance_travel_m", 0.0), 3),
            "vt1_departed": getattr(self, "rb_vt1_departed", False),
            "vt1_departure_time": getattr(
                self, "rb_vt1_departure_time", None),
            "vt1_drawn_route_finished": getattr(
                self, "rb_vt1_drawn_route_finished", False),
            "vt2_max_speed_mps": round(self.rb_vt2_max_speed, 4),
            "vt2_stationary": not self.rb_vt2_moved,
            "max_speed_kmh": round(self.rb_max_speed_mps * 3.6, 3),
            "observed_speed_limit_kmh": self.rb_observed_speed_limit_kmh,
            "speed_limit_observed": self.rb_speed_limit_observed,
            "speed_limit_unobservable_gap": self.rb_speed_limit_unobservable_gap,
            "speed_limit_exceeded": self.rb_speed_limit_exceeded,
            "speed_limit_violation": self.rb_speed_limit_exceeded,
            "solid_line_invasion": self.rb_solid_line_invasion,
            "lane_direction_violation": self.rb_lane_direction_violation,
            "lane_guidance_violation": self.rb_lane_guidance_violation,
            "lane_guidance_violation_detail": getattr(
                self, "rb_lane_guidance_violation_detail", None),
            "off_road": self.rb_off_road,
            "lane_invasion_events": self.rb_lane_invasion_events,
            "exit_indicator_observed": self.rb_exit_indicator_observed,
            "exit_indicator_evidence_source": self.rb_indicator_evidence_source,
            # The connected TCP controller has no turn-signal output channel.
            # Preserve CARLA/HMI observations for diagnosis, but do not turn
            # missing evidence into a tested-system violation or FAIL verdict.
            "turn_signal_violation": None,
            "indicator_policy": {
                "observed_signal": "RightBlinker",
                "lookback_s": self.rb_indicator_lookback,
                "evaluated_as_failure": False,
                "source": "optional engineering diagnostic",
            },
            "engineering_parameters": self.roundabout_engineering,
            "max_deceleration_mps2": round(self.rb_max_deceleration, 4),
            "emergency_braking": self.rb_emergency_braking,
            "emergency_braking_after_entry": self.rb_emergency_braking,
            "stopped_in_roundabout": self.rb_stopped_in_roundabout,
            "stopped_after_entry": self.rb_stopped_in_roundabout,
            "odd_alert_observed": self.rb_odd_alert_observed,
            "odd_alert_source": self.rb_odd_alert_source,
            "odd_alert_debug_configured": self.rb_odd_alert_debug_observed,
            "formal_hmi_evidence_required": self.rb_formal_hmi_evidence,
            "collision": bool(collision_occurred),
            "collision_vt1": self.rb_collision_vt1,
            "collision_vt2": self.rb_collision_vt2,
            "infrastructure_collision": self.rb_infrastructure_collision,
            "other_collision": self.rb_other_collision,
            "timed_out": self.rb_timed_out,
            "sut_error": self.rb_sut_error,
            "sut_control_failure": self.rb_sut_failure,
            "phase_history": self.rb_phase_history,
            "timeline_events": self.rb_events,
            "timeline_samples": self.timeline_samples,
            "pass": (not failures) if valid_trial else None,
            "failure_reasons": failures,
        }
        result.update(self._roundabout_telemetry_summary())
        return result
# ============================
# 车辆横穿场景（标准化 + TCP兼容）
# ============================
class CarCrossScene(BaseScene):
    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id)
        self.world = world
        self.map = self.world.get_map()
        self.cars = []
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

    def _get_lane_follow_steer(self, vehicle):
        """沿当前车道waypoints计算转向角"""
        loc = vehicle.get_location()
        wp = self.map.get_waypoint(loc, project_to_road=True)
        if not wp:
            return 0.0

        next_wps = wp.next(6.0)
        if not next_wps:
            return 0.0

        target_loc = next_wps[0].transform.location

        dx = target_loc.x - loc.x
        dy = target_loc.y - loc.y
        desired_yaw = math.atan2(dy, dx)

        trans = vehicle.get_transform()
        current_yaw = math.radians(trans.rotation.yaw)
        yaw_err = desired_yaw - current_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        steer = max(min(1.5 * yaw_err, 0.3), -0.3)
        return steer

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
        else:
            # 非TCP模式（behavior）：沿车道waypoints直行
            ego_steer = self._get_lane_follow_steer(self.ego)
            ego_ctrl = carla.VehicleControl()
            ego_ctrl.throttle = 0.45
            ego_ctrl.steer = ego_steer
            ego_ctrl.brake = 0.0
            self.ego.apply_control(ego_ctrl)

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
                    ctrl.brake = 0.0
                # Follow lane waypoints regardless of path
                ctrl.steer = self._get_lane_follow_steer(w)
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


# ============================
# GB/T 41798—2022 6.1.4 机动车信号灯（1.d）
# ============================
class MotorVehicleTrafficLightScene(EgoRouteFollowScene):
    """Circular motor-vehicle signal test with deterministic timing and scoring."""

    def __init__(self, client, world, config_path, town, route_id, model, model_path=None):
        super().__init__(client, world, config_path, town, route_id, model, model_path)
        if "signal_test" not in self.config:
            raise ValueError("1.d configuration is missing signal_test")
        self.signal_cfg = self.config["signal_test"]
        self.maneuver = self.signal_cfg["maneuver"]
        self.signal_case = self.signal_cfg["signal_case"]
        if self.maneuver not in ("straight", "left", "right"):
            raise ValueError("unsupported 1.d maneuver: {}".format(self.maneuver))
        if self.signal_case not in ("keep_green", "turn_red"):
            raise ValueError("unsupported 1.d signal case: {}".format(self.signal_case))
        self.trigger_distance = float(self.signal_cfg.get("trigger_distance_m", 50.0))
        self.valid_trigger_range = self.signal_cfg.get("valid_trigger_range_m", [40.0, 60.0])
        self.yellow_duration = float(self.signal_cfg.get("yellow_duration_s", 3.0))
        self.red_duration = float(self.signal_cfg.get("red_duration_s", 30.0))
        self.speed_limit_mps = float(self.signal_cfg.get("speed_limit_kmh", 40.0)) / 3.6
        self.stop_gap_max = float(self.signal_cfg.get("passenger_stop_gap_max_m", 2.0))
        self.restart_max = float(self.signal_cfg.get("passenger_restart_max_s", 3.0))
        self.stop_speed_threshold = float(self.signal_cfg.get("stop_speed_threshold_mps", 0.1))
        self.start_speed_threshold = float(self.signal_cfg.get("start_speed_threshold_mps", 0.5))
        self.unjustified_stop_duration = float(
            self.signal_cfg.get("unjustified_stop_duration_s", 1.0))
        self.timeout = float(self.config.get("timeout", 90.0))
        self.speed_limit = min(8.0, self.speed_limit_mps)
        self.route_completion_distance = float(
            self.signal_cfg.get("route_completion_distance_m", 4.0))

        stop_line = self.signal_cfg["stop_line"]
        center = stop_line["center"]
        self.stop_line_center = carla.Location(
            x=float(center["x"]), y=float(center["y"]), z=float(center.get("z", 0.0)))
        yaw = math.radians(float(stop_line["approach_yaw"]))
        self.approach_forward = carla.Vector3D(math.cos(yaw), math.sin(yaw), 0.0)

        self.traffic_light = None
        self._original_light = None
        self.phase = "Green"
        self.phase_started_at = None
        self.phase_history = []
        self.start_sim_time = None
        self.last_sim_time = None
        self.signal_triggered = False
        self.actual_trigger_distance = None
        self.yellow_started_at = None
        self.red_started_at = None
        self.green_after_red_at = None
        self.yellow_duration_actual = None
        self.red_duration_actual = None

        self.last_gap = None
        self.stop_gap = None
        self.stop_observed = False
        self.restart_delay = None
        self.crossed_line = False
        self.crossed_during_red = False
        self.crossed_during_protected_phase = False
        self.cross_phase = None
        self.unjustified_stop_time = 0.0
        self.unjustified_stop = False
        self.has_started_moving = False
        self.max_speed_mps = 0.0
        self.speed_limit_exceeded = False
        self.timed_out = False

    @staticmethod
    def _location_from_dict(data):
        return carla.Location(
            x=float(data["x"]), y=float(data["y"]), z=float(data.get("z", 0.0)))

    @staticmethod
    def _traffic_light_trigger_location(light):
        return light.get_transform().transform(light.trigger_volume.location)

    def _find_traffic_light(self):
        selector = self.signal_cfg.get("traffic_light_selector", {})
        expected = self._location_from_dict(
            selector.get("trigger_location", self.signal_cfg["stop_line"]["center"]))
        best_light = None
        best_distance = float("inf")
        for light in self.world.get_actors().filter("*traffic*light*"):
            try:
                distance = self._traffic_light_trigger_location(light).distance(expected)
            except (RuntimeError, AttributeError):
                continue
            if distance < best_distance:
                best_light = light
                best_distance = distance
        if best_light is None or best_distance > 20.0:
            raise RuntimeError(
                "cannot match configured traffic light (nearest distance {:.2f} m)".format(
                    best_distance))
        return best_light

    def _capture_light_state(self):
        frozen = False
        if hasattr(self.traffic_light, "is_frozen"):
            frozen = bool(self.traffic_light.is_frozen())
        self._original_light = {
            "state": self.traffic_light.get_state(),
            "green_time": self.traffic_light.get_green_time(),
            "yellow_time": self.traffic_light.get_yellow_time(),
            "red_time": self.traffic_light.get_red_time(),
            "frozen": frozen,
        }

    def _set_light_state(self, state, now=None):
        states = {
            "Green": carla.TrafficLightState.Green,
            "Yellow": carla.TrafficLightState.Yellow,
            "Red": carla.TrafficLightState.Red,
        }
        self.traffic_light.set_state(states[state])
        if hasattr(self.traffic_light, "freeze"):
            self.traffic_light.freeze(True)
        self.phase = state
        self.phase_started_at = now
        if now is not None:
            self.phase_history.append({"state": state, "sim_time": round(now, 4)})

    def _restore_light_state(self):
        if not self.traffic_light or not self._original_light:
            return
        try:
            original = self._original_light
            self.traffic_light.set_green_time(original["green_time"])
            self.traffic_light.set_yellow_time(original["yellow_time"])
            self.traffic_light.set_red_time(original["red_time"])
            self.traffic_light.set_state(original["state"])
            if hasattr(self.traffic_light, "freeze"):
                self.traffic_light.freeze(original["frozen"])
        except RuntimeError:
            pass

    def spawn(self):
        """Spawn actors and sensors without advancing the simulation clock."""
        self.ego = self.spawn_ego()
        if not self.ego:
            raise RuntimeError("EGO生成失败")
        self.load_ego_route()
        if len(self.route_points) < 2:
            raise RuntimeError("1.d requires at least two route points")
        self.ego.set_autopilot(False)

        if self.tcp_flag:
            self.planner = TCPRoutePlanner()
            self.tcp = TCPAgent(self.model_path, self.planner)
            waypoints = []
            for location in self.route_points:
                waypoint = self.map.get_waypoint(location, project_to_road=True)
                if waypoint:
                    waypoints.append(waypoint)
            self.planner.set_route(waypoints, maneuver=self.maneuver)
            self.spawn_camera()

        self.traffic_light = self._find_traffic_light()
        self._capture_light_state()
        now = self.world.get_snapshot().timestamp.elapsed_seconds
        self.start_sim_time = now
        self.last_sim_time = now
        self._set_light_state("Green", now)

    def _vehicle_speed(self):
        velocity = self.ego.get_velocity()
        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

    def _signed_stop_gap(self):
        """Distance from the foremost bounding-box vertex to the stop line."""
        transform = self.ego.get_transform()
        try:
            vertices = self.ego.bounding_box.get_world_vertices(transform)
        except AttributeError:
            extent = self.ego.bounding_box.extent
            forward = transform.get_forward_vector()
            right = transform.get_right_vector()
            center = transform.location
            vertices = [
                center + carla.Location(
                    x=forward.x * extent.x + sign * right.x * extent.y,
                    y=forward.y * extent.x + sign * right.y * extent.y)
                for sign in (-1.0, 1.0)
            ]
        progress = []
        for vertex in vertices:
            relative = vertex - self.stop_line_center
            progress.append(
                relative.x * self.approach_forward.x + relative.y * self.approach_forward.y)
        return -max(progress)

    def _advance_signal(self, now, gap):
        if self.signal_case != "turn_red":
            return
        if not self.signal_triggered and gap <= self.trigger_distance:
            self.signal_triggered = True
            self.actual_trigger_distance = gap
            self.yellow_started_at = now
            self._set_light_state("Yellow", now)
            return
        if self.phase == "Yellow" and now - self.yellow_started_at >= self.yellow_duration:
            self.yellow_duration_actual = now - self.yellow_started_at
            self.red_started_at = now
            self._set_light_state("Red", now)
            return
        if self.phase == "Red" and now - self.red_started_at >= self.red_duration:
            self.red_duration_actual = now - self.red_started_at
            self.green_after_red_at = now
            self._set_light_state("Green", now)

    def _apply_reference_control(self, gap):
        if self.finished:
            self.ego.apply_control(carla.VehicleControl(brake=1.0))
            return
        if self.current_target_idx >= len(self.route_points):
            self.finished = True
            self.ego.apply_control(carla.VehicleControl(brake=1.0))
            return

        ego_location = self.ego.get_location()
        target = self.route_points[self.current_target_idx]
        if ego_location.distance(target) < self.stop_distance:
            self.current_target_idx += 1
            if self.current_target_idx >= len(self.route_points):
                self.finished = True
                self.ego.apply_control(carla.VehicleControl(brake=1.0))
                return
            target = self.route_points[self.current_target_idx]

        transform = self.ego.get_transform()
        target_yaw = math.degrees(math.atan2(
            target.y - ego_location.y, target.x - ego_location.x))
        heading_error = (target_yaw - transform.rotation.yaw + 180.0) % 360.0 - 180.0
        speed = self._vehicle_speed()
        desired_speed = self.speed_limit
        must_stop = self.maneuver in ("straight", "left") and self.phase in ("Yellow", "Red")
        if must_stop:
            desired_speed = min(desired_speed, max(0.0, (gap - 1.0) * 0.65))
            if gap <= 1.2:
                desired_speed = 0.0

        control = carla.VehicleControl()
        control.steer = max(-1.0, min(1.0, heading_error * 0.08))
        speed_error = desired_speed - speed
        if speed_error > 0.2:
            control.throttle = min(0.45, 0.15 + speed_error * 0.08)
            control.brake = 0.0
        elif speed_error < -0.2:
            control.throttle = 0.0
            control.brake = min(1.0, 0.25 + (-speed_error) * 0.15)
        else:
            control.throttle = 0.08 if desired_speed > 0.2 else 0.0
            control.brake = 0.0 if desired_speed > 0.2 else 0.4
        self.ego.apply_control(control)

    def _update_route_progress(self):
        """Track route completion independently from the selected controller."""
        if self.finished or not self.route_points:
            return
        ego_location = self.ego.get_location()
        while self.current_target_idx < len(self.route_points):
            target = self.route_points[self.current_target_idx]
            if ego_location.distance(target) > self.route_completion_distance:
                break
            self.current_target_idx += 1
        if self.current_target_idx >= len(self.route_points):
            self.finished = True

    def _apply_tcp_control(self):
        if self.camera_data is None:
            self.ego.apply_control(carla.VehicleControl(brake=1.0))
            return
        action = self.tcp.get_action(self.camera_data, self.ego)
        control = carla.VehicleControl()
        control.throttle = max(0.0, min(1.0, float(action[0][0])))
        control.steer = max(-1.0, min(1.0, float(action[0][1])))
        control.brake = max(0.0, min(1.0, float(action[0][2]))) if action.shape[1] > 2 else 0.0
        if control.brake > 0.05:
            control.throttle = 0.0
        self.ego.apply_control(control)

    def _update_measurements(self, now, dt, gap, speed):
        self.max_speed_mps = max(self.max_speed_mps, speed)
        if speed > self.speed_limit_mps + 0.2:
            self.speed_limit_exceeded = True
        if speed >= self.start_speed_threshold:
            self.has_started_moving = True

        if self.last_gap is not None and self.last_gap > 0.0 and gap <= 0.0:
            self.crossed_line = True
            self.cross_phase = self.phase
            if self.phase == "Red":
                self.crossed_during_red = True
            if self.signal_triggered and self.green_after_red_at is None:
                self.crossed_during_protected_phase = True
        elif gap < 0.0 and self.signal_triggered and self.green_after_red_at is None:
            self.crossed_during_protected_phase = True

        protected_stop = (
            self.signal_case == "turn_red"
            and self.maneuver in ("straight", "left")
            and self.signal_triggered
            and self.green_after_red_at is None
        )
        if protected_stop and speed <= self.stop_speed_threshold:
            self.stop_observed = True
            if gap >= 0.0 and (self.stop_gap is None or gap < self.stop_gap):
                self.stop_gap = gap

        if self.green_after_red_at is not None and self.restart_delay is None:
            if speed >= self.start_speed_threshold:
                self.restart_delay = now - self.green_after_red_at

        should_not_stop = self.signal_case == "keep_green" or self.maneuver == "right"
        in_monitoring_zone = -10.0 <= gap <= 60.0 and self.has_started_moving and not self.finished
        if should_not_stop and in_monitoring_zone and speed <= self.stop_speed_threshold:
            self.unjustified_stop_time += dt
            if self.unjustified_stop_time > self.unjustified_stop_duration:
                self.unjustified_stop = True
        elif speed > self.stop_speed_threshold:
            self.unjustified_stop_time = 0.0

        self.last_gap = gap

    def tick(self):
        if not self.ego or not self.ego.is_alive:
            return False
        now = self.world.get_snapshot().timestamp.elapsed_seconds
        dt = max(0.0, now - self.last_sim_time)
        self.last_sim_time = now
        gap = self._signed_stop_gap()
        speed = self._vehicle_speed()

        self._advance_signal(now, gap)
        self._update_measurements(now, dt, gap, speed)
        self._update_route_progress()
        if self.tcp_flag:
            self._apply_tcp_control()
        else:
            self._apply_reference_control(gap)

        if now - self.start_sim_time >= self.timeout:
            self.timed_out = True
            return False
        if self.signal_case == "keep_green":
            return not self.finished
        if self.maneuver == "right":
            return not (self.finished and self.green_after_red_at is not None)
        return not (self.finished and self.green_after_red_at is not None)

    def get_result(self, collision_occurred=False):
        failures = []
        if collision_occurred:
            failures.append("collision")
        if self.timed_out:
            failures.append("timeout")
        if not self.finished:
            failures.append("route_incomplete")
        if self.speed_limit_exceeded:
            failures.append("speed_limit_exceeded")
        if self.unjustified_stop:
            failures.append("unjustified_stop")

        if self.signal_case == "turn_red":
            if self.actual_trigger_distance is None:
                failures.append("signal_not_triggered")
            elif not (float(self.valid_trigger_range[0]) <= self.actual_trigger_distance
                      <= float(self.valid_trigger_range[1])):
                failures.append("trigger_distance_out_of_range")
            if self.red_started_at is None:
                failures.append("red_phase_not_reached")
            if self.green_after_red_at is None:
                failures.append("green_recovery_not_reached")
            if self.maneuver == "right":
                if not self.crossed_during_red:
                    failures.append("right_turn_not_completed_during_red")
            else:
                if self.crossed_during_protected_phase:
                    failures.append("crossed_stop_line_before_green")
                if not self.stop_observed:
                    failures.append("stop_not_observed")
                if self.stop_gap is None or not (0.0 <= self.stop_gap <= self.stop_gap_max):
                    failures.append("stop_gap_out_of_range")
                if self.green_after_red_at is not None and (
                        self.restart_delay is None or self.restart_delay > self.restart_max):
                    failures.append("restart_delay_exceeded")

        reproducibility = self.signal_cfg.get("reproducibility", {})
        return {
            "matrix_id": self.signal_cfg.get("matrix_id"),
            "maneuver": self.maneuver,
            "trial_index": reproducibility.get("trial_index"),
            "signal_case": self.signal_case,
            "base_seed": reproducibility.get("base_seed"),
            "matrix_seed": reproducibility.get("matrix_seed"),
            "trial_seed": reproducibility.get("trial_seed"),
            "route_fingerprint": reproducibility.get("route_fingerprint"),
            "trigger_distance_m": round(self.actual_trigger_distance, 3)
            if self.actual_trigger_distance is not None else None,
            "yellow_duration_s": round(self.yellow_duration_actual, 3)
            if self.yellow_duration_actual is not None else None,
            "red_duration_s": round(self.red_duration_actual, 3)
            if self.red_duration_actual is not None else None,
            "stop_gap_m": round(self.stop_gap, 3) if self.stop_gap is not None else None,
            "restart_delay_s": round(self.restart_delay, 3)
            if self.restart_delay is not None else None,
            "crossed_stop_line_on_red": self.crossed_during_red,
            "crossed_before_green": self.crossed_during_protected_phase,
            "cross_phase": self.cross_phase,
            "unjustified_stop": self.unjustified_stop,
            "max_speed_kmh": round(self.max_speed_mps * 3.6, 3),
            "speed_limit_exceeded": self.speed_limit_exceeded,
            "collision": bool(collision_occurred),
            "route_complete": bool(self.finished),
            "timed_out": self.timed_out,
            "phase_history": self.phase_history,
            "pass": not failures,
            "failure_reasons": failures,
        }

    def destroy(self):
        self._restore_light_state()
        super().destroy()
