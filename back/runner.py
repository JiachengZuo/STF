#!/usr/bin/env python3
import carla
import json
import time
import pygame
import math
import numpy as np
import weakref

# ====================== 全局配置 ======================
JSON_PATH = "./trigger_person_output.json"
HOST = '127.0.0.1'
PORT = 2000
ACTOR_SPEED = 2.5
EGO_SPEED = 12.0
TOWN = "TOWN10HD_Opt"
ROUTE_ID = "route_01"
SCREEN_WIDTH = 512  # 总宽度：左BEV(800) + 右前置(800)
SCREEN_HEIGHT = 256  # 总高度
# ======================================================

# ============================
# 完整 Birdeye 渲染引擎（你的原版，无修改）
# ============================
COLOR_BUTTER_0 = pygame.Color(252, 233, 79)
COLOR_BUTTER_1 = pygame.Color(237, 212, 0)
COLOR_BUTTER_2 = pygame.Color(196, 160, 0)
COLOR_ORANGE_0 = pygame.Color(252, 175, 62)
COLOR_ORANGE_1 = pygame.Color(245, 121, 0)
COLOR_ORANGE_2 = pygame.Color(209, 92, 0)
COLOR_CHOCOLATE_0 = pygame.Color(233, 185, 110)
COLOR_CHOCOLATE_1 = pygame.Color(193, 125, 17)
COLOR_CHOCOLATE_2 = pygame.Color(143, 89, 2)
COLOR_CHAMELEON_0 = pygame.Color(138, 226, 52)
COLOR_CHAMELEON_1 = pygame.Color(115, 210, 22)
COLOR_CHAMELEON_2 = pygame.Color(78, 154, 6)
COLOR_SKY_BLUE_0 = pygame.Color(114, 159, 207)
COLOR_SKY_BLUE_1 = pygame.Color(52, 101, 164)
COLOR_SKY_BLUE_2 = pygame.Color(32, 74, 135)
COLOR_PLUM_0 = pygame.Color(173, 127, 168)
COLOR_PLUM_1 = pygame.Color(117, 80, 123)
COLOR_PLUM_2 = pygame.Color(92, 53, 102)
COLOR_SCARLET_RED_0 = pygame.Color(239, 41, 41)
COLOR_SCARLET_RED_1 = pygame.Color(204, 0, 0)
COLOR_SCARLET_RED_2 = pygame.Color(164, 0, 0)
COLOR_ALUMINIUM_0 = pygame.Color(238, 238, 236)
COLOR_ALUMINIUM_1 = pygame.Color(211, 215, 207)
COLOR_ALUMINIUM_2 = pygame.Color(186, 189, 182)
COLOR_ALUMINIUM_3 = pygame.Color(136, 138, 133)
COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83)
COLOR_ALUMINIUM_4_5 = pygame.Color(66, 62, 64)
COLOR_ALUMINIUM_5 = pygame.Color(46, 52, 54)
COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_BLACK = pygame.Color(0, 0, 0)

class Util(object):
    @staticmethod
    def blits(destination_surface, source_surfaces, rect=None, blend_mode=0):
        for surface in source_surfaces:
            destination_surface.blit(surface[0], surface[1], rect, blend_mode)
    @staticmethod
    def length(v):
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)
    @staticmethod
    def get_bounding_box(actor):
        bb = actor.trigger_volume.extent
        corners = [carla.Location(x=-bb.x, y=-bb.y), carla.Location(x=bb.x, y=-bb.y), carla.Location(x=bb.x, y=bb.y), carla.Location(x=-bb.x, y=bb.y), carla.Location(x=-bb.x, y=-bb.y)]
        corners = [x + actor.trigger_volume.location for x in corners]
        t = actor.get_transform()
        t.transform(corners)
        return corners

class MapImage(object):
    def __init__(self, carla_world, carla_map, pixels_per_meter, logger):
        logger.log('>> Drawing the map of the entire town. This may take a while...')
        self._pixels_per_meter = pixels_per_meter
        self.scale = 1.0
        waypoints = carla_map.generate_waypoints(2.0)
        margin = 50
        max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
        max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
        min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
        min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin
        self.width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)
        width_in_pixels = int(self._pixels_per_meter * self.width)
        self.big_map_surface = pygame.Surface((width_in_pixels, width_in_pixels)).convert()
        self.draw_road_map(self.big_map_surface, carla_world, carla_map, self.world_to_pixel, self.world_to_pixel_width)
        self.surface = self.big_map_surface

    def draw_road_map(self, map_surface, carla_world, carla_map, world_to_pixel, world_to_pixel_width):
        map_surface.fill(COLOR_BLACK)
        precision = 0.05
        def lane_marking_color_to_tango(lane_marking_color):
            tango_color = COLOR_BLACK
            if lane_marking_color == carla.LaneMarkingColor.White: tango_color = COLOR_ALUMINIUM_2
            elif lane_marking_color == carla.LaneMarkingColor.Blue: tango_color = COLOR_SKY_BLUE_0
            elif lane_marking_color == carla.LaneMarkingColor.Green: tango_color = COLOR_CHAMELEON_0
            elif lane_marking_color == carla.LaneMarkingColor.Red: tango_color = COLOR_SCARLET_RED_0
            elif lane_marking_color == carla.LaneMarkingColor.Yellow: tango_color = COLOR_ORANGE_0
            return tango_color
        def draw_solid_line(surface, color, closed, points, width):
            if len(points) >= 2: pygame.draw.lines(surface, color, closed, points, width)
        def draw_broken_line(surface, color, closed, points, width):
            broken_lines = [x for n, x in enumerate(zip(*(iter(points),) * 20)) if n % 3 == 0]
            for line in broken_lines: pygame.draw.lines(surface, color, closed, line, width)
        def get_lane_markings(lane_marking_type, lane_marking_color, waypoints, sign):
            margin = 0.25
            marking_1 = [world_to_pixel(lateral_shift(w.transform, sign * w.lane_width * 0.5)) for w in waypoints]
            if lane_marking_type == carla.LaneMarkingType.Broken or (lane_marking_type == carla.LaneMarkingType.Solid):
                return [(lane_marking_type, lane_marking_color, marking_1)]
            else:
                marking_2 = [world_to_pixel(lateral_shift(w.transform, sign * (w.lane_width * 0.5 + margin * 2))) for w in waypoints]
                if lane_marking_type == carla.LaneMarkingType.SolidBroken:
                    return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1), (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]
                elif lane_marking_type == carla.LaneMarkingType.BrokenSolid:
                    return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1), (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
                elif lane_marking_type == carla.LaneMarkingType.BrokenBroken:
                    return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1), (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
                elif lane_marking_type == carla.LaneMarkingType.SolidSolid:
                    return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1), (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]
            return [(carla.LaneMarkingType.NONE, carla.LaneMarkingColor.Other, [])]
        def draw_lane(surface, lane, color):
            for side in lane:
                lane_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in side]
                lane_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in side]
                polygon = lane_left_side + [x for x in reversed(lane_right_side)]
                polygon = [world_to_pixel(x) for x in polygon]
                if len(polygon) > 2:
                    pygame.draw.polygon(surface, color, polygon, 5)
                    pygame.draw.polygon(surface, color, polygon)
        def draw_lane_marking(surface, waypoints):
            draw_lane_marking_single_side(surface, waypoints[0], -1)
            draw_lane_marking_single_side(surface, waypoints[1], 1)
        def draw_lane_marking_single_side(surface, waypoints, sign):
            previous_marking_type = carla.LaneMarkingType.NONE
            previous_marking_color = carla.LaneMarkingColor.Other
            markings_list = []
            temp_waypoints = []
            current_lane_marking = carla.LaneMarkingType.NONE
            for sample in waypoints:
                lane_marking = sample.left_lane_marking if sign < 0 else sample.right_lane_marking
                if lane_marking is None: continue
                marking_type = lane_marking.type
                marking_color = lane_marking.color
                if current_lane_marking != marking_type:
                    markings = get_lane_markings(previous_marking_type, lane_marking_color_to_tango(previous_marking_color), temp_waypoints, sign)
                    current_lane_marking = marking_type
                    for marking in markings: markings_list.append(marking)
                    temp_waypoints = temp_waypoints[-1:]
                else:
                    temp_waypoints.append((sample))
                    previous_marking_type = marking_type
                    previous_marking_color = marking_color
            last_markings = get_lane_markings(previous_marking_type, lane_marking_color_to_tango(previous_marking_color), temp_waypoints, sign)
            for marking in last_markings: markings_list.append(marking)
            for markings in markings_list:
                if markings[0] == carla.LaneMarkingType.Solid: draw_solid_line(surface, markings[1], False, markings[2], 2)
                elif markings[0] == carla.LaneMarkingType.Broken: draw_broken_line(surface, markings[1], False, markings[2], 2)
        def lateral_shift(transform, shift):
            transform.rotation.yaw += 90
            return transform.location + shift * transform.get_forward_vector()
        def draw_topology(carla_topology, index):
            topology = [x[index] for x in carla_topology]
            topology = sorted(topology, key=lambda w: w.transform.location.z)
            set_waypoints = []
            for waypoint in topology:
                waypoints = [waypoint]
                nxt = waypoint.next(precision)
                if len(nxt) > 0:
                    nxt = nxt[0]
                    while nxt.road_id == waypoint.road_id:
                        waypoints.append(nxt)
                        nxt = nxt.next(precision)
                        if len(nxt) > 0: nxt = nxt[0]
                        else: break
                set_waypoints.append(waypoints)
                PARKING_COLOR = COLOR_ALUMINIUM_4_5
                SHOULDER_COLOR = COLOR_ALUMINIUM_5
                SIDEWALK_COLOR = COLOR_ALUMINIUM_3
                shoulder = [[], []]
                parking = [[], []]
                sidewalk = [[], []]
                for w in waypoints:
                    l = w.get_right_lane()
                    while l is not None and l.lane_type != carla.LaneType.Driving:
                        if l.lane_type == carla.LaneType.Shoulder: shoulder[0].append(l)
                        elif l.lane_type == carla.LaneType.Parking: parking[0].append(l)
                        elif l.lane_type == carla.LaneType.Sidewalk: sidewalk[0].append(l)
                        try: l = l.get_right_lane()
                        except: l = None
                    r = w.get_left_lane()
                    while r is not None and r.lane_type != carla.LaneType.Driving:
                        if r.lane_type == carla.LaneType.Shoulder: shoulder[1].append(r)
                        elif r.lane_type == carla.LaneType.Parking: parking[1].append(r)
                        elif r.lane_type == carla.LaneType.Sidewalk: sidewalk[1].append(r)
                        try: r = r.get_left_lane()
                        except: r = None
                draw_lane(map_surface, shoulder, SHOULDER_COLOR)
                draw_lane(map_surface, parking, PARKING_COLOR)
                draw_lane(map_surface, sidewalk, SIDEWALK_COLOR)
            for waypoints in set_waypoints:
                waypoint = waypoints[0]
                road_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
                road_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]
                polygon = road_left_side + [x for x in reversed(road_right_side)]
                polygon = [world_to_pixel(x) for x in polygon]
                if len(polygon) > 2:
                    pygame.draw.polygon(map_surface, COLOR_ALUMINIUM_5, polygon, 5)
                    pygame.draw.polygon(map_surface, COLOR_ALUMINIUM_5, polygon)
                if not waypoint.is_junction: draw_lane_marking(map_surface, [waypoints, waypoints])
        topology = carla_map.get_topology()
        draw_topology(topology, 0)

    def world_to_pixel(self, location, offset=(0, 0)):
        x = self.scale * self._pixels_per_meter * (location.x - self._world_offset[0])
        y = self.scale * self._pixels_per_meter * (location.y - self._world_offset[1])
        return [int(x - offset[0]), int(y - offset[1])]
    def world_to_pixel_width(self, width):
        return int(self.scale * self._pixels_per_meter * width)

class BirdeyeRender(object):
    def __init__(self, world, params, logger):
        self.params = params
        self.server_fps = 0.0
        self.simulation_time = 0
        self.server_clock = pygame.time.Clock()
        self.world = world
        self.town_map = self.world.get_map()
        self.actors_with_transforms = []
        self.hero_actor = None
        self.hero_id = None
        self.hero_transform = None
        self.heros_in_all_envs = []
        self.vehicle_polygons = []
        self.walker_polygons = []
        self.waypoints = None
        self.red_light = False
        self.map_image = MapImage(world, self.town_map, self.params['pixels_per_meter'], logger)
        self.original_surface_size = min(self.params['screen_size'][0], self.params['screen_size'][1])
        self.surface_size = self.map_image.big_map_surface.get_width()
        self.actors_surface = pygame.Surface((self.map_image.surface.get_width(), self.map_image.surface.get_height()))
        self.actors_surface.set_colorkey(COLOR_BLACK)
        self.waypoints_surface = pygame.Surface((self.map_image.surface.get_width(), self.map_image.surface.get_height()))
        self.waypoints_surface.set_colorkey(COLOR_BLACK)
        scaled_original_size = self.original_surface_size * (1.0 / 0.62)
        self.hero_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.result_surface = pygame.Surface((self.surface_size, self.surface_size)).convert()
        self.result_surface.set_colorkey(COLOR_BLACK)
        weak_self = weakref.ref(self)
        self.world.on_tick(lambda timestamp: BirdeyeRender.on_world_tick(weak_self, timestamp))

    def set_hero(self, hero_actor, hero_id):
        self.hero_actor = hero_actor
        self.hero_id = hero_id
        self.heros_in_all_envs.append(hero_id)

    def tick(self, clock):
        actors = self.world.get_actors()
        self.actors_with_transforms = [(actor, actor.get_transform()) for actor in actors]
        if self.hero_actor is not None: self.hero_transform = self.hero_actor.get_transform()

    @staticmethod
    def on_world_tick(weak_self, timestamp):
        self = weak_self()
        if not self: return
        self.server_clock.tick()
        self.server_fps = self.server_clock.get_fps()
        self.simulation_time = timestamp.elapsed_seconds

    def _split_actors(self):
        vehicles = []
        walkers = []
        for a in self.actors_with_transforms:
            actor = a[0]
            if 'vehicle' in actor.type_id: vehicles.append(a)
            elif 'walker.pedestrian' in actor.type_id: walkers.append(a)
        return vehicles, walkers

    def _render_hist_actors(self, surface, actor_polygons, actor_type, world_to_pixel, num):
        lp = len(actor_polygons)
        color = COLOR_SKY_BLUE_0
        for i in range(max(0, lp-num), lp):
            for ID, poly in actor_polygons[i].items():
                corners = []
                for p in poly: corners.append(carla.Location(x=p[0], y=p[1]))
                corners.append(carla.Location(x=poly[0][0], y=poly[0][1]))
                corners = [world_to_pixel(p) for p in corners]
                color_value = max(0.8 - 0.8/lp*(i+1), 0)
                if ID == self.hero_id or ID in self.heros_in_all_envs:
                    color = pygame.Color(255, math.floor(color_value*255), math.floor(color_value*255))
                else:
                    if actor_type == 'vehicle': color = pygame.Color(math.floor(color_value*255), 255, math.floor(color_value*255))
                    elif actor_type == 'walker': color = pygame.Color(255, 255, math.floor(color_value*255))
                pygame.draw.polygon(surface, color, corners)

    def render_waypoints(self, surface, waypoints, world_to_pixel):
        if not waypoints or len(waypoints) < 2: return
        if self.red_light: color = pygame.Color(128, 0, 128)
        else: color = pygame.Color(0,0,255)
        corners = []
        for p in waypoints: corners.append(carla.Location(x=p[0], y=p[1]))
        corners = [world_to_pixel(p) for p in corners]
        pygame.draw.lines(surface, color, False, corners, 10)

    def render_actors(self, surface, vehicles, walkers):
        self._render_hist_actors(surface, vehicles, 'vehicle', self.map_image.world_to_pixel, 10)
        self._render_hist_actors(surface, walkers, 'walker', self.map_image.world_to_pixel, 10)

    def clip_surfaces(self, clipping_rect):
        self.actors_surface.set_clip(clipping_rect)
        self.result_surface.set_clip(clipping_rect)

    def render(self, render_types=None):
        self.tick(self.server_clock)
        if self.actors_with_transforms is None: return
        self.result_surface.fill(COLOR_BLACK)
        self.actors_surface.fill(COLOR_BLACK)
        self.render_actors(self.actors_surface, self.vehicle_polygons, self.walker_polygons)
        self.waypoints_surface.fill(COLOR_BLACK)
        self.render_waypoints(self.waypoints_surface, self.waypoints, self.map_image.world_to_pixel)
        if render_types is None:
            surfaces = [(self.map_image.surface, (0,0)), (self.actors_surface, (0,0)), (self.waypoints_surface, (0,0))]
        else:
            surfaces = []
            if 'roadmap' in render_types: surfaces.append((self.map_image.surface, (0,0)))
            if 'waypoints' in render_types: surfaces.append((self.waypoints_surface, (0,0)))
            if 'actors' in render_types: surfaces.append((self.actors_surface, (0,0)))
        angle = self.hero_transform.rotation.yaw + 90.0 if self.hero_actor else 0.0
        if self.hero_actor is not None:
            hero_loc_screen = self.map_image.world_to_pixel(self.hero_transform.location)
            hero_front = self.hero_transform.get_forward_vector()
            translation_offset = (
                hero_loc_screen[0] - self.hero_surface.get_width()/2 + hero_front.x * self.params['pixels_ahead_vehicle'],
                hero_loc_screen[1] - self.hero_surface.get_height()/2 + hero_front.y * self.params['pixels_ahead_vehicle']
            )
            clip_rect = pygame.Rect(translation_offset[0], translation_offset[1], self.hero_surface.get_width(), self.hero_surface.get_height())
            self.clip_surfaces(clip_rect)
            Util.blits(self.result_surface, surfaces)
            self.hero_surface.fill(COLOR_BLACK)
            self.hero_surface.blit(self.result_surface, (-translation_offset[0], -translation_offset[1]))
            return pygame.transform.rotozoom(self.hero_surface, angle, 1.0).convert()
        else:
            raise ValueError('hero_actor is None')

# ============================
# Logger
# ============================
class SimpleLogger:
    def log(self, msg):
        print(f"[LOG] {msg}")

# ============================
# 工具函数：获取车辆/行人多边形
# ============================
def get_actor_polygons(world, filt):
    actor_poly_dict = {}
    for actor in world.get_actors().filter(filt):
        trans = actor.get_transform()
        x = trans.location.x
        y = trans.location.y
        yaw = trans.rotation.yaw / 180 * np.pi
        bb = actor.bounding_box
        l = bb.extent.x
        w = bb.extent.y
        poly_local = np.array([[l, w], [l, -w], [-l, -w], [-l, w]]).transpose()
        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        poly = np.matmul(R, poly_local).transpose() + np.repeat([[x, y]], 4, axis=0)
        actor_poly_dict[actor.id] = poly
    return actor_poly_dict

# ============================
# 加载JSON
# ============================
def load_trigger_config(json_path, town, route):
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return data[town][route][0]

# ============================
# 主函数（分屏版：左BEV + 右前置摄像头）
# ============================
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("左BEV鸟瞰图 + 右前置摄像头（1:1还原参考图）")
    clock = pygame.time.Clock()
    logger = SimpleLogger()

    # 连接CARLA
    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)
    world = client.get_world()
    world.apply_settings(carla.WorldSettings(synchronous_mode=True, fixed_delta_seconds=0.05))
    bp_lib = world.get_blueprint_library()
    scenario = load_trigger_config(JSON_PATH, TOWN, ROUTE_ID)

    # 触发点配置
    trig = scenario['trigger_position']
    trig_loc = carla.Location(float(trig['x']), float(trig['y']), float(trig['z']))
    trig_yaw = float(trig['yaw'])

    # 生成自车（在触发点后方35米）
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
            print("✅ 车辆已生成在JSON触发点后方！")
            break
        time.sleep(0.1)
    if not ego:
        print("❌ 车辆生成失败")
        return

    # ======================
    # 官方标准自动驾驶（能跑！）
    # ======================
    ego.set_autopilot(True)  # 就这一行！

    # ======================
    # 1. 初始化BEV渲染器
    # ======================
    birdeye_params = {
        'screen_size': [SCREEN_WIDTH//2, SCREEN_HEIGHT],
        'pixels_per_meter': 4.5,       # 变小 = 视野更大
        'pixels_ahead_vehicle': 0,     # 关键：设为0，车辆在正中心
    }
    birdeye_render = BirdeyeRender(world, birdeye_params, logger=logger)

    # ======================
    # 2. 初始化前置摄像头（右半屏）
    # ======================
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(SCREEN_WIDTH//2))
    cam_bp.set_attribute('image_size_y', str(SCREEN_HEIGHT))
    cam_bp.set_attribute('fov', '90')
    # 摄像头位置：车内第一人称视角（和参考图完全一致）
    camera = world.spawn_actor(
        cam_bp,
        carla.Transform(carla.Location(x=0.5, z=1.6), carla.Rotation(pitch=-5)),
        attach_to=ego
    )

    # 摄像头图像回调
    front_cam_data = None
    def front_cam_callback(image):
        nonlocal front_cam_data
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        front_cam_data = arr[:, :, :3][:, :, ::-1]  # BGRA → RGB

    camera.listen(front_cam_callback)

    # ======================
    # 3. 生成行人（触发横穿）
    # ======================
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
            # 行人垂直横穿马路（和你原逻辑一致）
            cross_yaw = yaw
            dx = math.cos(math.radians(cross_yaw))
            dy = math.sin(math.radians(cross_yaw))
            walker_targets.append((dx, dy, ACTOR_SPEED))

    
    # ======================
    # 4. 车辆未来行驶路线（车头正前方直线，和车行为完全一致）
    # ======================
    waypoints = []
    x0 = spawn_loc.x
    y0 = spawn_loc.y
    yaw_rad = math.radians(trig_yaw)

    # 生成前方 80 米的直线路径（车未来就是走这条）
    for i in range(0, 80, 2):
        x = x0 + i * math.cos(yaw_rad)
        y = y0 + i * math.sin(yaw_rad)
        waypoints.append((x, y))

    # ======================
    # 主循环（分屏渲染）
    # ======================
    triggered = False
    trigger_time = 0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # 触发行人横穿逻辑
        if not triggered and ego.get_location().distance(trig_loc) < 10:
            triggered = True
            trigger_time = time.time()
            print("🔥 触发！行人开始横穿马路")

        # 10秒后自动退出
        if triggered and time.time() - trigger_time > 5:
            print("⏹️ 5秒到，自动结束")
            break

        # 行人移动控制
        if triggered:
            for i, w in enumerate(walkers):
                dx, dy, speed = walker_targets[i]
                control = carla.WalkerControl()
                control.direction = carla.Vector3D(dx, dy, 0)
                control.speed = speed
                w.apply_control(control)

        # ======================
        # 分屏渲染核心
        # ======================
        screen.fill(COLOR_BLACK)

        # 左半屏：BEV鸟瞰图
        vehicle_polys = [get_actor_polygons(world, 'vehicle.*')]
        walker_polys = [get_actor_polygons(world, 'walker.*')]

        birdeye_render.set_hero(ego, ego.id)
        birdeye_render.vehicle_polygons = vehicle_polys
        birdeye_render.walker_polygons = walker_polys
        birdeye_render.waypoints = waypoints

        bev_surface = birdeye_render.render(['roadmap', 'actors', 'waypoints'])

        # 固定居中裁剪，车辆一定显示在画面正中心
        target_w = SCREEN_WIDTH // 2
        target_h = SCREEN_HEIGHT
        bev_cropped = pygame.Surface((target_w, target_h))

        # 核心修复：从BEV图正中心裁剪
        cx = bev_surface.get_width() // 2
        cy = bev_surface.get_height() // 2
        crop_rect = (cx - target_w//2, cy - target_h//2, target_w, target_h)

        bev_cropped.blit(bev_surface, (0, 0), crop_rect)
        screen.blit(bev_cropped, (0, 0))

        # 右半屏：前置摄像头画面
        if front_cam_data is not None:
            cam_surface = pygame.surfarray.make_surface(front_cam_data.swapaxes(0, 1))
            screen.blit(cam_surface, (SCREEN_WIDTH//2, 0))

        pygame.display.flip()
        world.tick()
        clock.tick(30)

    # 清理资源
    camera.stop()
    camera.destroy()
    ego.destroy()
    for w in walkers:
        w.destroy()
    pygame.quit()

if __name__ == '__main__':
    main()