#!/usr/bin/env python3
import carla
import json
import math
import pygame
import argparse
import os

# ====================== 核心配置 ======================
SCREEN_SIZE = (1200, 800)
ZOOM = 15
OFFSET_X = 600
OFFSET_Y = 400
ZOOM_SPEED = 1.2
ROUTE_ID = "route_01"
# ======================================================

class CarlaMapEditor0916:
    def __init__(self, host='127.0.0.1', port=2000, town_name="TOWN10HD_Opt", scenario=1, save_dir="output"):
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()

        # ======================
        # ✅ 100% 不报错红绿灯读取
        # ======================
        self.traffic_light_positions = []

        # 方式1：从 actor 读取（安全）
        for actor in self.world.get_actors().filter('*traffic*light*'):
            loc = actor.get_transform().location
            self.traffic_light_positions.append( (loc.x, loc.y) )

        # 方式2：从环境对象读取（安全写法，无 get_transform()）
        try:
            env_objs = self.world.get_environment_objects(carla.CityObjectLabel.TrafficLight)
            for obj in env_objs:
                loc = obj.transform.location  # ✅ 正确，没有 get_
                self.traffic_light_positions.append( (loc.x, loc.y) )
        except:
            pass

        print(f"✅ 成功读取红绿灯：{len(self.traffic_light_positions)} 个")

        # ======================
        # 原有代码
        # ======================
        self.town_name = town_name
        self.scenario = scenario
        self.save_dir = save_dir
        self.route_index = 0
        os.makedirs(save_dir, exist_ok=True)

        print("正在读取地图 waypoints... (CARLA 0.9.16 兼容)")
        self.waypoints = self.map.generate_waypoints(2.0)
        print(f"读取完成：共 {len(self.waypoints)} 个路点")

        self.trigger_point = None
        self.ego_point = None
        self.actor_points = []
        self.selected_agent_idx = -1
        self.selected_mode = None

        # ======================
        # ✅ 新增：自车路线点（ego route waypoints）
        # ======================
        self.ego_route_points = []

        self.dragging = False
        self.last_mouse_pos = (0, 0)
        self.zoom = ZOOM
        self.offset_x = OFFSET_X
        self.offset_y = OFFSET_Y

        pygame.init()
        self.screen = pygame.display.set_mode(SCREEN_SIZE)
        pygame.display.set_caption("CARLA 批量场景生成器 - 红绿灯+ego路线已支持")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 24)

    def world_to_screen(self, x, y):
        sx = int(self.offset_x + x * self.zoom)
        sy = int(self.offset_y - y * self.zoom)
        return sx, sy

    def screen_to_world(self, sx, sy):
        x = (sx - self.offset_x) / self.zoom
        y = (self.offset_y - sy) / self.zoom
        return x, y

    def get_nearest_waypoint(self, wx, wy):
        min_dist = 999999
        best_wp = None
        for wp in self.waypoints:
            dx = wp.transform.location.x - wx
            dy = wp.transform.location.y - wy
            dist = dx*dx + dy*dy
            if dist < min_dist:
                min_dist = dist
                best_wp = wp
        return best_wp

    def get_ground_z(self, x, y):
        location = carla.Location(x=x, y=y, z=100)
        waypoint = self.map.get_waypoint(location, project_to_road=False, lane_type=carla.LaneType.Any)
        if waypoint:
            return waypoint.transform.location.z + 0.2
        return 0.5

    def draw_waypoints(self):
        for wp in self.waypoints:
            x, y = self.world_to_screen(wp.transform.location.x, wp.transform.location.y)
            pygame.draw.circle(self.screen, (220, 220, 220), (x, y), 3)
            yaw_rad = math.radians(wp.transform.rotation.yaw)
            dx = math.cos(yaw_rad) * 10
            dy = -math.sin(yaw_rad) * 10
            pygame.draw.line(self.screen, (0, 255, 0), (x, y), (x + dx, y + dy), 3)

    def draw_traffic_lights(self):
        for (x, y) in self.traffic_light_positions:
            sx, sy = self.world_to_screen(x, y)
            pygame.draw.circle(self.screen, (255, 0, 0), (sx, sy), 10)
            self.screen.blit(self.font.render("TRAFFIC LIGHT", True, (255,0,0)), (sx+12, sy))

    def draw_points(self):
        self.draw_traffic_lights()

        # ======================
        # ✅ 新增：绘制 ego 行驶路线（连线+点）
        # ======================
        if len(self.ego_route_points) > 0:
            point_list = []
            for idx, p in enumerate(self.ego_route_points):
                sx, sy = self.world_to_screen(p['x'], p['y'])
                point_list.append((sx, sy))
                pygame.draw.circle(self.screen, (0, 255, 128), (sx, sy), 6)
                self.screen.blit(self.font.render(f"P{idx+1}", True, (0,255,128)), (sx+8, sy-18))
            if len(point_list) > 1:
                pygame.draw.lines(self.screen, (0, 255, 128), False, point_list, 3)

        if self.trigger_point:
            x, y = self.world_to_screen(self.trigger_point['x'], self.trigger_point['y'])
            color = (255, 0, 0) if self.selected_mode == 'trigger' else (255, 100, 100)
            pygame.draw.circle(self.screen, color, (x, y), 10)
            self.screen.blit(self.font.render("TRIGGER", True, color), (x+10, y))

        if self.ego_point:
            x, y = self.world_to_screen(self.ego_point['x'], self.ego_point['y'])
            color = (0, 255, 255) if self.selected_mode == 'ego' else (0, 200, 255)
            pygame.draw.circle(self.screen, color, (x, y), 11)
            self.screen.blit(self.font.render("EGO", True, color), (x+10, y))

        for i, p in enumerate(self.actor_points):
            x, y = self.world_to_screen(p['x'], p['y'])
            is_selected = (i == self.selected_agent_idx)
            atype = p.get("type", "person")

            if atype == "obstacle":
                color = (255, 165, 0) if is_selected else (255, 140, 0)
                label = "OBSTACLE"
            else:
                color = (255,255,0) if is_selected else (0,0,255)
                label = {"person":"PERSON","bike":"BIKE","car":"CAR"}[atype]

            pygame.draw.circle(self.screen, color, (x, y), 9 if atype=="car" else 7)
            self.screen.blit(self.font.render(label, True, color), (x+10, y))

            yaw = math.radians(p['yaw'])
            dx = math.cos(yaw) * 18
            dy = -math.sin(yaw) * 18
            pygame.draw.line(self.screen, color, (x,y), (x+dx, y+dy), 2)

    # ======================
    # 切换 AGENT 类型
    # ======================
    def set_agent_type(self, atype):
        if self.selected_agent_idx <0 or self.selected_agent_idx >= len(self.actor_points):
            print("❌ 请先选中一个 AGENT")
            return
        self.actor_points[self.selected_agent_idx]["type"] = atype
        print(f"✅ AGENT 已切换为：{atype.upper()}")

    # ======================
    # ✅ 新增：添加 ego 路线点
    # ======================
    def add_ego_route_point(self, wx, wy):
        wp = self.get_nearest_waypoint(wx, wy)
        if not wp:
            return
        self.ego_route_points.append({
            'x': round(wp.transform.location.x, 2),
            'y': round(wp.transform.location.y, 2),
            'z': round(wp.transform.location.z + 0.2, 2),
            'yaw': round(wp.transform.rotation.yaw, 2)
        })
        print(f"✅ 已添加 ego 路线点 {len(self.ego_route_points)}")

    # ======================
    # ✅ 新增：清空 ego 路线点
    # ======================
    def clear_ego_route(self):
        self.ego_route_points.clear()
        print("🗑️ 已清空 ego 路线")

    def save_current_route(self):
        if not self.ego_point or not self.trigger_point:
            print("❌ 必须设置 EGO 和 Trigger")
            return

        actors = []
        for p in self.actor_points:
            atype = p.get("type", "person")

            model_map = {
                "person": "walker.pedestrian.0001",
                "bike": "vehicle.diamondback.century",
                "car": "vehicle.tesla.model3",
                "obstacle": "static.prop.container"
            }
            model = model_map[atype]

            actors.append({
                "type": atype,
                "model": model,
                "transform": {
                    "pitch": "0.00",
                    "x": f"{round(p['x'],2):.2f}",
                    "y": f"{round(p['y'],2):.2f}",
                    "yaw": f"{round(p['yaw'],2):.2f}",
                    "z": f"{round(p['z'],2):.2f}"
                },
                "rolename": atype,
                "autopilot": True
            })

        filename = f"scenario_{self.scenario}_{self.route_index:04d}.json"
        path = os.path.join(self.save_dir, filename)

        # ======================
        # ✅ 新增：保存 ego_route 到 JSON
        # ======================
        ego_route_save = []
        for p in self.ego_route_points:
            ego_route_save.append({
                "x": p['x'],
                "y": p['y'],
                "z": p['z'],
                "yaw": p['yaw']
            })

        data = {
            self.town_name: {
                ROUTE_ID: [{
                    "name": f"DynamicObjectCrossing_{self.scenario}_{self.route_index:04d}",
                    "ego_start": {
                        "x": f"{round(self.ego_point['x'],2):.2f}",
                        "y": f"{round(self.ego_point['y'],2):.2f}",
                        "z": f"{round(self.ego_point['z'],2):.2f}",
                        "yaw": f"{round(self.ego_point['yaw'],2):.2f}"
                    },
                    "ego_route": ego_route_save,  # ✅ 路线保存
                    "trigger_position": {
                        "pitch": "0.0",
                        "x": f"{round(self.trigger_point['x'],2):.2f}",
                        "y": f"{round(self.trigger_point['y'],2):.2f}",
                        "yaw": f"{round(self.trigger_point['yaw'],2):.2f}",
                        "z": f"{round(self.trigger_point['z'],2):.2f}"
                    },
                    "trigger_radius": 2.0,
                    "other_actors": {"center": actors},
                    "timeout": 60.0,
                    "active": True
                }]
            }
        }

        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        print(f"✅ 已保存：{filename} | 路线点：{len(ego_route_save)} 个")
        self.clear_all_points()
        self.route_index += 1

    def clear_all_points(self):
        self.trigger_point = None
        self.ego_point = None
        self.actor_points.clear()
        self.selected_agent_idx = -1
        self.selected_mode = None
        print("🗑️ 已清空所有点")

    def select_click(self, sx, sy):
        wx, wy = self.screen_to_world(sx, sy)
        self.selected_agent_idx = -1
        self.selected_mode = None
        min_dist = 999
        sel_idx = -1

        for i, p in enumerate(self.actor_points):
            d = (p['x']-wx)**2 + (p['y']-wy)**2
            if d < min_dist and d < 10/(self.zoom+0.01):
                min_dist = d
                sel_idx = i

        if sel_idx >=0:
            self.selected_agent_idx = sel_idx
            self.selected_mode = 'agent'
            print(f"✅ 选中 AGENT {sel_idx+1}")
            return

        if self.trigger_point:
            d = (self.trigger_point['x']-wx)**2 + (self.trigger_point['y']-wy)**2
            if d < 15/(self.zoom+0.01):
                self.selected_mode = 'trigger'
                print("✅ 选中 Trigger")
                return

        if self.ego_point:
            d = (self.ego_point['x']-wx)**2 + (self.ego_point['y']-wy)**2
            if d < 15/(self.zoom+0.01):
                self.selected_mode = 'ego'
                print("✅ 选中 EGO")
                return

    def delete_selected(self):
        if self.selected_mode == 'trigger':
            self.trigger_point = None
        elif self.selected_mode == 'ego':
            self.ego_point = None
        elif self.selected_mode == 'agent' and 0 <= self.selected_agent_idx < len(self.actor_points):
            del self.actor_points[self.selected_agent_idx]
        self.selected_agent_idx = -1
        self.selected_mode = None
        print("🗑️ 已删除")

    def set_agent_yaw(self, yaw_type):
        if self.selected_agent_idx <0 or self.selected_agent_idx >= len(self.actor_points):
            print("❌ 请先选中一个 AGENT")
            return
        p = self.actor_points[self.selected_agent_idx]
        wp = self.get_nearest_waypoint(p['x'], p['y'])
        y = wp.transform.rotation.yaw
        if yaw_type ==1: ny = y
        elif yaw_type ==2: ny = y+180
        elif yaw_type ==3: ny = y-90
        elif yaw_type ==4: ny = y+90
        else: ny=0
        p['yaw'] = ny

    def run(self):
        running = True
        print("\n========== 批量生成模式 ==========")
        print("【Ctrl+左键】设置 EGO")
        print("【Shift+左键】设置 Trigger")
        print("【右键】添加 AGENT")
        print("【Alt+右键】添加 EGO 行驶路线点 ✅")
        print("【C】清空 EGO 路线 ✅")
        print("【5】行人  【6】自行车  【7】汽车  【8】障碍物")
        print("【S】保存 【Q】退出")
        print("==================================\n")

        while running:
            self.screen.fill((0,0,0))
            self.draw_waypoints()
            self.draw_points()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        running = False
                    if event.key == pygame.K_s:
                        self.save_current_route()
                    if event.key == pygame.K_c:
                        self.clear_ego_route()  # ✅ 清空路线
                    if event.key == pygame.K_1: self.set_agent_yaw(1)
                    if event.key == pygame.K_2: self.set_agent_yaw(2)
                    if event.key == pygame.K_3: self.set_agent_yaw(3)
                    if event.key == pygame.K_4: self.set_agent_yaw(4)
                    if event.key == pygame.K_5: self.set_agent_type("person")
                    if event.key == pygame.K_6: self.set_agent_type("bike")
                    if event.key == pygame.K_7: self.set_agent_type("car")
                    if event.key == pygame.K_8: self.set_agent_type("obstacle")
                    if event.key == pygame.K_DELETE:
                        self.delete_selected()

                if event.type == pygame.MOUSEWHEEL:
                    mx, my = pygame.mouse.get_pos()
                    wx, wy = self.screen_to_world(mx, my)
                    self.zoom *= ZOOM_SPEED if event.y>0 else 1/ZOOM_SPEED
                    nx, ny = self.world_to_screen(wx, wy)
                    self.offset_x += mx - nx
                    self.offset_y += my - ny

                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 2:
                        self.dragging = True
                        self.last_mouse_pos = pygame.mouse.get_pos()

                    if event.button == 1:
                        mods = pygame.key.get_mods()
                        sx, sy = event.pos
                        wx, wy = self.screen_to_world(sx, sy)
                        wp = self.get_nearest_waypoint(wx, wy)

                        if mods & pygame.KMOD_SHIFT:
                            self.trigger_point = {
                                'x': wp.transform.location.x,
                                'y': wp.transform.location.y,
                                'z': wp.transform.location.z,
                                'yaw': wp.transform.rotation.yaw
                            }
                            print("📍 设置 Trigger")
                        elif mods & pygame.KMOD_CTRL:
                            self.ego_point = {
                                'x': wp.transform.location.x,
                                'y': wp.transform.location.y,
                                'z': wp.transform.location.z + 0.3,
                                'yaw': wp.transform.rotation.yaw
                            }
                            print("🚗 设置 EGO")
                        else:
                            self.select_click(sx, sy)

                    # ======================
                    # ✅ 新增：Alt + 右键 = 添加 ego 路线点
                    # ======================
                    if event.button == 3:
                        mods = pygame.key.get_mods()
                        sx, sy = event.pos
                        wx, wy = self.screen_to_world(sx, sy)
                        if mods & pygame.KMOD_ALT:
                            self.add_ego_route_point(wx, wy)
                        else:
                            cx, cy = self.screen_to_world(sx, sy)
                            z = self.get_ground_z(cx, cy)
                            wp = self.get_nearest_waypoint(cx, cy)
                            self.actor_points.append({
                                'x': cx, 'y': cy, 'z': z,
                                'yaw': wp.transform.rotation.yaw,
                                'type': 'person'
                            })
                            print("👤 添加 AGENT")

                if event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 2:
                        self.dragging = False

                if event.type == pygame.MOUSEMOTION and self.dragging:
                    cx, cy = pygame.mouse.get_pos()
                    dx = cx - self.last_mouse_pos[0]
                    dy = cy - self.last_mouse_pos[1]
                    self.offset_x += dx
                    self.offset_y += dy
                    self.last_mouse_pos = (cx, cy)

            pygame.display.flip()
            self.clock.tick(30)
        pygame.quit()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--name', type=str, default='TOWN10HD_Opt', help='town/map name')
    parser.add_argument('--scenario', type=int, default=1, help='scenario group number')
    parser.add_argument('--save_dir', type=str, default='output', help='save folder')
    args = parser.parse_args()

    editor = CarlaMapEditor0916(
        host=args.host,
        port=args.port,
        town_name=args.name,
        scenario=args.scenario,
        save_dir=args.save_dir
    )
    editor.run()

if __name__ == '__main__':
    main()