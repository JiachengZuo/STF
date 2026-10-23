"""Two lane-following routes on the currently loaded CARLA map."""
import copy
import json
from pathlib import Path

import pygame
from waypoints_stf import CarlaMapEditor0916
from cross_right_2d import validate_baseline
from cross_right_2d_runtime import capture_baseline, require_map


class CrossRightEditor(CarlaMapEditor0916):
    def __init__(self, args, cfg):
        import carla
        client = carla.Client(args.host or cfg['host'], args.port or cfg['port'])
        client.set_timeout(20)
        name = client.get_world().get_map().name
        require_map(name, args.town)
        super().__init__(host=args.host or cfg['host'], port=args.port or cfg['port'],
                         town_name=name, scenario='2d', save_dir=args.save_dir or cfg['baseline_dir'])
        self.settings = cfg
        self.active_route = 'ego'
        self.routes = dict(ego=[], target=[])
        self.history = []
        self.message = 'E 编辑自车路线；N 编辑右侧目标车路线。'
        if args.baseline:
            baseline = json.loads(Path(args.baseline).read_text(encoding='utf-8'))
            validate_baseline(baseline, cfg)
            require_map(name, baseline['map_name'])
            import hashlib
            if hashlib.sha256(self.map.to_opendrive().encode()).hexdigest() != baseline['opendrive_sha256']:
                raise ValueError('baseline belongs to another map')
            self.routes = {k: copy.deepcopy(baseline[k+'_anchors']) for k in ('ego', 'target')}

    def remember(self):
        self.history.append(copy.deepcopy(self.routes))

    def place(self, pos, start=False):
        x, y = self.screen_to_world(*pos)
        wp = self.get_nearest_waypoint(x, y)
        if wp is None:
            raise ValueError('附近没有可用道路，请在道路上放置')
        route = self.routes[self.active_route]
        if not start and not route:
            raise ValueError('先用 Ctrl+左键放置该路线起点')
        p = wp.transform.location
        sample = dict(x=p.x, y=p.y, z=p.z, yaw=wp.transform.rotation.yaw)
        self.remember()
        if start:
            if route:
                route[0] = sample
            else:
                route.append(sample)
        else:
            route.append(sample)
        self.message = ('VUT' if self.active_route == 'ego' else '目标车')+'路线已更新'

    def save(self):
        b = capture_baseline(self.world, self.routes['ego'], self.routes['target'], self.settings)
        path = Path(self.save_dir)/('baseline_2d_'+b['baseline_fingerprint'][:12]+'.json')
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('x', encoding='utf-8') as stream:
            json.dump(b, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        self.message = '已保存 '+str(path)+'；可直接运行，或 --expand 展开天气。'
        if not b['geometry']['section_speed_limit_verified']:
            self.message += ' 地图限速标注仅作记录；试验按 40 km/h 上限记分。'
        print(self.message, flush=True)

    def draw_routes(self):
        for role, color, prefix in (('ego', (60, 220, 255), 'E'), ('target', (255, 185, 70), 'N')):
            route = self.routes[role]
            if len(route) > 1:
                self._draw_clipped_polyline([self.world_to_screen(p['x'], p['y']) for p in route], color, 3)
            for i, p in enumerate(route):
                xy = self.world_to_screen(p['x'], p['y'])
                if self._screen_point_visible(xy, margin=100):
                    self._draw_screen_circle(color, xy, 7)
                    self.screen.blit(self.font.render(prefix+str(i), True, color), (xy[0]+9, xy[1]))

    def panel(self):
        font = getattr(self, 'panel_font', self.font)
        lines = ['2.d 无信号灯十字路口 · 自车直行，右侧车辆直行横穿',
                 'E=编辑自车路线；N=编辑目标路线；当前：'+('自车' if self.active_route == 'ego' else '目标车'),
                 'Ctrl+左键放起点；Alt+右键加路点；滚轮缩放；中键拖动',
                 'Ctrl+Z撤销；C清当前路点；S保存一条基准；Q/Esc退出',
                 '没有 Trigger；目标车由实时 TTC 释放；天气可另用 --expand 展开。', self.message]
        width, wrapped = min(980, self.screen.get_width()-20), []
        for line in lines:
            current = ''
            for char in line:
                if current and font.size(current+char)[0] > width-20:
                    wrapped.append(current)
                    current = ''
                current += char
            wrapped.append(current)
        panel = pygame.Surface((width, len(wrapped)*26+16), pygame.SRCALPHA)
        panel.fill((15, 20, 27, 225))
        for i, line in enumerate(wrapped):
            panel.blit(font.render(line, True, (225, 235, 245)), (10, 8+i*26))
        self.screen.blit(panel, (10, 10))

    def run(self):
        pygame.display.set_caption('2.d 无信号灯路口右侧直行车辆 · 路线编辑器')
        clock, running = pygame.time.Clock(), True
        try:
            while running:
                for event in pygame.event.get():
                    try:
                        if event.type == pygame.QUIT:
                            running = False
                        elif event.type == pygame.KEYDOWN:
                            if event.key in (pygame.K_q, pygame.K_ESCAPE):
                                running = False
                            elif event.key in (pygame.K_e, pygame.K_n):
                                self.active_route = 'ego' if event.key == pygame.K_e else 'target'
                            elif event.key == pygame.K_s:
                                self.save()
                            elif event.key == pygame.K_c:
                                self.remember()
                                self.routes[self.active_route] = self.routes[self.active_route][:1]
                            elif event.key == pygame.K_z and pygame.key.get_mods() & pygame.KMOD_CTRL and self.history:
                                self.routes = self.history.pop()
                        elif event.type == pygame.MOUSEBUTTONDOWN:
                            if event.button == 2:
                                self.dragging, self.last_mouse_pos = True, event.pos
                            elif event.button in (4, 5):
                                self._zoom_at(*event.pos, 1 if event.button == 4 else -1)
                            elif event.button == 1 and pygame.key.get_mods() & pygame.KMOD_CTRL:
                                self.place(event.pos, start=True)
                            elif event.button == 3 and pygame.key.get_mods() & pygame.KMOD_ALT:
                                self.place(event.pos)
                        elif event.type == pygame.MOUSEBUTTONUP and event.button == 2:
                            self.dragging = False
                        elif event.type == pygame.MOUSEMOTION and self.dragging:
                            self.offset_x += event.pos[0]-self.last_mouse_pos[0]
                            self.offset_y += event.pos[1]-self.last_mouse_pos[1]
                            self.last_mouse_pos = event.pos
                    except (ValueError, RuntimeError, OSError) as exc:
                        self.message = str(exc)
                self.screen.fill((0, 0, 0))
                self.draw_waypoints()
                self.draw_routes()
                self.panel()
                pygame.display.flip()
                clock.tick(30)
        finally:
            pygame.quit()
