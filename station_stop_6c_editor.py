"""6c editor: snapped approach anchors, free curb/goal, vehicle and spawn previews."""
from __future__ import annotations
import copy
import hashlib
import json
import math
import time
from pathlib import Path
import pygame
import carla

from waypoints_stf import CarlaMapEditor0916
from station_stop_6c import (footprint, make_baseline, spawn_for_distance,
                            target_pose, verify, frame_point, frame_coordinates, trace_straight_approach)
from station_stop_6c_cli import write_json
from station_stop_6c_ground import GroundQuery


class StationStopEditor(CarlaMapEditor0916):
    def __init__(self, args, cfg):
        client = carla.Client(args.host or cfg['host'], args.port or cfg['port'])
        client.set_timeout(20)
        current = client.get_world().get_map().name
        town = args.town or cfg.get('town') or current
        from curve_emergency_5e import require_map
        require_map(current, town)
        super().__init__(host=args.host or cfg['host'], port=args.port or cfg['port'],
                         town_name=current, scenario='6c', save_dir=args.save_dir or cfg['baseline_dir'])
        self.settings = copy.deepcopy(cfg)
        self.mode = 'ego'
        self.ground_connection = dict(host=args.host or cfg['host'], port=args.port or cfg['port'],
            map_name=self.map.name, map_sha256=hashlib.sha256(self.map.to_opendrive().encode()).hexdigest(),
            rpc_timeout_s=float(cfg.get('editor', {}).get('ground_rpc_timeout_s', 2.)))
        self.ground_query = GroundQuery(cfg.get('editor', {}).get('ground_query_timeout_s', 5.))
        self.ground_feedback = None
        self.route_error = None
        self.geometry_revision = 0
        self.marks = {k: None for k in ('a', 'b', 'goal')}
        self.ground_evidence = {}
        self.vehicle_geometry = None
        self.preview_baseline = None
        self.history = []
        self.message = '先放 E，再向前画接近路线；V 会沿 E 上游车道自动补足最远起点。'
        if args.baseline:
            b = json.loads(Path(args.baseline).read_text(encoding='utf-8'))
            verify(b, 'baseline_fingerprint')
            if hashlib.sha256(self.map.to_opendrive().encode()).hexdigest() != b['opendrive_sha256']:
                raise ValueError('baseline belongs to another map')
            self.ego_point, self.ego_route_points = b.get('drawn_ego_start', b['ego_start']), b['route_anchors']
            self.marks = dict(a=b['station_edge'][0], b=b['station_edge'][1], goal=b['stop_pose'])
            self.ground_evidence = b['ground_evidence']
            if b['ego_blueprint'] == cfg['ego_blueprint']:
                self.vehicle_geometry = b['vehicle_geometry']
            else:
                self.message = '已沿用旧基准点位；车型已更换，V 将重新测量当前车型并生成新基准。'
        self.fit_view()

    def fit_view(self):
        points = ([self.ego_point] if self.ego_point else []) + self.ego_route_points
        points += [p for p in self.marks.values() if p]
        if not points:
            points = [dict(x=w.transform.location.x, y=w.transform.location.y) for w in self.waypoints]
        if not points:
            return
        x0, x1 = min(p['x'] for p in points), max(p['x'] for p in points)
        y0, y1 = min(p['y'] for p in points), max(p['y'] for p in points)
        width, height = self.screen.get_size()
        self.zoom = max(.2, min(40., (width-100)/max(10., x1-x0), (height-330)/max(10., y1-y0)))
        self.offset_x = width/2 - (x0+x1)/2*self.zoom
        self.offset_y = (height+240)/2 + (y0+y1)/2*self.zoom

    def require_ground_idle(self):
        if self.ground_query.pending is not None:
            raise ValueError('正在查询高程；请等待结果或按 Esc 取消，窗口仍可平移/缩放')

    def feedback(self, message):
        self.message = message
        print('[6c editor] ' + message, flush=True)

    @staticmethod
    def mode_label(mode):
        return {'ego': 'E（基准起点）', 'route': 'P（接近路线）',
                'a': 'A（站台边界）', 'b': 'B（站台边界）', 'goal': 'S（停车目标）'}[mode]

    def select_mode(self, mode):
        self.mode = mode
        self.feedback('已选择 ' + self.mode_label(mode) + '，请左键选点。')

    def remember(self):
        self.require_ground_idle()
        self.route_error = None
        self.geometry_revision += 1
        self.history.append(copy.deepcopy((self.ego_point, self.ego_route_points, self.marks,
                                          self.ground_evidence, self.vehicle_geometry)))
        self.preview_baseline = None

    def start_ground_query(self, x, y):
        if self.ego_point is None:
            raise ValueError('先放 E，以确定路面高程范围')
        request = dict(self.ground_connection, x=x, y=y, reference_z=self.ego_point['z'],
                       mode=self.mode, geometry_revision=self.geometry_revision)
        self.ground_query.start(request)
        self.ground_feedback = dict(x=x, y=y, mode=self.mode, state='pending')
        self.feedback('已接收 {} 点击 x={:.2f}, y={:.2f}；正在查询高程，Esc 可取消。'.format(
            self.mode_label(self.mode), x, y))

    def poll_ground_query(self):
        result = self.ground_query.poll()
        if result is None:
            return
        request = result['request']
        mode = request['mode']
        if request['geometry_revision'] != self.geometry_revision:
            self.ground_feedback = None
            self.feedback('查询期间几何已变化，已丢弃旧结果，请重新选点。')
            return
        if not result['ok']:
            self.ground_feedback = dict(x=request['x'], y=request['y'], mode=mode, state='failed')
            self.feedback(self.mode_label(mode) + ' 未放置：' + result['error'])
            return
        point = result['point']
        self.remember()
        self.marks[mode] = point
        self.ground_evidence[mode] = result['evidence']
        self.ground_feedback = None
        self.feedback('{} 已放置：x={:.2f}, y={:.2f}, z={:.2f}；V 三维预检，S 保存。'.format(
            self.mode_label(mode), point['x'], point['y'], point['z']))

    def cancel_ground_query(self):
        request = self.ground_query.cancel()
        if request is None:
            return False
        self.ground_feedback = None
        self.feedback('已取消 ' + self.mode_label(request['mode']) + ' 的高程查询；原有点位保留。')
        return True

    def undo(self):
        if self.cancel_ground_query():
            return
        if self.history:
            (self.ego_point, self.ego_route_points, self.marks,
             self.ground_evidence, self.vehicle_geometry) = self.history.pop()
            self.geometry_revision += 1
            self.preview_baseline = None
            self.ground_feedback = None
            self.route_error = None
            self.feedback('已撤销上一步。')

    def place(self, screen_position):
        self.require_ground_idle()
        x, y = self.screen_to_world(*screen_position)
        if self.mode in ('ego', 'route'):
            wp = self.get_nearest_waypoint(x, y)
            if wp is None or math.hypot(wp.transform.location.x - x, wp.transform.location.y - y) > 4:
                raise ValueError('请在接近车道中心附近放 E/P 点')
            p = dict(x=wp.transform.location.x, y=wp.transform.location.y,
                     z=wp.transform.location.z, yaw=wp.transform.rotation.yaw)
            if self.mode == 'route':
                if self.ego_point is None:
                    raise ValueError('先放 E')
                # Check this segment before accepting Pn, so save/preview does
                # not reveal a generic topology failure several actions later.
                prior = self.ego_route_points[-1] if self.ego_route_points else self.ego_point
                try:
                    prior_label = 'E' if not self.ego_route_points else 'P{}'.format(len(self.ego_route_points))
                    next_label = 'P{}'.format(len(self.ego_route_points) + 1)
                    trace_straight_approach(self.map, [prior, p], labels=[prior_label, next_label])
                except ValueError:
                    self.route_error = dict(start=prior, goal=p)
                    raise
            self.remember()
            if self.mode == 'ego':
                self.ego_point = p; self.vehicle_geometry = None
            else:
                self.ego_route_points.append(p)
        else:
            self.start_ground_query(x, y)
            return
        self.feedback('{}：x={:.2f}, y={:.2f}, z={:.2f}；V 三维预检，S 保存。'.format(
            self.mode_label(self.mode), p['x'], p['y'], p['z']))

    def read_vehicle_geometry(self):
        if self.vehicle_geometry is not None:
            return self.vehicle_geometry
        from station_stop_6c_runtime import transform, read_vehicle_geometry
        actor = self.world.try_spawn_actor(self.world.get_blueprint_library().find(self.settings['ego_blueprint']),
                                           transform(self.ego_point, .6))
        if actor is None:
            raise ValueError('E 位置无法生成所选车型，不能取得真实车身轮廓')
        try:
            geometry = read_vehicle_geometry(actor)
            self.vehicle_geometry = geometry
        finally:
            if not actor.destroy():
                raise RuntimeError('临时车型探针清理失败；请检查 CARLA 世界后再继续')
        return geometry

    def build(self):
        self.require_ground_idle()
        if self.ego_point is None or len(self.ego_route_points) < 2 or any(p is None for p in self.marks.values()):
            raise ValueError('需要 E、至少两个 P 点、站台 A/B 和停车目标 G(S)')
        draft = dict(ego_start=self.ego_point, route_anchors=self.ego_route_points,
                     station_edge=[self.marks['a'], self.marks['b']], stop_pose=self.marks['goal'],
                     vehicle_geometry=self.read_vehicle_geometry(), ground_evidence=self.ground_evidence,
                     geometry_review='operator_confirms_3d_preview_on_save',
                     boarding_zone=None, alighting_zone=None)
        return make_baseline(self.map, draft, self.settings)

    def preview(self):
        b = self.build()
        self.preview_baseline = b
        debug = self.world.debug
        def location(p):
            return carla.Location(x=p['x'], y=p['y'], z=p['z'] + .15)
        debug.draw_line(location(b['station_edge'][0]), location(b['station_edge'][1]),
                        thickness=.12, color=carla.Color(255, 210, 0), life_time=30.)
        corners = footprint(b['stop_pose'], b['vehicle_geometry'])
        for p, q in zip(corners, corners[1:] + corners[:1]):
            debug.draw_line(location(p), location(q), thickness=.08, color=carla.Color(0, 255, 100), life_time=30.)
        for d in self.settings['matrix']['initial_distances_m']:
            p, _ = spawn_for_distance(b, d)
            debug.draw_string(location(p), '{} m'.format(d), color=carla.Color(50, 180, 255), life_time=30.)
        debug.draw_arrow(location(b['stop_pose']), location(frame_point(3, 0, b['stop_pose'])),
                         thickness=.1, arrow_size=.4, color=carla.Color(0, 255, 100), life_time=30.)
        extended = (frame_coordinates(b['drawn_ego_start'], b['stop_pose'])[0]
                    - frame_coordinates(b['ego_start'], b['stop_pose'])[0])
        prefix = '已从 E 沿车道向上游补线 {:.1f} m；'.format(extended) if extended > .05 else ''
        self.message = '预检完成：' + prefix + '请在 CARLA 核对站台和停车轮廓；确认后按 S 保存。'

    def save(self):
        b = self.build()
        if self.preview_baseline is None or b['baseline_fingerprint'] != self.preview_baseline['baseline_fingerprint']:
            raise ValueError('请先按 V 检查当前几何，再按 S 确认保存')
        path = Path(self.save_dir) / ('baseline_6c_' + b['baseline_fingerprint'][:12] + '.json')
        if path.exists() and json.loads(path.read_text(encoding='utf-8')) != b:
            raise ValueError('baseline filename conflict')
        write_json(path, b)
        self.message = '已保存基准：' + str(path) + '；下一步 expand。'
        print(self.message, flush=True)

    def pixel(self, p):
        return tuple(round(v) for v in self.world_to_screen(p['x'], p['y']))

    def draw_6c(self):
        font = getattr(self, 'panel_font', self.font)
        route = ([self.ego_point] if self.ego_point else []) + self.ego_route_points
        if len(route) >= 2:
            pygame.draw.lines(self.screen, (30, 210, 220), False, [self.pixel(p) for p in route], 2)
        bad = getattr(self, 'route_error', None)
        if bad:
            a, b = self.pixel(bad['start']), self.pixel(bad['goal'])
            self._draw_clipped_polyline([a, b], (255, 80, 80), 2)
            pygame.draw.line(self.screen, (255, 80, 80), (b[0]-8, b[1]-8), (b[0]+8, b[1]+8), 2)
            pygame.draw.line(self.screen, (255, 80, 80), (b[0]+8, b[1]-8), (b[0]-8, b[1]+8), 2)
        points = [('E', self.ego_point)] + [('P' + str(i + 1), p) for i, p in enumerate(self.ego_route_points)]
        points += [(k.upper() if k != 'goal' else 'S', v) for k, v in self.marks.items()]
        for label, p in points:
            if p is not None:
                xy = self.pixel(p)
                pygame.draw.circle(self.screen, (255, 210, 60), xy, 5)
                self.screen.blit(font.render(label, True, (255, 255, 255)), (xy[0] + 6, xy[1] + 4))
        a, b, goal = (self.marks[k] for k in ('a', 'b', 'goal'))
        if a and b:
            pygame.draw.line(self.screen, (255, 210, 60), self.pixel(a), self.pixel(b), 3)
        if a and b and goal and route:
            try:
                pose = target_pose(a, b, goal, route[-1]['yaw'])
                if self.vehicle_geometry:
                    pygame.draw.polygon(self.screen, (50, 255, 100),
                                        [self.pixel(p) for p in footprint(pose, self.vehicle_geometry)], 2)
                pygame.draw.line(self.screen, (50, 255, 100), self.pixel(pose),
                                 self.pixel(frame_point(3, 0, pose)), 3)
            except ValueError:
                pass
        if self.preview_baseline:
            dense = self.preview_baseline['ego_route']
            drawn = self.preview_baseline['drawn_ego_start']
            e_index = min(range(len(dense)), key=lambda i: math.hypot(
                dense[i]['x'] - drawn['x'], dense[i]['y'] - drawn['y']))
            if e_index > 0:
                pygame.draw.lines(self.screen, (30, 210, 220), False,
                                  [self.pixel(p) for p in dense[:e_index + 1]], 2)
            for d in self.settings['matrix']['initial_distances_m']:
                p, _ = spawn_for_distance(self.preview_baseline, d)
                xy = self.pixel(p)
                pygame.draw.circle(self.screen, (90, 140, 255), xy, 7, 2)
                self.screen.blit(font.render(str(d) + 'm', True, (90, 170, 255)), (xy[0], xy[1] - 22))
        marker = self.ground_feedback
        if marker is not None:
            xy = self.pixel(marker)
            pending = marker['state'] == 'pending'
            color = (255, 175, 40) if pending else (255, 80, 80)
            pygame.draw.circle(self.screen, color, xy, 10, 2)
            pygame.draw.line(self.screen, color, (xy[0]-14, xy[1]), (xy[0]+14, xy[1]), 1)
            pygame.draw.line(self.screen, color, (xy[0], xy[1]-14), (xy[0], xy[1]+14), 1)
            label = ('S' if marker['mode'] == 'goal' else marker['mode'].upper()) + (' ?' if pending else ' !')
            self.screen.blit(font.render(label, True, color), (xy[0]+13, xy[1]+8))
        lines = ['6.c 一次停靠 | 一条基准 → 4起点×2初速度×120天气',
                 'E/ Ctrl+左键：接近锚点，上游自动补线；R/ Alt+右键：接近路线（吸附车道）',
                 'A、B、G 后左键：站台边界 A/B、停车目标 S（自由坐标）',
                 '车头方向由站台和接近方向自动确定；不画 Pn 到 S 的停车轨迹。',
                 'V 三维预检；S 保存；F 适应视图；Ctrl+Z 撤销；Esc 取消查询；Q 退出',
                 '范围：parking_only；车门/踏步条款未评价。当前：' + self.mode_label(self.mode), self.message]
        if self.ground_query.pending is not None:
            pending = self.ground_query.pending
            lines.append('查询 {} 高程中：{:.1f}/{:.1f}s；可平移、缩放或 Esc 取消。'.format(
                self.mode_label(pending['request']['mode']), time.monotonic()-pending['started'], self.ground_query.timeout_s))
        wrapped = []
        width = min(1080, self.screen.get_width() - 20)
        for line in lines:
            part = ''
            for char in line:
                if part and font.size(part + char)[0] > width - 20:
                    wrapped.append(part); part = ''
                part += char
            wrapped.append(part)
        panel = pygame.Surface((width, 26 * len(wrapped) + 12), pygame.SRCALPHA)
        panel.fill((18, 22, 30, 230))
        for i, line in enumerate(wrapped):
            panel.blit(font.render(line, True, (235, 235, 240)), (8, 6 + i * 26))
        self.screen.blit(panel, (10, 10))

    def run(self):
        pygame.display.set_caption('6.c 普通站台 · 独立路线编辑器')
        keys = {pygame.K_e: 'ego', pygame.K_r: 'route', pygame.K_a: 'a', pygame.K_b: 'b', pygame.K_g: 'goal'}
        running = True
        try:
            while running:
                for event in pygame.event.get():
                    try:
                        if event.type == pygame.QUIT:
                            running = False
                        elif event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_q: running = False
                            elif event.key == pygame.K_ESCAPE:
                                if not self.cancel_ground_query(): running = False
                            elif event.key in keys: self.select_mode(keys[event.key])
                            elif event.key == pygame.K_f: self.fit_view()
                            elif event.key == pygame.K_v: self.preview()
                            elif event.key == pygame.K_s: self.save()
                            elif event.key == pygame.K_c: self.remember(); self.ego_route_points = []
                            elif event.key == pygame.K_z and event.mod & pygame.KMOD_CTRL:
                                self.undo()
                        elif event.type == pygame.MOUSEBUTTONDOWN:
                            if event.button == 2: self.dragging = True; self.last_mouse_pos = event.pos
                            elif event.button in (4, 5): self._zoom_at(*event.pos, 1 if event.button == 4 else -1)
                            elif event.button == 1:
                                if pygame.key.get_mods() & pygame.KMOD_CTRL: self.mode = 'ego'
                                self.place(event.pos)
                            elif event.button == 3 and pygame.key.get_mods() & pygame.KMOD_ALT:
                                self.mode = 'route'; self.place(event.pos)
                        elif event.type == pygame.MOUSEBUTTONUP and event.button == 2: self.dragging = False
                        elif event.type == pygame.MOUSEWHEEL: self._zoom_at(*pygame.mouse.get_pos(), event.y)
                        elif event.type == pygame.MOUSEMOTION and self.dragging:
                            self.offset_x += event.pos[0] - self.last_mouse_pos[0]
                            self.offset_y += event.pos[1] - self.last_mouse_pos[1]
                            self.last_mouse_pos = event.pos
                    except (ValueError, RuntimeError, OSError, KeyError) as exc:
                        self.feedback(str(exc))
                self.poll_ground_query()
                self.screen.fill((0, 0, 0)); self.draw_waypoints(); self.draw_6c()
                pygame.display.flip(); self.clock.tick(30)
        finally:
            self.ground_query.close()
            pygame.quit()
