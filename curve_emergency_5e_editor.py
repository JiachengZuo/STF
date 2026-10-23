"""5e map drawing UI: one Ego route, P/T, and shared crossing endpoints."""
import copy
import json
from pathlib import Path

import pygame
from waypoints_stf import CarlaMapEditor0916
from curve_emergency_5e import make_baseline, verify_fingerprint, ensure_world, require_map


class CurveEditor(CarlaMapEditor0916):
    def __init__(self, args, cfg):
        import carla
        town = args.town or cfg['town']
        client = carla.Client(args.host or cfg['host'], args.port or cfg['port'])
        client.set_timeout(20)
        ensure_world(client, town)
        super().__init__(host=args.host or cfg['host'],port=args.port or cfg['port'],
                         town_name=town,scenario='5e',
                         save_dir=args.save_dir or cfg['baseline_dir'])
        require_map(self.map.name, town)
        self.settings = cfg
        self.mode = 'ego'
        self.marks = {k: None for k in ('risk_position','trigger_position','crossing_start','crossing_end')}
        self.history = []
        self.message = '选择Ego起点，沿弯道绘制路线，再放P/T及横穿起终点。'
        if args.baseline:
            b = json.loads(Path(args.baseline).read_text(encoding='utf-8'))
            verify_fingerprint(b,'baseline_fingerprint')
            require_map(b['map_name'], town)
            import hashlib
            if hashlib.sha256(self.map.to_opendrive().encode()).hexdigest() != b['opendrive_sha256']:
                raise ValueError('baseline belongs to another map')
            self.ego_point = b['ego_start']
            self.ego_route_points = b['route_anchors']
            self.marks = {k:b[k] for k in self.marks}

    def remember(self):
        self.history.append(copy.deepcopy((self.ego_point,self.ego_route_points,self.marks)))

    def place(self, pos):
        wx, wy = self.screen_to_world(*pos)
        wp = self.get_nearest_waypoint(wx,wy)
        if wp is None:
            raise ValueError('附近没有可用道路，请在地图范围内放置')
        location = wp.transform.location
        p = dict(x=location.x,y=location.y,z=location.z,yaw=wp.transform.rotation.yaw)
        self.remember()
        if self.mode == 'ego':
            self.ego_point = p
        elif self.mode == 'route':
            if not self.ego_point:
                raise ValueError('先放Ego起点')
            self.ego_route_points.append(p)
        else:
            if self.mode.startswith('crossing_'):
                # Off-road endpoints keep the clicked XY, unlike lane-center Ego anchors.
                p['x'],p['y'] = wx,wy
            self.marks[self.mode] = p
        self.message = '已放置 '+self.mode

    def save(self):
        if self.ego_point is None or any(v is None for v in self.marks.values()):
            raise ValueError('需要Ego、路线、P、T、横穿起点A与终点B')
        draft = dict(ego_start=self.ego_point,route_anchors=self.ego_route_points,**self.marks)
        b = make_baseline(self.map,draft,self.settings)
        p = Path(self.save_dir)/('baseline_5e_'+b['baseline_fingerprint'][:12]+'.json')
        p.parent.mkdir(parents=True,exist_ok=True)
        with p.open('x',encoding='utf-8') as stream:
            json.dump(b,stream,ensure_ascii=False,indent=2,allow_nan=False)
            stream.write('\n')
        self.message = '已保存 '+str(p)+'；下一步用 --expand 展开960条件。'
        print(self.message,flush=True)

    def draw_5e(self):
        route = ([self.ego_point] if self.ego_point else [])+self.ego_route_points
        if len(route)>1:
            self._draw_clipped_polyline([self.world_to_screen(p['x'],p['y']) for p in route],(50,230,140),3)
        a,b = self.marks['crossing_start'],self.marks['crossing_end']
        if a and b:
            self._draw_clipped_polyline([self.world_to_screen(p['x'],p['y']) for p in (a,b)],(255,180,60),3)
        labels = [('EGO',self.ego_point,(60,210,255))]
        labels += [('R'+str(i+1),p,(50,230,140)) for i,p in enumerate(self.ego_route_points)]
        labels += [(name,self.marks[key],color) for name,key,color in (
            ('P','risk_position',(255,80,100)),('T','trigger_position',(255,140,200)),
            ('A','crossing_start',(255,190,60)),('B','crossing_end',(255,190,60)))]
        for label,p,color in labels:
            if p:
                xy = self.world_to_screen(p['x'],p['y'])
                if self._screen_point_visible(xy,margin=100):
                    self._draw_screen_circle(color,xy,7)
                    self.screen.blit(self.font.render(label,True,color),(xy[0]+9,xy[1]))

    def panel(self):
        font = getattr(self,'panel_font',self.font)
        lines = ['5.e 固定转弯路线：3种静态障碍 + 5类横穿目标 × 120天气',
                 '按键选择后左键放置：E=Ego；R=路线；P=风险点；T=横穿触发点',
                 'A=横穿起点；B=横穿终点；A→P→B在一条直线上，给卡车留足空间',
                 'Ctrl+左键放Ego；Alt+右键加路点；滚轮缩放；中键拖动',
                 'Ctrl+Z撤销；C清路点；S校验保存基准；Q退出',
                 '当前：'+self.mode+'；Ego到达T后按模板时序释放横穿目标',self.message]
        width = min(960,self.screen.get_width()-20)
        wrapped = []
        for line in lines:
            current = ''
            for char in line:
                if current and font.size(current+char)[0] > width-20:
                    wrapped.append(current);current=''
                current += char
            wrapped.append(current)
        panel = pygame.Surface((width,len(wrapped)*26+16),pygame.SRCALPHA)
        panel.fill((15,20,27,225))
        for i,line in enumerate(wrapped):
            panel.blit(font.render(line,True,(225,235,245)),(10,8+i*26))
        self.screen.blit(panel,(10,10))

    def run(self):
        pygame.display.set_caption('5.e 紧急转弯危险情况 · 路线编辑器')
        keys = {pygame.K_e:'ego',pygame.K_r:'route',pygame.K_p:'risk_position',
                pygame.K_t:'trigger_position',pygame.K_a:'crossing_start',pygame.K_b:'crossing_end'}
        clock = pygame.time.Clock()
        running = True
        try:
            while running:
                for event in pygame.event.get():
                    try:
                        if event.type == pygame.QUIT:
                            running = False
                        elif event.type == pygame.KEYDOWN:
                            if event.key in (pygame.K_ESCAPE,pygame.K_q):running=False
                            elif event.key in keys:self.mode=keys[event.key]
                            elif event.key == pygame.K_s:self.save()
                            elif event.key == pygame.K_c:self.remember();self.ego_route_points=[]
                            elif event.key == pygame.K_z and pygame.key.get_mods() & pygame.KMOD_CTRL:
                                if self.history:self.ego_point,self.ego_route_points,self.marks=self.history.pop()
                        elif event.type == pygame.MOUSEBUTTONDOWN:
                            if event.button == 2:self.dragging=True;self.last_mouse_pos=event.pos
                            elif event.button in (4,5):self._zoom_at(*event.pos,1 if event.button==4 else -1)
                            elif event.button == 1:
                                if pygame.key.get_mods() & pygame.KMOD_CTRL:self.mode='ego'
                                self.place(event.pos)
                            elif event.button == 3 and pygame.key.get_mods() & pygame.KMOD_ALT:
                                self.mode='route';self.place(event.pos)
                        elif event.type == pygame.MOUSEBUTTONUP and event.button == 2:self.dragging=False
                        elif event.type == pygame.MOUSEMOTION and self.dragging:
                            self.offset_x+=event.pos[0]-self.last_mouse_pos[0]
                            self.offset_y+=event.pos[1]-self.last_mouse_pos[1]
                            self.last_mouse_pos=event.pos
                    except (ValueError,RuntimeError,OSError) as exc:
                        self.message=str(exc)
                self.screen.fill((0,0,0));self.draw_waypoints();self.draw_5e();self.panel()
                pygame.display.flip();clock.tick(30)
        finally:
            pygame.quit()
