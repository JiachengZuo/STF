"""Dedicated 5d workflow, reusing the existing map renderer only."""
import copy
import hashlib
import json
from pathlib import Path
import pygame
from waypoints_stf import CarlaMapEditor0916
from emergency_brake_5d import validate_geometry,digest,route_point_at


class BrakeEditor(CarlaMapEditor0916):
    def __init__(self,args,cfg):
        super().__init__(host=args.host or cfg['host'],port=args.port or cfg['port'],town_name=args.name or args.town or '5d',scenario='5d',save_dir=args.save_dir or cfg['baseline_dir'])
        self.settings=cfg;self.history=[];self.message='先放Ego并绘制路线；前车自动按20 m净车距生成。';self._preview_key=None
    def remember(self):
        self.history.append(copy.deepcopy((self.ego_point,self.actor_points,self.ego_route_points)))
    def refresh_lead_preview(self):
        key=digest([self.ego_point,self.ego_route_points,self.settings['engineering']['initial_net_gap_m'],self.settings['engineering']['vehicle_length_allowance_m']])
        if key==self._preview_key:return
        self._preview_key=key;self.actor_points=[]
        if not self.ego_point or not self.ego_route_points:return
        try:
            p=route_point_at([self.ego_point]+self.ego_route_points,self.settings['engineering']['initial_net_gap_m']+self.settings['engineering']['vehicle_length_allowance_m'])
            self.actor_points=[dict(p,type='car')]
        except ValueError:
            self.message='路线尚不足以放置前车，请继续向前添加路点。'

    def save(self):
        if not self.ego_point:
            raise ValueError('请先放置Ego，再绘制路线')
        b={'schema_version':1,'scenario':'5d','ego_start':self.ego_point,'ego_route':self.ego_route_points,'map_name':self.map.name,'opendrive_sha256':hashlib.sha256(self.map.to_opendrive().encode()).hexdigest()}
        route,gap=validate_geometry(self.map,b,self.settings['engineering'],auto_lead=True)
        self.actor_points=[dict(b['lead_start'],type='car')]
        b['lead_placement']={'mode':'automatic_along_route','initial_net_gap_m':self.settings['engineering']['initial_net_gap_m'],'preview_vehicle_length_m':self.settings['engineering']['vehicle_length_allowance_m']}
        b['route_length_m']=sum(((c['x']-a['x'])**2+(c['y']-a['y'])**2)**.5 for a,c in zip(route,route[1:]))
        b['initial_center_gap_m']=gap;b['baseline_fingerprint']=digest(b)
        directory=Path(self.save_dir);directory.mkdir(parents=True,exist_ok=True)
        p=directory/('baseline_5d_'+b['baseline_fingerprint'][:12]+'.json')
        if p.exists():raise ValueError('相同基准已保存；未覆盖')
        p.write_text(json.dumps(b,ensure_ascii=False,indent=2)+'\n')
        self.message='已保存 '+str(p)
        if b.get('geometry_warnings'):self.message+='；提示：'+'；'.join(b['geometry_warnings'])
    def run(self):
        pygame.display.set_caption('5.d 前方车辆紧急制动 · 专项路线编辑器')
        running=True;clock=pygame.time.Clock()
        while running:
            for e in pygame.event.get():
                try:
                    if e.type==pygame.QUIT:running=False
                    elif e.type==pygame.KEYDOWN:
                        if e.key in (pygame.K_ESCAPE,pygame.K_q):running=False
                        elif e.key==pygame.K_s:self.save()
                        elif e.key==pygame.K_z and pygame.key.get_mods() & pygame.KMOD_CTRL:
                            if self.history:self.ego_point,self.actor_points,self.ego_route_points=self.history.pop()
                        elif e.key==pygame.K_c:self.remember();self.ego_route_points=[]
                    elif e.type==pygame.MOUSEBUTTONDOWN:
                        if e.button==2:self.dragging=True;self.last_mouse_pos=e.pos
                        elif e.button in (4,5):
                            # Same origin-preserving scale adjustment as normal editor wheel handling.
                            self._zoom_at(*e.pos,1 if e.button==4 else -1)
                        elif e.button in (1,3):
                            mods=pygame.key.get_mods();wx,wy=self.screen_to_world(*e.pos)
                            wp=self.get_nearest_waypoint(wx,wy)
                            if wp is None:continue
                            p={'x':wp.transform.location.x,'y':wp.transform.location.y,'z':wp.transform.location.z+.3,'yaw':wp.transform.rotation.yaw}
                            if e.button==1 and mods & pygame.KMOD_CTRL:self.remember();self.ego_point=p
                            elif e.button==1 and mods & pygame.KMOD_SHIFT:self.message='前车沿路线自动生成，无需手动放置。'
                            elif e.button==3 and mods & pygame.KMOD_ALT:
                                if not self.ego_point:raise ValueError('请先放置Ego')
                                if self.ego_point:
                                    previous=self.ego_route_points[-1] if self.ego_route_points else self.ego_point
                                    import math
                                    yaw=math.radians(previous['yaw'])
                                    if (p['x']-previous['x'])*math.cos(yaw)+(p['y']-previous['y'])*math.sin(yaw)<=.5:
                                        raise ValueError('路点必须沿车道向前，间距大于0.5 m')
                                self.remember();self.ego_route_points.append(p)
                    elif e.type==pygame.MOUSEBUTTONUP and e.button==2:self.dragging=False
                    elif e.type==pygame.MOUSEMOTION and self.dragging:
                        dx=e.pos[0]-self.last_mouse_pos[0];dy=e.pos[1]-self.last_mouse_pos[1]
                        self.offset_x+=dx;self.offset_y+=dy;self.last_mouse_pos=e.pos
                except (ValueError,RuntimeError) as exc:self.message=str(exc)
            self.refresh_lead_preview()
            self.screen.fill((0,0,0));self.draw_waypoints();self.draw_points()
            font=getattr(self,'panel_font',self.font)
            lines=['5.d 专项绘制：沿同一车道绘制（junction/相邻车道/标线不拦截）', 'Ctrl+左键：放置/重放Ego；前车自动生成（净距{} m）'.format(self.settings['engineering']['initial_net_gap_m']), 'Alt+右键：沿车道向前添加P路点；Ctrl+Z撤销', '滚轮缩放；中键拖动；C清路点；S校验保存；Q退出', '不设置Trigger；稳定跟随{}秒后触发制动'.format(self.settings['engineering']['stable_follow_duration_s']), '速度矩阵在配置文件设置；保存基准时不生成天气', self.message]
            width=min(860,self.screen.get_width()-20)
            wrapped=[]
            for line in lines:
                current=''
                for char in line:
                    if current and font.size(current+char)[0]>width-20:
                        wrapped.append(current);current=''
                    current+=char
                wrapped.append(current)
            lines=wrapped
            panel=pygame.Surface((width,len(lines)*28+16),pygame.SRCALPHA);panel.fill((15,20,27,220))
            for i,line in enumerate(lines):panel.blit(font.render(line,True,(220,235,245)),(10,8+28*i))
            self.screen.blit(panel,(10,10));pygame.display.flip();clock.tick(30)
        pygame.quit()
