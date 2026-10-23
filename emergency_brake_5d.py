"""Pure configuration, geometry and trial-state rules for 5.d screening."""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest()


def positive(value, name):
    if isinstance(value, bool) or value is None:
        raise ValueError(name + ' must be a positive finite number')
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be a positive finite number')
    return value


def load_settings(path, require_speeds=False):
    import yaml
    cfg = yaml.safe_load(Path(path).read_text())
    if cfg.get('schema_version') != 1:
        raise ValueError('5d schema_version must be 1')
    if 'vmax_kmh' in cfg:
        raise ValueError('vmax_kmh no longer defines the library; use optional ads_vmax_kmh per controller')
    eng = cfg['engineering']
    for key in ('stable_follow_duration_s', 'setup_timeout_s', 'trial_timeout_s', 'camera_timeout_s',
                'speed_tolerance_kmh', 'stop_speed_threshold_kmh',
                'deceleration_target_mps2', 'deceleration_ramp_s',
                'lead_max_brake_torque_nm', 'lead_brake_feedforward', 'brake_kp', 'initial_net_gap_m', 'vehicle_length_allowance_m',
                'reserve_m', 'straight_tolerance_deg'):
        positive(eng[key], key)
    if not 0 < eng['lead_brake_feedforward'] <= 1:
        raise ValueError('lead_brake_feedforward must be in (0,1]')
    if eng['speed_tolerance_kmh'] > 2:
        raise ValueError('target-object speed tolerance may not exceed 2 km/h')
    if eng['deceleration_target_mps2'] != 6 or eng['deceleration_ramp_s'] != 1:
        raise ValueError('5d requires 6 m/s2 reached within 1 s')
    declarations = cfg.get('ads_vmax_kmh', {})
    if not isinstance(declarations, dict):
        raise ValueError('ads_vmax_kmh must be a controller mapping')
    cfg['ads_vmax_kmh'] = {model: positive(declarations[model], model + ' vmax')
                          if declarations.get(model) is not None else None
                          for model in ('behavior', 'tcp')}
    if require_speeds:
        speeds = cfg.get('test_speeds_kmh') or []
        if not isinstance(speeds, list) or not speeds:
            raise ValueError('test_speeds_kmh requires a nonempty list of distinct speeds')
        speeds = [positive(v, 'test speed') for v in speeds]
        if len(set(speeds)) != len(speeds):
            raise ValueError('test speeds must remain distinct after numeric conversion')
        cfg['test_speeds_kmh'] = speeds
    return cfg


def project(point, route, extend_ends=False):
    """Nearest XY polyline projection: longitudinal s and unsigned lateral error."""
    best = None; accumulated = 0.0
    for index,(a,b) in enumerate(zip(route, route[1:])):
        dx,dy=b['x']-a['x'],b['y']-a['y'];length=math.hypot(dx,dy)
        if length < 1e-8: continue
        raw=((point['x']-a['x'])*dx+(point['y']-a['y'])*dy)/length**2
        lower=-math.inf if extend_ends and index==0 else 0.
        upper=math.inf if extend_ends and index==len(route)-2 else 1.
        u=max(lower,min(upper,raw))
        distance=math.hypot(point['x']-a['x']-u*dx,point['y']-a['y']-u*dy)
        item=(distance,accumulated+u*length)
        if best is None or item[0] < best[0]:best=item
        accumulated+=length
    if best is None:raise ValueError('route has no nonzero segment')
    return {'s':best[1],'lateral':best[0],'length':accumulated}


class RouteLengthError(ValueError):
    pass


def moving_center_gap(speed_kmh, drawn_center_gap_m, eng, vehicle_length_m=None):
    length = eng['vehicle_length_allowance_m'] if vehicle_length_m is None else vehicle_length_m
    return eng['initial_net_gap_m'] + length


def minimum_route_length(speed_kmh, initial_gap_m, eng):
    # Moving start: no acceleration distance. Include configured following duration, a conservative
    # 1 s deceleration ramp allowance, ideal braking distance and response reserve.
    v = speed_kmh / 3.6
    return moving_center_gap(speed_kmh, initial_gap_m, eng) + (eng['stable_follow_duration_s']+eng['deceleration_ramp_s'])*v + v*v/(2*eng['deceleration_target_mps2']) + eng['reserve_m']


def route_feasibility(speed_kmh, center_gap_m, route_length_m, eng):
    required = minimum_route_length(speed_kmh, center_gap_m, eng)
    return {'initialization_mode': 'moving', 'initial_net_gap_m': eng['initial_net_gap_m'], 'required_route_length_m': required,
            'available_route_length_m': route_length_m,
            'planned_center_gap_m': moving_center_gap(speed_kmh, center_gap_m, eng),
            'route_length_sufficient': route_length_m >= required}


def route_point_at(route, s):
    if s < 0: raise ValueError('negative route distance')
    for a,b in zip(route,route[1:]):
        length = math.hypot(b['x']-a['x'], b['y']-a['y'])
        if length < 1e-8: continue
        if s <= length:
            u=s/length
            return dict(x=a['x']+u*(b['x']-a['x']), y=a['y']+u*(b['y']-a['y']),
                        z=a['z']+u*(b['z']-a['z']), yaw=math.degrees(math.atan2(b['y']-a['y'],b['x']-a['x'])))
        s-=length
    raise RouteLengthError('lead spawn exceeds available route')


def validate_geometry(map_obj, baseline, eng, auto_lead=False):
    import carla
    points=[baseline['ego_start']]+baseline['ego_route']
    if len(points)<3:raise ValueError('Ego plus at least two route points required')
    def wp(p):
        result=map_obj.get_waypoint(carla.Location(x=float(p['x']),y=float(p['y']),z=float(p.get('z',0))),project_to_road=True,lane_type=carla.LaneType.Driving)
        if result is None:raise ValueError('point is not on Driving lane')
        if math.hypot(result.transform.location.x-p['x'],result.transform.location.y-p['y'])>0.5:
            raise ValueError('place route/spawn on lane center, within 0.5 m')
        return result
    first=wp(points[0]);dense=[first];heading=first.transform.rotation.yaw
    warnings={}
    def check(w):
        # Custom maps may label straight connectors as junctions. Validate the
        # actual heading, lane boundaries and forward topology below instead.
        if abs((w.transform.rotation.yaw-heading+180)%360-180)>eng['straight_tolerance_deg']:
            warnings.setdefault('curvature', '路线方向变化超过配置角度；不阻止保存，需确认测试路段直线性。')
        # 5d engineering variant: adjacent driving lanes are allowed. The
        # selected route must still follow forward topology without lane changes.
        for marking in (w.left_lane_marking,w.right_lane_marking):
            if str(marking.type) not in ('Solid','SolidSolid'):
                warnings.setdefault('markings', 'OpenDRIVE未将两侧边界均标为实线；不阻止保存。')
    check(first)
    for p in points[1:]:
        target=wp(p);current=dense[-1]
        delta=target.transform.location-current.transform.location
        f=current.transform.get_forward_vector()
        if delta.x*f.x+delta.y*f.y<=0.5:raise ValueError('route points must progress forward, spaced >0.5 m')
        for _ in range(10000):
            if current.transform.location.distance(target.transform.location)<=1.1:break
            choices=current.next(1.)
            if len(choices)!=1:raise ValueError('route branches or ends before its next anchor')
            current=choices[0];check(current);dense.append(current)
        else:raise ValueError('route trace exceeded bound')
        if (current.road_id,current.lane_id)!=(target.road_id,target.lane_id):
            raise ValueError('anchor is not on traced lane')
        check(target);dense.append(target)
    route=[{'x':w.transform.location.x,'y':w.transform.location.y,'z':w.transform.location.z,'yaw':w.transform.rotation.yaw,'width':w.lane_width} for w in dense]
    if auto_lead:
        lead_point=route_point_at(route, eng['initial_net_gap_m']+eng['vehicle_length_allowance_m'])
        lead_point['z'] += .3
        baseline['lead_start']=lead_point
    lead=wp(baseline['lead_start']);projection=project(baseline['lead_start'],route)
    if projection['lateral']>0.5 or not 6 < projection['s'] < projection['length']-.1:
        raise ValueError('lead vehicle must be ahead on the route with positive clearance')
    for spawn in ('ego_start','lead_start'):
        if abs((baseline[spawn]['yaw']-wp(baseline[spawn]).transform.rotation.yaw+180)%360-180)>=90:
            raise ValueError('spawn heading opposes route')
    if auto_lead:baseline['geometry_warnings']=list(warnings.values())
    return route,projection['s']


class BrakeTrial:
    """Simulation-time state machine. Never substitutes controls for the tested ADS."""
    def __init__(self,speed_kmh,engineering,precondition_failure_is_failure=False):
        self.target=speed_kmh;self.eng=engineering;self.phase='ACQUIRE';self.start=None
        self.stable_since=None;self.brake_at=None;self.reached_decel_at=None
        self.invalid=[];self.failures=[];self.peak_decel=0.;self.stable_follow_met=False
        self.precondition_failure_is_failure=precondition_failure_is_failure
    def update(self,now,ego_kmh,lead_kmh,decel,ordered=True,collision=False,lane_violation=False):
        if self.start is None:self.start=now
        if self.phase in ('DONE','INVALID','FAILED'):return
        if collision:self.failures.append('collision');self.phase='FAILED';return
        if lane_violation:self.failures.append('solid_boundary_crossed');self.phase='FAILED';return
        stable=ordered and abs(lead_kmh-self.target)<=self.eng['speed_tolerance_kmh'] and abs(ego_kmh-lead_kmh)<=2
        if self.phase=='ACQUIRE':
            self.stable_since=(now if self.stable_since is None else self.stable_since) if stable else None
            if self.stable_since is not None and now-self.stable_since>=self.eng['stable_follow_duration_s']:
                self.phase='BRAKE';self.brake_at=now;self.stable_follow_met=True
            elif now-self.start>=self.eng['setup_timeout_s']:
                if self.precondition_failure_is_failure:
                    self.failures.append('stable_following_not_established');self.phase='FAILED'
                else:
                    self.invalid.append('stable_following_not_established');self.phase='INVALID'
        elif self.phase=='BRAKE':
            self.peak_decel=max(self.peak_decel,decel)
            target=self.eng['deceleration_target_mps2'];ramp=self.eng['deceleration_ramp_s']
            if decel>=target and self.reached_decel_at is None:self.reached_decel_at=now
            if now-self.brake_at>ramp+1e-6 and self.reached_decel_at is None:
                self.invalid.append('lead_deceleration_target_not_reached');self.phase='INVALID';return
            if lead_kmh<=self.eng['stop_speed_threshold_kmh']:
                if self.reached_decel_at is None:
                    self.invalid.append('lead_stopped_without_required_deceleration');self.phase='INVALID';return
                if self.reached_decel_at-self.brake_at>ramp+1e-6:
                    self.invalid.append('lead_deceleration_ramp_exceeded');self.phase='INVALID';return
                if ego_kmh<=self.eng['stop_speed_threshold_kmh']:self.phase='DONE'
            if now-self.brake_at>=self.eng['trial_timeout_s'] and self.phase=='BRAKE':
                self.invalid.append('trial_observation_timeout');self.phase='INVALID'
    def result(self):
        valid=not self.invalid and self.phase in ('DONE','FAILED')
        return {'stable_follow_duration_s':self.eng['stable_follow_duration_s'],'standard_follow_duration_met':self.stable_follow_met, 'trial_valid':valid,'pass':not self.failures if valid else None,'failure_reasons':self.failures,'invalid_reasons':self.invalid or ([] if valid else ['incomplete_trial']), 'phase':self.phase,'brake_start_sim_time':self.brake_at,'deceleration_reached_sim_time':self.reached_decel_at,'peak_lead_deceleration_mps2':self.peak_decel}


def speed_band(speed_kmh):
    """Engineering bands for the accepted scenario library, not GB categories."""
    return 'low' if speed_kmh <= 30 else 'medium' if speed_kmh <= 60 else 'high'


def capability_labels(speed_kmh, declared_vmax):
    return {'declared_vmax_kmh': declared_vmax,
            'within_declared_speed_range': None if declared_vmax is None else speed_kmh <= declared_vmax,
            'standard_speed_case': None if declared_vmax is None else abs(speed_kmh - .75 * declared_vmax) < 1e-6}
