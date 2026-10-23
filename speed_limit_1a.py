"""Pure geometry/validation and independent 1a virtual-rule scoring."""
import bisect
import hashlib
import json
import math

VERSION = '1a-virtual-rule-v1'


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


class Route:
    def __init__(self, points):
        self.points = points
        self.stations = [p['s_m'] for p in points]

    def at(self, s):
        if not 0 <= s <= self.stations[-1]:
            raise ValueError('station outside route')
        i = min(max(0, bisect.bisect_right(self.stations, s)-1), len(self.points)-2)
        a, b = self.points[i:i+2]
        t = (s-self.stations[i])/(self.stations[i+1]-self.stations[i])
        result = {k: a[k]+t*(b[k]-a[k]) for k in ('x','y','z')}
        result['yaw'] = a['yaw']+t*((b['yaw']-a['yaw']+180)%360-180)
        result['lane_width_m'] = a['lane_width_m']
        return result

    def project(self, x, y):
        best = None
        for i, (a,b) in enumerate(zip(self.points,self.points[1:])):
            dx,dy=b['x']-a['x'],b['y']-a['y']; length2=dx*dx+dy*dy
            if length2 < 1e-12: continue
            t=max(0.,min(1.,((x-a['x'])*dx+(y-a['y'])*dy)/length2))
            px,py=a['x']+t*dx,a['y']+t*dy
            d2=(x-px)**2+(y-py)**2
            if best is None or d2 < best[0]:
                best=(d2,a['s_m']+t*(b['s_m']-a['s_m']),i)
        return best[1],math.sqrt(best[0]),best[2]


def validate_definition(d):
    if d.get('scenario_code')!='1a': raise ValueError('not a 1a scenario')
    route=d['ego_route']; rule=d['speed_limit_test']
    if len(route)<2: raise ValueError('route requires at least two points')
    for p in route:
        if not all(finite(p.get(k)) for k in ('x','y','z','yaw','s_m','lane_width_m')):
            raise ValueError('nonfinite/missing route field')
        if p['lane_width_m']<=0: raise ValueError('invalid lane width')
    if route[0]['s_m']!=0: raise ValueError('route must begin at s=0')
    for a,b in zip(route,route[1:]):
        ds=b['s_m']-a['s_m']; chord=math.hypot(b['x']-a['x'],b['y']-a['y'])
        if ds<=0 or ds>5.01 or abs(ds-chord)>.1:
            raise ValueError('discontinuous/inconsistent route stations')
    B,C,D,E=(rule[k] for k in ('restriction_start_s_m','release_s_m','final_check_s_m','route_end_s_m'))
    if not all(finite(v) for v in (B,D,E)) or not 0<B<D<E:
        raise ValueError('invalid B/D/E order')
    if abs(E-route[-1]['s_m'])>.001: raise ValueError('E differs from route end')
    v,u,r=(rule[k] for k in ('initial_limit_kmh','restricted_limit_kmh','restored_limit_kmh'))
    if not finite(v) or not finite(u) or not 0<u<v: raise ValueError('invalid speed limits')
    if C is not None:
        if not finite(C) or not B<C<D or C-B<100 or abs(D-C-200)>.001:
            raise ValueError('invalid release/check spacing')
        if not finite(r) or r!=v: raise ValueError('invalid restored speed')
    elif r is not None: raise ValueError('restored speed without release')
    lo,hi=rule['approach_window_s_m']
    if not all(finite(x) for x in (lo,hi)) or not 0<lo<hi<B:
        raise ValueError('invalid approach window')
    if not finite(d['timeout']) or d['timeout']<=0: raise ValueError('invalid timeout')
    if rule.get('truth_only_not_controller_input') is not True: raise ValueError('control boundary missing')
    thresholds={'minimum_approach_speed_kmh':.75*v,'speed_at_B_max_kmh':u,
                'restricted_speed_min_kmh':.75*u,
                'restored_speed_at_D_min_kmh':.75*r if r else None}
    if any(rule.get(k)!=value for k,value in thresholds.items()):
        raise ValueError('inconsistent derived speed thresholds')
    segments=[{'from_s_m':0,'to_s_m':B,'limit_kmh':v},
              {'from_s_m':B,'to_s_m':C if C is not None else E,'limit_kmh':u}]
    if C is not None: segments.append({'from_s_m':C,'to_s_m':E,'limit_kmh':r})
    if rule.get('speed_segments')!=segments: raise ValueError('inconsistent speed segments')
    geometry=Route(route)
    for name,s in {'S':0,'B':B,'C':C,'D':D,'E':E,
                   'approach_window_start':lo,'approach_window_end':hi}.items():
        point=rule['test_points'].get(name)
        if s is None:
            if point is not None: raise ValueError('unexpected release point')
            continue
        expected=geometry.at(s)
        if point is None or point['s_m']!=s or any(abs(point[k]-expected[k])>.02 for k in ('x','y','z')):
            raise ValueError('test point inconsistent with route: '+name)
    start=d['ego_start']
    if not all(finite(start.get(k)) for k in ('x','y','z','yaw')):
        raise ValueError('invalid ego spawn')
    if math.hypot(start['x']-route[0]['x'],start['y']-route[0]['y'])>.1:
        raise ValueError('spawn differs from route start')
    if not all(finite(value) for value in d['weather'].values()): raise ValueError('invalid weather')
    return geometry


def decode_action(action):
    """Validate, never clamp or replace the model's final three channels."""
    if hasattr(action,'tolist'): action=action.tolist()
    if not isinstance(action,(list,tuple)) or not action:
        raise ValueError('empty/invalid TCP action')
    row=action[0] if isinstance(action[0],(tuple,list)) else action
    if len(row)!=3 or not all(finite(x) for x in row): raise ValueError('invalid TCP action')
    throttle,steer,brake=map(float,row)
    if not (0<=throttle<=1 and -1<=steer<=1 and 0<=brake<=1):
        raise ValueError('TCP action outside API ranges')
    return throttle,steer,brake


class Trial:
    """Observes measurements only; never returns driving controls."""
    def __init__(self, definition, stall_timeout=30.):
        self.d=definition;self.r=definition['speed_limit_test']
        self.B=self.r['restriction_start_s_m'];self.C=self.r['release_s_m']
        self.D=self.r['final_check_s_m'];self.E=self.r['route_end_s_m']
        self.previous=None;self.events=[];self.failures=[];self.invalid=[]
        self.done=False;self.phase='PREPARE';self.approach_min=None
        self.approach_complete=False;self.checked_D=False;self.route_complete=False
        self.passed=set();self.stall_timeout=stall_timeout;self.progress_s=0.;self.progress_time=0.

    def fail(self,reason):
        if reason not in self.failures: self.failures.append(reason)
        self.done=True

    def invalidate(self,reason):
        self.invalid.append(reason);self.done=True

    def observe(self, obs):
        if self.done:return
        if not all(finite(obs.get(k)) for k in ('time_s','front_s_m','speed_kmh','center_s_m')):
            self.invalidate('nonfinite_measurement');return
        t,s,v=obs['time_s'],obs['front_s_m'],obs['speed_kmh']
        if self.previous and t<=self.previous['time_s']:
            self.invalidate('nonmonotonic_simulation_time');return
        if obs.get('collision'):self.fail('collision');return
        if obs.get('outside_route'):self.fail('outside_route');return
        if s+5<self.progress_s:self.fail('reverse_progress');return
        if s>self.progress_s+.5:self.progress_s=s;self.progress_time=t
        old=self.previous or obs
        p=old['front_s_m'];pv=old['speed_kmh']
        def interp(station):
            ratio=max(0.,min(1.,(station-p)/(s-p))) if s>p+1e-9 else 1.
            return pv+ratio*(v-pv)
        lo,hi=self.r['approach_window_s_m']
        if s>=lo and p<=hi:
            values=[interp(max(p,lo)),interp(min(s,hi))]
            self.approach_min=min(values+[self.approach_min] if self.approach_min is not None else values)
        if not self.approach_complete and s>=hi:
            self.approach_complete=True
            self.events.append({'event':'APPROACH','time_s':t,'minimum_speed_kmh':self.approach_min})
            if self.approach_min is None:
                self.invalidate('approach_measurement_missing');return
            if self.approach_min<self.r['minimum_approach_speed_kmh']-1e-6:
                self.phase='APPROACH';self.fail('approach_speed_insufficient');return
        # Exact signed vehicle-front support distance to each gate, supplied
        # from bounding-box world vertices. Scalar s is used only offline.
        for label,station in (('B',self.B),('C',self.C),('D',self.D)):
            if station is None or label in self.passed:continue
            before=old.get('gates',{}).get(label,p-station)
            after=obs.get('gates',{}).get(label,s-station)
            if before<0<=after:
                ratio=-before/(after-before)
                speed=pv+ratio*(v-pv)
                self.events.append({'event':label,'s_m':station,'speed_kmh':speed,
                                    'time_s':old['time_s']+ratio*(t-old['time_s']),
                                    'frame_before':old.get('frame'),'frame_after':obs.get('frame'),
                                    'interpolation':'linear_between_front_gate_samples'})
                self.passed.add(label)
                if label=='B' and speed>self.r['restricted_limit_kmh']+1e-6:
                    self.phase='LIMITED';self.fail('speed_at_B_exceeded');return
                if label=='D':
                    self.checked_D=True
                    if self.C is not None and speed<self.r['restored_speed_at_D_min_kmh']-1e-6:
                        self.phase='RECOVERY';self.fail('recovery_speed_insufficient');return
        limited_end=self.C if self.C is not None else self.D
        if s>=self.B and p<limited_end:
            values=[interp(max(p,self.B)),interp(min(s,limited_end))]
            if max(values)>self.r['restricted_limit_kmh']+1e-6:
                self.phase='LIMITED';self.fail('restricted_speed_exceeded');return
            if min(values)<self.r['restricted_speed_min_kmh']-1e-6:
                self.phase='LIMITED';self.fail('restricted_speed_too_low');return
        if self.C is not None and s>=self.C and p<=self.D:
            if max(interp(max(p,self.C)),interp(min(s,self.D)))>self.r['restored_limit_kmh']+1e-6:
                self.phase='RECOVERY';self.fail('restored_limit_exceeded');return
        if p<self.B and min(s,self.B)>=lo and interp(min(s,self.B))>self.r['initial_limit_kmh']+1e-6:
            self.fail('initial_limit_exceeded');return
        self.phase=('PREPARE' if s<lo else 'APPROACH' if s<self.B else
                    'LIMITED' if s<limited_end else 'RECOVERY' if s<self.D else 'END_MARGIN')
        if obs.get('at_end'):
            required={'B','D'} | ({'C'} if self.C is not None else set())
            if not self.checked_D or not self.approach_complete or not required<=self.passed:
                self.invalidate('route_ended_without_required_events');return
            self.route_complete=True;self.phase='COMPLETE';self.done=True
        elif t>=self.d['timeout']:
            self.fail('precondition_timeout' if not self.approach_complete else 'scenario_timeout')
        elif t-self.progress_time>=self.stall_timeout:
            self.fail('stalled')
        self.previous=dict(obs)

    def result(self):
        valid=not self.invalid
        passed=self.done and self.route_complete and not self.failures and valid
        return {'result_status':'INVALID' if not valid else 'PASS' if passed else 'FAILED',
                'trial_valid':valid,'pass':passed if valid else None,'phase':self.phase,
                'failure_reasons':list(self.failures),'invalid_reasons':list(self.invalid),
                'failure_stage':self.phase if self.failures else None,
                'response_evaluation':'EVALUATED' if self.approach_complete and self.approach_min is not None and self.approach_min>=self.r['minimum_approach_speed_kmh'] else 'NOT_EVALUATED',
                'route_complete':self.route_complete,'approach_established':self.approach_complete and
                self.approach_min is not None and self.approach_min>=self.r['minimum_approach_speed_kmh'],
                'approach_minimum_speed_kmh':self.approach_min,'final_check_completed':self.checked_D,
                'events':self.events,'scoring_version':VERSION,
                'evaluation_scope':'virtual_rule_screening','visual_sign_recognition_tested':False,
                'physical_field_test_compliance_claimed':False}
