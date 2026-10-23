"""CARLA BehaviorAgent adapter. Its inputs contain no scenario or scorer state."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

from cross_right_2d import project, stations


def agents_directory():
    spec = importlib.util.find_spec('agents')
    if spec and spec.submodule_search_locations:
        for location in spec.submodule_search_locations:
            path = Path(location)
            if (path/'navigation/behavior_agent.py').is_file():
                return path
    roots = [Path(os.environ[key]).expanduser() for key in
             ('CARLA_UE4_ROOT', 'CARLA_0916_ROOT', 'CARLA_ROOT') if os.environ.get(key)]
    roots.append(Path.home()/'Carla/carla')
    for root in roots:
        path = root/'PythonAPI/carla/agents'
        if (path/'navigation/behavior_agent.py').is_file():
            sys.path.insert(0, str(path.parent))
            return path
    raise RuntimeError('CARLA BehaviorAgent unavailable; set CARLA_UE4_ROOT to the CARLA installation')


def source_identity():
    path = agents_directory()
    files = {str(p.relative_to(path)): hashlib.sha256(p.read_bytes()).hexdigest()
             for directory in ('navigation', 'tools') for p in sorted((path/directory).glob('*.py'))}
    return dict(implementation='carla_behavior_agent', directory=str(path), files=files,
                sha256=hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest())


class VehicleWithSectionLimit:
    """Expose the experiment's declared road limit, delegating all live state."""
    def __init__(self, vehicle, speed_limit_kmh):
        self.vehicle, self.speed_limit_kmh = vehicle, float(speed_limit_kmh)

    def get_speed_limit(self):
        return self.speed_limit_kmh

    def __getattr__(self, name):
        return getattr(self.vehicle, name)


class BehaviorController:
    def __init__(self, ego, route, target_speed_kmh, speed_limit_kmh, dt):
        agents_directory()
        import carla
        from agents.navigation.behavior_agent import BehaviorAgent
        from agents.navigation.local_planner import RoadOption

        class ObservedAgent(BehaviorAgent):
            """Record decisions without changing upstream detection or controls."""
            def collision_and_car_avoid_manager(self, waypoint):
                result = super().collision_and_car_avoid_manager(waypoint)
                found, actor, distance = result
                self.evidence['vehicle_hazard'] = dict(detected=bool(found),
                    actor_id=actor.id if actor else None, distance_m=float(distance))
                return result

            def pedestrian_avoid_manager(self, waypoint):
                result = super().pedestrian_avoid_manager(waypoint)
                found, actor, distance = result
                self.evidence['pedestrian_hazard'] = dict(detected=bool(found),
                    actor_id=actor.id if actor else None, distance_m=float(distance))
                return result

            def traffic_light_manager(self):
                result = super().traffic_light_manager()
                self.evidence['red_light'] = bool(result)
                return result

            def emergency_stop(self):
                self.evidence['emergency_stop'] = True
                return super().emergency_stop()

            def run_step(self, debug=False):
                self.evidence = dict(emergency_stop=False)
                return super().run_step(debug=debug)

        self.agent = ObservedAgent(VehicleWithSectionLimit(ego, speed_limit_kmh),
                                   behavior='normal', opt_dict={'dt': dt})
        # BehaviorAgent's run_step overwrites set_target_speed(). Configure its
        # cruise ceiling instead; obstacle following can still request less.
        self.agent._behavior.max_speed = float(target_speed_kmh)
        self.agent._behavior.speed_lim_dist = 0.
        location = ego.get_location()
        start_s = project(dict(x=location.x, y=location.y), route)['s']
        plan = []
        for p, s in zip(route, stations(route)):
            if s < start_s:
                continue
            wp = self.agent._map.get_waypoint(carla.Location(x=p['x'], y=p['y'], z=p['z']))
            if wp is None:
                raise RuntimeError('behavior_route_waypoint_missing')
            plan.append((wp, RoadOption.STRAIGHT if wp.is_junction else RoadOption.LANEFOLLOW))
        if not plan:
            raise RuntimeError('behavior_route_has_no_forward_waypoints')
        self.agent.set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)
        self.last_debug = {}
        self.metadata = dict(implementation='carla_behavior_agent', profile='normal',
            requested_cruise_speed_kmh=float(target_speed_kmh),
            section_speed_limit_kmh=float(speed_limit_kmh), speed_lim_dist_kmh=0.,
            decision_inputs=['ego_state', 'surrounding_actor_states', 'map', 'ego_route', 'speed_request'],
            uses_camera_perception=False, reads_scenario_state=False, modifies_agent_actions=False)

    def run_step(self):
        control = self.agent.run_step()
        self.last_debug = dict(self.agent.evidence,
            planned_speed_kmh=self.agent.get_local_planner()._target_speed,
            final_throttle=control.throttle, final_steer=control.steer, final_brake=control.brake)
        return control
