"""No-server coverage of permissive drawing and topology-based target extension."""
import math
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from cross_right_2d import load_settings, route_intersection
from cross_right_2d_routes import extend_target_upstream, route_samples, trace_waypoints
from test_cross_right_2d import ROOT, fake_runtime


class Location:
    def __init__(self, x=0., y=0., z=0.):
        self.x, self.y, self.z = x, y, z

    def distance(self, other):
        return math.dist((self.x, self.y, self.z), (other.x, other.y, other.z))

    def __sub__(self, other):
        return Location(self.x-other.x, self.y-other.y, self.z-other.z)


class Waypoint:
    def __init__(self, s, road=1):
        self.s, self.road_id = s, road
        self.section_id, self.lane_id = 0, -1
        self.lane_width, self.is_junction = 2.7, False
        yaw = 0. if road == 1 else -90.
        location = Location(x=s-100.) if road == 1 else Location(y=40.-s)
        self.transform = SimpleNamespace(location=location, rotation=SimpleNamespace(yaw=yaw),
            get_forward_vector=lambda: Location(x=math.cos(math.radians(yaw)), y=math.sin(math.radians(yaw))))

    def next(self, distance):
        return [Waypoint(self.s+distance, self.road_id)] if self.s+distance <= 200 else []

    def previous(self, distance):
        return [Waypoint(max(0., self.s-distance), self.road_id)] if self.s > 1e-8 else []

    def get_junction(self):
        return None


class Map:
    name = 'short_routes'

    def get_waypoint(self, location, **kwargs):
        return Waypoint(location.x+100., 1) if abs(location.y) < 1e-8 else Waypoint(40.-location.y, 2)

    def to_opendrive(self):
        return '<OpenDRIVE><road id="1"><type s="0"><speed max="50" unit="km/h"/></type></road></OpenDRIVE>'


class RouteTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fake_runtime()
        self.carla = self.runtime.carla
        self.carla.Location = Location
        self.carla.LaneType = SimpleNamespace(Driving=1)
        self.cfg = load_settings(ROOT/'config/cross_right_2d.yaml')

    def test_start_and_end_only_and_close_anchors(self):
        with patch.dict(sys.modules, {'carla': self.carla}):
            route = trace_waypoints(Map(), [dict(x=-10., y=0., z=0.), dict(x=-9.5, y=0., z=0.)])
        self.assertEqual(len(route), 2)
        self.assertAlmostEqual(route[-1].s-route[0].s, .5)

    def test_unreachable_forward_route_is_still_reported(self):
        with patch.dict(sys.modules, {'carla': self.carla}), self.assertRaisesRegex(ValueError, '无法沿当前车道'):
            trace_waypoints(Map(), [dict(x=10., y=0., z=0.), dict(x=5., y=0., z=0.)])

    def test_upstream_extension_uses_real_waypoints_and_stops_at_road_start(self):
        original = [Waypoint(40.), Waypoint(41.)]
        extended = extend_target_upstream(original, 20.5)
        self.assertAlmostEqual(extended[0].s, 19.5)
        self.assertEqual(extended[-2:], original)
        self.assertEqual(extend_target_upstream([Waypoint(2.), Waypoint(3.)], 10.)[0].s, 0.)

    def capture(self, signal_query_fails=False):
        def actors():
            if signal_query_fails:
                raise RuntimeError('query unavailable')
            return SimpleNamespace(filter=lambda _: [])
        world = SimpleNamespace(get_map=lambda: Map(), get_actors=actors)
        ego = [dict(x=-10., y=0., z=0.), dict(x=10., y=0., z=0.)]
        target = [dict(x=0., y=10., z=0.), dict(x=0., y=-5., z=0.)]
        with patch.dict(sys.modules, {'carla': self.carla}):
            return self.runtime.capture_baseline(world, ego, target, self.cfg)

    def test_capture_saves_short_narrow_nonjunction_routes_with_automatic_wait_pose(self):
        b = self.capture()
        self.assertEqual(len(b['ego_anchors']), 2)
        self.assertEqual(len(b['target_anchors']), 2)
        self.assertAlmostEqual(b['geometry']['target_wait_s'], 0.)
        self.assertAlmostEqual(b['geometry']['target_wait_pose']['y'], 20/3.6*5.5)
        self.assertAlmostEqual(b['geometry']['ego_half_width_m'], 1.35)
        self.assertIsNone(b['geometry']['directed_arm_count'])
        self.assertTrue(b['geometry']['signal_query_clear'])
        self.assertFalse(b['geometry']['section_speed_limit_verified'])
        self.assertEqual(b['geometry']['exit_gate_s'], b['geometry']['ego_length_m'])

    def test_missing_optional_signal_query_does_not_block_save(self):
        self.assertIsNone(self.capture(signal_query_fails=True)['geometry']['signal_query_clear'])


if __name__ == '__main__':
    unittest.main()
