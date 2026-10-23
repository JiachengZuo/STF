"""A curved CARLA connector is a legitimate 6c approach before the station."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import carla

from station_stop_6c import (geometry_metrics, make_baseline, trace_straight_approach)
from station_stop_6c_runtime import StationStopScene
from test_station_stop_6c import fixture


class CurvedMap:
    name = 'offline_station'

    def __init__(self):
        samples = ([(float(x), 0., 0., 67) for x in range(61)]
                   + [(61., 0., 0., 752), (62., -.1, -10., 752),
                      (63., -.5, -25., 752), (63.5, -1.1, -45., 752),
                      (63.9, -1.8, -55., 752), (64.4, -2.6, -45., 752),
                      (65.2, -3.3, -25., 752), (66., -3.8, -10., 752)]
                   + [(float(x), -4., 0., 14) for x in range(67, 91)])
        self.points = []
        for i, (x, y, yaw, road) in enumerate(samples):
            w = SimpleNamespace(transform=carla.Transform(carla.Location(x=x, y=y, z=0.),
                                                           carla.Rotation(yaw=yaw)),
                                road_id=road, section_id=0, lane_id=-1, lane_width=4., s=float(i))
            self.points.append(w)
        for i, w in enumerate(self.points):
            w.next = lambda _step, i=i: [self.points[i + 1]] if i + 1 < len(self.points) else []

    def to_opendrive(self):
        return 'connected synthetic curve'

    def get_waypoint(self, location, project_to_road, lane_type):
        w = min(self.points, key=lambda p: p.transform.location.distance(location))
        if w.transform.location.distance(location) > .8:
            return None
        return w

    def anchors(self):
        indices = [0, 40, 60, 67, 88]
        return [dict(x=self.points[i].transform.location.x,
                     y=self.points[i].transform.location.y, z=0.,
                     yaw=self.points[i].transform.rotation.yaw) for i in indices]


class CurvedConnectorTests(unittest.TestCase):
    def setUp(self):
        self.map = CurvedMap()
        self.anchors = self.map.anchors()

    def baseline(self):
        cfg, reference = fixture()
        draft = dict(ego_start=self.anchors[0], route_anchors=self.anchors[1:],
                     station_edge=[dict(x=90., y=-2., z=0.),
                                   dict(x=110., y=-2., z=0.)],
                     stop_pose=dict(x=100., y=-3.2, z=0.),
                     vehicle_geometry=reference['vehicle_geometry'], ground_evidence={})
        return make_baseline(self.map, draft, cfg), cfg

    def test_connected_curve_passes_with_large_midpoint_heading_change(self):
        route = trace_straight_approach(self.map, self.anchors)
        self.assertEqual(route[-1]['x'], self.anchors[-1]['x'])
        self.assertEqual({p['road_id'] for p in route}, {67, 752, 14})
        self.assertLess(min(p['yaw'] for p in route), -40.)

    def test_curve_then_straight_station_approach_has_valid_geometry(self):
        baseline, cfg = self.baseline()
        self.assertEqual(baseline['route_geometry_source'],
                         'directed_topology_with_verified_straight_fallback')
        self.assertGreater(baseline['geometry']['docking_length_m'], 10.)
        self.assertEqual(geometry_metrics(baseline, cfg['engineering']), baseline['geometry'])

    def test_curved_road_before_pn_is_not_outside_station_rectangle(self):
        baseline, cfg = self.baseline()
        definition = dict(baseline=baseline, ego_route=baseline['ego_route'],
                          engineering=cfg['engineering'], evaluation=cfg['evaluation'])
        scene = StationStopScene(Mock(), definition, 'behavior', None, 'hybrid')
        actor = Mock()
        actor.get_transform.return_value = carla.Transform(carla.Location(x=30., y=0., z=0.),
                                                            carla.Rotation(yaw=0.))
        actor.get_velocity.return_value = carla.Vector3D(x=2.)
        actor.bounding_box = carla.BoundingBox(carla.Location(z=.8),
                                               carla.Vector3D(x=2.4, y=.9, z=.8))
        scene.ego = actor
        _, _, _, outside = scene.observation()
        self.assertFalse(outside)

    def test_connector_cannot_be_replaced_by_a_straight_chord(self):
        self.map.points[60].next = lambda _step: []
        with self.assertRaisesRegex(ValueError, 'P2→P3'):
            trace_straight_approach(self.map, self.anchors)


if __name__ == '__main__':
    unittest.main()
