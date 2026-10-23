"""Optional real CARLA client geometry test; no server, rendering or vehicles."""

import hashlib
import math
import unittest
from unittest import mock

try:
    import carla
except ImportError:
    carla = None

from tests.test_tunnel_2a import valid_tunnel_config
from tunnel_2a import tunnel_condition_fingerprint, tunnel_route_fingerprint
from tunnel_expansion import expand_baseline, read_approach_geometry


OPENDRIVE = '''<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="TunnelGeometryFixture" version="1" date="" north="0" south="0" east="0" west="0"><geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs]]></geoReference></header>
  <road name="two_lane_approach" length="400" id="1" junction="-1">
    <planView>
      <geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry>
      <geometry s="100" x="100" y="0" hdg="0" length="300"><arc curvature="0.002"/></geometry>
    </planView>
    <elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile>
    <lanes><laneSection s="0">
      <center><lane id="0" type="none" level="false"><roadMark sOffset="0" type="solid" weight="standard" color="white" width="0.15"/></lane></center>
      <right>
        <lane id="-1" type="driving" level="false"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><roadMark sOffset="0" type="broken" weight="standard" color="white" width="0.15"/></lane>
        <lane id="-2" type="driving" level="false"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><roadMark sOffset="0" type="solid" weight="standard" color="white" width="0.15"/></lane>
      </right>
    </laneSection></lanes>
  </road>
</OpenDRIVE>'''


@unittest.skipUnless(carla is not None and hasattr(carla, "Map"), "CARLA Python API unavailable")
class CarlaTunnelGeometryTest(unittest.TestCase):
    def test_real_client_topology_and_arc_distance_expansion_without_server(self):
        import tunnel_geometry
        carla_map = carla.Map("TunnelGeometryFixture", OPENDRIVE)

        def point(s):
            tf = carla_map.get_waypoint_xodr(1, -1, s).transform
            return dict(x=tf.location.x, y=tf.location.y, z=tf.location.z, yaw=tf.rotation.yaw)

        baseline = valid_tunnel_config()
        baseline["ego_start"] = point(20)
        baseline["ego_start"]["z"] += 0.3
        baseline["ego_route"] = [point(s) for s in (40, 150, 225, 300, 350)]
        section = baseline["tunnel_test"]
        section["map_name"] = "TunnelGeometryFixture"
        section["opendrive_sha256"] = hashlib.sha256(carla_map.to_opendrive().encode()).hexdigest()
        section["route_fingerprint"] = tunnel_route_fingerprint(baseline)
        section["condition_fingerprint"] = tunnel_condition_fingerprint(baseline)
        # Runtime unit tests can have imported the geometry module with a
        # CARLA stub. This scoped patch ensures this test uses the real API.
        with mock.patch.object(tunnel_geometry, "carla", carla):
            approach = read_approach_geometry(carla_map, baseline)
            configs, summary = expand_baseline(baseline, approach, list(range(20, 101, 10)))
        self.assertEqual(summary["condition_count"], 1080)
        for config in configs[::120]:
            start = config["ego_start"]
            waypoint = carla_map.get_waypoint(carla.Location(x=start["x"], y=start["y"], z=start["z"]))
            self.assertEqual(waypoint.lane_id, -1)
            # Independent analytic length: 100 m straight, then a radius-500
            # left curve whose right lane centre has radius 501.75 m.
            straight = max(0.0, 100 - waypoint.s)
            curved = (150 - max(100, waypoint.s)) * (501.75 / 500)
            requested = config["tunnel_test"]["expansion"]["start_distance_m"]
            self.assertAlmostEqual(straight + curved, requested, delta=0.1)
            # Check lateral placement analytically; get_waypoint's nearest
            # road lookup also has a small longitudinal discretization error.
            lateral = (abs(math.hypot(start["x"] - 100, start["y"] + 500) - 501.75)
                       if start["x"] >= 100 else abs(start["y"] - 1.75))
            self.assertLess(lateral, 0.01)


if __name__ == "__main__":
    unittest.main()
