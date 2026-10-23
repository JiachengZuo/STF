import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import Mock
import carla
import unittest

from station_stop_6c import (StationStopEvaluator, build_definitions, generation_spec, geometry_metrics, load_settings, reference_control,
                            reference_path, seal, spawn_for_distance, target_pose, trace_straight_approach, validate_definitions)
from station_stop_6c_cli import artifact_paths, dispatch_arguments, expand, summarize, recover_records, write_json

ROOT = Path(__file__).resolve().parents[1]


def fixture():
    cfg = load_settings(ROOT / 'config/station_stop_6c.yaml')
    route = [dict(x=float(i), y=0., z=0., yaw=0., width=4., road_id=1, lane_id=-1) for i in range(81)]
    b = dict(schema_version=1, scenario_code='6c', map_name='offline_station', opendrive_sha256='offline',
             ego_blueprint=cfg['ego_blueprint'], ego_start=route[0], ego_route=route,
             route_anchors=[route[40], route[80]], stop_pose=dict(x=100., y=.8, z=0., yaw=0.),
             station_edge=[dict(x=90., y=2., z=0.), dict(x=110., y=2., z=0.)],
             vehicle_geometry=dict(extent=dict(x=2.4, y=.9, z=.8), location=dict(x=0., y=0., z=.8), wheelbase_m=2.8))
    b['geometry'] = geometry_metrics(b, cfg['engineering'])
    return cfg, seal(b, 'baseline_fingerprint')


_CACHED_DEFINITION = None


def definition():
    global _CACHED_DEFINITION
    if _CACHED_DEFINITION is None:
        cfg, b = fixture()
        _CACHED_DEFINITION = build_definitions(b, generation_spec(cfg))[0]
    return copy.deepcopy(_CACHED_DEFINITION)


class StraightApproachMap:
    """Connected road surface with deliberately disconnected OpenDRIVE next()."""
    def __init__(self, gap=None):
        self.gap = gap
        self.name = 'offline_station'

    def to_opendrive(self):
        return 'synthetic OpenDRIVE road-ID seam'

    def get_waypoint(self, location, project_to_road, lane_type):
        x, y = location.x, location.y
        if x < 0 or x > 120 or abs(y) > 5:
            return None
        if not project_to_road and self.gap and self.gap[0] < x < self.gap[1]:
            return None
        center = 0. if abs(y) < 1.75 else 3.5 if y > 0 else -3.5
        yaw = 0. if center == 0 else 180.
        waypoint = SimpleNamespace(
            transform=SimpleNamespace(location=carla.Location(x=x, y=center, z=0.),
                                      rotation=carla.Rotation(yaw=yaw)),
            lane_width=4., road_id=1 if x < 40 else 2,
            section_id=0, lane_id=-1 if center == 0 else 1, s=float(x))
        waypoint.next = Mock(return_value=[])  # Deliberately disconnected straight-road seam.
        return waypoint


def anchor(x, y=0, yaw=0):
    return dict(x=float(x), y=float(y), z=0., yaw=float(yaw))


class StraightTopologyTests(unittest.TestCase):
    def test_continuous_straight_lane_across_road_id_seam(self):
        carla_map = StraightApproachMap()
        points = trace_straight_approach(carla_map, [anchor(0), anchor(40), anchor(80)])
        self.assertGreater(len(points), 100)
        self.assertEqual({p['road_id'] for p in points}, {1, 2})
        self.assertEqual(points[-1]['x'], 80.)

    def test_baseline_accepts_physically_continuous_road_id_seam(self):
        cfg, original = fixture()
        draft = dict(ego_start=original['ego_start'], route_anchors=original['route_anchors'],
                     station_edge=original['station_edge'], stop_pose=original['stop_pose'],
                     vehicle_geometry=original['vehicle_geometry'], ground_evidence={})
        from station_stop_6c import make_baseline
        b = make_baseline(StraightApproachMap(), draft, cfg)
        self.assertEqual(b['route_geometry_source'], 'directed_topology_with_verified_straight_fallback')
        self.assertGreater(len(b['ego_route']), 100)

    def test_e_anchor_extends_upstream_to_cover_farthest_start(self):
        cfg, original = fixture()
        draft = dict(ego_start=anchor(40), route_anchors=[anchor(60), anchor(80)],
                     station_edge=original['station_edge'], stop_pose=original['stop_pose'],
                     vehicle_geometry=original['vehicle_geometry'], ground_evidence={})
        from station_stop_6c import make_baseline
        b = make_baseline(StraightApproachMap(), draft, cfg)
        self.assertEqual(b['drawn_ego_start']['x'], 40.)
        self.assertLessEqual(b['ego_start']['x'], 30.)
        self.assertAlmostEqual(spawn_for_distance(b, 70)[0]['x'], 30.)
        self.assertAlmostEqual(spawn_for_distance(b, 40)[0]['x'], 60.)

    def test_upstream_gap_is_reported_instead_of_inventing_a_spawn(self):
        cfg, original = fixture()
        draft = dict(ego_start=anchor(40), route_anchors=[anchor(60), anchor(80)],
                     station_edge=original['station_edge'], stop_pose=original['stop_pose'],
                     vehicle_geometry=original['vehicle_geometry'], ground_evidence={})
        from station_stop_6c import make_baseline
        with self.assertRaisesRegex(ValueError, r'E 上游.*无连续的顺行 Driving 车道'):
            make_baseline(StraightApproachMap(gap=(35, 36)), draft, cfg)

    def test_missing_driving_road_reports_specific_segment_and_location(self):
        with self.assertRaisesRegex(ValueError, r'E→P1.*25\.'):
            trace_straight_approach(StraightApproachMap(gap=(25, 26)), [anchor(0), anchor(40)])

    def test_reversed_and_neighboring_lanes_are_rejected(self):
        m = StraightApproachMap()
        with self.assertRaisesRegex(ValueError, 'P1→P2'):
            trace_straight_approach(m, [anchor(0), anchor(40), anchor(30)])
        with self.assertRaisesRegex(ValueError, 'E→P1.*方向相反.*road/lane'):
            trace_straight_approach(m, [anchor(0), anchor(40, 3.5, 180)])


class GeometryAndMatrixTests(unittest.TestCase):
    def test_960_unique_conditions_same_goal_and_four_actual_starts(self):
        cfg, baseline = fixture()
        rows = build_definitions(baseline, generation_spec(cfg))
        self.assertEqual(len(rows), 960)
        self.assertEqual(len({r['condition_fingerprint'] for r in rows}), 960)
        self.assertEqual(len({r['profile_id'] for r in rows}), 8)
        self.assertEqual({r['ego_start']['x'] for r in rows}, {30., 40., 50., 60.})
        for r in rows:
            self.assertEqual(r['baseline'], baseline)
            self.assertEqual(r['ego_route'][0], r['ego_start'])
            self.assertEqual(r['ego_route'][-1]['x'], 80.)
            self.assertTrue(all(p['x'] >= r['ego_start']['x'] for p in r['ego_route']))
            self.assertAlmostEqual(100 - r['ego_start']['x'], r['initial_distance_m'])

    def test_missing_start_is_rejected_without_clamping(self):
        cfg, b = fixture()
        b['ego_route'] = b['ego_route'][40:]
        with self.assertRaisesRegex(ValueError, '未覆盖'):
            spawn_for_distance(b, 70)

    def test_station_goal_keeps_lateral_offset_and_forward_heading(self):
        _, b = fixture()
        pose = target_pose(*reversed(b['station_edge']), b['stop_pose'], 0)
        self.assertAlmostEqual(pose['yaw'] % 360, 0.)
        self.assertEqual(pose['y'], .8)

    def test_short_station_edge_only_needs_to_contain_goal(self):
        cfg, b = fixture()
        b['station_edge'] = [dict(x=98.4, y=2., z=0.), dict(x=101.6, y=2., z=0.)]
        metrics = geometry_metrics(b, cfg['engineering'])
        self.assertEqual(metrics, {'docking_length_m': 20.})
        # The car spans x=97.6..102.4, beyond both A and B.
        self.assertLess(b['stop_pose']['x'] - b['vehicle_geometry']['extent']['x'], 98.4)
        self.assertGreater(b['stop_pose']['x'] + b['vehicle_geometry']['extent']['x'], 101.6)

    def test_goal_must_be_within_station_edge_segment(self):
        cfg, b = fixture()
        b['station_edge'] = [dict(x=90., y=2., z=0.), dict(x=95., y=2., z=0.)]
        with self.assertRaisesRegex(ValueError, '停车目标 G'):
            geometry_metrics(b, cfg['engineering'])

    def test_goal_and_vehicle_may_cross_station_reference_line(self):
        cfg, b = fixture()
        for y in (1.5, 2.):
            b['stop_pose']['y'] = y
            self.assertEqual(geometry_metrics(b, cfg['engineering']),
                             {'docking_length_m': 20.})

    def test_station_reference_may_be_left_of_approach_lane(self):
        cfg, b = fixture()
        b['station_edge'] = [dict(x=90., y=-2., z=0.), dict(x=110., y=-2., z=0.)]
        b['stop_pose']['y'] = -.8
        b['geometry'] = geometry_metrics(b, cfg['engineering'])
        self.assertEqual(b['geometry'], {'docking_length_m': 20.})
        d = definition()
        d['baseline'] = b
        path = reference_path(d)
        self.assertAlmostEqual(path[-1]['y'], -.8)
        self.assertLess(reference_control(d['ego_route'][-1], 2., d, path)[1], 0.)

    def test_incompatible_vehicle_or_misleading_scope_rejected(self):
        cfg, b = fixture()
        cfg['ego_blueprint'] = 'vehicle.audi.tt'
        with self.assertRaises(ValueError):
            build_definitions(b, generation_spec(cfg))
        cfg['evaluation']['scope'] = 'full_gbt'
        with self.assertRaises(ValueError):
            generation_spec(cfg)

    def test_rotated_geometry_uses_station_axis(self):
        cfg, b = fixture()
        def rotate(p):
            p['x'], p['y'] = -p['y'] + 12., p['x'] - 18.
            if 'yaw' in p: p['yaw'] += 90.
        for p in b['ego_route']: rotate(p)
        for p in b['station_edge']: rotate(p)
        rotate(b['stop_pose'])
        metrics = geometry_metrics(b, cfg['engineering'])
        self.assertAlmostEqual(metrics['docking_length_m'], 20.)
        p, _ = spawn_for_distance(b, 40)
        self.assertAlmostEqual(p['x'], 12.)
        self.assertAlmostEqual(p['y'], 42.)

    def test_expansion_is_immutable_and_runtime_uses_archived_spec(self):
        cfg, b = fixture()
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); path = root / 'baseline.json'; path.write_text(json.dumps(b))
            args = SimpleNamespace(baseline=str(path), input_dir=str(root / 'definitions'), dry_run=False)
            expand(args, cfg)
            rows, manifest = validate_definitions(args.input_dir)
            self.assertEqual(len(rows), 960)
            cfg['evaluation']['timeout_s'] = 500  # Does not reinterpret an already saved batch.
            reread, _ = validate_definitions(args.input_dir)
            self.assertEqual(reread[0]['evaluation']['timeout_s'], 60)
            with self.assertRaises(ValueError): expand(args, cfg)
            victim = Path(args.input_dir) / (rows[0]['scenario'] + '.json')
            changed = json.loads(victim.read_text()); changed['initial_speed_kmh'] = 19.
            victim.write_text(json.dumps(seal(changed, 'condition_fingerprint')))
            with self.assertRaises(ValueError): validate_definitions(args.input_dir)


class ScoringTests(unittest.TestCase):
    def setUp(self):
        self.d = definition(); self.goal = self.d['baseline']['stop_pose']
        self.trial = StationStopEvaluator(self.d)
        self.trial.start(0., 100)
        self.frame = 100; self.now = 0.

    def sample(self, pose=None, speed=0., signed=None, **kwargs):
        self.trial.update(self.now, self.frame, pose or self.goal, speed,
                          speed if signed is None else signed, **kwargs)
        self.frame += 1; self.now = round(self.now + .05, 8)

    def hold(self, seconds=2.2, **kwargs):
        for _ in range(round(seconds / .05)):
            self.sample(**kwargs)

    def test_first_valid_stop_and_required_hold_pass(self):
        self.hold()
        self.assertEqual(self.trial.status, 'PASS')
        self.assertEqual(self.trial.first_stop['sim_time'], 0.)
        self.assertEqual(self.trial.result()['standard_checks']['door_open_delay'], 'NOT_EVALUATED')

    def test_each_error_axis_and_early_stop_fail_first_attempt(self):
        for key, offset, reason in [('x', -10, 'stop_longitudinal_error'),
                                    ('x', 1, 'stop_longitudinal_error'),
                                    ('y', .3, 'stop_lateral_error'),
                                    ('yaw', 6, 'stop_heading_error')]:
            self.setUp(); pose = dict(self.goal); pose[key] += offset
            self.hold(1.2, pose=pose)
            self.assertEqual(self.trial.status, 'FAILED')
            self.assertIn(reason, self.trial.reasons)
            self.hold(pose=self.goal)
            self.assertEqual(self.trial.status, 'FAILED')

    def test_hold_must_remain_inside_each_goal_tolerance(self):
        for axis, initial, drift in (('x', .49, .02), ('y', .19, .02), ('yaw', 4.9, .2)):
            with self.subTest(axis=axis):
                self.setUp()
                pose = dict(self.goal)
                pose[axis] += initial
                self.hold(1.1, pose=pose)
                self.assertEqual(self.trial.phase, 'HOLD')
                pose[axis] += drift
                self.sample(pose=pose)
                self.assertEqual(self.trial.status, 'FAILED')
                self.assertEqual(self.trial.reasons, ['pose_outside_tolerance_during_hold'])

    def test_stop_then_reposition_fails(self):
        self.hold(1.2)
        self.assertEqual(self.trial.phase, 'HOLD')
        self.sample(speed=.3)
        self.assertEqual(self.trial.reasons, ['moved_after_first_stop'])

    def test_creeping_does_not_pass(self):
        self.trial.e['timeout_s'] = 3.
        for i in range(70):
            p = dict(self.goal, x=self.goal['x'] + i * .004)
            self.sample(pose=p, speed=.08)
        self.assertEqual(self.trial.status, 'FAILED')
        self.assertIn('task_timeout', self.trial.reasons)

    def test_reverse_requires_motion_not_gear_bit(self):
        self.hold(.3, speed=.2, signed=-.2)
        self.assertEqual(self.trial.reasons, ['reverse_adjustment'])

    def test_collision_and_outside_fail_and_survive_later_platform_error(self):
        for kwargs in (dict(collision=True), dict(outside=True), dict(solid_line=True)):
            self.setUp(); self.sample(speed=2., **kwargs)
            self.assertEqual(self.trial.status, 'FAILED')
            self.trial.end('INVALID', 'cleanup_failed')
            self.assertEqual(self.trial.status, 'FAILED')

    def test_frame_gap_invalid_and_nonfinite_invalid(self):
        self.sample(speed=2.)
        self.frame += 1; self.sample(speed=2.)
        self.assertEqual(self.trial.status, 'INVALID')
        self.setUp(); self.sample(speed=float('nan'))
        self.assertEqual(self.trial.status, 'INVALID')

    def test_late_collision_revises_provisional_pass(self):
        self.hold(); self.trial.end('FAILED', 'collision')
        self.assertEqual(self.trial.status, 'FAILED')

    def test_never_stops_is_failed_not_invalid(self):
        self.trial.e['timeout_s'] = .2
        self.hold(.4, speed=1.)
        self.assertEqual(self.trial.reasons, ['task_timeout'])

    def test_spawn_rest_cannot_be_scored_before_start(self):
        trial = StationStopEvaluator(self.d)
        with self.assertRaises(ValueError): trial.update(0, 1, self.goal, 0, 0)


class ControllerAndEntrypointTests(unittest.TestCase):
    def test_reference_terminal_brake_and_smooth_path(self):
        d = definition(); path = reference_path(d)
        self.assertEqual(path[-1]['x'], d['baseline']['stop_pose']['x'])
        self.assertEqual(path[-1]['y'], d['baseline']['stop_pose']['y'])
        self.assertEqual(reference_control(d['baseline']['stop_pose'], 0., d, path)[2], 1.)
        self.assertTrue(all(a['x'] < b['x'] for a, b in zip(path, path[1:])))

    def test_reference_steering_uses_the_archived_vehicle_limit(self):
        d = definition()
        path = reference_path(d)
        d['baseline']['vehicle_geometry']['max_steer_angle_deg'] = 35.
        old = reference_control(d['ego_route'][-1], 2., d, path)[1]
        d['baseline']['vehicle_geometry']['max_steer_angle_deg'] = 70.
        measured = reference_control(d['ego_route'][-1], 2., d, path)[1]
        self.assertGreater(old, 0.)
        self.assertAlmostEqual(measured, old / 2.)

    def test_video_overlay_stays_at_goal_near_route_end(self):
        from station_stop_6c_video import lookahead_xy
        path = reference_path(definition())
        points = lookahead_xy(path, path[-1])
        self.assertEqual(len(points), 12)
        self.assertTrue(all(abs(x - path[-1]['x']) < 1e-6
                            and abs(y - path[-1]['y']) < 1e-6 for x, y in points))

    def test_raw_planner_preserves_off_center_goal(self):
        from station_stop_6c_runtime import CoordinatePlanner
        d = definition(); goal = d['baseline']['stop_pose']
        planner = CoordinatePlanner(d['ego_route'], goal)
        planner.cursor = len(planner.points) - 1
        p, command = planner.run_step(SimpleNamespace(x=goal['x'], y=0))
        self.assertEqual(list(p), [goal['x'], goal['y']])
        self.assertEqual(command, 4)

    def test_dispatch_and_help_are_independent(self):
        for argv in (['-6c'], ['--scenario', '6.c'], ['--scenario=6C']):
            self.assertEqual(dispatch_arguments(argv), [])
        self.assertIsNone(dispatch_arguments(['--scenario', '5e']))
        for argv in (['-6c', '-5e'], ['-6c', '--scenario', '2b']):
            with self.assertRaises(ValueError): dispatch_arguments(argv)
        for entry in ('waypoints_stf.py', 'run_stf.py'):
            r = subprocess.run([sys.executable, '-B', str(ROOT / entry), '-6c', '--help'],
                               capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn('--tcp-mode', r.stdout)

    def test_recover_finalized_failed_attempt_without_rerunning_it(self):
        d = definition()
        row = seal(dict(scenario=d['scenario'], attempt=2, profile_id=d['profile_id'], weather_id=d['weather_id'],
                        condition_fingerprint=d['condition_fingerprint'], controller='behavior_reference_parking',
                        result_status='FAILED', reasons=['collision']), 'result_fingerprint')
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            paths = artifact_paths(out, d, 2)
            self.assertEqual(paths['video'].relative_to(out).parts,
                             (d['scenario'] + '_attempt_2.mp4',))
            self.assertEqual(paths['result'].relative_to(out).parts,
                             ('details', d['scenario'] + '_attempt_2.json'))
            write_json(paths['condition'], d)
            write_json(paths['result'], row)
            records = recover_records(out, [], [d], 'behavior_reference_parking')
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0]['result_status'], 'FAILED')
            again = recover_records(out, records, [d], 'behavior_reference_parking')
            self.assertEqual(len(again), 1)

    def test_summary_uses_conditions_not_attempt_count(self):
        rows = [dict(scenario='a', result_status='INVALID'), dict(scenario='a', result_status='PASS'),
                dict(scenario='b', result_status='FAILED')]
        summary = summarize(rows, 3)
        self.assertEqual(summary['counts'], dict(PASS=1, FAILED=1, INVALID=0))
        self.assertEqual(summary['attempts'], 3)
        self.assertEqual(summary['pass_rate_valid'], .5)
        self.assertEqual(summary['unattempted_conditions'], 1)


if __name__ == '__main__':
    unittest.main()
