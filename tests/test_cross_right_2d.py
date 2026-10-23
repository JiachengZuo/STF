"""2d contract tests: numeric geometry, sampled scoring and fake actor calls."""
import copy
from contextlib import redirect_stdout
import importlib.util
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from cross_right_2d import (CrossRightTrial, ENTRY_SPEEDS, at, build_geometry, digest, directed_arms,
    final_status, live_ttc_s, load_settings, make_baseline, make_definitions,
    release_transition, reference_cruise_control, speed_control, validate_definitions)
from cross_right_2d_cli import dispatch_arguments, expand, load_run_definitions, write_json

ROOT = Path(__file__).resolve().parents[1]


def fixture():
    cfg = load_settings(ROOT/'config/cross_right_2d.yaml')
    ego = [dict(x=float(x), y=0., z=0., yaw=0., width=3.5, is_junction=abs(x) <= 6,
                signal_hit=False, road_id=1, lane_id=-1) for x in range(-100, 31)]
    target = [dict(x=0., y=float(y), z=0., yaw=-90., width=3.5, is_junction=abs(y) <= 6,
                   signal_hit=False, road_id=2, lane_id=-1) for y in range(40, -31, -1)]
    ego[0]['junction_entry_headings'] = [0, 90, 180, 270]
    b = make_baseline(ego, target, cfg, map_name='offline_cross', opendrive_sha256='offline')
    return cfg, b


def definition(speed=40):
    cfg, b = fixture()
    return make_definitions(b, cfg, speed=speed, weather='sunny_07')[0]


def corners(x, y, yaw=0., length=2.5, width=.9):
    a = math.radians(yaw)
    return [dict(x=x+i*length*math.cos(a)-j*width*math.sin(a),
                 y=y+i*length*math.sin(a)+j*width*math.cos(a), z=0.)
            for i, j in ((-1, -1), (-1, 1), (1, 1), (1, -1))]


def observation(x=-80., v=40/3.6, target_y=30.5555555556, target_x=0., target_v=20/3.6, frame=1):
    return dict(frame=frame, ego=dict(x=x, y=0., z=0.), target=dict(x=target_x, y=target_y, z=0.),
                ego_corners=corners(x, 0.), target_corners=corners(target_x, target_y, -90.),
                ego_speed_mps=v, target_speed_mps=target_v, ego_yaw=0.)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg, self.b = fixture()

    def test_crossing_and_reference_lengths(self):
        g = self.b['geometry']
        self.assertEqual(g['ego_conflict_s'], 100)
        self.assertEqual(g['target_conflict_s'], 40)
        self.assertEqual(g['directed_arm_count'], 4)
        self.assertAlmostEqual(40-g['target_wait_s'], 20/3.6*5.5)
        self.assertAlmostEqual(g['ego_conflict_near_s'], 98.25)
        self.assertFalse(g['section_speed_limit_verified'])

    def test_four_directed_arms_keep_opposites_and_merge_wraparound(self):
        self.assertEqual(len(directed_arms([0, 90, 180, 270, 359, 1], 25)), 4)
        b = copy.deepcopy(self.b)
        b['ego_route'][0]['junction_entry_headings'] = [0, 90, 180]
        self.assertEqual(build_geometry(b['ego_route'], b['target_route'], self.cfg)['directed_arm_count'], 3)

    def test_width_signals_junction_flags_and_side_are_observations(self):
        b = copy.deepcopy(self.b)
        for p in b['ego_route']+b['target_route']:
            p['width'] = 4.2
            p.pop('is_junction', None)
        b['ego_route'][0].pop('junction_entry_headings')
        b['ego_route'][0]['signal_hit'] = True
        g = build_geometry(b['ego_route'], b['target_route'], self.cfg)
        self.assertEqual(g['ego_half_width_m'], 2.1)
        self.assertIsNone(g['directed_arm_count'])
        self.assertFalse(g['signal_query_clear'])
        target = list(reversed(self.b['target_route']))
        g = build_geometry(self.b['ego_route'], target, self.cfg)
        self.assertLess(g['right_side_lateral_m'], 0)

    def test_invalid_coordinates_and_missing_crossing_still_rejected(self):
        b = copy.deepcopy(self.b)
        b['ego_route'][0]['x'] = float('nan')
        with self.assertRaisesRegex(ValueError, '坐标'):
            build_geometry(b['ego_route'], b['target_route'], self.cfg)
        b = copy.deepcopy(self.b)
        for p in b['target_route']:
            p['x'] = 500.
        with self.assertRaisesRegex(ValueError, '没有交点'):
            build_geometry(b['ego_route'], b['target_route'], self.cfg)

    def test_two_point_short_routes_need_no_map_annotations(self):
        ego = [dict(x=-3., y=0., z=0.), dict(x=4., y=0., z=0.)]
        target = [dict(x=0., y=2., z=0.), dict(x=0., y=-2., z=0.)]
        b = make_baseline(ego, target, self.cfg, map_name='offline', opendrive_sha256='offline')
        g = b['geometry']
        self.assertEqual(g['exit_gate_s'], 7.)
        self.assertLess(g['target_wait_s'], 0.)
        self.assertAlmostEqual(g['target_wait_pose']['y'], 20/3.6*5.5)
        self.assertIsNone(g['signal_query_clear'])
        self.assertIn('yaw', b['ego_route'][0])
        self.assertEqual(len(make_definitions(b, self.cfg, speed=40, weather='sunny_07')), 1)

    def test_map_speed_annotation_is_recorded_without_blocking_baseline_or_run(self):
        for limit in (30., 40., 50.):
            with self.subTest(map_limit=limit):
                routes = copy.deepcopy((self.b['ego_route'], self.b['target_route']))
                for route in routes:
                    for p in route:
                        p['speed_limit_kmh'] = limit
                b = make_baseline(*routes, self.cfg, map_name='offline_cross', opendrive_sha256='offline')
                self.assertEqual(b['geometry']['section_speed_limit_verified'], limit == 40.)
                self.assertEqual(b['ego_route'][0]['speed_limit_kmh'], limit)
                d = make_definitions(b, self.cfg, speed=40, weather='sunny_07')[0]
                self.assertEqual(d['section_speed_limit_kmh'], 40.)
                trial = CrossRightTrial(d)
                trial.update(0., observation(x=-60., v=41/3.6))
                self.assertIn('section_speed', trial.failures)
        routes = copy.deepcopy((self.b['ego_route'], self.b['target_route']))
        routes[0][0]['speed_limit_kmh'] = 40.
        routes[1][0]['speed_limit_kmh'] = 40.
        routes[1][-1]['speed_limit_kmh'] = 50.
        self.assertFalse(build_geometry(*routes, self.cfg)['section_speed_limit_verified'])

    def test_junction_width_not_used_for_conflict_or_gate(self):
        b = copy.deepcopy(self.b)
        for p in b['ego_route']+b['target_route']:
            if p['is_junction']:
                p['width'] = 20.
        self.assertEqual(build_geometry(b['ego_route'], b['target_route'], self.cfg)['ego_half_width_m'], 1.75)

    def test_curved_target_route_records_ray_error_without_rejecting(self):
        b = copy.deepcopy(self.b)
        # Tangent drift of ~4 m over the approach, with only 15 degrees change
        # across the 40 m straightness window, leaving room for segment sampling.
        radius = 40/math.radians(14)
        for p in b['target_route']:
            s = p['y']
            p['x'] = radius*(1-math.cos(s/radius))
            p['y'] = radius*math.sin(s/radius)
            p['yaw'] = -90-math.degrees(s/radius)
        g = build_geometry(b['ego_route'], b['target_route'], self.cfg)
        self.assertGreater(g['target_ray_max_error_m'], self.cfg['engineering']['target_lateral_tolerance_m'])

    def test_short_routes_and_near_endpoint_crossings_can_be_saved(self):
        for ego in (self.b['ego_route'][20:], self.b['ego_route'][100:]):
            ego = copy.deepcopy(ego)
            ego[0]['junction_entry_headings'] = [0, 90, 180, 270]
            g = build_geometry(ego, self.b['target_route'], self.cfg)
            self.assertLessEqual(g['ego_conflict_s'], 80.)


    def test_oblique_crossing_and_long_route_are_not_drawing_errors(self):
        target = copy.deepcopy(self.b['target_route'])
        for p in target:
            p['x'] = p['y']*math.sqrt(3)
            p['yaw'] = -150.
        g = build_geometry(self.b['ego_route'], target, self.cfg)
        self.assertAlmostEqual(g['crossing_angle_deg'], 150.)
        ego = [dict(x=-1000., y=0., z=0.), dict(x=100., y=0., z=0.)]
        g = build_geometry(ego, self.b['target_route'], self.cfg)
        self.assertGreater(g['timeout_s'], 600.)


class TTCTests(unittest.TestCase):
    def test_undefined_and_past_conflict(self):
        self.assertIsNone(live_ttc_s(10, .15, .15))
        self.assertEqual(live_ttc_s(0, 0, .15), 0)
        self.assertEqual(live_ttc_s(-1, 0, .15), 0)
        for speed, expected in ((40, 550/9), (10, 275/18)):
            self.assertLess(abs((speed/3.6*5.5)-expected), .01)
            self.assertAlmostEqual(live_ttc_s(expected, speed/3.6, .15), 5.5)

    def test_transition_truth_table(self):
        for previous, current, seen, expected in (
                (None, None, False, 'hold'), (None, 5.4, False, 'release'),
                (None, 4.4, False, 'never_observed'), (None, 0, False, 'never_observed'),
                (5.6, 4.4, True, 'skipped'), (6., 5.4, True, 'release'),
                (None, 4.4, True, 'hold'), (None, 0, True, 'hold')):
            with self.subTest(previous=previous, current=current, seen=seen):
                self.assertEqual(release_transition(previous, current, seen), expected)
        self.assertEqual(release_transition(5.4, 5.2, True, already_released=True), 'hold')
        self.assertEqual(release_transition(6., 5., True, ttc_latched=True), 'hold')

    def test_reference_cruise_compensates_drag_without_altering_yield_brake(self):
        cfg, _ = fixture()
        eng = cfg['engineering']
        v, integral, desired = 10/3.6, 0., 10/3.6
        for _ in range(600):
            throttle, brake, integral = reference_cruise_control(desired, v, eng, integral)
            v = max(0., v+(3*throttle-3*brake-.35)*eng['fixed_delta_seconds'])
        self.assertLess(abs(v-desired)*3.6, .1)
        throttle, brake, integral = reference_cruise_control(0., v, eng, integral)
        self.assertEqual((throttle, brake), speed_control(0., v, eng))
        self.assertEqual(integral, 0.)



class ScoringTests(unittest.TestCase):
    def test_all_terminal_branches(self):
        self.assertEqual(final_status(['ttc_window_skipped'], True, 'tcp', True, True,
                                      ['collision_with_target']), ('INVALID', 'ttc_window_skipped'))
        for controller, gate, expected in [('behavior', False, ('FAILED', 'corresponding_lane_not_entered')),
                                           ('behavior', True, ('INVALID', 'ttc_window_not_entered')),
                                           ('tcp', False, ('FAILED', 'corresponding_lane_not_entered')),
                                           ('tcp', True, ('INVALID', 'ttc_window_not_entered'))]:
            self.assertEqual(final_status([], False, controller, gate, False, []), expected)
        self.assertEqual(final_status([], True, 'tcp', True, True, []), ('PASS', None))
        self.assertEqual(final_status([], True, 'tcp', True, False, []), ('FAILED', 'corresponding_lane_not_entered'))
        self.assertEqual(final_status([], True, 'tcp', True, False, ['yield_not_established', 'collision_with_target']),
                         ('FAILED', 'collision_with_target'))

    def test_stopping_forever_is_failed_for_both_algorithms(self):
        for controller, status in [('tcp', 'FAILED'), ('behavior', 'FAILED')]:
            t = CrossRightTrial(definition(), controller)
            t.update(0., observation(v=0.))
            t.update(t.g['timeout_s'], observation(v=0.))
            self.assertTrue(t.done)
            self.assertFalse(t.released)
            self.assertEqual(t.result()['result_status'], status)

    def test_valid_yield_and_gate_pass(self):
        t = CrossRightTrial(definition(), 'tcp')
        t.update(-1.1, observation(x=-80))
        self.assertEqual(t.update(0, observation(x=-60)), 'release')
        t.update(6.3, observation(x=-30, v=0, target_y=-4.3))
        self.assertEqual(t.t_clear, 6.3)
        t.update(10., observation(x=18., v=5., target_y=-10., target_v=0.))
        self.assertTrue(t.done)
        self.assertEqual(t.result()['result_status'], 'PASS')
        self.assertTrue(t.result()['lane_entry_evidence']['lane_success'])

    def test_short_route_end_does_not_prevent_body_clearance_or_gate(self):
        cfg, b = fixture()
        ego = [p for p in b['ego_route'] if -10 <= p['x'] <= 5]
        target = [p for p in b['target_route'] if p['y'] >= -1]
        b = make_baseline(ego, target, cfg, map_name='offline', opendrive_sha256='offline')
        t = CrossRightTrial(make_definitions(b, cfg, speed=5, weather='sunny_07')[0], 'tcp')
        t.update(-1.1, observation(x=-10., v=5/3.6))
        t.update(0., observation(x=-7., v=5/3.6))
        self.assertTrue(t.released)
        t.update(6.3, observation(x=-6., v=0., target_y=-4.3))
        self.assertEqual(t.t_clear, 6.3)
        t.update(10., observation(x=8., v=3., target_y=-10., target_v=0.))
        self.assertEqual(t.result()['result_status'], 'PASS')

    def test_same_frame_clear_and_entry_is_not_yield(self):
        t = CrossRightTrial(definition(), 'tcp')
        t.update(-1.1, observation(x=-80))
        t.update(0., observation(x=-60))
        t.update(6.3, observation(x=-4., v=5., target_y=-4.3))
        self.assertEqual(t.t_clear, t.t_enter)
        t.update(10., observation(x=18., v=5., target_y=-10., target_v=0.))
        self.assertEqual(t.result()['primary_reason'], 'yield_not_established')

    def test_collision_closes_trial_before_future_faults_and_preserves_prior_faults(self):
        for before in (True, False):
            t = CrossRightTrial(definition(35), 'tcp')
            v = 35/3.6
            t.update(-1.1, observation(x=-80., v=v))
            o = observation(x=-5.6*v, v=v)
            o['collision_target'] = before
            t.update(0., o)
            o = observation(x=-4.4*v, v=v)
            o['collision_target'] = not before
            t.update(.05, o)
            r = t.result()
            if before:
                self.assertEqual(r['primary_reason'], 'collision_with_target')
                self.assertFalse(r['pass'])
                self.assertNotIn('ttc_window_skipped', r['latches'])
            else:
                self.assertEqual(r['primary_reason'], 'ttc_window_skipped')
                self.assertIsNone(r['pass'])
            self.assertIn('collision_with_target', r['failure_reasons'])

    def test_none_breaks_adjacency_and_gate_does_not_require_target_clear(self):
        t = CrossRightTrial(definition(35), 'tcp')
        t.update(-1.1, observation(x=-80, v=35/3.6))
        t.update(0, observation(x=-60, v=35/3.6))
        t.update(.05, observation(x=-60, v=0))
        self.assertEqual(t.update(.1, observation(x=20, v=10)), 'hold')
        self.assertTrue(t.gate_crossed)
        self.assertFalse(t.lane_success)
        self.assertEqual(t.result()['primary_reason'], 'ttc_window_not_entered')

    def test_first_defined_below_window_retains_tcp_speed_failure(self):
        for model, status, reason in (('tcp', 'FAILED', 'required_speed_not_reached'),
                                      ('behavior', 'FAILED', 'required_speed_not_reached')):
            t = CrossRightTrial(definition(), model)
            t.update(0, observation(x=-40, v=10))
            self.assertEqual((t.result()['result_status'], t.result()['primary_reason']), (status, reason))
            self.assertIn('ttc_window_never_observed', t.result()['latches'])

    def test_target_tail_controls_clearance_and_speed_monitoring(self):
        t = CrossRightTrial(definition(), 'tcp')
        t.update(-1.1, observation(x=-80))
        t.update(0, observation(x=-60))
        t.update(5.9, observation(x=-30, v=0, target_y=-1.8))
        self.assertIsNone(t.t_clear)
        t.update(6.4, observation(x=-30, v=0, target_y=-4.3))
        self.assertEqual(t.t_clear, 6.4)
        t.update(7., observation(x=-30, v=0, target_y=-31., target_v=0.))
        self.assertNotIn('target_speed_not_held', t.latches)
        self.assertNotIn('target_left_route', t.latches)
        self.assertAlmostEqual(2.5/(20/3.6), .45)

    def test_target_origin_only_for_lateral_error(self):
        t = CrossRightTrial(definition(), 'tcp')
        t.update(-1.1, observation(x=-80))
        t.update(0, observation(x=-60))
        t.update(2., observation(x=-30, v=0, target_y=15.))
        self.assertNotIn('target_left_route', t.latches)  # 0.9 m box corners are allowed.
        t.update(2.05, observation(x=-30, v=0, target_y=14., target_x=.8))
        self.assertIn('target_left_route', t.latches)

    def test_speed_cap_exact_and_only_before_gate(self):
        for kmh, failed in ((40, False), (41, True)):
            t = CrossRightTrial(definition(), 'tcp')
            t.update(0., observation(x=-60, v=kmh/3.6))
            self.assertEqual('section_speed' in t.failures, failed)

    def test_behavior_checks_measured_speed_without_second_hold_period(self):
        t = CrossRightTrial(definition(), 'behavior')
        t.update(0., observation(x=-80))
        t.update(1.8, observation(x=-60))
        self.assertNotIn('reference_speed_not_held', t.latches)
        t = CrossRightTrial(definition(), 'behavior')
        t.update(0., observation(x=-60))
        self.assertNotIn('reference_speed_not_held', t.latches)
        t = CrossRightTrial(definition(), 'behavior')
        t.update(0., observation(x=-50, v=35/3.6))
        self.assertEqual(t.result()['result_status'], 'FAILED')
        self.assertEqual(t.result()['primary_reason'], 'required_speed_not_reached')
        self.assertEqual(t.latches, [])


    def test_tcp_window_checks_speed_without_repeating_initialization_hold(self):
        for actual, hold, expected in ((18., 1., True), (20., 1., True), (22., 1., True),
                                       (17.99, 2., False), (22.01, 2., False), (20., .05, True)):
            with self.subTest(actual=actual, hold=hold):
                t = CrossRightTrial(definition(20), 'tcp')
                v = actual/3.6
                t.update(0., observation(x=-8*v, v=v))
                t.update(hold, observation(x=-5.4*v, v=v))
                t.update(hold+6.3, observation(x=-10., v=0., target_y=-4.3))
                t.update(hold+10., observation(x=18., v=5., target_y=-10., target_v=0.))
                r = t.result()
                self.assertEqual(r['result_status'], 'PASS' if expected else 'FAILED')
                self.assertEqual(r['speed_requirement_met'], expected)
                self.assertEqual(r['required_speed_kmh'], 20)
                self.assertIsNone(r['commanded_speed_kmh'])
                self.assertAlmostEqual(r['speed_requirement_evidence']['observed_speed_kmh'], actual)
                if not expected:
                    self.assertEqual(r['primary_reason'], 'required_speed_not_reached')

    def test_tcp_speed_must_still_be_held_when_window_is_entered(self):
        t = CrossRightTrial(definition(20), 'tcp')
        t.update(0., observation(x=-80., v=20/3.6))
        t.update(1., observation(x=-70., v=20/3.6))
        t.update(2., observation(x=-15., v=10/3.6))
        self.assertTrue(t.speed_ever_qualified)
        self.assertEqual(t.result()['primary_reason'], 'required_speed_not_reached')

    def test_tcp_unmet_speed_ends_at_window_without_inventing_later_failures(self):
        t = CrossRightTrial(definition(40), 'tcp')
        t.update(0., observation(x=-60., v=20/3.6))
        t.update(1., observation(x=-30., v=20/3.6))
        self.assertTrue(t.done)
        r = t.result()
        self.assertEqual(r['result_status'], 'FAILED')
        self.assertEqual(r['termination_reason'], 'required_speed_not_reached')
        self.assertEqual(r['elapsed_sim_time_s'], 1.)
        self.assertEqual(r['failure_reasons'], ['required_speed_not_reached'])
        self.assertIsNone(r['yield_evidence']['t_clear'])
        later = observation(x=18.)
        later['collision_target'] = True
        t.update(2., later)
        self.assertEqual(t.result()['failure_reasons'], ['required_speed_not_reached'])

    def test_tcp_timeout_without_required_speed_fails(self):
        t = CrossRightTrial(definition(), 'tcp')
        t.update(0., observation(v=0.))
        t.update(t.g['timeout_s'], observation(v=0.))
        r = t.result()
        self.assertTrue(t.done)
        self.assertEqual((r['result_status'], r['primary_reason']), ('FAILED', 'required_speed_not_reached'))
        self.assertFalse(r['speed_requirement_met'])
        self.assertEqual(r['invalid_reasons'], [])

    def test_tcp_speed_failure_keeps_ttc_evidence_and_real_faults_invalid(self):
        for fault, expected in ((None, ('FAILED', 'required_speed_not_reached')),
                                ('camera_timeout', ('INVALID', 'camera_timeout'))):
            t = CrossRightTrial(definition(), 'tcp')
            t.update(0., observation(x=-56., v=10.))
            t.update(.05, observation(x=-44., v=10.))
            if fault:
                t.latch(fault)
            r = t.result()
            self.assertEqual((r['result_status'], r['primary_reason']), expected)
            self.assertIn('ttc_window_skipped', r['latches'])
            self.assertIn('required_speed_not_reached', r['failure_reasons'])
            if not fault:
                self.assertEqual(r['invalid_reasons'], [])


class DefinitionTests(unittest.TestCase):
    def test_shared_matrix_identity_and_shape(self):
        cfg, b = fixture()
        docs = make_definitions(b, cfg)
        self.assertEqual(len(docs), 960)
        self.assertEqual(len({d['condition_fingerprint'] for d in docs}), 960)
        self.assertTrue(all('controller' not in d and 'commanded_speed_kmh' not in d for d in docs))
        self.assertEqual({v: sum(d['required_speed_kmh'] == v for d in docs) for v in ENTRY_SPEEDS},
                         {v: 120 for v in ENTRY_SPEEDS})
        self.assertTrue(all(d['scenario'].startswith('scenario_2d_v') for d in docs))
        self.assertNotIn('start_delay_s', json.dumps(docs))
        with patch('stf_weather_catalog.WEATHER_CATALOG_VERSION', 3):
            with self.assertRaisesRegex(ValueError, 'version 2'):
                make_definitions(b, cfg)

    def test_single_definition_and_direct_baseline_run(self):
        cfg, b = fixture()
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'baseline.json'
            write_json(p, b)
            args = SimpleNamespace(model='behavior', speed=40, weather='sunny_07', baseline=str(p), input_dir=None)
            docs = load_run_definitions(args, cfg)
            self.assertEqual(len(docs), 1)
            dpath = Path(tmp)/(docs[0]['scenario']+'.json')
            write_json(dpath, docs[0])
            self.assertEqual(len(validate_definitions([dpath], cfg)), 1)
            with self.assertRaisesRegex(ValueError, 'incomplete'):
                validate_definitions([dpath], cfg, require_complete=True)
            args.model = 'tcp'
            self.assertEqual(load_run_definitions(args, cfg), docs)
            args.speed = None
            self.assertEqual(len(load_run_definitions(args, cfg)), 8)
            args.baseline, args.input_dir, args.speed = None, tmp, 40
            self.assertEqual(load_run_definitions(args, cfg), docs)
            args.model = 'behavior'
            self.assertEqual(load_run_definitions(args, cfg), docs)

    def test_fingerprints_and_legacy_fields_rejected(self):
        cfg, b = fixture()
        d = make_definitions(b, cfg, speed=40, weather='sunny_07')[0]
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/(d['scenario']+'.json')
            d['weather']['wetness'] += 1
            write_json(p, d)
            with self.assertRaisesRegex(ValueError, 'fingerprint'):
                validate_definitions([p], cfg)
            d['start_delay_s'] = 0
            write_json(p, d)
            with self.assertRaisesRegex(ValueError, 'legacy'):
                validate_definitions([p], cfg)

    def test_expansion_refuses_hidden_file_and_dry_run_does_not_write(self):
        cfg, b = fixture()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            baseline, output = root/'baseline.json', root/'expanded'
            write_json(baseline, b)
            args = SimpleNamespace(baseline=str(baseline), input_dir=str(output), dry_run=True)
            with redirect_stdout(io.StringIO()):
                expand(args, cfg)
            self.assertFalse(output.exists())
            output.mkdir()
            (output/'.keep').touch()
            args.dry_run = False
            with self.assertRaisesRegex(ValueError, 'not empty'):
                expand(args, cfg)
            self.assertEqual(list(output.iterdir()), [output/'.keep'])


class DispatchTests(unittest.TestCase):
    def test_aliases_and_conflicts(self):
        for args in (['-2d'], ['--scenario', '2.d'], ['--scenario=2D'], ['-2d', '--scenario', '2.d']):
            self.assertEqual(dispatch_arguments(args), [])
        for selector in ('-6c', '-1a', '-5e', '-5d', '-2c', '-2e'):
            with self.assertRaises(ValueError):
                dispatch_arguments([selector, '-2d'])
        for code in ('2c', '2e', '5e'):
            self.assertIsNone(dispatch_arguments(['--scenario', code]))
        with self.assertRaises(ValueError):
            dispatch_arguments(['-2d', '--scenario', '2c'])

    def test_early_help_and_offline_cli_without_carla(self):
        for script in ('run_stf.py', 'waypoints_stf.py'):
            result = subprocess.run([sys.executable, '-B', str(ROOT/script), '-2d', '--help'],
                                    cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('live TTC', result.stdout)
        cfg, b = fixture()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'baseline.json'
            write_json(path, b)
            for model in ('behavior', 'tcp'):
                result = subprocess.run([sys.executable, '-B', str(ROOT/'run_stf.py'), '--scenario', '2.d',
                                         '--baseline', str(path), '--weather', 'sunny_07', '--speed', '40',
                                         '--model', model, '--dry-run'], cwd=ROOT, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('1 selected', result.stdout)


def fake_runtime():
    carla = ModuleType('carla')
    carla.Vector3D = lambda **kwargs: SimpleNamespace(**kwargs)
    carla.VehicleControl = lambda **kwargs: SimpleNamespace(**dict(dict(throttle=0., brake=0., steer=0., gear=0, manual_gear_shift=False), **kwargs))
    helpers = ModuleType('emergency_brake_5d_runtime')
    helpers.CameraFrames = lambda: SimpleNamespace()
    helpers.speed = lambda a: a.speed
    helpers.point = lambda a: a.position
    helpers.steer = lambda a, route: 0.
    helpers.crosses_solid = lambda e: any('Solid' in m for m in e['markings'])
    spec = importlib.util.spec_from_file_location('_test_2d_runtime', ROOT/'cross_right_2d_runtime.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {'carla': carla, 'emergency_brake_5d_runtime': helpers}):
        spec.loader.exec_module(module)
    return module


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fake_runtime()

    def scene(self, controller='tcp'):
        scene = self.runtime.CrossRightScene(SimpleNamespace(tick=Mock()), definition(), model=controller)
        scene.ego = SimpleNamespace(id=1, speed=0., apply_control=Mock(), set_target_velocity=Mock(),
                                    get_transform=lambda: SimpleNamespace(get_forward_vector=lambda: SimpleNamespace(x=1, y=0, z=0)))
        scene.target = SimpleNamespace(id=2, apply_control=Mock(), enable_constant_velocity=Mock(), disable_constant_velocity=Mock())
        return scene

    def test_target_steers_along_saved_route_until_body_clears(self):
        scene = self.scene()
        scene.trial.release_time = 1.
        with patch.object(self.runtime, 'steer', side_effect=[.4, -.2]) as track:
            scene.target_step()
            self.assertEqual(scene.target.apply_control.call_args.args[0].steer, .4)
            scene.target_step()
            self.assertEqual(scene.target.apply_control.call_args.args[0].steer, -.2)
            self.assertEqual(track.call_count, 2)
            scene.target.enable_constant_velocity.assert_called_once()
            scene.trial.t_clear = 8.
            scene.target_step()
            self.assertEqual(track.call_count, 2)
            scene.target.disable_constant_velocity.assert_called_once()

    def test_target_control_order_and_full_body_clearance(self):
        scene = self.scene()
        calls = []
        scene.target.apply_control.side_effect = lambda c: calls.append(('control', c.brake))
        scene.target.enable_constant_velocity.side_effect = lambda v: calls.append(('velocity', v.x))
        scene.trial.update(-1.1, observation(x=-80))
        scene.trial.update(0., observation(x=-60))
        scene.target_step()
        self.assertEqual(calls, [('control', 0.), ('velocity', 20/3.6)])
        scene.trial.update(5.9, observation(x=-30, v=0, target_y=-1.8))
        scene.target_step()
        scene.target.disable_constant_velocity.assert_not_called()
        scene.trial.update(6.4, observation(x=-30, v=0, target_y=-4.3))
        scene.target_step()
        scene.target.disable_constant_velocity.assert_called_once()
        self.assertEqual(calls[-1], ('control', 1.))

    def test_frame_discontinuity_stops_before_observation(self):
        scene = self.scene()
        scene.previous_frame = 10
        scene.world.get_snapshot = lambda: SimpleNamespace(frame=12, timestamp=SimpleNamespace(elapsed_seconds=1.))
        scene.observation = Mock()
        with self.assertRaisesRegex(RuntimeError, 'world_frame_discontinuity'):
            scene.step()
        scene.observation.assert_not_called()

    def test_late_collision_callback_keeps_gate_frame_but_excludes_later_frames(self):
        scene = self.scene()
        scene.trial.gate_frame = 100
        scene.collisions = [dict(frame=101, other_id=2)]
        self.assertFalse(scene.event_failures()[0])
        scene.collisions.append(dict(frame=100, other_id=2))
        self.assertTrue(scene.event_failures()[0])
        scene.lane_events = [dict(frame=100, markings=['Solid'])]
        self.assertIn('solid_line', scene.event_failures()[1])

    def test_map_assertion_has_no_loading_side_effect(self):
        self.runtime.require_map('/Game/Maps/Cross', 'Cross')
        self.runtime.require_map('/Game/Maps/Cross', None)
        with self.assertRaisesRegex(ValueError, 'loaded map'):
            self.runtime.require_map('/Game/Maps/Cross', 'Other')


if __name__ == '__main__':
    unittest.main()
