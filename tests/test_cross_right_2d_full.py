"""Completion, overlap prevention and wall-time estimates for the full 2d run."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from scripts.launch_cross_right_2d_full import behavior_ready, completion, estimate_remaining, remaining_behavior


def ledger(names=('a', 'b'), states=('FAILED', 'FAILED'), status='completed'):
    return dict(metadata=dict(status=status), records=[
        dict(scenario=n, condition_fingerprint=n+'-fp', controller='behavior', result_status=s,
             cleanup=dict(remaining_actor_ids=[])) for n, s in zip(names, states)])


class FullRunTests(unittest.TestCase):
    def test_algorithm_failures_do_not_block_next_stage(self):
        self.assertEqual(completion(ledger(), {'a':'a-fp','b':'b-fp'}, 'behavior'), 'completed')

    def test_cleanly_attempted_invalid_trials_are_execution_complete(self):
        data=ledger(states=('FAILED','INVALID'),status='incomplete')
        self.assertEqual(completion(data, {'a':'a-fp','b':'b-fp'}, 'behavior'), 'completed_with_invalid')

    def test_partial_or_interrupted_batch_is_not_complete(self):
        self.assertIsNone(completion(ledger(names=('a',),states=('FAILED',)), {'a':'a-fp','b':'b-fp'}, 'behavior'))
        self.assertIsNone(completion(ledger(status='interrupted'), {'a':'a-fp','b':'b-fp'}, 'behavior'))

    def test_actor_remnants_or_different_conditions_prevent_handoff(self):
        for key,value in (('cleanup',{'remaining_actor_ids':[42]}),('condition_fingerprint','changed')):
            data=ledger();data['records'][0][key]=value
            self.assertIsNone(completion(data, {'a':'a-fp','b':'b-fp'}, 'behavior'))

    def test_pilot_valid_cases_are_counted_once_and_invalid_cases_remain(self):
        plan=dict(expected_conditions={'a':'a-fp','b':'b-fp','c':'c-fp'})
        self.assertEqual(remaining_behavior(plan,ledger(states=('FAILED','INVALID'))),{'b':'b-fp','c':'c-fp'})

    def test_estimate_weights_all_speeds_and_reuses_six_pilot_cases(self):
        measured=(5,10,20,25,35,40)
        samples=[dict(speed_kmh=v,wall_seconds=10.+500./v) for v in measured]
        remaining=[v for v in (5,10,15,20,25,30,35,40) for _ in range(120-(v in measured))]
        estimate=estimate_remaining(samples,remaining,startup_seconds=3.)
        self.assertEqual(sum(estimate['remaining_counts_per_speed'].values()),954)
        self.assertAlmostEqual(estimate['remaining_wall_seconds'],3.+sum(10.+500./v for v in remaining),places=7)
        self.assertAlmostEqual(estimate['predicted_seconds_per_speed'][15],10.+500./15)

    def test_tcp_waits_for_behavior_worker_exit_even_if_ledger_is_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);plan=dict(behavior_output=directory,expected_conditions={'a':'a-fp','b':'b-fp'})
            pilot=ledger(names=('a',),states=('FAILED',))
            (root/'2d_result.json').write_text(json.dumps(ledger(names=('b',),states=('FAILED',))))
            state=root/'launch_status.json';state.write_text(json.dumps(dict(state='running')))
            self.assertFalse(behavior_ready(plan,pilot)[0])
            state.write_text(json.dumps(dict(state='completed')))
            self.assertTrue(behavior_ready(plan,pilot)[0])


if __name__=='__main__':
    unittest.main()
