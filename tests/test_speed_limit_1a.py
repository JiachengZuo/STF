import copy
import json
from pathlib import Path
import pytest
from speed_limit_1a import Trial,validate_definition,decode_action
from speed_limit_1a_cli import dispatch_arguments

ROOT=Path(__file__).resolve().parents[1]

def case(number=3):
    return json.loads((ROOT/f'save_scenarios/1a/definitions/scenario_1a_{number:02d}_B.json').read_text())['1a']['route_01'][0]

def observation(s,v,t,**kwargs):
    return dict(time_s=t,front_s_m=s,center_s_m=s-2,speed_kmh=v,**kwargs)

def approach(trial):
    for s,v,t in [(0,0,0),(250,65,20),(270,65,22),(390,60,30)]:
        trial.observe(observation(s,v,t))


def test_full_trial_requires_D_and_E():
    trial=Trial(case());approach(trial)
    for s,v,t in [(400,60,31),(600,55,45),(700,55,53),(800,65,60),(900,65,68)]:
        trial.observe(observation(s,v,t))
    assert trial.checked_D and not trial.done
    trial.observe(observation(1100,10,90,at_end=True))
    assert trial.result()['result_status']=='PASS'
    assert [e['event'] for e in trial.events]==['APPROACH','B','C','D']


def test_low_approach_is_failed_not_invalid():
    t=Trial(case(7))
    for s,v,ts in [(0,0,0),(250,28.8,32),(270,28.8,35)]:t.observe(observation(s,v,ts))
    assert t.result()['result_status']=='FAILED'
    assert t.failures==['approach_speed_insufficient']
    assert not t.result()['approach_established']


def test_B_uses_front_plane_interpolated_speed():
    t=Trial(case());approach(t)
    t.observe(observation(399,59,31,gates={'B':-.5,'C':-301,'D':-501}))
    t.observe(observation(401,69,32,gates={'B':.5,'C':-299,'D':-499}))
    assert t.failures==['speed_at_B_exceeded']
    assert t.events[-1]['speed_kmh']==64
    assert t.events[-1]['time_s']==31.5


@pytest.mark.parametrize('v,reason',[(61,'restricted_speed_exceeded'),(44,'restricted_speed_too_low')])
def test_restricted_band(v,reason):
    t=Trial(case());approach(t)
    t.observe(observation(400,60,31));t.observe(observation(500,v,40))
    assert t.failures==[reason]


def test_recovery_lower_bound_applies_at_D_only():
    t=Trial(case());approach(t)
    for s,v,ts in [(400,55,31),(700,50,50),(800,55,60)]:t.observe(observation(s,v,ts))
    assert not t.done
    t.observe(observation(900,59,70))
    assert t.failures==['recovery_speed_insufficient']


def test_no_release_never_creates_C_or_restore_requirement():
    t=Trial(case(7))
    for s,v,ts in [(0,0,0),(250,32,30),(270,32,33),(390,30,45),(400,30,46),(500,25,58)]:
        t.observe(observation(s,v,ts))
    assert t.checked_D and not t.done
    t.observe(observation(700,20,90,at_end=True))
    assert t.result()['pass'] is True
    assert [e['event'] for e in t.events]==['APPROACH','B','D']


def test_timeout_and_stall_are_controller_failures():
    t=Trial(case(),stall_timeout=300)
    t.observe(observation(0,0,0));t.observe(observation(10,0,180))
    assert t.failures==['precondition_timeout']
    t=Trial(case(),stall_timeout=30)
    t.observe(observation(0,0,0));t.observe(observation(0,0,30))
    assert t.failures==['stalled']


def test_bad_measurement_is_invalid():
    t=Trial(case());t.observe(observation(0,float('nan'),0))
    assert t.result()['result_status']=='INVALID' and t.result()['pass'] is None


def test_late_collision_is_not_hidden_by_D_completion():
    t=Trial(case());approach(t)
    for s,v,ts in [(400,60,31),(700,60,50),(900,60,70)]:t.observe(observation(s,v,ts))
    t.observe(observation(950,60,74,collision=True))
    assert t.failures==['collision']


def test_three_channels_preserved_and_invalid_action_rejected():
    assert decode_action([[.3,-.4,.8]])==(.3,-.4,.8)
    for action in ([[.2,0,float('nan')]],[[2,0,0]],[[0,0,-1]]):
        with pytest.raises(ValueError):decode_action(action)


def test_definition_derived_fields_not_trusted():
    d=case();validate_definition(d)
    for mutation in ('station','speed','point'):
        bad=copy.deepcopy(d)
        if mutation=='station':bad['speed_limit_test']['final_check_s_m']+=1
        if mutation=='speed':bad['speed_limit_test']['speed_at_B_max_kmh']=80
        if mutation=='point':bad['speed_limit_test']['test_points']['B']['x']+=10
        with pytest.raises(ValueError):validate_definition(bad)


def test_dispatch_aliases_and_conflicts():
    assert dispatch_arguments(['-1a','--model','tcp'])==['--model','tcp']
    assert dispatch_arguments(['--scenario','1.a','--dry-run'])==['--dry-run']
    assert dispatch_arguments(['--scenario=1a'])==[]
    assert dispatch_arguments(['-5e']) is None
    for args in (['-1a','-5e'],['-1a','--scenario','2b']):
        with pytest.raises(ValueError):dispatch_arguments(args)


def test_multiple_gates_in_one_step_are_ordered():
    t=Trial(case());approach(t)
    t.observe(observation(901,60,60))
    assert [e['event'] for e in t.events]==['APPROACH','B','C','D']
    assert t.checked_D and not t.done


def test_missing_gate_prevents_pass():
    t=Trial(case());approach(t)
    t.observe(observation(400,60,31))
    t.observe(observation(901,60,60))
    t.passed.remove('B')
    t.observe(observation(1100,20,90,at_end=True))
    assert t.result()['result_status']=='INVALID'
    assert 'route_ended_without_required_events' in t.invalid


def test_native_video_name_and_retry_preserves_previous(tmp_path):
    from speed_limit_1a_artifacts import artifact_paths,archive_previous_video
    name='scenario_1a_01_cloudy_01'
    paths=artifact_paths(tmp_path,name+'.json',2)
    assert paths['video']==tmp_path/(name+'.mp4')
    assert paths['telemetry'].parent==tmp_path/'details'
    paths['video'].write_bytes(b'old completed recording')
    old={'scenario':name,'attempt':1,'video_path':str(paths['video'])}
    record=archive_previous_video(paths,name,[old])
    assert not paths['video'].exists()
    assert Path(record['video_path']).name==name+'.mp4'
    assert Path(record['video_path']).read_bytes()==b'old completed recording'
    paths['video'].write_bytes(b'new recording')
    assert Path(record['video_path']).read_bytes()==b'old completed recording'


def test_native_output_does_not_overwrite_unowned_video(tmp_path):
    from speed_limit_1a_artifacts import artifact_paths,archive_previous_video
    paths=artifact_paths(tmp_path,'scenario_1a_01_cloudy_01.json',1)
    paths['video'].write_bytes(b'existing unrelated file')
    with pytest.raises(ValueError):archive_previous_video(paths,'scenario_1a_01_cloudy_01',[])
    assert paths['video'].read_bytes()==b'existing unrelated file'


def test_video_overlay_uses_same_route_and_correct_panel_layout():
    import numpy as np
    from types import SimpleNamespace
    from speed_limit_1a import Route
    from speed_limit_1a_video import SpeedLimitVideo,VIDEO_METADATA
    d=case();route=Route(d['ego_route']);point=route.points[-1]
    calls={}
    viz=SpeedLimitVideo.__new__(SpeedLimitVideo)
    viz.viz=SimpleNamespace(ego=SimpleNamespace(get_location=lambda:SimpleNamespace(x=point['x'],y=point['y'])),screen=object(),render=lambda points:calls.update(points=points))
    viz.pygame=SimpleNamespace(event=SimpleNamespace(pump=lambda:None),surfarray=SimpleNamespace(array3d=lambda screen:np.zeros((512,256,3),dtype=np.uint8)))
    rgb=np.zeros((256,256,3),dtype=np.uint8)
    result=viz.frame(rgb,route)
    assert result.shape==(256,512,3)
    assert viz.viz.front_cam_data is rgb
    assert len(calls['points'])==12
    assert all(abs(p[0]-point['x'])<.001 and abs(p[1]-point['y'])<.001 for p in calls['points'])
    assert VIDEO_METADATA['fps']==20 and VIDEO_METADATA['codec']=='h264'
