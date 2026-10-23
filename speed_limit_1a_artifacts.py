"""Native 1a output paths: a video's stem is exactly its input JSON's stem."""
from pathlib import Path


def artifact_paths(output, input_file, attempt):
    output=Path(output).resolve()
    source=Path(input_file)
    if source.suffix!='.json' or not source.stem.startswith('scenario_1a_') or attempt<1:
        raise ValueError('invalid 1a JSON filename or attempt')
    stem=source.stem+f'_attempt_{attempt}'
    details=output/'details'
    return {'video':output/(source.stem+'.mp4'),
            'telemetry':details/(stem+'.telemetry.jsonl.gz'),
            'events':details/(stem+'.events.json'),
            'result':details/(stem+'.json')}


def archive_previous_video(paths, scenario, records):
    """Preserve a prior attempt before directly recording the new JSON-named video."""
    video=paths['video']
    if not video.exists():return None
    matching=[r for r in records if r['scenario']==scenario and r.get('video_path')
              and Path(r['video_path']).resolve()==video]
    if not matching:raise ValueError('video exists without matching attempt record: '+str(video))
    record=matching[-1]
    archive=video.parent/'details'/'prior_attempt_videos'/f"attempt_{record['attempt']:02d}"/video.name
    if archive.exists():raise ValueError('previous-attempt video archive already exists')
    archive.parent.mkdir(parents=True,exist_ok=True)
    video.replace(archive)
    record['video_path']=str(archive)
    return record
