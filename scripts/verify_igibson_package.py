"""Audit real simulator artifacts, frame integrity and close-range outcomes."""
import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image


def verify(directory):
    directory = Path(directory).resolve()
    results = []
    for path in sorted(directory.rglob('*-summary.json')):
        summary = json.loads(path.read_text(encoding='utf-8'))
        if summary.get('source') != 'igibson_static_scan' or 'profile' not in summary:
            continue
        name = summary['policy']
        trace_path = path.with_name(name + '-trace.json')
        trace = json.loads(trace_path.read_text(encoding='utf-8'))
        frames = trace['frames']
        assert frames and all(a['time_s'] <= b['time_s'] for a,b in zip(frames,frames[1:]))
        assert abs(frames[-1]['time_s'] - summary['sim_time_s']) < .002
        assert abs(frames[-1]['distance_m'] - summary['executed_distance_m']) < .002
        video = trace.get('video')
        if video:
            video_path = (path.parent / video['file']).resolve()
            assert directory in video_path.parents and video_path.is_file()
            assert video['max_gap_s'] <= 1/video['fps'] + .00001
            assert len(video['frames']) >= summary['sim_time_s']*video['fps']-1
            assert abs(video['frames'][-1]['distance_m']-summary['executed_distance_m']) < .002
            import imageio_ffmpeg
            reader = imageio_ffmpeg.read_frames(str(video_path))
            metadata = next(reader)
            assert metadata['codec'] == 'h264' and metadata['size'] == (1440,900)
            assert abs(metadata['fps']-video['fps']) < .001
            reader.close()
        for frame in frames:
            image_path = (path.parent / frame['image']).resolve()
            assert directory in image_path.parents
            with Image.open(image_path) as image:
                assert image.size == (1440, 900)
                image.verify()
        records = [json.loads(line) for line in path.with_name(name + '.jsonl').read_text(encoding='utf-8').splitlines()]
        initial = next(r['state'] for r in records if r['event'] == 'decision')
        assert initial['target_claim'] is None
        assert not any(o['class_name'] == initial['target_class'] for o in initial['semantic_objects'])
        assert summary['physical_robot'] and summary['robot'] == 'Locobot'
        if summary['termination'] == 'success':
            evaluation = next(r for r in records if r['event'] == 'terminal_evaluation')
            assert evaluation['valid_target']
            assert len(set(evaluation['claim']['evidence_observation_ids'])) >= 2
            assert summary['target_distance_m'] <= summary['close_distance_limit_m']
        if name == 'llm_guided' and summary['backend'] == 'replay':
            replay = json.loads(path.with_name('igibson.llm-offline-responses.json').read_text(encoding='utf-8'))
            assert 'NOT LLM inference' in replay['provenance']
            assert summary['fallbacks'] == 0
        results.append({'scenario': summary['profile'], 'policy': name,
            'result': summary['termination'], 'frames': len(frames),
            'contacts': summary['collision_steps'], 'replans': summary['replans'],
            'navigation_failures': summary['navigation_failures'],
            'log_sha256': hashlib.sha256(path.with_name(name+'.jsonl').read_bytes()).hexdigest()})
    assert results, 'No current iGibson episodes found'
    assert (directory / 'index.html').is_file()
    report = {'artifact_checks_passed': True, 'episodes': results,
              'scope': 'Simulator artifact integrity and recorded outcomes; no browser screenshot check'}
    (directory / 'verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    verify(parser.parse_args().directory)
