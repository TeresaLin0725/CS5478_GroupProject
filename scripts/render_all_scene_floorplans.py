"""Render one native inserted-object layer for each frozen bowl48 task block."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    args = parser.parse_args()
    root = args.run_root.resolve()
    protocol = json.loads((root / 'protocol.json').read_text(encoding='utf-8'))
    if protocol.get('design') != 'bowl48':
        raise ValueError('Expected a frozen bowl48 run')
    script = Path(__file__).with_name('render_rs_scene_overhead.py')
    seen = set()
    records = []
    for job in protocol['jobs']:
        key = (job['target_location'], job['profile'], job['start_id'])
        if key in seen:
            continue
        seen.add(key)
        output = root / 'scene-floorplans' / (
            f'{key[0]}-{key[1]}-start{key[2]}-objects.png')
        if not output.exists() or not output.with_suffix('.json').exists():
            print(f'Rendering {len(seen)}/12: {output.name}', flush=True)
            subprocess.run([sys.executable, str(script), '--run-root', str(root),
                            '--job-id', job['job_id'], '--output', str(output),
                            '--isolated-objects'], check=True)
        else:
            print(f'Using saved {len(seen)}/12: {output.name}', flush=True)
        from PIL import Image
        with Image.open(output) as im:
            if im.mode != 'RGBA' or im.getchannel('A').getbbox() is None:
                raise ValueError(f'Missing rendered inserted objects: {output}')
        records.append({'task_block': key, 'source_job_id': job['job_id'],
                        'image': str(output.relative_to(root)),
                        'metadata': str(output.with_suffix('.json').relative_to(root))})
    if len(seen) != 12:
        raise ValueError(f'Expected 12 task blocks, found {len(seen)}')
    index = {'status': 'complete', 'planned': 12, 'rendered': len(records),
             'method': 'Native iGibson overhead RGB + instance mask for inserted objects at t=0',
             'base': 'Original Rs floorplan scan; direct overhead RGB is ceiling occluded',
             'dynamic_cart': 'Initial cart pose only; motion during episodes remains in event logs',
             'records': records}
    (root / 'scene-floorplans' / 'index.json').write_text(
        json.dumps(index, indent=2), encoding='utf-8')
    print('Rendered 12 scene backgrounds', flush=True)


if __name__ == '__main__':
    main()
