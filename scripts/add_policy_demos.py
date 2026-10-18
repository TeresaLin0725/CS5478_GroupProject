"""Add verified per-policy recordings to the consolidated report as separate demos.

Each demo keeps its own recording, non-recording validation run and recording gate
under report/evidence/demo/<policy>/. Native logs and video bytes are copied unchanged.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from build_demo_report import PROFILES, build_report, integrated
from publish_paired_benchmark_links import publish

ORDER = ('random', 'nearest', 'information_gain', 'llm_guided')
NAMES = {'random': '随机策略', 'nearest': '最近可达目标策略',
         'information_gain': '信息增益策略', 'llm_guided': '在线大语言模型策略'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def copy_tree(source, target):
    for path in sorted(source.rglob('*')):
        if path.is_file() and path.name != 'index.html' and '__pycache__' not in path.parts:
            destination = target/path.relative_to(source)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, destination)
            assert sha(path) == sha(destination)


def restructure(demo):
    """Move the single-demo layout into demo/llm_guided/ once."""
    if not (demo/'recording').is_dir():
        return
    moves = {'recording': 'llm_guided/recording', 'dryrun-multiview': 'llm_guided/dryrun',
             'validation.json': 'llm_guided/validation.json',
             'spectator-isolation.json': 'llm_guided/spectator-isolation.json',
             'checks-multiview': 'checks'}
    for old, new in moves.items():
        (demo/new).parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(demo/old), str(demo/new))
    relocation = demo.parent/'relocation.json'
    text = relocation.read_text(encoding='utf-8')
    for old, new in moves.items():
        text = text.replace('"evidence/demo/%s' % old, '"evidence/demo/%s' % new)
    relocation.write_text(text, encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--demo', type=Path, action='append', default=[],
                        help='Per-policy run directory containing recording/, dryrun/ and validation.json.')
    args = parser.parse_args()
    report = args.report.resolve(); demo = report/'evidence/demo'
    restructure(demo)
    for run in (p.resolve() for p in args.demo):
        summary = read(run/'recording/summary.json')
        assert summary['termination'] == 'success' and read(run/'recording/artifact-verification.json')['passed']
        assert read(run/'validation.json')['passed']
        target = demo/summary['policy']
        if target.exists():
            shutil.rmtree(target)
        copy_tree(run/'recording', target/'recording')
        copy_tree(run/'dryrun', target/'dryrun')
        shutil.copy2(run/'validation.json', target/'validation.json')
    cases = []
    for policy in ORDER:
        recording = demo/policy/'recording'
        if recording.is_dir():
            summary = read(recording/'summary.json')
            cases.append(integrated(None, policy, PROFILES[summary['profile']]+' · '+NAMES[policy],
                                    recording=recording))
    notes = read(report/'scene-update.json')
    notes['summary'] = ('Locobot 在 Gibson Rs 公寓中搜索杯子，四种前沿选择策略各收录一段通过独立验收的录像；'
                        '同一源码、检测模型、控制器与验收条件。检测阈值、目标位置及独立验收条件按原设计执行。')
    notes['diagnostic_scope'] = ('每种策略保留一次通过完整验证的录制及其同源无录制试跑；'
                                 '单次录制只用于展示，不用于估计成功率或比较策略性能，策略比较以 36 回合配对实验为准。')
    notes['demos'] = {c['key']: dict(title=c['title'], recording='evidence/demo/%s/recording' % c['key'],
                                     dryrun='evidence/demo/%s/dryrun/summary.json' % c['key'],
                                     validation='evidence/demo/%s/validation.json' % c['key']) for c in cases}
    for key in ('dryrun_summary', 'validation'):
        notes.pop(key, None)
    default = 'llm_guided' if any(c['key'] == 'llm_guided' for c in cases) else cases[0]['key']
    build_report(report, default=default, cases=cases, notes=notes)
    (report/'inventory.json').write_text(json.dumps([dict(key=c['key'], title=c['title'],
        duration_s=c['duration'], distance_m=c['distance'], contacts=c['contacts'],
        model_calls=c['model_calls'], source_fingerprint=c['source_fingerprint']) for c in cases],
        ensure_ascii=False, indent=2), encoding='utf-8')
    publish(report/'evidence/benchmark', report, final=True)
    print(json.dumps([dict(key=c['key'], title=c['title'], duration_s=round(c['duration'], 2)) for c in cases],
                     ensure_ascii=False))


if __name__ == '__main__':
    main()
