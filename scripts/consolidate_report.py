"""Collect one verified demo and one complete benchmark into a portable report.

Copies retained evidence before any cleanup. This command never deletes runs.
Frozen protocols, source snapshots, native logs and video bytes stay unchanged.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil

from build_demo_report import build_report, integrated
from publish_paired_benchmark_links import publish


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def consolidate(demo, benchmark, output):
    demo, benchmark, output = (Path(p).resolve() for p in (demo, benchmark, output))
    if output.exists():
        raise FileExistsError('Choose an empty report destination; existing evidence is never overwritten.')
    read = lambda path: json.loads(path.read_text(encoding='utf-8'))
    assert read(demo/'recording/artifact-verification.json')['passed']
    stats = read(benchmark/'results.json')
    assert stats['complete'] and stats['completed'] == 36 and stats['complete_pairs'] == 9
    assert read(benchmark/'audit.json')['passed']
    output.mkdir(parents=True)
    demo_out, benchmark_out = output/'evidence/demo', output/'evidence/benchmark'
    demo_out.mkdir(parents=True)
    entries = []

    def copy_tree(source, target, skip=()):
        for path in sorted(source.rglob('*')):
            relative = path.relative_to(source)
            if any(part in ('__pycache__', 'profile') for part in relative.parts):
                continue
            if relative.parts[0] in skip:
                continue
            if not path.is_file() or path.suffix in ('.pyc', '.zip'):
                continue
            if source == demo/'recording' and relative.name == 'index.html':
                continue
            destination = target/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, destination)
            assert sha(path) == sha(destination)
            entries.append(dict(source=str(path), path=destination.relative_to(output).as_posix(),
                                bytes=path.stat().st_size, sha256=sha(destination)))

    for name in ('recording', 'dryrun-multiview', 'checks-multiview'):
        copy_tree(demo/name, demo_out/name)
    for name in ('validation.json', 'spectator-isolation.json'):
        source, destination = demo/name, demo_out/name
        shutil.copy2(source, destination)
        entries.append(dict(source=str(source), path=destination.relative_to(output).as_posix(),
                            bytes=source.stat().st_size, sha256=sha(destination)))
    # Keep every final episode and recovery record, including all failures.
    # Superseded-suite provenance and old page screenshots belong to earlier report versions.
    copy_tree(benchmark, benchmark_out, skip=('superseded-suites.json', 'browser-check',
                                              'browser-check-demo', 'browser-check-live'))

    notes = read(demo/'scene-update.json')
    notes.pop('comparison', None)
    notes['summary'] = ('Locobot 在 Rs 公寓中搜索杯子，途中有推车移动干扰；在线大语言模型选择探索前沿，'
                        '推车瓶子使用 YCB 纹理网格。检测阈值、目标位置及独立验收条件按原设计执行。')
    notes['diagnostic_scope'] = ('只保留最新一次通过完整验证的录制、同源验证运行和 36 回合配对实验；'
                                 '单次录制不用于估计成功率或策略性能。')
    notes['dryrun_summary'] = 'evidence/demo/dryrun-multiview/summary.json'
    notes['validation'] = 'evidence/demo/validation.json'
    case = integrated(None, 'current-0', '移动推车干扰搜索 · 在线大语言模型策略',
                      recording=demo_out/'recording')
    build_report(output, default='current-0', cases=[case], notes=notes)
    write(output/'inventory.json', [dict(key=case['key'], title=case['title'],
          duration_s=case['duration'], distance_m=case['distance'], contacts=case['contacts'],
          model_calls=case['model_calls'], source_fingerprint=case['source_fingerprint'])])

    for name in ('index.html', 'analysis.html', 'findings.md', 'performance.png', 'performance.svg',
                 'trajectories.png', 'trajectories.svg', 'failure-facts.json'):
        source = benchmark_out/'report'/name
        target = output/('benchmark.html' if name == 'index.html' else name)
        if source.suffix in ('.html', '.md'):
            text = source.read_text(encoding='utf-8')
            text = re.sub(r'((?:href|src)=[\"\'])\.\./', r'\1evidence/benchmark/', text)
            text = text.replace('href="index.html"', 'href="benchmark.html"')
            text = text.replace('(index.html)', '(benchmark.html)')
            text = re.sub(r' · <a href="evidence/benchmark/superseded-suites.json">[^<]*</a>', '', text)
            text = re.sub(r'\n旧 v1 只有预检查[^\n]*\n', '\n', text)
            text = re.sub(r'(\]\()\.\./', r'\1evidence/benchmark/', text)
            target.write_text(text, encoding='utf-8')
        else:
            shutil.copy2(source, target)
    publish(benchmark_out, output, final=True)
    write(output/'evidence/relocation.json', dict(
        original_demo_root=str(demo), original_benchmark_root=str(benchmark),
        frozen_protocol_paths_are_original=True,
        demo_validation_paths=dict(run='demo/dryrun-multiview', tests='demo/checks-multiview/tests.log'),
        benchmark_snapshot='benchmark/frozen',
        note='Original hash-bound JSON paths are retained as provenance; resolve copied evidence relative to this directory.',
        files=entries))
    print(json.dumps(dict(output=str(output), verified_copied_files=len(entries),
                          demo_cases=1, benchmark_episodes=36, complete_pairs=9)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo-run', type=Path, required=True)
    parser.add_argument('--benchmark-run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    consolidate(args.demo_run, args.benchmark_run, args.output)
