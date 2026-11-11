"""Package the Chinese report and compact evidence for a repository submission."""
import argparse
from datetime import datetime, timezone
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import shutil


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.targets = []

    def handle_starttag(self, tag, attrs):
        if tag == 'a':
            self.targets.append(dict(attrs)['href'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source, output = args.run_root.resolve(), args.output.resolve()
    if output == source or source in output.parents:
        raise ValueError('Archive must be outside the source run')
    results = json.loads((source / 'results.json').read_text(encoding='utf-8'))
    assert results['complete'] and results['completed'] == results['planned'] == 48
    output.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source / 'report-zh', output / 'report-zh', dirs_exist_ok=True)
    evidence = ['results.json', 'episodes.csv', 'protocol.json', 'protocol.sha256',
                'suite-verification.json', 'configuration-audit.json', 'setup-provenance.json',
                'trajectory-scene-status.json']
    if (source / 'submission-validation.json').is_file():
        evidence.append('submission-validation.json')
    for name in evidence:
        shutil.copy2(source / name, output / name)
    from PIL import Image
    for row in results['results']:
        original = source / 'episodes' / row['job_id']
        archived = output / 'episodes' / row['job_id']
        archived.mkdir(parents=True, exist_ok=True)
        with Image.open(original / 'trajectory-scene.png') as image:
            image = image.convert('RGB')
            image.thumbnail((594, 738), Image.Resampling.LANCZOS)
            image.save(archived / 'trajectory-scene.webp', quality=90, method=6)
        for name in ('result.json', 'summary.json', 'trajectory-scene-metadata.json'):
            shutil.copy2(original / name, archived / name)
    note = ('仓库存档中的逐轮轨迹使用WebP预览；完整分辨率PNG保留在原实验目录。'
            'WebP仅用于展示，统计数据与逐轮JSON均直接复制自原结果。')
    for name in ('bowl48-report-zh.html', 'bowl48-report-zh.md'):
        path = output / 'report-zh' / name
        text = path.read_text(encoding='utf-8').replace('/trajectory-scene.png', '/trajectory-scene.webp')
        if name.endswith('.html'):
            text = text.replace('<h3>逐轮轨迹图索引</h3>', '<h3>逐轮轨迹图索引</h3><p>' + note + '</p>')
        else:
            text = text.replace('### 逐轮轨迹图索引\n', '### 逐轮轨迹图索引\n\n' + note + '\n')
        path.write_text(text, encoding='utf-8')
    links = Links()
    links.feed((output / 'report-zh/bowl48-report-zh.html').read_text(encoding='utf-8'))
    assert len(links.targets) == 53
    assert all((output / 'report-zh' / name).is_file() for name in links.targets)
    for name in evidence:
        assert (output / name).read_bytes() == (source / name).read_bytes()
    digest = hashlib.sha256((output / 'protocol.json').read_bytes()).hexdigest()
    assert digest == (output / 'protocol.sha256').read_text().strip()
    hashes = {p.relative_to(output).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'archive-manifest.json'}
    manifest = {'source_run': source.name, 'archived_utc': datetime.now(timezone.utc).isoformat(),
                'report': 'report-zh/bowl48-report-zh.html', 'completed': 48,
                'trajectory_previews': 48, 'preview_format': 'WebP quality 90, at most 594x738 pixels',
                'scope': 'Report, statistics, frozen protocol, per-episode results and summaries; '
                         'full event logs, model assets, credentials and frozen source tree remain local.',
                'files_sha256': hashes}
    (output / 'README.md').write_text(
        '# 48轮有噪声碗搜索实验存档\n\n'
        '[打开中文报告](report-zh/bowl48-report-zh.html) · '
        '[Markdown版](report-zh/bowl48-report-zh.md)\n\n'
        '本批次48轮全部完成，共45轮成功；LLM与IG均为12/12成功。报告包含成功率、时间、路径、'
        '两处目标与两个起点分析、四组四方法轨迹对比、配对不确定性和全部48轮明细。\n\n'
        + note + '\n\n'
        '原始protocol.json及其SHA256完整保留；其中机器路径属于实验时的来源记录，不能直接用作其他电脑的运行配置。'
        '图像是原始Rs平面图与原生渲染新增物体的合成，动态推车显示初始位置。'
        '原生资源和完整事件日志位于本机runs/bowl-48-noise-20261110-02目录。\n\n'
        '复现报告存档：\n\n```powershell\n'
        'python scripts/build_bowl48_chinese_report.py --run-root runs/bowl-48-noise-20261110-02\n'
        'python scripts/archive_bowl48_report.py --run-root runs/bowl-48-noise-20261110-02 '
        '--output report/bowl-48-noise-20261110-02\n```\n', encoding='utf-8')
    manifest['files_sha256'] = {
        p.relative_to(output).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'archive-manifest.json'}
    (output / 'archive-manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'archive': str(output), 'episodes': 48, 'validated_links': len(links.targets),
                      'total_bytes': sum(p.stat().st_size for p in output.rglob('*') if p.is_file())}))


if __name__ == '__main__':
    main()
