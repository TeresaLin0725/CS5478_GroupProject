# coding: utf-8
"""Link a benchmark and analysis from the current demo without historical runs."""
import argparse
from html import escape
import json
import os
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def publish(root, demo, final=False):
    root, demo = Path(root).resolve(), Path(demo).resolve()
    compact = (demo/'benchmark.html').is_file()
    link = lambda path: os.path.relpath(path, demo).replace('\\', '/')
    overview = 'benchmark.html' if compact else link(root/'report/index.html')
    analysis = 'analysis.html' if compact else link(root/'report/analysis.html')
    path = demo/'index.html'
    text = path.read_text(encoding='utf-8')
    nav = '<a href="%s">四策略表现</a><a href="%s">逐任务分析</a>' % (overview, analysis)
    if '四策略表现</a>' not in text:
        text = text.replace('<a href="#design-check">设计核对</a>', nav+'<a href="#design-check">设计核对</a>', 1)
    if final:
        stats = json.loads((root/'results.json').read_text(encoding='utf-8'))
        assert stats['complete'] and json.loads((root/'audit.json').read_text(encoding='utf-8'))['passed']
        table = ''
        for policy, label in (('random', 'Random'), ('nearest', 'Nearest'),
                              ('information_gain', 'IG'), ('llm_guided', 'LLM-guided')):
            s = stats['paired_policy_stats'][policy]
            table += ('<tr><td>%s</td><td>%d / %d</td><td>%.1f%%</td><td>%.1f s</td></tr>' %
                      (label, s['successes'], stats['complete_pairs'], s['success_rate']*100,
                       s['mean_penalized_time_s']))
        section = ('<section class="report" id="paired-performance"><div class="eyebrow">'
                   'PAIRED NATIVE BENCHMARK / 36 EPISODES</div><h2>四策略目前的实际表现</h2>'
                   '<p class="note">同一 Rs 房屋的原始、拥挤、动态三种条件 × 3 组种子 / 初始朝向 '
                   '× 4 策略；全部 36 回合完成，9 组严格配对，全部失败纳入统计，无单回合重试。</p>'
                   '<div class="table-wrap"><table><thead><tr><th>策略</th><th>成功 / 全部</th>'
                   '<th>成功率</th><th>失败计 600 s 的平均时间</th></tr></thead><tbody>'+table+
                   '</tbody></table></div><p><a href="'+escape(overview, quote=True)+'">完整统计与 95% 区间 →</a>　'
                   '<a href="'+escape(analysis, quote=True)+'">逐任务、实际轨迹与失败分析 →</a></p>'
                   '<p class="note">演示录像单独保留，不计入这 36 个回合。IG 在本批成功率最高；'
                   '小样本未显示校正后的显著差异，当前结果不能证明跨房屋泛化。</p></section>')
        if 'id="paired-performance"' in text:
            text = re.sub(r'<section class="report" id="paired-performance">.*?</section>',
                          lambda match: section, text, flags=re.S)
        else:
            text = text.replace('<section class="report" id="design-check">',
                                section+'<section class="report" id="design-check">', 1)
        for old, new in (
            ('代码具备 / 实验待齐', '已完成 / 9 组配对'),
            ('共享候选和导航；当前演示不能代替四策略的完整配对比较。', '共享候选、导航与验收；完整统计见四策略表现。'),
            ('多场景、多种子、统计与不确定性', '三条件、多种子、配对统计与不确定性'),
            ('尚未完成验收', '36 回合已完成'),
            ('还需冻结实验配置，保留全部成功与失败回合，报告配对统计和区间。',
             '冻结配置、全部终止结果、配对区间与独立复核均保留；跨房屋实验尚未覆盖。')):
            text = text.replace(old, new)
    path.write_text(text, encoding='utf-8')
    if compact:
        nav = ('<nav aria-label="报告导航" style="display:flex;flex-wrap:wrap;gap:24px;margin:0 0 24px">'
               '<a href="index.html">最新 Demo</a><a href="benchmark.html">四策略统计</a>'
               '<a href="analysis.html">逐任务分析</a></nav>')
        for name in ('benchmark.html', 'analysis.html'):
            page = demo/name
            content = page.read_text(encoding='utf-8')
            if 'aria-label="报告导航"' not in content:
                content = content.replace('<main>', '<main>'+nav, 1)
            page.write_text(content, encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--demo', type=Path, default=ROOT/'report')
    parser.add_argument('--final', action='store_true')
    args = parser.parse_args()
    publish(args.run_root, args.demo, args.final)
