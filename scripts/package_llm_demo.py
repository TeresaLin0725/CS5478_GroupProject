"""Expose genuine hosted decisions in an accepted local LLM recording viewer."""
import argparse
from html import escape
import json
import os
from pathlib import Path
from urllib.parse import urlsplit


def package(directory, baseline):
    summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    gate=json.loads((directory.parent/'validation.json').read_text(encoding='utf-8'))
    artifact=json.loads((directory/'artifact-verification.json').read_text(encoding='utf-8'))
    assert summary['termination']=='success' and summary['policy']=='llm_guided'
    assert summary['recorded'] and summary['source_unchanged'] and gate['passed'] and artifact['passed']
    assert summary['source_fingerprint']==gate['source_fingerprint']
    rows=[json.loads(line) for line in (directory/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    decisions={row['state']['revision']:row for row in rows if row['event']=='decision'}
    dispatched=[row['request']['subgoal'] for row in rows if row['event']=='dispatch']
    records=[row['record'] for row in rows if row['event']=='llm_call']
    assert records and len(records)==summary['llm_calls'] and summary['llm_fallbacks']==0
    calls=[]
    for record in records:
        provider=record['provider'];decision=decisions[record['state_revision']]
        assert not record['fallback_reason'] and not record['fallback_selected_action']
        assert provider.get('backend')!='replay' and provider.get('response_model')
        assert urlsplit(provider['endpoint']).hostname=='generativelanguage.googleapis.com'
        assert json.loads(provider['request']['messages'][1]['content'])==record['input']
        assert record['response'] in {c['frontier_id'] for c in record['input']['candidates']}
        assert decision['selection']['subgoal']['frontier_id']==record['response']
        assert any(goal['frontier_id']==record['response']
            and goal['selected_from_revision']==record['state_revision'] for goal in dispatched)
        calls.append(dict(simulated_time_s=decision['state']['timestamp_s'],state_revision=record['state_revision'],
            model=record['model'],response_model=provider['response_model'],frontier_id=record['response'],
            reason=provider['reason'],latency_s=record['latency_s'],usage=record['usage'],
            previous_failure=(record['input'].get('previous_attempt') or {}).get('failure_reason'),
            raw_response=provider['raw_response']))
    models=sorted({call['model'] for call in calls})
    assert models==gate['llm_models']
    usage={key:sum((call['usage'] or {}).get(key,0) for call in calls)
           for key in ('prompt_tokens','completion_tokens','total_tokens')}
    report=dict(passed=True,hosted_calls=len(calls),fallbacks=0,models=models,
        response_models=sorted({call['response_model'] for call in calls}),
        all_responses_eligible=True,all_responses_match_dispatched_goals=True,logged_inputs_match_requests=True,
        provider_host='generativelanguage.googleapis.com',usage=usage,
        calls_with_navigation_failure_feedback=sum(bool(call['previous_failure']) for call in calls),
        local_map_replans=sum(row['event']=='navigation_map_feedback' and row.get('kind')=='replan' for row in rows),
        source_fingerprint=summary['source_fingerprint'])
    (directory/'llm-calls.json').write_text(json.dumps(calls,ensure_ascii=False,indent=2),encoding='utf-8')
    (directory/'llm-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    model_text=escape(', '.join(models))
    baseline_href=escape(os.path.relpath(baseline/'index.html',directory).replace('\\','/'),quote=True)
    baseline_summary=json.loads((baseline/'summary.json').read_text(encoding='utf-8'))
    same_source=baseline_summary['source_fingerprint']==summary['source_fingerprint']
    baseline_label='Nearest 基线版' if same_source else 'Nearest 早期参考'
    banner=('<section><a href="index.html">LLM 实调用版</a><a href="'+baseline_href+'">'+baseline_label+'</a></section>'
        '<p><strong>真实在线模型：'+model_text+'</strong>；'+str(len(calls))+' 次有效 API 决策，0 次回退。'
        'LLM 选择探索目标；目标出现后，由确定性接近控制和独立验收完成确认。</p>'
        '<p>高难度动态场景：%.2f 秒仿真、%.2f 米行驶、%d 次接触；W3 近距离停稳证据验收通过。</p>' %
        (summary['simulated_time_s'],summary['actual_distance_m'],summary['collisions']))
    if not same_source:
        banner+='<p>参考运行的源码或配置不同；各自配置保存在 manifest.json，单次录像差异不代表策略性能的配对比较。</p>'
    if report['calls_with_navigation_failure_feedback']:
        banner+='<p>%d 次模型决策接收了 B 的导航失败反馈；相应上下文与回答保存在模型记录中。</p>' % report['calls_with_navigation_failure_feedback']
    table='<h2>真实模型决策记录</h2><p>时间为仿真时钟；API 耗时为墙钟时间。完整请求与响应保留在事件日志。</p>'
    table+='<table style="width:100%;border-spacing:12px"><thead><tr><th>仿真时间</th><th>模型选择</th><th>API 耗时</th><th>模型理由原文</th></tr></thead><tbody>'
    for call in calls:
        table+='<tr><td>%.2fs</td><td>%s</td><td>%.2fs</td><td>%s</td></tr>' % (
            call['simulated_time_s'],escape(call['frontier_id']),call['latency_s'],escape(call['reason']))
    table+='</tbody></table>'
    text=(directory/'index.html').read_text(encoding='utf-8')
    assert '<!-- hosted-llm -->' not in text,'Viewer is already packaged'
    text=text.replace('<video id="video"','<!-- hosted-llm -->'+banner+'\n<video id="video"',1)
    text=text.replace('<pre id="summary">',
        '<section><a href="llm-calls.json">模型回答</a><a href="llm-verification.json">实调用校验</a>'
        '<a href="final.png">最终验收画面</a><a href="evaluation-only.json">独立证据</a></section>'+table+'<pre id="summary">',1)
    text=text.replace('collisions:s.collisions,evaluation:s.evaluation',
        'collisions:s.collisions,llm_calls:s.llm_calls,llm_fallbacks:s.llm_fallbacks,evaluation:s.evaluation')
    (directory/'index.html').write_text(text,encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    parser.add_argument('--baseline',type=Path,required=True)
    args=parser.parse_args();package(args.directory,args.baseline)
