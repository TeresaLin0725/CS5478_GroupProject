"""Audit a completed frozen suite against raw native logs and retained evidence."""
import argparse,csv,hashlib,importlib.util,json,math,sys
from pathlib import Path
from datetime import datetime,timezone
from urllib.parse import urlsplit

def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def iou(a,b):
    overlap=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    return overlap/((a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-overlap)
def distance(a,b,dimensions=('x_m','y_m','z_m')):
    return math.sqrt(sum((a[k]-b[k])**2 for k in dimensions))

def evidence_check(folder,manifest,events=None):
    """Recompute scoring from persisted detections/truth/motion, not success flags."""
    data=read(folder/'evaluation-only.json');bundle=data['evidence'];detections=bundle['detections']
    claim=read(folder/'final-state.json')['target_claim'];cfg=manifest['evaluation']
    assert bundle['episode_id']==manifest['episode_id'] and bundle['claim_id']==claim['claim_id']
    assert claim['target_class']==manifest['target_class']
    assert len(detections)>=cfg['required_evidence_frames']
    assert len({d['capture_sequence'] for d in detections})==len(detections)
    assert len({d['timestamp_s'] for d in detections})==len(detections)
    assert set(claim['evidence_observation_ids'])=={d['observation_id'] for d in detections}
    frames={f['capture_sequence']:f for f in data['frames']};common=None;details=[]
    for d in detections:
        f=frames[d['capture_sequence']];motion=data['motion'][str(d['capture_sequence'])]
        assert f['episode_id']==manifest['episode_id'] and abs(f['timestamp_s']-d['timestamp_s'])<1e-9
        assert d['class_name']==claim['target_class'] and d['track_id']==claim['track_id']
        # Captured physical motion defines stationary evidence. A small active
        # steering command is not the same as measured body rotation.
        assert d['timestamp_s']<=claim['timestamp_s']
        assert motion['linear_mps']<=.04 and motion['angular_radps']<=.10
        matches={}
        for truth in f['instances']:
            if truth['class_name']!=claim['target_class'] or truth['visibility']<cfg['minimum_visibility']:continue
            if (iou(d['bbox_xyxy_px'],truth['bbox_xyxy_px'])>=cfg['minimum_bbox_iou']
                and distance(d['position_world'],truth['position_world'])<=cfg['localization_tolerance_m']
                and distance(motion['pose'],truth['position_world'],('x_m','y_m'))<=.95):
                matches[truth['instance_id']]=dict(distance_m=distance(motion['pose'],truth['position_world'],('x_m','y_m')),
                    visibility=truth['visibility'],bbox_iou=iou(d['bbox_xyxy_px'],truth['bbox_xyxy_px']))
        common=set(matches) if common is None else common&set(matches)
        details.append(dict(sequence=d['capture_sequence'],timestamp_s=d['timestamp_s'],matches=matches,
            linear_mps=motion['linear_mps'],angular_radps=motion['angular_radps']))
    assert common,'No consistent visible near stationary instance across all evidence'
    if events is None:
        events=[json.loads(line) for line in (folder/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    scoring=[(i,e) for i,e in enumerate(events) if e['event']=='target_evaluation']
    assert len(scoring)==1
    index,score=scoring[0]
    assert score['result']['valid_target'] is True and score['result']['matched_instance_id'] in common
    assert score['evidence']==bundle and score['claim']==claim
    stops=[i for i,e in enumerate(events[:index]) if e['event']=='module_end' and e.get('module')=='stop']
    moves=[i for i,e in enumerate(events[:index]) if e['event']=='motion_sample']
    assert stops and (not moves or stops[-1]>moves[-1]),'Terminal command must be stopped before scoring'
    return dict(passed=True,instances=sorted(common),captures=details)

def audit(root):
    root=Path(root).resolve();p=read(root/'protocol.json');s=read(root/'results.json')
    assert sha(root/'protocol.json')==(root/'protocol.sha256').read_text().strip()
    assert not (root/'cancel.request').exists(),'Cancelled suite cannot be final'
    assert s['complete'] and s['completed']==s['planned']==len(p['jobs'])==36
    assert s['complete_pairs']==9 and read(root/'progress.json')['status']=='complete'
    supervisor=read(root/'suite-verification.json')
    assert all(supervisor[k] for k in ('complete','frozen_snapshot_unchanged','shared_assets_unchanged','no_retries','all_result_integrity_checks_passed'))
    assert read(root/'preflight/index.json')['passed']
    snapshot=root/'frozen'
    for name,digest in p['snapshot_seals'].items():assert sha(snapshot/name)==digest,name
    assetroot=Path(p['asset_paths']['scenes']).parent
    for name,digest in p['asset_hashes'].items():assert sha(assetroot/name)==digest,name
    rows=[];checks=[];artifacts={}
    for job in p['jobs']:
        folder=root/'episodes'/job['job_id'];m=read(root/job['manifest_path'])
        assert sha(root/job['manifest_path'])==job['manifest_sha256']
        summary=read(folder/'summary.json');row=read(folder/'result.json')
        events=[json.loads(line) for line in (folder/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        assert len([e for e in events if e['event']=='episode_start'])==1
        assert next(e['manifest'] for e in events if e['event']=='episode_start')==m
        assert all(row['integrity_checks'].values()),job['job_id']
        assert summary['source_fingerprint']==p['engine_fingerprint'] and summary['source_unchanged']
        assert row['termination']==summary['termination']
        terminals=[e for e in events if e['event']=='episode_terminal']
        if terminals:
            assert len(terminals)==1 and terminals[0]['termination']==summary['termination']
            for key in ('simulated_time_s','actual_distance_m','collisions','decisions','attempts'):
                assert terminals[0][key]==summary[key],('native terminal mismatch',job['job_id'],key)
        else:assert summary.get('partial_metrics') is True and row['termination']=='pipeline_error'
        if summary.get('post_terminal_export_recovery'):
            recovery=read(folder/'postprocess-recovery.json')
            assert recovery==summary['post_terminal_export_recovery']
            assert sha(folder/'summary-supervisor-original.json')==recovery['original_summary_sha256']
            assert sha(folder/'events.jsonl')==recovery['native_events_sha256']
            assert sha(folder/'console.log')==recovery['console_sha256']
            assert recovery['native_outcome_unchanged']==summary['termination'] and recovery['no_episode_retry']
        for k in ('simulated_time_s','actual_distance_m','collisions','decisions','attempts'):
            assert row[k]==summary[k],(job['job_id'],k)
        calls=[e['record'] for e in events if e['event']=='llm_call']
        assert read(folder/'llm-calls.json')==calls
        model_calls=sum(bool(c.get('model_called',
            c.get('fallback_reason') not in ('unconfigured_client','call_cap'))) for c in calls)
        assert model_calls==row['llm_calls']==summary['llm_calls']
        assert len(calls)==summary.get('llm_attempts',len(calls))
        assert row['llm_fallbacks']==sum(bool(c.get('fallback_reason')) for c in calls)
        assert job['policy']=='llm_guided' or not calls
        decisions={e['state']['revision']:e for e in events if e['event']=='decision'}
        for c in calls:
            provider=c.get('provider')
            if provider:
                assert provider.get('backend')!='replay'
                assert urlsplit(provider['endpoint']).hostname=='generativelanguage.googleapis.com'
                assert json.loads(provider['request']['messages'][1]['content'])==c['input']
            if not c.get('fallback_reason'):
                assert provider and provider.get('response_model') and provider.get('raw_response')
                assert c['response'] in {candidate['frontier_id'] for candidate in c['input']['candidates']}
                assert decisions[c['state_revision']]['selection']['subgoal']['frontier_id']==c['response']
        captures=[e for e in events if e['event']=='capture_processed']
        assert all(b['sequence']>a['sequence'] for a,b in zip(captures,captures[1:]))
        item=dict(job_id=job['job_id'],termination=row['termination'],captures=len(captures),
                  calls=model_calls,llm_attempts=len(calls),passed=True)
        if row['success']:item['independent_evidence_recomputation']=evidence_check(folder,m,events)
        rows.append(row);checks.append(item)
        for name in ('summary.json','result.json','events.jsonl','supervisor.json','evaluation-only.json','llm-calls.json'):
            if (folder/name).exists():artifacts[str((folder/name).relative_to(root)).replace('\\','/')]=sha(folder/name)
    # Recompute the predeclared final 10,000-draw statistics from frozen code.
    spec=importlib.util.spec_from_file_location('frozen_benchmark_statistics',snapshot/'harness/benchmark_protocol.py')
    frozen=importlib.util.module_from_spec(spec);spec.loader.exec_module(frozen)
    rebuilt=frozen.aggregate(rows,p['jobs'],draws=10000)
    for key in rebuilt:assert rebuilt[key]==s[key],('statistics do not reproduce',key)
    with (root/'episodes.csv').open(encoding='utf-8-sig',newline='') as f:csvrows=list(csv.DictReader(f))
    assert len(csvrows)==36 and {r['job_id'] for r in csvrows}=={j['job_id'] for j in p['jobs']}
    page=(root/'report/index.html').read_text(encoding='utf-8')
    assert '36 / 36' in page and 'http-equiv="refresh"' not in page
    assert not any(marker in page for marker in ('__CARDS__','__TABLE__','__COMPLETED__'))
    from PIL import Image
    with Image.open(root/'report/performance.png') as image:image.verify()
    for file in ('results.json','episodes.csv','report/index.html','report/performance.png','report/performance.svg',
                 'report/analysis.html','report/findings.md','report/trajectories.png','report/trajectories.svg','report/failure-facts.json'):
        if (root/file).exists():artifacts[file]=sha(root/file)
    result=dict(passed=True,completed=36,complete_pairs=9,successes=sum(r['success'] for r in rows),
        successful_evidence_independently_recomputed=True,final_statistics_reproduced_draws=10000,
        protocol_sha256=sha(root/'protocol.json'),engine_fingerprint=p['engine_fingerprint'],
        checks=checks,artifact_sha256=artifacts,checked_utc=datetime.now(timezone.utc).isoformat())
    (root/'audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_root',type=Path);args=parser.parse_args()
    r=audit(args.run_root);print(json.dumps({k:r[k] for k in ('passed','completed','complete_pairs','successes')}))
