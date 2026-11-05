"""Recover post-terminal JSON export errors; never rerun or alter native outcomes."""
import argparse,hashlib,json
from datetime import datetime,timezone
from pathlib import Path
from benchmark_episode import atomic,read
from run_paired_benchmark import check_seals,result_row
from check_paired_benchmark import evidence_check

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def recover(root):
    root=Path(root).resolve();protocol=read(root/'protocol.json');check_seals(root,protocol)
    recovered=[]
    for job in protocol['jobs']:
        folder=root/'episodes'/job['job_id'];summaryfile=folder/'summary.json'
        if not summaryfile.exists() or not (folder/'supervisor.json').exists() or not (folder/'result.json').exists():continue
        original=read(summaryfile)
        if original.get('post_terminal_export_recovery'):continue
        if original.get('detail')!='worker_exit_without_summary':continue
        console=(folder/'console.log').read_text(encoding='utf-8')
        if "TypeError: Object of type PolicyName is not JSON serializable" not in console:continue
        assert "atomic(out/'llm-calls.json',calls)" in console,'Different error must not be silently recovered'
        events=[json.loads(line) for line in (folder/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        terminals=[e for e in events if e['event']=='episode_terminal']
        assert len(terminals)==1 and events[-1]==terminals[0],'Recovery requires a complete native terminal record'
        terminal=terminals[0];assert terminal['episode_id']==job['job_id'] and terminal['source']=='igibson'
        assert job['policy']=='llm_guided'
        calls=[e['record'] for e in events if e['event']=='llm_call']
        assert calls and (folder/'final-state.json').exists(),'Native execution/export did not complete to the expected point'
        summary={k:v for k,v in terminal.items() if k not in ('event','source','episode_id')}
        evaluation={}
        if terminal['termination']=='success':
            verification=evidence_check(folder,read(root/job['manifest_path']))
            instance=verification['instances'][0]
            evaluation=dict(w3_valid=True,distinct_times=True,stationary_evidence=True,near_evidence=True,terminal_stopped=True,
                evidence_distances_m=[c['matches'][instance]['distance_m'] for c in verification['captures']])
        else:
            evaluation['raw_target_evaluation_events']=[e for e in events if e['event']=='target_evaluation']
        backup=folder/'summary-supervisor-original.json'
        assert not backup.exists(),'Original export diagnosis must remain immutable'
        atomic(backup,original)
        if (folder/'result.json').exists():atomic(folder/'result-supervisor-original.json',read(folder/'result.json'))
        provenance=dict(reason='Native search already terminated. Only final llm-calls JSON export failed on nested PolicyName Enum.',
            native_outcome_unchanged=terminal['termination'],no_episode_retry=True,
            original_summary_sha256=sha(backup),native_events_sha256=sha(folder/'events.jsonl'),console_sha256=sha(folder/'console.log'),
            recovery_script_sha256=sha(__file__),recovered_utc=datetime.now(timezone.utc).isoformat())
        summary.update(job,recorded=False,source_fingerprint=protocol['engine_fingerprint'],source_unchanged=True,
            evaluation=evaluation,llm_attempts=len(calls),
            llm_calls=sum(bool(c.get('model_called',
                c.get('fallback_reason') not in ('unconfigured_client','call_cap'))) for c in calls),
            llm_fallbacks=sum(bool(c.get('fallback_reason')) for c in calls),
            partial_metrics=False,post_terminal_export_recovery=provenance)
        atomic(folder/'llm-calls.json',calls);atomic(summaryfile,summary);atomic(folder/'postprocess-recovery.json',provenance)
        result_row(root,job,protocol);recovered.append(dict(job_id=job['job_id'],termination=terminal['termination']))
    records=[]
    for path in (root/'episodes').glob('*/postprocess-recovery.json'):
        records.append(dict(job_id=path.parent.name,**read(path)))
    atomic(root/'export-recovery.json',dict(scope='Post-terminal artifact serialization only. All actual native outcomes preserved, no retries, no excluded episodes.',
        frozen_snapshot_unchanged=True,records=sorted(records,key=lambda r:r['job_id'])))
    return recovered

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_root',type=Path)
    print(json.dumps(dict(recovered=recover(p.parse_args().run_root))))
