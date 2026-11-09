"""Issue the recording gate only after regression tests and a real successful dry run."""
import argparse
from pathlib import Path
import json
import hashlib
import re

ROOT=Path(__file__).resolve().parents[1]


def source_fingerprint():
    digest=hashlib.sha256()
    for path in sorted((ROOT/'src/frontier_search').rglob('*.py')):
        digest.update(str(path.relative_to(ROOT)).encode());digest.update(path.read_bytes())
    digest.update((ROOT/'weights/yolo11/yolo11s.pt').read_bytes())
    return digest.hexdigest()


def validate(run,tests):
    summary=json.loads((run/'summary.json').read_text(encoding='utf-8'))
    # PowerShell may produce UTF-16 redirected logs; accept its explicit BOM.
    raw=tests.read_bytes()
    test_text=raw.decode('utf-16') if raw.startswith((b'\xff\xfe',b'\xfe\xff')) else raw.decode('utf-8')
    checks=dict(test_suite_passed=bool(re.search(r'^OK\s*$',test_text,re.M)) and 'FAILED (' not in test_text,
        native_success=summary['termination']=='success',no_recording_in_validation=summary['recorded'] is False,
        exact_source=summary.get('source_unchanged') is True and summary['source_fingerprint']==source_fingerprint(),
        independent_evaluation=all(summary['evaluation'].get(k) is True for k in
            ('w3_valid','distinct_times','stationary_evidence','near_evidence','terminal_stopped')),
        physical_evidence_present=(run/'evaluation-only.json').is_file())
    certificate=json.loads(tests.with_suffix('.json').read_text(encoding='utf-8'))
    checks['test_process_exit_and_source']=(certificate['returncode']==0 and certificate['source_unchanged']
                                         and certificate['source_fingerprint']==source_fingerprint())
    rows=[json.loads(line) for line in (run/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    runtime=summary.get('runtime_configuration')
    starts=[row for row in rows if row['event']=='episode_start']
    navigation_records=[]
    for kind,detail in summary.get('navigator_events',[]):
        if kind=='navigation_configuration':
            navigation_records.append(json.loads(detail))
    checks['runtime_configuration_recorded']=(
        isinstance(runtime,dict) and runtime.get('schema_version')=='c-navigation-recovery-v1'
        and runtime.get('profile')==summary.get('profile')
        and runtime.get('policy')==summary.get('policy')
        and len(starts)==1 and runtime.get('recovery')==starts[0]['manifest'].get('recovery')
        and len(navigation_records)==1 and runtime.get('navigation')==navigation_records[0])
    checks['sensor_noise_configuration_recorded']=(
        isinstance(runtime,dict) and isinstance(runtime.get('sensor_noise'),dict)
        and len(starts)==1
        and runtime['sensor_noise']==starts[0]['manifest'].get('sensor_noise')
        and runtime['sensor_noise']==summary.get('sensor_noise'))
    captures=[r for r in rows if r['event']=='capture_processed']
    checks['fresh_capture_barriers']=all(b['sequence']>a['sequence'] and b['simulated_time_s']>=a['simulated_time_s']
        for a,b in zip(captures,captures[1:])) and len(captures)>2
    checks['no_pipeline_errors']=not any(r['event'] in ('pipeline_error','cleanup_error') for r in rows)
    decisions=[r for r in rows if r['event']=='decision']
    checks['shared_decision_loop']=bool(decisions) and all(r['context']['evaluation']['state_revision']==r['state']['revision']
        and (not r['selection']['subgoal'] or r['selection']['subgoal']['selected_from_revision']==r['state']['revision']) for r in decisions)
    checks['no_oracle_room_labels']=all(not r['manifest']['oracle_room_labels'] for r in rows if r['event']=='episode_start')
    checks['native_calibration_logged']=any(r['event']=='sensor_calibration' for r in rows)
    checks['verified_start_area_logged']=any(r['event']=='verified_start_area' for r in rows)
    checks['all_contact_ticks_stop']=all(r.get('feedback',{}).get('status')=='failed'
        for r in rows if r['event']=='replan_request')
    llm=[r['record'] for r in rows if r['event']=='llm_call']
    if summary['policy']=='llm_guided':
        # A successful Nearest fallback must not certify a hosted-LLM demo.
        checks['hosted_llm_responses']=bool(llm) and all(
            isinstance(record.get('provider'),dict)
            and record['provider'].get('response_model')
            and isinstance(record['provider'].get('raw_response'),str)
            and bool(record['provider']['raw_response'].strip())
            and record['provider'].get('backend')!='replay'
            for record in llm)
        checks['llm_no_fallback']=bool(llm) and all(not record.get('fallback_reason')
            and not record.get('fallback_selected_action') for record in llm)
        checks['llm_responses_eligible']=bool(llm) and all(record.get('response') in
            {candidate['frontier_id'] for candidate in record['input']['candidates']}
            for record in llm)
        model_calls=sum(bool(record.get('model_called',
            record.get('fallback_reason') not in ('unconfigured_client','call_cap'))) for record in llm)
        checks['llm_count_matches_summary']=(model_calls==summary.get('llm_calls')
            and len(llm)==summary.get('llm_attempts',len(llm)))
    match=re.search(r'Ran (\d+) tests',test_text)
    return dict(passed=all(checks.values()),checks=checks,source_fingerprint=source_fingerprint(),
        profile=summary['profile'],policy=summary['policy'],tests_count=int(match.group(1)) if match else None,
        runtime_configuration=runtime,
        tests_file=str(tests.resolve()),tests_sha256=hashlib.sha256(raw).hexdigest(),
        run=str(run.resolve()),summary_sha256=hashlib.sha256((run/'summary.json').read_bytes()).hexdigest(),
        dry_run_summary=summary,llm_attempts=len(llm),
        llm_calls=sum(bool(record.get('model_called',
            record.get('fallback_reason') not in ('unconfigured_client','call_cap'))) for record in llm),
        llm_models=sorted({record['model'] for record in llm}),
        llm_response_models=sorted({record['provider']['response_model'] for record in llm
            if isinstance(record.get('provider'),dict) and record['provider'].get('response_model')}),
        llm_fallbacks=sum(bool(r.get('fallback_reason')) for r in llm))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--tests',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=validate(args.run,args.tests)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'],checks=report['checks']),indent=2))
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
