"""Package the audited paired dataset using an explicit artifact allowlist."""
import argparse,hashlib,json,zipfile
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    root=a.run_root.resolve();read=lambda f:json.loads((root/f).read_text(encoding='utf-8'))
    assert read('audit.json')['passed'] and read('results.json')['complete']
    assert read('browser-check/verification.json')['passed'] and read('browser-check/verification.json')['complete']
    files=set()
    def include(relative):
        file=root/relative
        if file.is_file():files.add(file)
    for name in ('protocol.json','protocol.sha256','results.json','episodes.csv','progress.json','suite-verification.json','audit.json','superseded-suites.json','export-recovery.json','finalization.json'):
        include(name)
    for job in read('protocol.json')['jobs']:
        include(job['manifest_path'])
        for name in ('summary.json','result.json','events.jsonl','supervisor.json','evaluation-only.json','llm-calls.json','final-state.json','last-processed-rgb.png','console.log','summary-supervisor-original.json','result-supervisor-original.json','postprocess-recovery.json'):
            include('episodes/'+job['job_id']+'/'+name)
    include('preflight/index.json')
    for profile in ('apartment','clutter','dynamic'):
        for name in ('preflight.json','oracle-grid.json','target-view.png','console.log'):include('preflight/'+profile+'/'+name)
    for name in ('index.html','analysis.html','findings.md','performance.png','performance.svg','trajectories.png','trajectories.svg','failure-facts.json'):
        include('report/'+name)
    for file in (root/'browser-check').glob('benchmark-*.png'):files.add(file)
    include('browser-check/verification.json')
    include('checks/harness-tests.log');include('checks/harness-tests.json')
    for relative in read('protocol.json')['snapshot_seals']:include('frozen/'+relative)
    entries=[];a.output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(a.output,'w',compression=zipfile.ZIP_DEFLATED) as archive:
        for file in sorted(files):
            relative=file.relative_to(root).as_posix()
            assert not any(part.startswith('.env') or part in ('profile','__pycache__') for part in file.relative_to(root).parts)
            archive.write(file,root.name+'/'+relative)
            entries.append(dict(path=relative,bytes=file.stat().st_size,sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
        archive.writestr(root.name+'/package-manifest.json',json.dumps(entries,indent=2))
        archive.writestr(root.name+'/README.txt',
            'Extract the entire folder, then open report/index.html and report/analysis.html.\n'
            '36 fresh native episodes: one Rs house, 3 conditions x 3 seed/heading blocks x 4 policies.\n'
            'All failures retained. No episode retries. Earlier infrastructure-invalid suites are excluded as entire suites.\n'
            'audit.json independently recomputes successful target evidence and all final statistics.\n'
            'Oracle geometry/truth is evaluator-only; never supply preflight files to a search policy.\n'
            'Secrets, virtual environments, browser profiles and external iGibson assets are excluded.\n'
            'Reproduction requires local native assets/environment and a new output folder; protocol asset paths describe the original run.\n')
    with zipfile.ZipFile(a.output) as archive:
        assert archive.testzip() is None
        for e in entries:assert hashlib.sha256(archive.read(root.name+'/'+e['path'])).hexdigest()==e['sha256']
    certificate=dict(output=str(a.output.resolve()),files=len(entries),bytes=a.output.stat().st_size,
        sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),all_entries_match=True,zip_crc_verified=True,
        excludes_secrets_environments_browser_profiles_and_external_assets=True)
    (root/'package-verification.json').write_text(json.dumps(certificate,indent=2),encoding='utf-8')
    print(json.dumps(certificate))

if __name__=='__main__':main()
