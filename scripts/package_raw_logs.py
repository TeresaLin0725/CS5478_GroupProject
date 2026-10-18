"""Usage: package_raw_logs.py report/evidence report/raw-logs-<date>.zip [skipped prefixes...]

Zip report/evidence (except git-tracked demo recordings) with a SHA-256 manifest."""
import hashlib, json, sys, zipfile
from pathlib import Path
root = Path(sys.argv[1]).resolve(); out = Path(sys.argv[2]).resolve()
skip_prefix = tuple(sys.argv[3:])  # e.g. demo/random/recording/, already tracked in git
entries = []
with zipfile.ZipFile(out, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as z:
    for f in sorted(p for p in root.rglob('*') if p.is_file()):
        rel = f.relative_to(root).as_posix()
        if rel.startswith(skip_prefix) or any(part in ('profile', '__pycache__') or part.startswith('.env') for part in f.relative_to(root).parts):
            continue
        data = f.read_bytes(); z.writestr('evidence/' + rel, data)
        entries.append(dict(path='evidence/' + rel, bytes=len(data), sha256=hashlib.sha256(data).hexdigest()))
    z.writestr('package-manifest.json', json.dumps(entries, indent=2))
    z.writestr('README.txt', 'Extract into the report/ folder so that report/evidence/ is restored.\n'
               'Contains the raw logs of the 36-episode paired benchmark and the demo validation runs.\n'
               'Demo recordings (video, events) are already tracked in git under report/evidence/demo/.\n')
with zipfile.ZipFile(out) as z:
    assert z.testzip() is None
    for e in entries: assert hashlib.sha256(z.read(e['path'])).hexdigest() == e['sha256']
print(json.dumps(dict(files=len(entries), raw_mb=round(sum(e['bytes'] for e in entries)/1e6, 1),
                      zip_mb=round(out.stat().st_size/1e6, 1))))
