"""Snapshot current project source and setup instructions, excluding local assets/secrets."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raw = subprocess.check_output(['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'], cwd=str(ROOT))
    candidates = [Path(p) for p in raw.decode('utf-8').split('\0') if p]
    top_files = {'README.md', 'pyproject.toml', '.gitignore', '.env.example', 'demo.html'}
    files = []
    for relative in candidates:
        if not (ROOT / relative).is_file():
            continue
        if any(part.startswith('.env') and part != '.env.example' for part in relative.parts):
            continue
        if relative.parts[0] in ('src', 'tests', 'configs', 'scripts', 'Documentation') or (
            len(relative.parts) == 1 and (relative.name in top_files or
                relative.name.startswith(('requirements-', 'constraints-')) and relative.suffix == '.txt')):
            files.append(relative)
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=str(ROOT), encoding='utf-8').strip()
    entries = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for relative in sorted(files):
            path = ROOT / relative
            archive.write(path, 'frontier-search/' + relative.as_posix())
            entries.append(dict(path=relative.as_posix(), bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        archive.writestr('frontier-search/source-manifest.json', json.dumps(dict(
            based_on_commit=revision, includes_current_working_files=True, files=entries), indent=2))
    print(json.dumps(dict(output=str(args.output), files=len(files), bytes=args.output.stat().st_size)))


if __name__ == '__main__':
    main()
