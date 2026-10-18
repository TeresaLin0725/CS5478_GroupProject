"""Package one current report and its local evidence for another computer."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True, help='Parent containing report/, recording/ or llm/recording/.')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.run_root.resolve()
    report = root / 'report'
    if not (report / 'index.html').is_file():
        parser.error('Generate run-root/report/index.html first.')
    files = {p for p in (report/'index.html', report/'inventory.json', report/'scene-update.json') if p.is_file()}
    recordings = [root / 'recording', root / 'llm/recording', root / 'nearest/recording']
    for directory in recordings:
        if not directory.is_dir():
            continue
        for name in ('integrated.mp4', 'start.png', 'final.png', 'summary.json',
                     'events.jsonl', 'manifest.json', 'evaluation-only.json', 'video.json',
                     'artifact-verification.json', 'llm-calls.json', 'llm-verification.json', 'controller.json'):
            path = directory / name
            if path.is_file():
                files.add(path)
        validation = directory.parent / 'validation.json'
        if validation.is_file():
            files.add(validation)
    for path in (root / 'environment.json', root / 'environment.png', root / 'scene-update.json',
                 root / 'spectator-isolation.json', root / 'comparison.json',
                 root / 'report/README.txt'):
        if path.is_file():
            files.add(path)
    # Include the exact dry run and certified test files referenced by the gate,
    # only when they are inside this run root and match the evidence allowlist.
    for validation in list(files):
        if validation.name != 'validation.json':
            continue
        gate = json.loads(validation.read_text(encoding='utf-8'))
        dryrun = Path(gate['run']).resolve()
        if root in dryrun.parents:
            for name in ('summary.json', 'events.jsonl', 'manifest.json', 'evaluation-only.json', 'last-processed-rgb.png'):
                if (dryrun / name).is_file():
                    files.add(dryrun / name)
        tests = Path(gate['tests_file']).resolve()
        if root in tests.parents and tests.suffix == '.log':
            for path in (tests, tests.with_suffix('.json')):
                if path.is_file():
                    files.add(path)
    for path in [root / 'browser-check/verification.json'] + list((root / 'browser-check').glob('report-*.png')):
        if path.is_file():
            files.add(path)
    # Copy only the explicit artifact allowlist; never traverse a .env, browser
    # profile, virtual environment, source checkout or the rest of the workspace.
    inventory = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(files):
            relative = path.relative_to(root).as_posix()
            archive.write(path, root.name + '/' + relative)
            inventory.append(dict(path=relative, bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        archive.writestr(root.name + '/package-manifest.json', json.dumps(inventory, indent=2))
    print(json.dumps(dict(output=str(args.output), files=len(files), bytes=args.output.stat().st_size)))


if __name__ == '__main__':
    main()
