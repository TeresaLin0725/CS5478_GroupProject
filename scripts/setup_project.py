"""Install the complete verified Windows runtime and checksum-verified YOLO model."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
MODEL_SHA256 = '85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5'
MODEL_URL = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s.pt'


def run(*args):
    subprocess.run([str(a) for a in args], cwd=str(ROOT), check=True)


def download_model():
    model = ROOT / 'weights/yolo11/yolo11s.pt'
    model.parent.mkdir(parents=True, exist_ok=True)
    if not model.exists():
        partial = model.with_suffix('.pt.part')
        with urllib.request.urlopen(MODEL_URL, timeout=60) as response, partial.open('wb') as output:
            shutil.copyfileobj(response, output)
        if hashlib.sha256(partial.read_bytes()).hexdigest() != MODEL_SHA256:
            raise RuntimeError('YOLO download checksum mismatch: ' + str(partial))
        partial.replace(model)
    if hashlib.sha256(model.read_bytes()).hexdigest() != MODEL_SHA256:
        raise RuntimeError('Existing YOLO weights have an unexpected checksum: ' + str(model))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', choices=('cuda', 'cpu'), default='cuda')
    args = parser.parse_args()
    os.environ.setdefault('UV_CACHE_DIR', str(ROOT / 'data/uv-cache'))
    if os.name != 'nt':
        parser.error('This installer builds Windows DLLs. See Documentation/chinese/running.md for Linux setup.')
    for executable in ('git', 'uv'):
        if shutil.which(executable) is None:
            parser.error('Install ' + executable + ' and reopen the terminal first.')
    run(sys.executable, ROOT / 'scripts/setup_igibson.py')
    python = ROOT / 'data/venv-igibson/Scripts/python.exe'
    requirements = 'requirements-integrated.txt' if args.device == 'cuda' else 'requirements-integrated-cpu.txt'
    index = 'cu121' if args.device == 'cuda' else 'cpu'
    run('uv', 'pip', 'install', '--python', python, '-r', ROOT / requirements,
        '-c', ROOT / 'constraints-windows-py38.txt',
        '--extra-index-url', 'https://download.pytorch.org/whl/' + index,
        '--index-strategy', 'unsafe-best-match')
    run('uv', 'pip', 'install', '--python', python, '--no-deps', '-e', ROOT)
    download_model()
    run(python, ROOT / 'scripts/check_environment.py', '--output', ROOT / 'runs/environment.json')
    print('Ready. Run: python scripts/run_integrated.py --policy nearest --output runs/my-first-run')


if __name__ == '__main__':
    main()
