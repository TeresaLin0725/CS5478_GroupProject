"""Run the report checks in a hidden browser with a separate local profile."""
from pathlib import Path
import os
import subprocess
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]


def main():
    candidates = [Path(os.environ.get('PROGRAMFILES', 'C:/Program Files')) / 'Google/Chrome/Application/chrome.exe',
                  Path(os.environ.get('PROGRAMFILES(X86)', 'C:/Program Files (x86)')) / 'Microsoft/Edge/Application/msedge.exe']
    browser = next((p for p in candidates if p.is_file()), None)
    if browser is None:
        raise RuntimeError('A local Chromium browser is required for media playback verification.')
    output = ROOT / 'runs/demo-report-20261018/browser-check'
    output.mkdir(parents=True, exist_ok=True)
    args = [str(browser), '--headless=new', '--disable-gpu', '--disable-extensions',
            '--disable-background-networking', '--no-first-run', '--no-default-browser-check',
            '--remote-debugging-address=127.0.0.1', '--remote-debugging-port=9227',
            '--user-data-dir=' + str(output / 'profile'), 'about:blank']
    with (output / 'stdout.log').open('w') as stdout, (output / 'stderr.log').open('w') as stderr:
        process = subprocess.Popen(args, stdout=stdout, stderr=stderr,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            for _ in range(50):
                try:
                    with urlopen('http://127.0.0.1:9227/json/version', timeout=1):
                        break
                except OSError:
                    time.sleep(.1)
            subprocess.run(['node', 'scripts/check_demo_report.js'], cwd=str(ROOT),
                           check=True, timeout=120)
        finally:
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=15)


if __name__ == '__main__':
    main()
