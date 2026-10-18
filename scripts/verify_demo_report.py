"""Run the report checks in a hidden browser with a separate local profile."""
from pathlib import Path
import argparse
import os
import subprocess
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, action='append', help='Report directory to check; may be repeated.')
    parser.add_argument('--output', type=Path, default=ROOT / 'report/verification/demo')
    parser.add_argument('--port', type=int, default=9227)
    parser.add_argument('--checker', type=Path, default=ROOT / 'scripts/check_demo_report.js')
    options = parser.parse_args()
    candidates = [Path(os.environ.get('PROGRAMFILES', 'C:/Program Files')) / 'Google/Chrome/Application/chrome.exe',
                  Path(os.environ.get('PROGRAMFILES(X86)', 'C:/Program Files (x86)')) / 'Microsoft/Edge/Application/msedge.exe']
    browser = next((p for p in candidates if p.is_file()), None)
    if browser is None:
        raise RuntimeError('A local Chromium browser is required for media playback verification.')
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    args = [str(browser), '--headless=new', '--disable-gpu', '--disable-extensions',
            '--disable-background-networking', '--no-first-run', '--no-default-browser-check',
            '--remote-debugging-address=127.0.0.1', '--remote-debugging-port=' + str(options.port),
            '--user-data-dir=' + str(output / 'profile'), 'about:blank']
    with (output / 'stdout.log').open('w') as stdout, (output / 'stderr.log').open('w') as stderr:
        process = subprocess.Popen(args, stdout=stdout, stderr=stderr,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            for _ in range(50):
                try:
                    with urlopen('http://127.0.0.1:' + str(options.port) + '/json/version', timeout=1):
                        break
                except OSError:
                    time.sleep(.1)
            result = subprocess.run(['node', str(options.checker.resolve())] + [str(p.resolve()) for p in (options.report or [])],
                           cwd=str(ROOT), env=dict(os.environ, DEMO_REPORT_CHECK_OUTPUT=str(output),
                               DEMO_REPORT_CHECK_PORT=str(options.port)),
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding='utf-8', timeout=120)
            (output / 'node.log').write_text(result.stdout, encoding='utf-8')
            result.check_returncode()
            print(result.stdout)
        finally:
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=15)


if __name__ == '__main__':
    main()
