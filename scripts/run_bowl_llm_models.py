"""Re-run only the 12 bowl48 llm_guided episodes with a different hosted model.

Everything except the model endpoint is the bowl48 protocol: kitchen_bowl base
task, bowl48 design, 1 cm depth noise, 1 % dropout, base noise seed 123, two
concurrent simulators. Keys are read from the project .env and passed only via
the process environment (never printed or written to disk).
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MODELS = {
    'deepseek-flash': ('https://api.deepseek.com/v1', 'DEEPSEEK_LLM_API_KEY'),
    'deepseek-v4-pro': ('https://api.deepseek.com/v1', 'DEEPSEEK_LLM_API_KEY'),
    'gpt-4o': ('https://api.openai.com/v1', 'OPENAI_API_KEY'),
}


def read_env(path):
    values = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        key, sep, value = line.strip().partition('=')
        if sep and not key.startswith('#'):
            values[key.strip()] = value.strip().strip('"\'')
    return values


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', choices=sorted(MODELS), required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--tests-certificate', type=Path, required=True)
    p.add_argument('--env-file', type=Path, default=ROOT.parent / '.env')
    p.add_argument('--resume', action='store_true', help='run an already frozen suite')
    p.add_argument('--freeze-only', action='store_true')
    a = p.parse_args()
    base_url, key_name = MODELS[a.model]
    secrets = read_env(a.env_file)
    if not secrets.get(key_name):
        raise SystemExit('Missing ' + key_name + ' in ' + str(a.env_file))
    env = dict(os.environ, LLM_BASE_URL=base_url, LLM_MODEL=a.model, LLM_API_KEY=secrets[key_name])
    script = str(ROOT / 'scripts/run_paired_benchmark.py')
    if a.resume:
        cmd = [sys.executable, script, 'run', '--output', str(a.output), '--only-llm']
    else:
        action = 'freeze' if a.freeze_only else 'all'
        cmd = [sys.executable, script, action, '--output', str(a.output)] + ([] if a.freeze_only else ['--only-llm']) + [
               '--workers', '2', '--task', 'kitchen_bowl', '--design', 'bowl48',
               '--tests-certificate', str(a.tests_certificate),
               '--noise', '--depth-noise-std-m', '.01', '--depth-dropout-prob', '.01',
               '--noise-seed', '123']
    raise SystemExit(subprocess.call(cmd, cwd=str(ROOT), env=env))


if __name__ == '__main__':
    main()
