"""Launch from any directory, with a local native venv or an explicit Python."""
from pathlib import Path
import os
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
override=os.environ.get('FRONTIER_PYTHON')
candidates=[root/'data/venv-igibson/Scripts/python.exe',root/'data/venv-igibson/bin/python']
python=Path(override) if override else next((p for p in candidates if p.is_file()),Path(sys.executable))
arguments=sys.argv[1:]
if '--env-file' not in arguments and not any(a.startswith('--env-file=') for a in arguments):
    env_file=next((p for p in (root/'.env',root.parent/'.env') if p.is_file()),root/'.env')
    arguments += ['--env-file',str(env_file)]
env=dict(os.environ,PYTHONPATH=str(root/'src'),PYTHONIOENCODING='utf-8')
raise SystemExit(subprocess.call([str(python),'-m','frontier_search.runner.integrated',*arguments],
    cwd=str(root),env=env))
