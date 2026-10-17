"""Portable launcher; preserve Windows execution policy and use the native venv."""
from pathlib import Path
import os
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
python=root/'data/venv-igibson/Scripts/python.exe'
env=dict(os.environ,PYTHONPATH=str(root/'src'),PYTHONIOENCODING='utf-8')
raise SystemExit(subprocess.call([str(python),'-m','frontier_search.runner.integrated',*sys.argv[1:]],
    cwd=str(root),env=env,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0)))
