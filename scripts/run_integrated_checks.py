"""Capture regression output/exit status without PowerShell native-stderr translation."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from validate_integrated import source_fingerprint,ROOT

parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();args.output.parent.mkdir(parents=True,exist_ok=True)
before=source_fingerprint()
with args.output.open('w',encoding='utf-8') as output:
    result=subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-q'],
        cwd=str(ROOT),env=dict(os.environ,PYTHONPATH=str(ROOT/'src'),
            YOLO_CONFIG_DIR=str(ROOT/'data/yolo-config')),
        stdout=output,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
text=args.output.read_text(encoding='utf-8');match=re.search(r'Ran (\d+) tests',text)
report=dict(returncode=result.returncode,source_fingerprint=before,
            source_unchanged=before==source_fingerprint(),tests_count=int(match.group(1)) if match else None)
args.output.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2));raise SystemExit(result.returncode)
