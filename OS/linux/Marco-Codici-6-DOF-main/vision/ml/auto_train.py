#!/usr/bin/env python3
"""Auto-retrain: export dataset + training + export ONNX.

Lanciato dal server (server.js) quando ci sono abbastanza campioni approvati nuovi.
"""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def run(script, *extra):
    print(f'=== {script} {" ".join(extra)} ===', flush=True)
    r = subprocess.run([sys.executable, script, *extra], cwd=ROOT)
    if r.returncode:
        print(f'FALLITO: {script} (exit {r.returncode})', flush=True)
        sys.exit(r.returncode)


run('export_dataset.py')
run('train.py', '--epochs', '100')
print('AUTO-TRAIN OK', flush=True)
