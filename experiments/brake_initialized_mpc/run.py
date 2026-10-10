#!/usr/bin/env python3
"""Use unchanged frozen plant/runner with initialization-only MPC variant."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import mpc

ROOT = Path(__file__).resolve().parent
BASE = ROOT.parent / 'reverse_parking_mpc'


def load_runner():
    spec = importlib.util.spec_from_file_location('unchanged_reverse_runner', BASE / 'run.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.mpc = mpc
    return module


def verify():
    manifest = json.loads((ROOT / 'version.json').read_text())
    for name, digest in manifest['source_sha256'].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError('Version hash mismatch: ' + name)
    for name, digest in manifest['parent_core_sha256'].items():
        if hashlib.sha256((BASE / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError('Parent baseline hash mismatch: ' + name)
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT / 'results')
    parser.add_argument('--seed', type=int, default=20261010)
    parser.add_argument('--mode', choices=['two', 'single'], default='two')
    args = parser.parse_args()
    manifest = verify()
    args.output.mkdir(parents=True, exist_ok=True)
    config = json.loads((ROOT / 'config.json').read_text())
    controls = [np.array(json.loads((ROOT / f'reference_{car}_controls.json').read_text())) for car in 'AB']
    runner = load_runner()
    runner.write(args.output / 'manifest.json', dict(version=manifest, config=config))
    summary = runner.run(config, controls, args.output, args.seed, args.mode == 'two')
    print(json.dumps(summary, indent=2), flush=True)
    raise SystemExit(0 if summary['success'] else 1)


if __name__ == '__main__':
    main()
