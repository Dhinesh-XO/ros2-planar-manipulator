#!/usr/bin/env python3
"""Package committed source/history and an explicit evidence allowlist.

No venv, build tree, downloaded dependency, failed recording or crash-test log
is swept into the bundle. The manifest records hashes and evidence provenance.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--tracked-only', action='store_true',
                        help='Use Git-tracked submission evidence; works from a fresh clone')
    args = parser.parse_args()
    if git('status', '--porcelain'):
        parser.error('Commit/review the worktree first; the bundle must match its Git history.')
    if args.destination.exists():
        parser.error('Destination exists; choose a new filename to preserve earlier bundles.')
    artifacts = [
        'artifacts/enhanced-release-demo.mp4',
        'artifacts/gazebo-final-demo.mp4',
        'artifacts/validation.json',
        'artifacts/telemetry_position.csv',
        'artifacts/telemetry_velocity_pid.csv',
        'artifacts/gazebo_validation.json',
        'artifacts/submission-notes/DESIGN_NOTE.pdf',
        'artifacts/submission-notes/HARDWARE_TRANSITION.pdf',
        'artifacts/submission-audit/reliability-final/report.json',
        'artifacts/submission-audit/physics/gazebo_validation.json',
        'artifacts/submission-audit/physics/gazebo_validation.log',
    ]
    for folder in ('artifacts/submission-audit/acceptance',
                   'artifacts/submission-audit/physics/telemetry',
                   'artifacts/submission-audit/clean-package'):
        artifacts.extend(str(p.relative_to(ROOT)) for p in sorted((ROOT / folder).rglob('*'))
                         if p.is_file())
    if args.tracked_only:
        artifacts = []
        subprocess.run([sys.executable, str(ROOT / 'scripts/prepare_submission.py'), '--check'],
                       cwd=ROOT, check=True)
    else:
        for report in ('artifacts/submission-audit/reliability-final/report.json',
                       'artifacts/submission-audit/physics/gazebo_validation.json'):
            assert json.loads((ROOT / report).read_text())['passed'], f'Failed evidence: {report}'
    files = [*git('ls-files').splitlines(), *artifacts]
    files.extend(str(p.relative_to(ROOT)) for p in sorted((ROOT / '.git').rglob('*'))
                 if p.is_file() and not p.name.endswith('.lock'))
    assert all((ROOT / name).is_file() for name in files), 'Missing evidence file'
    manifest = {
        'source_commit': git('rev-parse', 'HEAD'),
        'recordings_source_commit': 'b0bb113',
        'notes': ['Plain-English submission notes; candidate must confirm understanding before sending.',
                  'Recordings precede fault-path hardening; audit reports cover those fixes.',
                  'Clean extraction tests use the same Ubuntu/Humble system, not a fresh OS.',
                  'Gazebo and its compatible bridge must be installed separately.'],
        'sha256': {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                   for name in sorted(set(files))},
    }
    with zipfile.ZipFile(args.destination, 'x', compression=zipfile.ZIP_DEFLATED) as bundle:
        for name in sorted(set(files)):
            bundle.write(ROOT / name, name)
        bundle.writestr('BUNDLE_MANIFEST.json', json.dumps(manifest, indent=2)+'\n')
    with zipfile.ZipFile(args.destination) as bundle:
        assert bundle.testzip() is None
    digest = hashlib.sha256(args.destination.read_bytes()).hexdigest()
    print(json.dumps({'archive': str(args.destination.resolve()), 'sha256': digest,
                      'source_commit': manifest['source_commit'], 'files': len(set(files))}, indent=2))


if __name__ == '__main__':
    main()
