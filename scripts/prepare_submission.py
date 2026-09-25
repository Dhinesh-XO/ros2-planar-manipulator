#!/usr/bin/env python3
"""Export selected local evidence into a Git-tracked reviewer handover.

Run after an audit and PDF generation, then review/commit the resulting changes.
Use --check on a fresh Git clone; checking needs neither ROS nor local artifacts.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
AUDITED_COMMIT = 'd42119d'
RECORDING_COMMIT = 'b0bb113'
LIBRARY_SHA256 = '1e1c5f4bb5718839da924d192458fb2a679cc25cbded89cccf04381ad16f3dde'
FILES = {
    'design-note.pdf': 'artifacts/submission-notes/DESIGN_NOTE.pdf',
    'simulation-to-hardware.pdf': 'artifacts/submission-notes/HARDWARE_TRANSITION.pdf',
    'media/pick-and-place.mp4': 'artifacts/enhanced-release-demo.mp4',
    'media/gazebo-pick-and-place.mp4': 'artifacts/gazebo-final-demo.mp4',
    'media/workcell.png': 'artifacts/enhanced-release-preview.png',
    'evidence/software-acceptance.json': 'artifacts/submission-audit/clean-package/validation.json',
    'evidence/telemetry-position.csv': 'artifacts/submission-audit/clean-package/telemetry_position.csv',
    'evidence/telemetry-velocity-pid.csv': 'artifacts/submission-audit/clean-package/telemetry_velocity_pid.csv',
    'evidence/clean-build-and-test.log': 'artifacts/submission-audit/clean-package/build-and-test.log',
    'evidence/final-zip-build-and-test.log': 'artifacts/submission-audit/final-bundle-check.log',
    'evidence/reliability.json': 'artifacts/submission-audit/reliability-final/report.json',
    'evidence/gazebo.json': 'artifacts/submission-audit/physics/gazebo_validation.json',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    for name, expected in manifest['sha256'].items():
        assert digest(directory / name) == expected, f'Changed evidence: {name}'
    library = ROOT / 'src/planar_arm_control/planar_arm_control/planar_arm.py'
    assert digest(library) == LIBRARY_SHA256, 'Supplied kinematics changed'
    software = json.loads((directory / 'evidence/software-acceptance.json').read_text())
    assert software['library_sha256'] == LIBRARY_SHA256
    assert {m['mode'] for m in software['modes']} == {'position', 'velocity_pid'}
    assert all(m['passed'] for m in software['modes'])
    reliability = json.loads((directory / 'evidence/reliability.json').read_text())
    assert reliability['passed'] and len(reliability['checks']) == 25
    assert all(c['passed'] for c in reliability['checks'])
    assert json.loads((directory / 'evidence/gazebo.json').read_text())['passed']
    print(f'Verified {len(manifest["sha256"])} deliverable files and supplied-library hash.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    destination = ROOT / 'submission'
    if not args.check:
        subprocess.run(['git', 'diff', '--exit-code', AUDITED_COMMIT, '--', 'src'],
                       cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
        for source in FILES.values():
            assert (ROOT / source).is_file(), f'Missing local artifact: {source}'
        for name, source in FILES.items():
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / source, target)
        manifest = {
            'schema_version': 1,
            'audited_runtime_commit': AUDITED_COMMIT,
            'recordings_commit': RECORDING_COMMIT,
            'supplied_library_sha256': LIBRARY_SHA256,
            'written_answers': 'Review drafts; candidate must review in their own words.',
            'validation_scope': 'Simulation on Ubuntu 22.04 / ROS 2 Humble; not hardware or fresh-OS validation.',
            'source_artifacts': FILES,
            'sha256': {name: digest(destination / name) for name in FILES},
        }
        (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    verify(destination)


if __name__ == '__main__':
    main()
