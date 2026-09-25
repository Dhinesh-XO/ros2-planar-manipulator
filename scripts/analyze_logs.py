#!/usr/bin/env python3
"""Offline audit of the reused logger's output; no ROS or planner imports."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def read_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def analyze(directory):
    actual = read_rows(directory / 'joint_states.csv')
    command = read_rows(directory / 'joint_commands.csv')
    key = lambda row: (int(row['source_sec']), int(row['source_nanosec']), row['joint'])
    references = {key(row): row for row in command}
    errors = {}
    matched = 0
    for row in actual:
        reference = references.get(key(row))
        if reference is None:
            continue
        errors.setdefault(row['joint'], []).append(
            float(reference['position_rad'])-float(row['position_rad']))
        matched += 1
    stamps = sorted({int(row['source_sec'])*10**9+int(row['source_nanosec']) for row in actual})
    intervals = [(b-a)*1e-9 for a,b in zip(stamps,stamps[1:])]
    with (directory / 'events.jsonl').open() as stream:
        events = [json.loads(line) for line in stream]
    return {
        'session': str(directory), 'actual_rows': len(actual), 'command_rows': len(command),
        'matched_rows': matched, 'unmatched_actual_rows': len(actual)-matched,
        'mean_publication_rate_hz': 1/statistics.mean(intervals) if intervals else None,
        'max_publication_gap_ms': max(intervals)*1000 if intervals else None,
        'joint_error_rad': {joint: {'rms': math.sqrt(statistics.mean(e*e for e in values)),
                                     'max_absolute': max(abs(e) for e in values)}
                            for joint,values in errors.items()},
        'phases': [event['state'] for event in events if event['event'] == 'controller_transition'],
        'cleanly_closed': bool(events and events[-1]['event'] == 'session_end'),
        'effort_available': any(row['effort'] != '' for row in actual),
        'note': 'Observed matching/timing only; no lossless or hardware-safety guarantee.',
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.session), indent=2))
