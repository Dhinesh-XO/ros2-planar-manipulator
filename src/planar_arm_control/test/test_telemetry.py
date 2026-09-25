"""Prove the reused helper's behavior, including its overwrite caveat."""
import csv
import json

from planar_arm_control.vendor.telemetry_csv import CsvLog, JsonlLog


def test_csv_header_rows_and_flush(tmp_path):
    log = CsvLog(tmp_path / 'joints.csv', ['joint', 'position'])
    log.write(['joint_1', 0.5])
    log.write(['joint_2', -0.3])
    log.flush()
    assert log.count == 2
    with log.path.open() as stream:
        assert list(csv.reader(stream)) == [
            ['joint', 'position'], ['joint_1', '0.5'], ['joint_2', '-0.3']]
    log.close()


def test_jsonl_appends_complete_records(tmp_path):
    path = tmp_path / 'events.jsonl'
    for phase in ('PICKING', 'PLACING'):
        log = JsonlLog(path)
        log.write({'state': phase, 'note': 'quoted "text"\nnewline'})
        log.close()
    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert [record['state'] for record in records] == ['PICKING', 'PLACING']


def test_csv_reopening_overwrites_so_adapter_must_use_unique_directories(tmp_path):
    path = tmp_path / 'sample.csv'
    log = CsvLog(path, ['first'])
    log.write(['old'])
    log.close()
    replacement = CsvLog(path, ['new'])
    replacement.close()
    assert path.read_text().strip() == 'new'
