# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Registro locale per missione: eventi, risultati JSON e resoconto leggibile."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
import uuid


def now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


class MissionJournal:
    def __init__(self, root, plan):
        self.run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
        self.directory = Path(root).expanduser() / self.run_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.started = time.monotonic()
        self.summary = {'run_id': self.run_id, 'started_at': now(), 'status': 'RUNNING',
                        'evidence': 'Feedback del simulatore Webots; nessuna validazione hardware.',
                        'plan': plan, 'steps': []}
        self.event('mission_started', {'plan_id': plan['plan_id'], 'transfers': plan['transfers']})
        self.save()

    def event(self, kind, data):
        record = {'at': now(), 'event': kind, **data}
        with (self.directory / 'events.jsonl').open('a') as output:
            output.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
            output.flush()
            os.fsync(output.fileno())

    def step_started(self, step):
        self.event('step_started', {'step': step})

    def step_finished(self, step, status, duration, feedback):
        item = {'step': step, 'status': status, 'duration_seconds': round(duration, 3), 'feedback': feedback}
        self.event('step_finished', item)
        self.summary['steps'].append(item)
        self.save()

    def finish(self, status, reason):
        self.summary.update(status=status, reason=reason, ended_at=now(),
                            duration_seconds=round(time.monotonic() - self.started, 3))
        self.event('mission_finished', {'status': status, 'reason': reason})
        self.save()

    def save(self):
        path = self.directory / 'report.json'
        temp = self.directory / 'report.json.tmp'
        temp.write_text(json.dumps(self.summary, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
        os.replace(temp, path)
        lines = [f"# {self.summary['plan']['title']}", '', f"Esecuzione: {self.run_id}",
                 f"Stato: {self.summary['status']}", f"Avvio UTC: {self.summary['started_at']}",
                 '', self.summary['evidence'], '',
                 '| Passo | Campione | Origine → destinazione | Esito | Durata osservata |',
                 '| --- | --- | --- | --- | --- |']
        for item in self.summary['steps']:
            step = item['step']
            lines.append(f"| {step['index']} | {step['sample']} | {step['source']} → {step['destination']} | {item['status']} | {item['duration_seconds']} s |")
        if 'reason' in self.summary:
            lines += ['', self.summary['reason'], f"Durata complessiva: {self.summary['duration_seconds']} s"]
        lines += ['', 'Le durate includono controlli preliminari e comunicazione, oltre al moto simulato.',
                  'Il report JSON contiene piano e feedback osservati; events.jsonl conserva la sequenza degli eventi.']
        temp = self.directory / 'report.md.tmp'
        temp.write_text('\n'.join(lines) + '\n')
        os.replace(temp, self.directory / 'report.md')
