# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Missioni pianificate, confermate e documentate sopra il controller esistente."""
import time
from .knowledge import normalize
from .planner import fingerprint, positions
from .journal import MissionJournal


class MissionDirector:
    def __init__(self, node, planner, journal_root, plan_ttl=300.0):
        self.node, self.planner, self.journal_root = node, planner, journal_root
        self.plan_ttl = plan_ttl
        self.prepared = None
        self.prepared_at = 0.0
        self.generation = 0
        self.running = False
        self.executing = None
        self.journal = None
        self.index = 0
        self.step_started_at = 0.0
        self.last_event = {'status': 'IDLE', 'message': 'Nessuna missione pianificata.'}

    def emit(self, status, message, **data):
        event = {'status': status, 'message': message, **data}
        if self.journal is not None and self.executing is not None:
            event.update(run_id=self.journal.run_id, completed_steps=self.index,
                         total_steps=len(self.executing['steps']),
                         report_path=str(self.journal.directory / 'report.md'))
        self.last_event = event
        self.node.mission_event(event)

    def handle(self, text):
        value = normalize(text)
        for prefix in ('ehi doc ', 'doc '):
            if value.startswith(prefix):
                value = value[len(prefix):]
                break
        if value == 'esegui piano':
            self.commit()
        elif value == 'annulla piano':
            if self.running:
                raise ValueError('Missione in esecuzione: usa stop per arrestarla.')
            self.invalidate()
            self.emit('CANCELLED', 'Piano annullato. Nessun trasferimento avviato.')
        elif value == 'stato missione':
            self.node.mission_event(self.last_event)
        elif value == 'pianifica':
            raise ValueError('Specifica un protocollo: prepara, riordina, ciclo completo o scambia supporti.')
        else:
            protocol = self.planner.resolve(value)
            if protocol is None:
                return False
            self.plan(protocol)
        return True

    def invalidate(self):
        self.generation += 1
        self.prepared = None

    def plan(self, protocol):
        if self.running or self.node.active:
            raise ValueError('Attendi la richiesta in corso prima di pianificare.')
        self.invalidate()
        generation = self.generation
        self.journal = None
        self.executing = None
        self.emit('PLANNING', 'Leggo lo stato del laboratorio per costruire il piano.')

        def received(scene, error):
            if generation != self.generation:
                return
            if error:
                self.emit('REJECTED', error)
                return
            try:
                self.prepared = self.planner.build(protocol, scene)
                self.prepared_at = time.monotonic()
                count = self.prepared['transfers']
                self.emit('PLAN_READY', f'Piano pronto: {count} trasferimenti. Controlla i passaggi e usa esegui piano.',
                          plan=self.prepared)
            except (ValueError, KeyError, TypeError) as exc:
                self.prepared = None
                self.emit('REJECTED', str(exc))
        self.node.inspect_scene(received)

    def commit(self):
        if self.running or self.node.active:
            raise ValueError('Una richiesta è già in corso.')
        if self.prepared is None:
            raise ValueError('Prima crea un piano con pianifica ciclo completo.')
        if time.monotonic() - self.prepared_at > self.plan_ttl:
            self.invalidate()
            raise ValueError('Piano scaduto: ripeti la pianificazione sullo stato attuale.')
        generation = self.generation
        plan = self.prepared
        self.emit('VALIDATING', 'Ricontrollo lo stato prima di eseguire il piano.')

        def received(scene, error):
            if generation != self.generation:
                return
            if error:
                self.invalidate()
                self.emit('REJECTED', error)
                return
            try:
                self.planner.ready(scene)
                if fingerprint(scene) != plan['scene_fingerprint']:
                    raise ValueError('La scena è cambiata dopo l’anteprima. Pianifica di nuovo.')
                self.journal = MissionJournal(self.journal_root, plan)
            except (ValueError, KeyError, TypeError, OSError) as exc:
                self.invalidate()
                self.emit('REJECTED', str(exc))
                return
            self.prepared = None
            self.executing = plan
            self.running = True
            self.index = 0
            self.start_next()
        self.node.inspect_scene(received)

    def start_next(self):
        if not self.running:
            return
        if self.index == len(self.executing['steps']):
            self.terminal('COMPLETED', 'Missione completata: tutti gli stati finali sono stati confermati dal simulatore.')
            return
        step = self.executing['steps'][self.index]
        self.step_started_at = time.monotonic()
        try:
            self.journal.step_started(step)
            self.emit('RUNNING', f"Passo {step['index']}/{len(self.executing['steps'])}: {step['sample']} → {step['destination']}", step=step)
            self.node.request({'action': 'TRANSFER', 'params': {
                'sample': step['sample'], 'destination': step['destination']}},
                from_mission=True, expected_positions=step['expected_before'])
        except (ValueError, OSError) as exc:
            self.node.active = False
            self.terminal('FAILED', f'Impossibile avviare il passo: {exc}')

    def step_result(self, status, message, feedback):
        if not self.running:
            return
        step = self.executing['steps'][self.index]
        if status == 'DONE':
            try:
                if positions(feedback) != step['expected_after']:
                    raise ValueError('Le posizioni finali dei campioni non coincidono con il piano.')
            except (ValueError, KeyError, TypeError) as exc:
                status, message = 'UNCONFIRMED', str(exc)
        duration = time.monotonic() - self.step_started_at
        try:
            self.journal.step_finished(step, status, duration, feedback)
        except (ValueError, OSError) as exc:
            self.terminal('JOURNAL_ERROR', f'Registro non aggiornabile; missione interrotta: {exc}')
            return
        if status != 'DONE':
            self.terminal(status if status in ('STOPPED', 'UNCONFIRMED') else 'FAILED', message)
            return
        self.index += 1
        self.emit('STEP_COMPLETED', f"Passo {step['index']} confermato in {duration:.1f} s.", step=step,
                  duration_seconds=round(duration, 3))
        self.start_next()

    def terminal(self, status, reason):
        self.running = False
        self.node.active = False
        try:
            self.journal.finish(status, reason)
        except (ValueError, OSError) as exc:
            status, reason = 'JOURNAL_ERROR', f'{reason} Registro non salvato completamente: {exc}'
        self.emit(status, reason)

    def shutdown(self):
        self.invalidate()
        if self.running:
            self.running = False
            try:
                self.journal.finish('INTERRUPTED', 'Coordinatore chiuso durante la missione. Stato del moto non confermato; nessuna ripresa automatica.')
            except (ValueError, OSError) as exc:
                self.node.get_logger().error(f'Impossibile chiudere il registro: {exc}')
