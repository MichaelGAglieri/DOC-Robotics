# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Demo Webots: due flaconi, quattro supporti e presa a tre dita.

La presa e' un vincolo cinematico del supervisore, verificato in prossimita'
del flacone. Non e' una prova di attrito o carico del gripper reale.
"""
import json
import math
import os
from pathlib import Path
import socket
import time
import uuid

from controller import Supervisor

CONFIG_FILE = Path(__file__).resolve().parents[2] / 'laboratorio.json'
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint']
SPEED = 0.7
POSITION_TOLERANCE = 0.003
GRASP_TOLERANCE = 0.004


def quintic(u):
    return u**3 * (10 - 15*u + 6*u*u)


class Motion:
    def __init__(self, lab, target):
        self.lab, self.target, self.elapsed = lab, target, 0.0
        if lab.location == target:
            self.path = [lab.config['waypoint'][target]] * 2
        else:
            key = f'{lab.location}->{target}'
            if key not in lab.config['routes']:
                raise ValueError(f'Tratta non definita: {key}. Ricaricare il mondo dopo ABORT.')
            self.path = lab.config['routes'][key]
        actual = lab.joints()
        if max(abs(a-b) for a, b in zip(actual, self.path[0])) > 0.04:
            raise ValueError('Braccio fuori dalla posa iniziale: ricaricare il mondo.')
        slope = max(abs(b[j]-a[j]) * (len(self.path)-1)
                    for a, b in zip(self.path, self.path[1:]) for j in range(len(JOINTS)))
        self.duration = max(0.5, 1.875*slope/SPEED)
        self.lab.phase = f'Movimento: {target}'

    def tick(self):
        self.elapsed += self.lab.dt
        u = quintic(min(1.0, self.elapsed/self.duration)) * (len(self.path)-1)
        i = min(int(u), len(self.path)-2)
        fraction = u-i
        goal = [a+(b-a)*fraction for a, b in zip(self.path[i], self.path[i+1])]
        for motor, q in zip(self.lab.motors, goal):
            motor.setPosition(q)
        if self.elapsed >= self.duration:
            if max(abs(a-b) for a, b in zip(self.lab.joints(), self.path[-1])) < POSITION_TOLERANCE:
                self.lab.location = self.target
                return True
            if self.elapsed > self.duration+3.0:
                raise ValueError('Giunti non arrivati al bersaglio entro 3 s: possibile collisione.')
        return False


class Grip:
    def __init__(self, lab, closed, opening=None):
        self.lab, self.closed, self.elapsed = lab, closed, 0.0
        self.goal = 0.026 if closed else 0.0
        if opening is not None:
            self.goal = 0.026 * max(0, min(1, (170-opening)/160))
            self.closed = self.goal >= 0.0253
        lab.phase = 'Chiusura pinza' if closed else 'Apertura pinza'
        for motor in lab.fingers:
            motor.setPosition(self.goal)

    def tick(self):
        self.elapsed += self.lab.dt
        arrived = all(abs(s.getValue()-self.goal) < 0.0007 for s in self.lab.finger_sensors)
        if arrived and self.elapsed >= 0.35:
            self.lab.closed = self.closed
            return True
        if self.elapsed > 2:
            values = [round(s.getValue(), 5) for s in self.lab.finger_sensors]
            raise ValueError(f'Pinza non arrivata al bersaglio {self.goal}: {values}')
        return False


class Transfer:
    def __init__(self, lab, sample, destination):
        self.lab = lab
        if sample not in lab.positions or destination not in lab.config['stations']:
            raise ValueError('Campione o destinazione sconosciuti.')
        if lab.held:
            raise ValueError('Un campione e\' ancora agganciato: ricaricare la scena.')
        source = lab.positions[sample]
        self.source = source  # Metadata for the read-only telemetry channel.
        if source == destination:
            raise ValueError('Campione gia\' nella destinazione richiesta.')
        if destination in lab.positions.values():
            raise ValueError(f'Destinazione occupata: {destination}')
        self.steps = [
            ('move', f'{source}_approach'), ('grip', False),
            ('move', f'{source}_grip'), ('grip', True), ('attach', sample),
            ('move', f'{source}_approach'), ('move', f'{destination}_approach'),
            ('move', f'{destination}_grip'), ('grip', False), ('release', destination),
            ('move', f'{destination}_approach'), ('move', 'home'),
        ]
        self.current = None

    def tick(self):
        if self.current:
            if not self.current.tick():
                return False
            self.current = None
        while self.steps:
            kind, value = self.steps.pop(0)
            if kind == 'move':
                self.current = Motion(self.lab, value)
                return False
            if kind == 'grip':
                self.current = Grip(self.lab, value)
                return False
            if kind == 'attach':
                self.lab.attach(value)
            elif kind == 'release':
                self.lab.release(value)
        return True


class Lab:
    def __init__(self, robot):
        self.robot = robot
        self.config = json.loads(CONFIG_FILE.read_text())
        if self.config.get('joint_names') != JOINTS or self.config.get('arm_dof') != len(JOINTS):
            raise RuntimeError('Waypoints incompatibili con il braccio a 5 assi: rigenerare il laboratorio.')
        self.ts = int(robot.getBasicTimeStep())
        self.dt = self.ts/1000
        self.motors = [robot.getDevice(n) for n in JOINTS]
        self.sensors = [m.getPositionSensor() for m in self.motors]
        self.fingers = [robot.getDevice(f'lab_finger_{i}') for i in range(1, 4)]
        self.finger_sensors = [m.getPositionSensor() for m in self.fingers]
        for s in self.sensors+self.finger_sensors:
            s.enable(self.ts)
        for m, q in zip(self.motors, self.config['waypoint']['home']):
            m.setVelocity(SPEED)
            m.setPosition(q)
        for m in self.fingers:
            m.setPosition(0)
        self.tcp = robot.getFromDef('LAB_TCP')
        self.nodes = {s: robot.getFromDef(s.upper()) for s in self.config['samples']}
        if self.tcp is None or any(n is None for n in self.nodes.values()):
            raise RuntimeError('Mondo incompleto: manca TCP o un flacone.')
        self.positions = dict(self.config['samples'])
        self.location, self.held, self.closed = 'home', None, False
        self.phase = 'Pronto'
        self.keyboard = robot.getKeyboard()
        self.keyboard.enable(self.ts)

    def joints(self):
        q = [s.getValue() for s in self.sensors]
        if not all(math.isfinite(v) for v in q):
            raise ValueError('Sensori dei giunti non disponibili.')
        return q

    def tilt(self):
        return math.degrees(math.acos(max(-1, min(1, -self.tcp.getOrientation()[8]))))

    def near(self, point):
        return math.dist(self.tcp.getPosition(), point) <= GRASP_TOLERANCE and self.tilt() < 2

    def attach(self, sample):
        actual = self.nodes[sample].getPosition()
        offset = self.config['center_offset']
        grasp = [actual[i]-offset[i] for i in range(3)]
        if not self.closed or not self.near(grasp):
            raise ValueError('Presa rifiutata: pinza aperta, lontana o inclinata.')
        self.held = sample
        self.follow()

    def follow(self):
        if self.held:
            node = self.nodes[self.held]
            p = self.tcp.getPosition()
            node.getField('translation').setSFVec3f([p[i]+self.config['center_offset'][i] for i in range(3)])
            node.getField('rotation').setSFRotation([0, 0, 1, 0])

    def release(self, destination):
        if not self.held or self.closed or not self.near(self.config['stations'][destination]):
            raise ValueError('Deposito rifiutato: campione assente o pinza fuori supporto.')
        # Il centro misurato resta dove arriva il TCP: nessuno scatto sul supporto.
        self.follow()
        self.positions[self.held] = destination
        self.held = None

    def diagnose(self):
        return {'tcp': self.tcp.getPosition(), 'tilt_degrees': self.tilt(),
                'fingers': [s.getValue() for s in self.finger_sensors], 'closed': self.closed,
                'joints': self.joints(), 'held': self.held, 'location': self.location,
                'samples': {s: {'station': self.positions[s], 'center': n.getPosition(),
                                'orientation': n.getOrientation()} for s, n in self.nodes.items()},
                'phase': self.phase, 'scenario': 'laboratorio',
                'arm_dof': len(JOINTS), 'joint_names': JOINTS}

    def freeze(self):
        for m, q in zip(self.motors, self.joints()):
            m.setPosition(q)
        for m, s in zip(self.fingers, self.finger_sensors):
            m.setPosition(s.getValue())
        self.location = None
        self.phase = 'Arrestato. R per ricaricare la scena.'

    def hud(self):
        self.robot.setLabel(0, 'DOC  |  LABORATORIO', 0.02, 0.02, 0.043, 0x242729, 0)
        self.robot.setLabel(1, 'D: demo   A/B: trasferisci campione   SPAZIO: stop   R: reset',
                            0.02, 0.058, 0.027, 0x45494A, 0)
        self.robot.setLabel(2, self.phase, 0.02, 0.086, 0.031, 0x313637, 0)



class Telemetry:
    """Read-only snapshots on a separate, non-blocking loopback UDP channel."""
    PHASES = [
        ('approach', 'Avvicinamento al campione'),
        ('open', 'Apertura della pinza'),
        ('lower_source', 'Discesa sul campione'),
        ('grasp', 'Chiusura della pinza'),
        ('check_grasp', 'Verifica della presa'),
        ('lift', 'Sollevamento verticale'),
        ('transfer', 'Trasferimento in quota'),
        ('lower_destination', 'Discesa sulla destinazione'),
        ('open_destination', 'Apertura della pinza'),
        ('deposit', 'Verifica del deposito'),
        ('clear', 'Allontanamento dal supporto'),
        ('home', 'Ritorno HOME'),
    ]

    def __init__(self):
        self.session = uuid.uuid4().hex
        self.sequence, self.last_sent = 0, 0.0
        self.last_result, self.force = None, True
        self.udp = None
        try:
            port = int(os.getenv('DOC_TELEMETRY_PORT', '10000'))
            if not 1 <= port <= 65535:
                raise ValueError('porta non valida')
            self.target = ('127.0.0.1', port)
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.setblocking(False)
        except (OSError, ValueError) as error:
            print(f'Telemetria facoltativa non disponibile: {error}', flush=True)

    def record(self, cmd, status, reason=None):
        if cmd.get('action') not in ('TRANSFER', 'MOVE_TO', 'GRIPPER', 'ABORT'):
            return
        params = cmd.get('params')
        params = params if isinstance(params, dict) else {}
        self.last_result = {
            'id': str(cmd['id']) if cmd.get('id') is not None else cmd.setdefault('_telemetry_id', uuid.uuid4().hex),
            'action': cmd.get('action'), 'status': status,
            'sample': params.get('sample'), 'destination': params.get('destination'),
            'source': cmd.get('_telemetry_source'), 'reason': reason,
        }
        self.force = True

    def snapshot(self, lab, active, pending):
        diagnosis = lab.diagnose()
        mission = None
        phase_code, phase_label = 'ready', 'Pronto'
        if active:
            cmd, operation = active
            params = cmd.get('params')
            params = params if isinstance(params, dict) else {}
            mission = {
                'id': str(cmd['id']) if cmd.get('id') is not None else cmd.setdefault('_telemetry_id', uuid.uuid4().hex),
                'action': cmd.get('action'), 'sample': params.get('sample'),
                'source': getattr(operation, 'source', None),
                'destination': params.get('destination'),
            }
            if isinstance(operation, Transfer):
                step = max(1, min(len(self.PHASES), len(self.PHASES)-len(operation.steps)))
                phase_code, phase_label = self.PHASES[step-1]
                mission.update(step=step, total_steps=len(self.PHASES))
            else:
                phase_code, phase_label = 'direct', lab.phase
        status = 'running' if active else 'queued' if pending else 'idle'
        if lab.phase.startswith('Arrestato'):
            status, phase_code, phase_label = 'stopped', 'stopped', 'Arrestato · R per ripristinare'
        elif lab.phase.startswith('Errore:'):
            status, phase_code, phase_label = 'failed', 'failed', 'Errore nel trasferimento'
        elif not active and not pending and self.last_result:
            status = {'DONE': 'completed', 'FAILED': 'failed', 'ABORTED': 'stopped'}.get(self.last_result['status'], 'idle')
            phase_code = status
            phase_label = {'completed': 'Comando completato', 'failed': 'Richiesta fallita', 'stopped': 'Arrestato · R per ripristinare'}.get(status, 'Pronto')
        error = lab.phase if lab.phase.startswith('Errore:') else (
            self.last_result.get('reason') if self.last_result and self.last_result['status'] in ('FAILED', 'ABORTED') else None)
        return {
            'schema': 'doc.lab.telemetry.v1', 'session_id': self.session,
            'sequence': self.sequence, 'scenario': 'laboratorio', 'model': 'DocArm5',
            'arm_dof': len(JOINTS), 'sim_time': lab.robot.getTime(),
            'status': status, 'phase': phase_label, 'phase_code': phase_code,
            'controller_phase': lab.phase, 'mission': mission,
            'queue_length': len(pending), 'last_result': self.last_result,
            'error': error, 'joint_names': JOINTS, 'joints': diagnosis['joints'],
            'gripper': {'closed': diagnosis['closed'], 'held': diagnosis['held'],
                        'positions': diagnosis['fingers']},
            'tcp': {'position': diagnosis['tcp'], 'tilt_degrees': diagnosis['tilt_degrees']},
            'samples': {name: {'station': data['station'], 'center': data['center']}
                        for name, data in diagnosis['samples'].items()},
            'stations': lab.config['stations'],
        }

    def publish(self, lab, active, pending):
        now = time.monotonic()
        if not self.udp or (not self.force and now-self.last_sent < 0.1):
            return
        self.last_sent, self.force = now, False
        self.sequence += 1
        try:
            payload = json.dumps(self.snapshot(lab, active, pending), allow_nan=False).encode()
            if len(payload) <= 32768:
                self.udp.sendto(payload, self.target)
        except (OSError, ValueError, TypeError):
            # The presentation must never block or interrupt the robot controller.
            pass

    def close(self):
        if self.udp:
            self.udp.close()


def run():
    robot = Supervisor()
    lab = Lab(robot)
    telemetry = Telemetry()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Nessun SO_REUSEADDR: due controllori non devono condividere la stessa porta.
    udp.bind(('127.0.0.1', int(os.getenv('DOC_COMMAND_PORT', '9999'))))
    udp.setblocking(False)
    feedback_port = int(os.getenv('DOC_FEEDBACK_PORT', '9998'))
    active = None
    pending = []
    last_heartbeat = 0

    def reply(cmd, status, reason=None, extra=None):
        telemetry.record(cmd, status, reason)
        if cmd.get('_addr'):
            out = {'id': cmd.get('id'), 'action': cmd.get('action'), 'status': status}
            if reason:
                out['reason'] = reason
            out.update(extra or {})
            udp.sendto(json.dumps(out).encode(), (cmd['_addr'][0], feedback_port))

    def stop(cmd):
        nonlocal active
        if active:
            reply(active[0], 'FAILED', 'annullato da ABORT')
            active = None
        for c in pending:
            reply(c, 'FAILED', 'annullato da ABORT')
        pending.clear()
        lab.freeze()
        reply(cmd, 'DONE')
        telemetry.record({'action': 'ABORT', 'id': cmd.get('id')}, 'ABORTED', 'Arrestato. R per ripristinare la scena.')

    def demo():
        for sample, dest in [('campione_a', 'postazione_1'), ('campione_b', 'postazione_2'),
                             ('campione_a', 'supporto_a'), ('campione_b', 'supporto_b')]:
            pending.append({'action': 'TRANSFER', 'params': {'sample': sample, 'destination': dest}})

    print('DOC laboratorio pronto. D: demo; A/B: trasferimento; SPAZIO: stop; R: reset.', flush=True)
    try:
        while robot.step(lab.ts) != -1:
            now = time.monotonic()
            if now-last_heartbeat >= 1:
                udp.sendto(b'{"action":"ALIVE","scenario":"laboratorio"}', ('127.0.0.1', int(os.getenv('DOC_HEARTBEAT_PORT', '9997'))))
                last_heartbeat = now
            while True:
                try:
                    raw, addr = udp.recvfrom(65535)
                except BlockingIOError:
                    break
                cmd = None
                try:
                    cmd = json.loads(raw)
                    if not isinstance(cmd, dict):
                        continue
                    cmd['_addr'] = addr
                    action = cmd.get('action')
                    if action == 'ABORT':
                        stop(cmd)
                    elif action == 'PING':
                        reply(cmd, 'DONE', extra={'scenario': 'laboratorio', 'busy': bool(active or pending),
                                                  'arm_dof': len(JOINTS), 'joint_names': JOINTS})
                    elif action == 'DIAGNOSI':
                        reply(cmd, 'DONE', extra=lab.diagnose())
                    elif action == 'SCATTA':
                        robot.exportImage((cmd.get('params') or {}).get('file', '/tmp/doc_laboratorio.jpg'), 95)
                        reply(cmd, 'DONE')
                    elif action in ('TRANSFER', 'MOVE_TO', 'GRIPPER'):
                        params = cmd.get('params')
                        if action in ('TRANSFER', 'MOVE_TO') and not isinstance(params, dict):
                            raise ValueError('params deve essere un oggetto JSON.')
                        if action == 'TRANSFER' and not all(isinstance(params.get(n), str) for n in ('sample', 'destination')):
                            raise ValueError('sample e destination devono essere stringhe.')
                        if action == 'GRIPPER':
                            value = float(params)
                            if not math.isfinite(value) or not 0 <= value <= 170:
                                raise ValueError('Apertura pinza fuori scala 0..170.')
                        if active or pending:
                            reply(cmd, 'FAILED', 'Laboratorio occupato: attendere il completamento.')
                        else:
                            pending.append(cmd)
                    else:
                        reply(cmd, 'FAILED', 'Azione non supportata dal laboratorio.')
                except (ValueError, TypeError, KeyError, AttributeError) as error:
                    if 'cmd' in locals() and isinstance(cmd, dict):
                        reply(cmd, 'FAILED', str(error))
            key = lab.keyboard.getKey()
            while key != -1:
                if key == ord(' '):
                    stop({})
                elif key == ord('R'):
                    robot.simulationReset()
                    return
                elif not active and not pending:
                    if key == ord('D'):
                        demo()
                    elif key in (ord('A'), ord('B')):
                        suffix = chr(key).lower()
                        pending.append({'action': 'TRANSFER', 'params': {
                            'sample': f'campione_{suffix}',
                            'destination': f'postazione_{1 if suffix == "a" else 2}'
                            if lab.positions[f'campione_{suffix}'].startswith('supporto') else f'supporto_{suffix}'}})
                key = lab.keyboard.getKey()
            try:
                if active is None and pending:
                    cmd = pending.pop(0)
                    params = cmd.get('params') or {}
                    if cmd['action'] == 'TRANSFER':
                        operation = Transfer(lab, params.get('sample'), params.get('destination'))
                        cmd['_telemetry_source'] = operation.source
                    elif cmd['action'] == 'MOVE_TO':
                        loc, off = params.get('location', ''), params.get('offset', '')
                        operation = Motion(lab, f'{loc}_{off}' if off and off != 'none' else loc)
                    else:
                        operation = Grip(lab, False, opening=float(cmd.get('params', 170)))
                    active = (cmd, operation)
                if active and active[1].tick():
                    reply(active[0], 'DONE', extra=lab.diagnose())
                    active = None
                    lab.phase = 'Pronto'
                lab.follow()
            except (ValueError, TypeError, KeyError, AttributeError) as error:
                reply(active[0] if active else cmd, 'FAILED', str(error))
                for queued in pending:
                    reply(queued, 'FAILED', 'sequenza interrotta dopo un errore')
                pending.clear()
                if active:
                    lab.freeze()
                active = None
                lab.phase = f'Errore: {error}'
                print(lab.phase, flush=True)
            lab.hud()
            telemetry.publish(lab, active, pending)
    finally:
        telemetry.close()
        udp.close()


if __name__ == '__main__':
    run()
