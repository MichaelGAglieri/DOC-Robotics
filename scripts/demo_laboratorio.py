#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Comanda e verifica la demo di laboratorio via UDP, senza ROS o Neo4j."""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


class Client:
    def __init__(self):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(('127.0.0.1', int(os.getenv('DOC_FEEDBACK_PORT', '9998'))))
        self.rx.settimeout(1)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = ('127.0.0.1', int(os.getenv('DOC_COMMAND_PORT', '9999')))

    def command(self, action, params=None, timeout=90):
        cid = uuid.uuid4().hex
        self.tx.sendto(json.dumps({'id': cid, 'action': action, 'params': params}).encode(), self.target)
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            try:
                data, _ = self.rx.recvfrom(65535)
            except socket.timeout:
                continue
            result = json.loads(data)
            if result.get('id') != cid:
                continue
            if result.get('status') != 'DONE':
                raise RuntimeError(result.get('reason', str(result)))
            return result
        raise TimeoutError(f'Nessuna risposta a {action}. Aprire laboratorio.wbt e avviare la simulazione.')

    def close(self):
        self.rx.close()
        self.tx.close()


def verify(result, config, sample=None, destination=None):
    if (result.get('joint_names') != config['joint_names']
            or result.get('arm_dof') != config['arm_dof']
            or len(result.get('joints', [])) != config['arm_dof']):
        raise RuntimeError('Il modello attivo non corrisponde alla calibrazione a cinque assi.')
    if result.get('held') is not None or result.get('location') != 'home':
        raise RuntimeError('Ciclo incompleto: campione ancora in presa o braccio fuori home.')
    max_error, max_tilt = 0, 0
    for name, state in result['samples'].items():
        if name == sample and state['station'] != destination:
            raise RuntimeError('Destinazione dichiarata diversa dalla richiesta.')
        grasp = config['stations'][state['station']]
        expected = [grasp[i]+config['center_offset'][i] for i in range(3)]
        error = math.dist(expected, state['center'])
        # Un contenitore verticale ha asse locale z allineato a +z del mondo.
        tilt = math.degrees(math.acos(max(-1, min(1, state['orientation'][8]))))
        max_error, max_tilt = max(max_error, error), max(max_tilt, tilt)
        if error > 0.005 or tilt > 2:
            raise RuntimeError(f'{name}: deposito fuori tolleranza ({error*1000:.2f} mm, {tilt:.2f} gradi).')
    print(f'OK: {sample or "scena"}, errore massimo {max_error*1000:.2f} mm, '
          f'inclinazione {max_tilt:.2f} gradi, braccio in home.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cycles', type=int, default=1, help='cicli completi di andata e ritorno')
    parser.add_argument('--diagnose', action='store_true')
    parser.add_argument('--abort', action='store_true')
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error('--cycles deve essere almeno 1')
    client = Client()
    try:
        ready = client.command('PING', timeout=5)
        if ready.get('scenario') != 'laboratorio':
            raise RuntimeError('Il controller attivo non e\' quello del laboratorio.')
        if ready.get('arm_dof') != 5:
            raise RuntimeError('Ricaricare laboratorio.wbt: serve il nuovo controller a cinque assi.')
        if args.abort:
            print(client.command('ABORT'))
        elif args.diagnose:
            print(json.dumps(client.command('DIAGNOSI'), indent=2))
        elif args.screenshot:
            path = str(args.screenshot.resolve())
            client.command('SCATTA', {'file': path})
            print(path)
        else:
            if ready.get('busy'):
                raise RuntimeError('Una demo e\' gia\' in corso.')
            config = json.loads((ROOT/'webots_sim/laboratorio.json').read_text())
            initial = client.command('DIAGNOSI')
            if {s: v['station'] for s, v in initial['samples'].items()} != config['samples']:
                raise RuntimeError('Demo non nella scena iniziale: premere R in Webots.')
            verify(initial, config)
            sequence = [('campione_a', 'postazione_1'), ('campione_b', 'postazione_2'),
                        ('campione_a', 'supporto_a'), ('campione_b', 'supporto_b')]
            for cycle in range(args.cycles):
                print(f'Ciclo {cycle+1}/{args.cycles}', flush=True)
                for sample, dest in sequence:
                    print(f'  {sample} -> {dest}', flush=True)
                    result = client.command('TRANSFER', {'sample': sample, 'destination': dest})
                    verify(result, config, sample, dest)
            print(f'{args.cycles} cicli completi verificati.', flush=True)
    finally:
        client.close()


if __name__ == '__main__':
    main()
