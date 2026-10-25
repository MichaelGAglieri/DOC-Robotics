# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Catalogo statico e comandi espliciti; lo stato corrente proviene da Webots."""
import json
from pathlib import Path
import re
import unicodedata


def normalize(text):
    text = unicodedata.normalize('NFKD', text.lower())
    text = ''.join(c for c in text if not unicodedata.combining(c))
    return ' '.join(re.sub(r'[^a-z0-9 ]', ' ', text).split())


class Knowledge:
    def __init__(self, path):
        self.data = json.loads(Path(path).read_text())
        if self.data['scenario'] != 'laboratorio' or self.data['arm_dof'] != 5:
            raise ValueError('Catalogo incompatibile con il laboratorio a cinque assi.')
        if len(self.data['joint_names']) != 5:
            raise ValueError('Servono cinque nomi di giunto.')
        for sample in self.data['campioni'].values():
            if sample['supporto_iniziale'] not in self.data['postazioni']:
                raise ValueError('Supporto iniziale sconosciuto.')

    def validate_request(self, request):
        if not isinstance(request, dict) or request.get('action') not in self.data['azioni']:
            raise ValueError('Azioni ammesse: TRANSFER, DIAGNOSI, ABORT.')
        action = request['action']
        if action == 'TRANSFER':
            params = request.get('params')
            if not isinstance(params, dict):
                raise ValueError('TRANSFER richiede sample e destination.')
            if not isinstance(params.get('sample'), str) or params['sample'] not in self.data['campioni']:
                raise ValueError('Campione sconosciuto.')
            if not isinstance(params.get('destination'), str) or params['destination'] not in self.data['postazioni']:
                raise ValueError('Postazione sconosciuta.')
            return {'action': action, 'params': {'sample': params['sample'], 'destination': params['destination']}}
        return {'action': action, 'params': None}

    def parse(self, text):
        text = normalize(text)
        for prefix in ('ehi doc ', 'doc '):
            if text.startswith(prefix):
                text = text[len(prefix):]
                break
        if text in ('stato', 'stato laboratorio', 'diagnosi'):
            return {'action': 'DIAGNOSI'}
        if text in ('stop', 'ferma', 'ferma laboratorio', 'arresta laboratorio'):
            return {'action': 'ABORT'}
        match = re.fullmatch(r'(?:sposta|porta|trasferisci) (campione [a-z]) (?:in|alla|a) (postazione [0-9]+|supporto [a-z])', text)
        if match:
            return self.validate_request({'action': 'TRANSFER', 'params': {
                'sample': match[1].replace(' ', '_'), 'destination': match[2].replace(' ', '_')}})
        match = re.fullmatch(r'riporta (campione [a-z])(?: al supporto)?', text)
        if match:
            sample = match[1].replace(' ', '_')
            if sample not in self.data['campioni']:
                raise ValueError('Campione sconosciuto.')
            return {'action': 'TRANSFER', 'params': {'sample': sample,
                    'destination': self.data['campioni'][sample]['supporto_iniziale']}}
        if text.startswith(('sposta', 'porta', 'trasferisci', 'riporta', 'ferma', 'arresta')):
            raise ValueError('Usa per esempio: sposta campione a in postazione 1.')
        return None

    def validate_scene(self, result):
        if (result.get('status') != 'DONE' or result.get('scenario') != 'laboratorio'
                or result.get('arm_dof') != 5 or result.get('joint_names') != self.data['joint_names']):
            raise ValueError('Risposta assente o scenario incompatibile: aprire laboratorio.wbt a cinque assi.')

    def validate_transfer(self, request, scene):
        self.validate_scene(scene)
        samples = scene.get('samples', {})
        if set(samples) != set(self.data['campioni']):
            raise ValueError('Campioni del mondo diversi dal catalogo.')
        if scene.get('held') is not None or scene.get('location') != 'home':
            raise ValueError('Scena non pronta: campione in presa o braccio fuori home. Ripristinare Webots.')
        destination = request['params']['destination']
        if any(value.get('station') == destination for value in samples.values()):
            raise ValueError('Destinazione occupata o campione già presente.')

    def validate_completed(self, request, scene):
        self.validate_scene(scene)
        sample, destination = request['params']['sample'], request['params']['destination']
        if (scene.get('samples', {}).get(sample, {}).get('station') != destination
                or scene.get('held') is not None or scene.get('location') != 'home'):
            raise ValueError('DONE ricevuto, ma stato finale del trasferimento non confermato.')
