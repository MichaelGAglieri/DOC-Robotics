# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Pianificazione discreta delle postazioni, senza generare traiettorie robotiche."""
from collections import deque
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import uuid

from .knowledge import normalize


def positions(scene):
    return {name: sample['station'] for name, sample in scene['samples'].items()}


def fingerprint(scene):
    data = {key: scene.get(key) for key in ('scenario', 'arm_dof', 'joint_names', 'held', 'location')}
    data['positions'] = positions(scene)
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


class MissionPlanner:
    def __init__(self, knowledge, protocols_path):
        self.knowledge = knowledge
        self.protocols = json.loads(Path(protocols_path).read_text())['protocolli']
        self.aliases = {}
        for key, protocol in self.protocols.items():
            for alias in [key, protocol['titolo']] + protocol.get('alias', []):
                self.aliases[normalize(alias)] = key

    def resolve(self, text):
        value = normalize(text)
        if value.startswith('ehi doc '):
            value = value[8:]
        elif value.startswith('doc '):
            value = value[4:]
        if value.startswith('pianifica '):
            name = value[10:]
            if name not in self.aliases:
                raise ValueError('Protocollo sconosciuto. Disponibili: ' + ', '.join(self.protocols))
            return self.aliases[name]
        return self.aliases.get(value)

    def ready(self, scene):
        self.knowledge.validate_scene(scene)
        if scene.get('held') is not None or scene.get('location') != 'home':
            raise ValueError('Per pianificare serve il braccio in home, senza campioni in presa.')
        current = positions(scene)
        if set(current) != set(self.knowledge.data['campioni']):
            raise ValueError('I campioni osservati non coincidono con il catalogo.')
        if any(slot not in self.knowledge.data['postazioni'] for slot in current.values()):
            raise ValueError('Una posizione osservata è sconosciuta.')
        if len(set(current.values())) != len(current):
            raise ValueError('Più campioni dichiarati sulla stessa postazione.')
        joints = scene.get('joints')
        if not isinstance(joints, list) or len(joints) != 5 or not all(isinstance(q, (float, int)) and math.isfinite(q) for q in joints):
            raise ValueError('Lettura dei cinque giunti non disponibile.')
        return current

    def route(self, start, targets):
        """BFS: minimo numero di trasferimenti, un campione per postazione."""
        if not isinstance(targets, dict) or not set(targets).issubset(start):
            raise ValueError('Campioni del protocollo sconosciuti.')
        goal = dict(start, **targets)
        slots = tuple(sorted(self.knowledge.data['postazioni']))
        if any(slot not in slots for slot in goal.values()) or len(set(goal.values())) != len(goal):
            raise ValueError('Obiettivo impossibile: destinazione sconosciuta o assegnata a più campioni.')
        names = tuple(sorted(start))
        initial = tuple(start[name] for name in names)
        final = tuple(goal[name] for name in names)
        queue = deque([initial])
        previous = {initial: None}
        while queue:
            state = queue.popleft()
            if state == final:
                result = []
                while previous[state] is not None:
                    old, move = previous[state]
                    result.append(move)
                    state = old
                return list(reversed(result))
            for index, name in enumerate(names):
                for target in slots:
                    if target in state:
                        continue
                    new_state = list(state)
                    new_state[index] = target
                    new_state = tuple(new_state)
                    if new_state in previous:
                        continue
                    previous[new_state] = (state, {'sample': name, 'source': state[index],
                        'destination': target, 'temporary': target != goal[name]})
                    queue.append(new_state)
        raise ValueError('Nessun piano possibile con le postazioni libere disponibili.')

    def build(self, protocol_id, scene):
        if protocol_id not in self.protocols:
            raise ValueError('Protocollo sconosciuto.')
        initial = self.ready(scene)
        current = dict(initial)
        protocol = self.protocols[protocol_id]
        steps = []
        for phase in protocol['fasi']:
            for move in self.route(current, phase['obiettivi']):
                before = dict(current)
                current[move['sample']] = move['destination']
                steps.append(dict(move, index=len(steps) + 1, phase=phase['nome'],
                    expected_before=before, expected_after=dict(current),
                    reason='Appoggio temporaneo per liberare una destinazione occupata.' if move['temporary']
                           else 'Raggiungimento della postazione prevista dal protocollo.'))
        return {'plan_id': uuid.uuid4().hex[:12], 'protocol': protocol_id, 'title': protocol['titolo'],
                'initial_positions': initial, 'expected_final_positions': current,
                'scene_fingerprint': fingerprint(scene), 'steps': steps, 'transfers': len(steps),
                'catalog_sha256': hashlib.sha256(json.dumps(self.knowledge.data, sort_keys=True).encode()).hexdigest(),
                'planning_scope': 'Postazioni e occupazione; traiettorie eseguite dal controller Webots esistente.',
                'initial_scene': deepcopy(scene)}
