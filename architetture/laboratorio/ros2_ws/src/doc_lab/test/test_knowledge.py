# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
import copy
import json
from pathlib import Path

import pytest
from doc_lab.knowledge import Knowledge


@pytest.fixture
def kb():
    return Knowledge(Path(__file__).resolve().parents[1] / 'config/base_conoscenza.json')


@pytest.fixture
def scene(kb):
    return {'status': 'DONE', 'scenario': 'laboratorio', 'arm_dof': 5,
            'joint_names': kb.data['joint_names'], 'held': None, 'location': 'home',
            'samples': {name: {'station': data['supporto_iniziale']}
                        for name, data in kb.data['campioni'].items()}}


def test_explicit_transfer_and_return(kb):
    assert kb.parse('Doc, sposta campione A in postazione 1.') == {
        'action': 'TRANSFER', 'params': {'sample': 'campione_a', 'destination': 'postazione_1'}}
    assert kb.parse('riporta campione a')['params']['destination'] == 'supporto_a'
    assert kb.parse('stato')['action'] == 'DIAGNOSI'
    assert kb.parse('stop')['action'] == 'ABORT'
    assert kb.parse('Come funziona la robotica?') is None


@pytest.mark.parametrize('text', ['sposta campione a', 'sposta campione z in postazione 1',
                                 'sposta campione a in postazione 99', 'porta tutto'])
def test_ambiguous_or_unknown_commands_rejected(kb, text):
    with pytest.raises(ValueError):
        kb.parse(text)


@pytest.mark.parametrize('payload', [None, [], {'action': 'SET_JOINTS'},
                                    {'action': 'TRANSFER'}, {'action': 'TRANSFER', 'params': {'sample': []}}])
def test_invalid_structured_request(kb, payload):
    with pytest.raises(ValueError):
        kb.validate_request(payload)


def test_preflight_requires_measured_lab_state(kb, scene):
    request = kb.parse('sposta campione a in postazione 1')
    kb.validate_transfer(request, scene)
    with pytest.raises(ValueError, match='occupata'):
        kb.validate_transfer(kb.parse('sposta campione a in supporto b'), scene)
    for mutation in ({'scenario': 'cassettiera'}, {'arm_dof': 6}, {'held': 'campione_a'}, {'location': None}):
        broken = dict(scene, **mutation)
        with pytest.raises(ValueError):
            kb.validate_transfer(request, broken)


def test_done_does_not_invent_sample_position(kb, scene):
    request = kb.parse('sposta campione a in postazione 1')
    with pytest.raises(ValueError, match='non confermato'):
        kb.validate_completed(request, scene)
    updated = copy.deepcopy(scene)
    updated['samples']['campione_a']['station'] = 'postazione_1'
    kb.validate_completed(request, updated)
    assert kb.data['campioni']['campione_a']['supporto_iniziale'] == 'supporto_a'
