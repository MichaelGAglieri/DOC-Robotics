# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
from copy import deepcopy
import json
from pathlib import Path
import pytest
from doc_lab.knowledge import Knowledge
from doc_lab.planner import MissionPlanner, positions
from doc_lab.missions import MissionDirector


@pytest.fixture
def planner():
    config = Path(__file__).resolve().parents[1] / 'config'
    return MissionPlanner(Knowledge(config / 'base_conoscenza.json'), config / 'protocolli.json')


@pytest.fixture
def scene(planner):
    return {'status': 'DONE', 'scenario': 'laboratorio', 'arm_dof': 5,
            'joint_names': planner.knowledge.data['joint_names'], 'joints': [0.0] * 5,
            'held': None, 'location': 'home', 'samples': {
                'campione_a': {'station': 'supporto_a'}, 'campione_b': {'station': 'supporto_b'}}}


def test_cycle_uses_current_state_and_predicts_each_step(planner, scene):
    plan = planner.build('ciclo completo', scene)
    assert plan['transfers'] == 4
    current = positions(scene)
    for step in plan['steps']:
        assert step['expected_before'] == current
        assert step['destination'] not in current.values()
        current[step['sample']] = step['destination']
        assert step['expected_after'] == current
    assert current == plan['initial_positions'] == plan['expected_final_positions']
    scene['samples']['campione_a']['station'] = 'postazione_1'
    assert planner.build('prepara', scene)['transfers'] == 1
    assert plan['initial_scene']['samples']['campione_a']['station'] == 'supporto_a'


def test_swap_needs_one_temporary_support_and_three_moves(planner, scene):
    plan = planner.build('scambia supporti', scene)
    assert plan['transfers'] == 3
    assert sum(step['temporary'] for step in plan['steps']) == 1
    assert plan['expected_final_positions'] == {'campione_a': 'supporto_b', 'campione_b': 'supporto_a'}


def test_impossible_or_unknown_goals_rejected(planner, scene):
    for targets in ({'campione_a': 'supporto_b'}, {'campione_a': 'ignoto'}, {'ignoto': 'postazione_1'}):
        with pytest.raises(ValueError):
            planner.route(positions(scene), targets)


def test_invalid_scene_and_text(planner, scene):
    for change in ({'scenario': 'cassettiera'}, {'held': 'campione_a'}, {'location': None}, {'joints': [float('nan')] * 5}):
        with pytest.raises(ValueError):
            planner.build('prepara', dict(scene, **change))
    assert planner.resolve('Doc pianifica ciclo completo') == 'ciclo completo'
    assert planner.resolve('porta entrambi i campioni alle postazioni e poi riportali ai supporti') == 'ciclo completo'
    with pytest.raises(ValueError):
        planner.resolve('pianifica una procedura sconosciuta')


class FakeNode:
    def __init__(self, scene):
        self.scene = deepcopy(scene)
        self.active = False
        self.events = []
        self.commands = []
        self.deferred = False
        self.callback = None

    def mission_event(self, event):
        self.events.append(deepcopy(event))

    def inspect_scene(self, callback):
        if self.deferred:
            self.active = True
            self.callback = callback
        else:
            callback(deepcopy(self.scene), None)

    def deliver_scene(self):
        self.active = False
        self.callback(deepcopy(self.scene), None)

    def request(self, request, from_mission=False, expected_positions=None):
        assert from_mission
        assert expected_positions == positions(self.scene)
        self.active = True
        self.commands.append(request)

    def complete(self, director):
        request = self.commands[-1]
        self.scene['samples'][request['params']['sample']]['station'] = request['params']['destination']
        self.active = False
        director.step_result('DONE', 'OK', deepcopy(self.scene))


def make(planner, scene, tmp_path):
    node = FakeNode(scene)
    director = MissionDirector(node, planner, tmp_path)
    return node, director


def test_preview_never_moves_and_stale_plan_cannot_execute(planner, scene, tmp_path):
    node, director = make(planner, scene, tmp_path)
    director.handle('pianifica ciclo completo')
    assert node.events[-1]['status'] == 'PLAN_READY' and node.commands == []
    assert list(tmp_path.iterdir()) == []
    node.scene['samples']['campione_a']['station'] = 'postazione_1'
    director.commit()
    assert node.events[-1]['status'] == 'REJECTED'
    assert node.commands == [] and not director.running


def test_cancel_during_commit_prevents_late_execution(planner, scene, tmp_path):
    node, director = make(planner, scene, tmp_path)
    director.plan('prepara')
    node.deferred = True
    director.commit()
    director.handle('annulla piano')
    node.deliver_scene()
    assert node.commands == [] and director.prepared is None


def test_completed_cycle_has_observed_report(planner, scene, tmp_path):
    node, director = make(planner, scene, tmp_path)
    director.plan('ciclo completo')
    director.commit()
    for _ in range(4):
        node.complete(director)
    assert not director.running and len(node.commands) == 4
    assert node.events[-1]['status'] == 'COMPLETED'
    report = json.loads((director.journal.directory / 'report.json').read_text())
    assert report['status'] == 'COMPLETED'
    assert len(report['steps']) == 4
    assert report['steps'][-1]['feedback']['samples'] == scene['samples']
    assert all(item['duration_seconds'] >= 0 for item in report['steps'])
    events = [json.loads(line) for line in (director.journal.directory / 'events.jsonl').read_text().splitlines()]
    assert [event['event'] for event in events].count('step_finished') == 4
    assert events[-1]['event'] == 'mission_finished'


@pytest.mark.parametrize('status', ['STOPPED', 'UNCONFIRMED', 'FAILED'])
def test_stop_failure_and_timeout_never_continue(planner, scene, tmp_path, status):
    node, director = make(planner, scene, tmp_path)
    director.plan('ciclo completo')
    director.commit()
    node.active = False
    director.step_result(status, 'Esito non completato', {'status': status})
    assert len(node.commands) == 1 and not director.running
    assert json.loads((director.journal.directory / 'report.json').read_text())['status'] == status


def test_inconsistent_done_cannot_advance(planner, scene, tmp_path):
    node, director = make(planner, scene, tmp_path)
    director.plan('prepara')
    director.commit()
    node.active = False
    director.step_result('DONE', 'OK', scene)
    assert len(node.commands) == 1
    assert node.events[-1]['status'] == 'UNCONFIRMED'


def test_expired_plan_and_shutdown_are_explicit(planner, scene, tmp_path):
    node, director = make(planner, scene, tmp_path)
    director.plan('prepara')
    director.prepared_at -= 301
    with pytest.raises(ValueError, match='scaduto'):
        director.commit()
    director.plan('prepara')
    director.commit()
    director.shutdown()
    assert json.loads((director.journal.directory / 'report.json').read_text())['status'] == 'INTERRUPTED'


def test_noop_mission_records_zero_transfers(planner, scene, tmp_path):
    node, director = make(planner, scene, tmp_path)
    director.plan('riordina')
    director.commit()
    assert node.events[-1]['status'] == 'COMPLETED'
    assert node.commands == []
