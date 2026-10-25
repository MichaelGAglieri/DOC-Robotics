# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Verifiche geometriche e di sequenza; i contatti richiedono Webots reale."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import types

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'architetture/laboratorio/ros2_ws/src/doc_lab'))
from doc_lab.doc5_kinematics import fk, LIMITI, NOMI_GIUNTI

CONFIG = json.loads((ROOT/'webots_sim/laboratorio.json').read_text())


@pytest.fixture
def module(monkeypatch):
    monkeypatch.setitem(sys.modules, 'controller', types.SimpleNamespace(Supervisor=None))
    spec = importlib.util.spec_from_file_location('lab_controller',
        ROOT/'webots_sim/controllers/doc_laboratorio/doc_laboratorio.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


class Sensor:
    def __init__(self, motor): self.motor = motor
    def enable(self, _): pass
    def getValue(self): return self.motor.position


class Motor:
    def __init__(self, position=0):
        self.position = self.target = position
        self.blocked = False
    def getPositionSensor(self): return Sensor(self)
    def setVelocity(self, _): pass
    def setPosition(self, value): self.target = value
    def step(self):
        if not self.blocked:
            self.position = self.target


class Field:
    def __init__(self, node, name): self.node, self.name = node, name
    def setSFVec3f(self, value): self.node.position = value
    def setSFRotation(self, value): self.node.rotation = value


class Sample:
    def __init__(self, station):
        self.position = [a+b for a, b in zip(CONFIG['stations'][station], CONFIG['center_offset'])]
        self.rotation = [0, 0, 1, 0]
    def getPosition(self): return self.position
    def getOrientation(self): return [1, 0, 0, 0, 1, 0, 0, 0, 1]
    def getField(self, name): return Field(self, name)


class TCP:
    def __init__(self, robot): self.robot = robot
    def pose(self):
        return fk([self.robot.motors[n].position for n in NOMI_GIUNTI])
    def getPosition(self): return (self.pose()[:3, 3]+np.array(CONFIG['base'])).tolist()
    def getOrientation(self): return self.pose()[:3, :3].flatten().tolist()


class Robot:
    def __init__(self, module):
        self.motors = {n: Motor(v) for n, v in zip(module.JOINTS, CONFIG['waypoint']['home'])}
        self.nodes = {s.upper(): Sample(p) for s, p in CONFIG['samples'].items()}
        self.nodes['LAB_TCP'] = TCP(self)
    def getBasicTimeStep(self): return 16
    def getDevice(self, name): return self.motors.setdefault(name, Motor())
    def getFromDef(self, name): return self.nodes[name]
    def getKeyboard(self): return types.SimpleNamespace(enable=lambda _: None)
    def step(self):
        for m in self.motors.values(): m.step()


def execute(lab, operation, robot):
    for _ in range(10000):
        robot.step()
        done = operation.tick()
        lab.follow()
        if done:
            return
    pytest.fail('Operazione bloccata')


def test_all_routes_keep_tcp_vertical_and_above_bench():
    max_error, max_tilt = 0, 0
    for key, path in CONFIG['routes'].items():
        start, end = key.split('->')
        a, b = np.array(CONFIG['points'][start]), np.array(CONFIG['points'][end])
        for i, q in enumerate(path):
            assert all(lo <= v <= hi for v, (lo, hi) in zip(q, LIMITI))
            assert abs(q[2]) <= math.pi
            # Verifica anche i tratti interpolati tra i campioni IK.
            for u in (0, 0.5):
                if i == len(path)-1 and u:
                    continue
                qi = np.array(q) if not u else (np.array(q)+np.array(path[i+1]))/2
                pose = fk(qi)
                expected = a+(b-a)*(i+u)/(len(path)-1)
                max_error = max(max_error, float(np.linalg.norm(pose[:3, 3]-expected)))
                max_tilt = max(max_tilt, math.acos(np.clip(-pose[2, 2], -1, 1)))
                # Origini dei link al di sopra del banco; non prova le collisioni delle mesh.
                assert all(fk(qi, fino_a=j)[2, 3]+CONFIG['base'][2] >= CONFIG['bench_z']
                           for j in range(1, len(NOMI_GIUNTI)+1))
    assert max_error < 0.0002
    assert max_tilt < math.radians(0.1)


def test_two_samples_round_trip_and_measured_deposit(module):
    robot = Robot(module)
    lab = module.Lab(robot)
    for s, dest in [('campione_a', 'postazione_1'), ('campione_b', 'postazione_2'),
                    ('campione_a', 'supporto_a'), ('campione_b', 'supporto_b')]:
        execute(lab, module.Transfer(lab, s, dest), robot)
        assert lab.held is None and lab.location == 'home'
        expected = np.array(CONFIG['stations'][dest])+np.array(CONFIG['center_offset'])
        assert np.linalg.norm(lab.nodes[s].getPosition()-expected) < 0.004
    assert lab.positions == CONFIG['samples']


def test_occupied_station_and_wrong_sample_are_rejected(module):
    lab = module.Lab(Robot(module))
    with pytest.raises(ValueError, match='occupata'):
        module.Transfer(lab, 'campione_a', 'supporto_b')
    with pytest.raises(ValueError, match='sconosciuti'):
        module.Transfer(lab, 'campione_inesistente', 'postazione_1')


def test_grasp_requires_closed_gripper_and_proximity(module):
    lab = module.Lab(Robot(module))
    with pytest.raises(ValueError, match='Presa rifiutata'):
        lab.attach('campione_a')
    lab.closed = True
    with pytest.raises(ValueError, match='Presa rifiutata'):
        lab.attach('campione_a')


def test_stalled_joint_fails_instead_of_done(module):
    robot = Robot(module)
    lab = module.Lab(robot)
    robot.motors[module.JOINTS[0]].blocked = True
    with pytest.raises(ValueError, match='non arrivati'):
        execute(lab, module.Motion(lab, 'supporto_a_approach'), robot)


def test_abort_freezes_joint_targets_and_retains_payload(module):
    robot = Robot(module)
    lab = module.Lab(robot)
    transfer = module.Transfer(lab, 'campione_a', 'postazione_1')
    for _ in range(10000):
        robot.step()
        transfer.tick()
        lab.follow()
        if lab.held:
            break
    assert lab.held == 'campione_a'
    before = lab.nodes['campione_a'].getPosition()[:]
    lab.freeze()
    for _ in range(10):
        robot.step()
        lab.follow()
    assert lab.location is None
    assert lab.nodes['campione_a'].getPosition() == before
    assert all(m.target == m.position for m in robot.motors.values())


def assert_same_vrml(actual, generated):
    """Keep node/text equality; allow sub-picometre libm differences across OSes."""
    import re
    numeric = r'([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)'
    a, b = re.split(numeric, actual), re.split(numeric, generated)
    assert a[::2] == b[::2], 'VRML structure differs'
    assert [float(v) for v in a[1::2]] == pytest.approx(
        [float(v) for v in b[1::2]], rel=0, abs=1e-12)


def test_vrml_comparison_rejects_geometric_and_structural_changes():
    assert_same_vrml('Pose { translation -0.015499999999999993 0 0 }',
                     'Pose { translation -0.015499999999999995 0 0 }')
    with pytest.raises(AssertionError):
        assert_same_vrml('Pose { translation 0 0 0 }', 'Pose { translation 0.001 0 0 }')
    with pytest.raises(AssertionError):
        assert_same_vrml('Pose { translation 0 0 0 }', 'Solid { translation 0 0 0 }')


def test_world_and_proto_are_generated_from_config():
    sys.path.insert(0, str(ROOT/'scripts'))
    import genera_laboratorio
    assert_same_vrml((ROOT/'webots_sim/worlds/laboratorio.wbt').read_text(), genera_laboratorio.world_source(CONFIG))
    assert_same_vrml((ROOT/'webots_sim/protos/DocLabGripper.proto').read_text(), genera_laboratorio.gripper_source())
    import genera_doc5
    assert_same_vrml((ROOT/'webots_sim/protos/DocArm5.proto').read_text(), genera_doc5.source())


def test_five_axis_model_has_no_terminal_joint(module):
    import re
    source = (ROOT/'webots_sim/protos/DocArm5.proto').read_text()
    assert source.count('HingeJoint {') == 5
    assert re.findall(r'name "([^"]+_joint)"', source) == NOMI_GIUNTI
    assert 'name "wrist_3_link"' not in source
    assert 'translation 0.0 0.0505 0.0854\n                                      children IS toolSlot' in source
    assert CONFIG['arm_dof'] == 5 and CONFIG['joint_names'] == module.JOINTS == NOMI_GIUNTI
    assert all(len(q) == 5 for q in CONFIG['waypoint'].values())
    assert all(len(q) == 5 for path in CONFIG['routes'].values() for q in path)
    with pytest.raises(ValueError, match='cinque angoli'):
        fk([0]*6)


def test_laboratory_assets_are_complete_and_local():
    """La scena deve aprirsi senza rete anche dopo uno spostamento della cartella."""
    import hashlib
    import re

    vendor = ROOT/'webots_sim/protos/vendor/webots'
    manifest = json.loads((vendor/'manifest.json').read_text())
    for asset in manifest['assets']:
        data = (vendor/asset['path']).read_bytes()
        assert hashlib.sha256(data).hexdigest() == asset['sha256'], asset['path']

    pending = [ROOT/'webots_sim/worlds/laboratorio.wbt']
    visited = set()
    while pending:
        path = pending.pop().resolve()
        if path in visited:
            continue
        visited.add(path)
        # I commenti contengono URL di documentazione e licenze, non dipendenze.
        source = re.sub(r'(?m)^\s*#.*$', '', path.read_text())
        for url in re.findall(r'"([^"\n]+\.(?:proto|obj|png|jpg|hdr))"', source):
            assert not url.startswith(('https:', 'http:', 'webots:')), url
            dependency = (path.parent/url).resolve()
            assert dependency.is_relative_to(ROOT/'webots_sim'), url
            assert dependency.is_file(), dependency
            if dependency.suffix == '.proto':
                pending.append(dependency)
