# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Cinematica del modello Webots a cinque assi derivato dall'UR3e.

Il modulo wrist_3 e' rimosso. La pinza e' fissata alla faccia del quinto
link: centro z=0.0854, faccia y=0.002+0.097/2 nel PROTO R2025a.
L'IK vincola posizione e direzione del TCP, lasciando libera la rotazione
attorno all'asse del contenitore. Queste quote non calibrano il DOC reale.
"""
import math
import numpy as np

from .ur3e_kinematics import CATENA as UR3E_CHAIN, _rot, _omogenea

CATENA = UR3E_CHAIN[:5]
NOMI_GIUNTI = [joint[0] for joint in CATENA]
TOOLSLOT = (0.0, 0.0505, 0.0854)
GRIPPER_ROT_X = -1.5707953071795862
GRIPPER_TCP = (0.0, 0.0, 0.17696)
LIMITI = [(-2*math.pi, 2*math.pi), (-2*math.pi, 2*math.pi),
          (-math.pi, math.pi), (-2*math.pi, 2*math.pi), (-2*math.pi, 2*math.pi)]


def fk(q, fino_a=None):
    if len(q) != len(CATENA) or not np.all(np.isfinite(q)):
        raise ValueError('Il braccio richiede esattamente cinque angoli finiti.')
    transform = np.eye(4)
    for i in range(len(CATENA) if fino_a is None else fino_a):
        _, axis, anchor, rest = CATENA[i]
        transform = transform @ _omogenea(_rot(axis, q[i]+rest), np.array(anchor))
    if fino_a is not None:
        return transform
    return (transform @ _omogenea(np.eye(3), np.array(TOOLSLOT))
            @ _omogenea(_rot('x', GRIPPER_ROT_X), np.zeros(3))
            @ _omogenea(np.eye(3), np.array(GRIPPER_TCP)))


def ik_axis(point, seed, direction=(0, 0, -1), iterations=350,
            toll_pos=2e-6, toll_ang=2e-5):
    """Cinque vincoli: tre coordinate e due componenti della direzione."""
    target = np.array(point, dtype=float)
    axis = np.array(direction, dtype=float)
    axis /= np.linalg.norm(axis)
    q = np.array(seed, dtype=float)
    for _ in range(iterations):
        pose = fk(q)
        e_pos, e_axis = target-pose[:3, 3], axis-pose[:3, 2]
        ep = float(np.linalg.norm(e_pos))
        ea = math.acos(float(np.clip(axis @ pose[:3, 2], -1, 1)))
        if ep < toll_pos and ea < toll_ang:
            return q, ep, ea, True
        jacobian = np.zeros((6, len(q)))
        for i in range(len(q)):
            shifted = q.copy()
            shifted[i] += 1e-6
            new_pose = fk(shifted)
            jacobian[:3, i] = (new_pose[:3, 3]-pose[:3, 3])/1e-6
            jacobian[3:, i] = (new_pose[:3, 2]-pose[:3, 2])/1e-6
        error = np.concatenate([e_pos, e_axis])
        step = np.linalg.solve(jacobian.T @ jacobian+0.003**2*np.eye(len(q)), jacobian.T @ error)
        maximum = float(np.max(np.abs(step)))
        if maximum > 0.15:
            step *= 0.15/maximum
        q += step
    pose = fk(q)
    return q, float(np.linalg.norm(target-pose[:3, 3])), math.acos(float(np.clip(axis @ pose[:3, 2], -1, 1))), False
