#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Cinematica diretta e inversa dell'UR3e come lo modella Webots.

Perche' non una libreria: i parametri DH pubblicati da Universal Robots
descrivono il robot reale, non il PROTO di Webots, che ha assi e pose di
riposo propri. Usare i DH "da catalogo" produce pose che sembrano giuste e
finiscono a dieci centimetri dal bersaglio. Qui la catena e' trascritta
direttamente da UR3e.proto (R2025a), giunto per giunto.

Semantica di un HingeJoint Webots:

    genitore -> figlio = Trans(anchor) . Rot(asse, q) . Trans(-anchor)
                         . Trans(endPoint.translation) . Rot(endPoint.rotation)

Nel PROTO dell'UR3e anchor e endPoint.translation coincidono per tutti e sei i
giunti, quindi i due Trans opposti si annullano e resta:

    genitore -> figlio = Trans(anchor) . Rot(asse, q) . Rot(riposo)

La validazione contro il simulatore sta in scripts/valida_cinematica.py: non
fidarsi di questo file finche' quel confronto non e' verde.
"""

import math

import numpy as np

# ----------------------------------------------------------------------
# Catena, trascritta da UR3e.proto (R2025a)
# ----------------------------------------------------------------------
# (nome, asse di rotazione, anchor, rotazione di riposo attorno allo stesso
#  asse; None se il PROTO non ne dichiara)
CATENA = [
    ('shoulder_pan_joint',  'z', (0.0,  0.0,   0.152), 0.0),
    ('shoulder_lift_joint', 'y', (0.0,  0.12,  0.0),   1.570796),
    ('elbow_joint',         'y', (0.0, -0.093, 0.244), 0.0),
    ('wrist_1_joint',       'y', (0.0,  0.0,   0.213), 1.570796),
    ('wrist_2_joint',       'z', (0.0,  0.104, 0.0),   0.0),
    ('wrist_3_joint',       'y', (0.0,  0.0,   0.085), 0.0),
]

NOMI_GIUNTI = [c[0] for c in CATENA]

# wrist_3_link -> toolSlot
TOOLSLOT = (0.0, 0.09, 0.0)

# Robotiq 2F-140 montata con 'rotation 1 0 0 -pi/2'.
#
# Il punto di presa sta a 177 mm lungo il +z della pinza, e le dita si chiudono
# lungo il suo y. Ricavato percorrendo la catena dei polpastrelli nel PROTO
# tenendo conto delle rotazioni dei nodi: sommare solo le traslazioni, come
# avevo fatto prima, dava 115 mm lungo y — sbagliato sia in modulo sia in
# direzione, e la pinza finiva a puntare per terra.
#
# Controprova: cosi' i due polpastrelli aperti distano 136 mm, cioe' la corsa
# che da' il nome alla "2F-140".
GRIPPER_ROT_X = -1.5707953071795862
GRIPPER_TCP = (0.0, 0.0, 0.17696)
APERTURA_DITA = 0.136

# Limiti articolari UR3e (rad). Il PROTO non li impone, ma generare pose fuori
# da questi valori significa produrre waypoint che il robot vero rifiuterebbe.
LIMITI = [(-2 * math.pi, 2 * math.pi)] * 6


def _rot(asse, q):
    c, s = math.cos(q), math.sin(q)
    if asse == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)
    if asse == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _omogenea(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def fk(q, fino_a=None):
    """Posa 4x4 del TCP nel frame base del robot.

    fino_a=None arriva al punto di presa fra le dita; fino_a=k si ferma
    all'uscita del giunto k (serve allo jacobiano).
    """
    T = np.eye(4)
    n = len(CATENA) if fino_a is None else fino_a
    for i in range(n):
        _, asse, anchor, riposo = CATENA[i]
        Ti = _omogenea(np.eye(3), np.array(anchor)) @ _omogenea(_rot(asse, q[i]), np.zeros(3))
        if riposo:
            Ti = Ti @ _omogenea(_rot(asse, riposo), np.zeros(3))
        T = T @ Ti
    if fino_a is not None:
        return T
    T = T @ _omogenea(np.eye(3), np.array(TOOLSLOT))
    T = T @ _omogenea(_rot('x', GRIPPER_ROT_X), np.zeros(3))
    T = T @ _omogenea(np.eye(3), np.array(GRIPPER_TCP))
    return T


def tcp(q):
    """Solo la posizione del punto di presa."""
    return fk(q)[:3, 3]


def _jacobiano_posizione(q, eps=1e-6):
    """Jacobiano 3x6 per differenze finite: piu' corto e piu' facile da
    verificare della forma analitica, e il costo non conta perche' la
    generazione dei waypoint e' offline."""
    J = np.zeros((3, 6))
    p0 = tcp(q)
    for i in range(6):
        qp = list(q)
        qp[i] += eps
        J[:, i] = (tcp(qp) - p0) / eps
    return J


def ik_posizione(bersaglio, q_iniziale, iterazioni=300, tolleranza=1e-4,
                 smorzamento=0.05):
    """Minimi quadrati smorzati sulla sola posizione del TCP.

    Sulla sola posizione e non sull'orientamento perche' per infilare la pinza
    nella maniglia di un cassetto cio' che conta e' arrivarci: l'orientamento
    viene imposto a parte, scegliendo il seme (q_iniziale) gia' con il polso
    nell'assetto voluto. Ritorna (q, errore_metri, convergiuto).
    """
    q = np.array(q_iniziale, dtype=float)
    bersaglio = np.array(bersaglio, dtype=float)

    for _ in range(iterazioni):
        errore = bersaglio - tcp(q)
        norma = float(np.linalg.norm(errore))
        if norma < tolleranza:
            return q, norma, True

        J = _jacobiano_posizione(q)
        # (J J^T + l^2 I)^-1 e' sempre invertibile: niente esplosioni vicino
        # alle singolarita', al prezzo di un passo leggermente piu' corto.
        JJt = J @ J.T + (smorzamento ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(JJt, errore)

        # Passo limitato: senza questo la prima iterazione da lontano sbatte il
        # braccio dall'altra parte dell'inviluppo e l'IK non torna piu' indietro.
        massimo = float(np.max(np.abs(dq)))
        if massimo > 0.2:
            dq *= 0.2 / massimo
        q = q + dq

    return q, float(np.linalg.norm(bersaglio - tcp(q))), False


def raggiungibile(punto, margine=0.02):
    """Test rapido prima di sprecare iterazioni: distanza dalla spalla contro
    la lunghezza massima della catena braccio+avambraccio+polso+pinza."""
    spalla = np.array([0.0, 0.0, 0.152])
    portata = 0.12 + 0.244 + 0.213 + 0.104 + 0.085 + 0.09 + GRIPPER_TCP[1]
    return float(np.linalg.norm(np.array(punto) - spalla)) <= portata - margine


# ----------------------------------------------------------------------
# Cinematica inversa su posa completa
# ----------------------------------------------------------------------
# Per agganciare la maniglia di un cassetto non basta portarci il punto di
# presa: la pinza deve anche arrivarci diritta, altrimenti le dita sbattono
# sul frontale. Servono quindi sei vincoli, tanti quanti i giunti.
#
# Nel frame del TCP le dita si estendono lungo +z e si chiudono lungo y.
# Quindi T[:3, 2] e' l'asse che deve puntare dentro al cassetto, e T[:3, 1]
# quello che deve restare orizzontale per stringere una maniglia verticale.

def asse_avvicinamento(T):
    """Versore lungo cui si estendono le dita, dato un frame TCP 4x4."""
    return T[:3, 2]


def asse_chiusura(T):
    """Versore lungo cui le due dita si avvicinano."""
    return T[:3, 1]


def _log_rotazione(R):
    """Vettore rotazione (asse * angolo) da una matrice: l'errore angolare in
    forma utilizzabile dallo jacobiano."""
    traccia = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angolo = math.acos(traccia)
    if angolo < 1e-9:
        return np.zeros(3)
    if abs(angolo - math.pi) < 1e-6:
        # Vicino a mezzo giro il denominatore sin(angolo) va a zero: si ricava
        # l'asse dalla diagonale, che li' resta ben condizionata.
        d = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        k = int(np.argmax(d))
        asse = np.zeros(3)
        asse[k] = d[k]
        for j in range(3):
            if j != k:
                asse[j] = (R[j, k] + R[k, j]) / (4.0 * d[k]) if d[k] > 1e-9 else 0.0
        n = np.linalg.norm(asse)
        return (asse / n) * angolo if n > 1e-9 else np.zeros(3)
    return (angolo / (2.0 * math.sin(angolo))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def _jacobiano_posa(q, eps=1e-6):
    """Jacobiano 6x6 (3 di posizione + 3 di orientamento) per differenze finite."""
    J = np.zeros((6, 6))
    T0 = fk(q)
    p0, R0 = T0[:3, 3], T0[:3, :3]
    for i in range(6):
        qp = list(q)
        qp[i] += eps
        T1 = fk(qp)
        J[:3, i] = (T1[:3, 3] - p0) / eps
        J[3:, i] = _log_rotazione(T1[:3, :3] @ R0.T) / eps
    return J


def ik_posa(T_bersaglio, q_iniziale, iterazioni=400, toll_pos=1e-4,
            toll_ang=1e-3, smorzamento=0.05):
    """Minimi quadrati smorzati su posizione e orientamento.

    Ritorna (q, errore_posizione_m, errore_angolo_rad, convergiuto).
    """
    q = np.array(q_iniziale, dtype=float)
    p_b = np.array(T_bersaglio)[:3, 3]
    R_b = np.array(T_bersaglio)[:3, :3]

    for _ in range(iterazioni):
        T = fk(q)
        e_pos = p_b - T[:3, 3]
        e_ang = _log_rotazione(R_b @ T[:3, :3].T)
        n_pos, n_ang = float(np.linalg.norm(e_pos)), float(np.linalg.norm(e_ang))
        if n_pos < toll_pos and n_ang < toll_ang:
            return q, n_pos, n_ang, True

        J = _jacobiano_posa(q)
        errore = np.concatenate([e_pos, e_ang])
        JJt = J @ J.T + (smorzamento ** 2) * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, errore)

        massimo = float(np.max(np.abs(dq)))
        if massimo > 0.2:
            dq *= 0.2 / massimo
        q = q + dq

    T = fk(q)
    return (q, float(np.linalg.norm(p_b - T[:3, 3])),
            float(np.linalg.norm(_log_rotazione(R_b @ T[:3, :3].T))), False)


def posa_bersaglio(punto, avvicinamento, su=(0.0, 0.0, 1.0)):
    """Posa 4x4 del TCP da un punto e da una direzione di avvicinamento.

    'avvicinamento' e' dove devono puntare le dita: diventa l'asse z. L'asse y,
    lungo cui le dita si chiudono, viene messo orizzontale, cosi' la pinza
    stringe una maniglia verticale invece di scavalcarla.
    """
    z = np.array(avvicinamento, dtype=float)
    z /= np.linalg.norm(z)
    su = np.array(su, dtype=float)
    if abs(float(np.dot(su, z))) > 0.99:      # avvicinamento quasi verticale
        su = np.array([1.0, 0.0, 0.0])
    y = np.cross(su, z)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
    T[:3, 3] = np.array(punto, dtype=float)
    return T
