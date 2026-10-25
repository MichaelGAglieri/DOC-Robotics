#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Genera banco, pinza a tre dita e traiettorie della demo di laboratorio.

Geometria e pose hanno un'unica sorgente. DocArm5 conserva i primi cinque
assi dell'UR3e: queste pose non sono comandi per il DOC fisico.
"""
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'architetture/laboratorio/ros2_ws/src/doc_lab'))
from doc_lab.doc5_kinematics import fk, ik_axis, LIMITI, NOMI_GIUNTI, GRIPPER_TCP
from genera_doc5 import generate as generate_arm

BASE = (0.0, 0.0, 0.77)
BENCH_Z = 0.75
HEIGHT = 0.080
RADIUS = 0.019
GRIP_HEIGHT = 0.064
SUPPORT_HEIGHT = 0.006
GRIP_Z = BENCH_Z - BASE[2] + SUPPORT_HEIGHT + GRIP_HEIGHT
TRAVEL_Z = 0.23
HOME = (0.31, -0.03, TRAVEL_Z)
STATIONS = {
    'supporto_a': (0.32, -0.15),
    'supporto_b': (0.32, -0.03),
    'postazione_1': (0.32, 0.09),
    'postazione_2': (0.32, 0.21),
}
SAMPLES = {'campione_a': 'supporto_a', 'campione_b': 'supporto_b'}
CONFIG = ROOT / 'webots_sim/laboratorio.json'
WORLD = ROOT / 'webots_sim/worlds/laboratorio.wbt'
GRIPPER = ROOT / 'webots_sim/protos/DocLabGripper.proto'


def world_point(p):
    return [float(p[i] + BASE[i]) for i in range(3)]


def solve(p, seed):
    q, ep, ea, ok = ik_axis(p, seed, toll_pos=2e-6, toll_ang=2e-5)
    if not ok or any(not lo <= v <= hi for v, (lo, hi) in zip(q, LIMITI)) or abs(q[2]) > math.pi:
        raise ValueError(f'Posa irraggiungibile: {p}, errore {ep:.6f} m / {ea:.6f} rad')
    return [round(float(v), 8) for v in q]


def generate_config():
    points = {'home': list(HOME)}
    for station, (x, y) in STATIONS.items():
        points[f'{station}_approach'] = [x, y, TRAVEL_Z]
        points[f'{station}_grip'] = [x, y, GRIP_Z]
    home = solve(HOME, [0, -1.4, 1.6, -1.75, -1.5708])
    poses = {'home': home}
    for name, p in points.items():
        if name != 'home':
            poses[name] = solve(p, home)
    high = ['home'] + [f'{s}_approach' for s in STATIONS]
    edges = [(a, b) for a in high for b in high if a != b]
    edges += [(f'{s}_{a}', f'{s}_{b}') for s in STATIONS
              for a, b in [('approach', 'grip'), ('grip', 'approach')]]
    routes = {}
    for start, end in edges:
        a, b = np.array(points[start]), np.array(points[end])
        count = max(2, math.ceil(float(np.linalg.norm(b - a)) / 0.004))
        path = [poses[start]]
        for i in range(1, count):
            path.append(solve(a + (b - a) * i / count, path[-1]))
        q_end = solve(b, path[-1])
        if max(abs(x - y) for x, y in zip(q_end, poses[end])) > 0.01:
            raise ValueError(f'Cambio di ramo IK sulla tratta {start} -> {end}')
        path.append(poses[end])
        routes[f'{start}->{end}'] = path
    return {
        'scenario': 'laboratorio', 'robot': 'DocArm5, UR3e-derived 5 axes',
        'arm_dof': len(NOMI_GIUNTI), 'joint_names': NOMI_GIUNTI,
        'base': list(BASE), 'bench_z': BENCH_Z,
        'sample_height': HEIGHT, 'sample_radius': RADIUS,
        'grip_height': GRIP_HEIGHT, 'support_height': SUPPORT_HEIGHT,
        'center_offset': [0, 0, HEIGHT / 2 - GRIP_HEIGHT],
        'stations': {s: world_point((x, y, GRIP_Z)) for s, (x, y) in STATIONS.items()},
        'samples': SAMPLES, 'points': points, 'waypoint': poses, 'routes': routes,
    }


def appearance(color, roughness=0.45, metalness=0, texture=None):
    maps = ''
    if texture:
        maps = f'''baseColorMap ImageTexture {{ url ["../textures/{texture}_color.png"] }}
          roughnessMap ImageTexture {{ url ["../textures/{texture}_rough.png"] }}
          normalMap ImageTexture {{ url ["../textures/{texture}_normal.png"] }} normalMapFactor 0.35'''
    return f'PBRAppearance {{ baseColor {color} roughness {roughness} metalness {metalness} {maps} }}'


def box(p, size, color, solid=False, name='', roughness=0.45, metalness=0, texture=None):
    node = 'Solid' if solid else 'Pose'
    collision = f'boundingObject Box {{ size {size} }}' if solid else ''
    return f'''{node} {{ translation {p}
      children [ Shape {{ appearance {appearance(color, roughness, metalness, texture)}
        geometry Box {{ size {size} }} }} ]
      {collision} {f'name "{name}"' if solid else ''} }}\n'''


def cylinder(p, radius, height, color, roughness=0.4, metalness=0):
    return f'''Pose {{ translation {p} children [ Shape {{
      appearance {appearance(color, roughness, metalness)}
      geometry Cylinder {{ radius {radius} height {height} subdivision 64 }} }} ] }}\n'''


def lathe(profile, segments=64):
    """Superficie di rivoluzione: profilo (z, r) con raccordi e normali lisce."""
    vertices = []
    faces = []
    for z, r in profile:
        for i in range(segments):
            a = 2*math.pi*i/segments
            vertices.append(f'{r*math.cos(a):.7f} {r*math.sin(a):.7f} {z:.7f}')
    for row in range(len(profile)-1):
        for i in range(segments):
            a, b = row*segments+i, row*segments+(i+1)%segments
            faces.append(f'{a} {b} {b+segments} {a+segments} -1')
    faces.append(' '.join(str(i) for i in reversed(range(segments)))+' -1')
    faces.append(' '.join(str((len(profile)-1)*segments+i) for i in range(segments))+' -1')
    return ('IndexedFaceSet { creaseAngle 0.65 coord Coordinate { point [ '
            + ', '.join(vertices)+' ] } coordIndex [ '+', '.join(faces)+' ] }')


def camera_pose(eye, target):
    forward = np.array(target)-np.array(eye)
    forward /= np.linalg.norm(forward)
    left = np.cross([0, 0, 1], forward)
    left /= np.linalg.norm(left)
    up = np.cross(forward, left)
    rotation = np.column_stack([forward, left, up])
    angle = math.acos(np.clip((np.trace(rotation)-1)/2, -1, 1))
    axis = np.array([rotation[2, 1]-rotation[1, 2], rotation[0, 2]-rotation[2, 0],
                     rotation[1, 0]-rotation[0, 1]])
    axis /= np.linalg.norm(axis)
    return ' '.join(f'{v:.9f}' for v in [*axis, angle])


def plaque(name, p, size):
    # Webots non implementa il nodo VRML Text: etichette su texture PNG locali.
    return f'''Pose {{ translation {p}
      children [ Shape {{ appearance PBRAppearance {{
        baseColorMap ImageTexture {{ url ["../textures/{name}.png"] }}
        roughness 1 metalness 0 }} geometry Plane {{ size {size} }} }} ] }}'''


def render_labels():
    from PIL import Image, ImageDraw, ImageFont
    textures = ROOT/'webots_sim/textures'
    textures.mkdir(exist_ok=True)
    font_path = Path('/System/Library/Fonts/Supplemental/Arial Bold.ttf')
    font = ImageFont.truetype(str(font_path), 70) if font_path.exists() else ImageFont.load_default(size=70)
    labels = {'titolo': 'DOC  /  LABORATORIO', 'supporto_a': 'SUPPORTO A',
              'supporto_b': 'SUPPORTO B', 'postazione_1': 'POSTAZIONE 1', 'postazione_2': 'POSTAZIONE 2'}
    for name, label in labels.items():
        canvas = Image.new('RGB', (1024, 256), (232, 232, 226))
        draw = ImageDraw.Draw(canvas)
        draw.rectangle((4, 4, 1019, 251), outline=(115, 118, 116), width=3)
        draw.text((512, 128), label, font=font, anchor='mm', fill=(33, 35, 36))
        canvas.save(textures/f'{name}.png')
    for sample in SAMPLES:
        canvas = Image.new('RGB', (1024, 256), (234, 234, 225))
        draw = ImageDraw.Draw(canvas)
        suffix = sample[-1].upper()
        draw.text((512, 55), f'CAMPIONE {suffix}   |   DOC-LAB', font=font, anchor='mm', fill=(25, 28, 29))
        small = ImageFont.truetype(str(font_path), 35) if font_path.exists() else ImageFont.load_default(size=35)
        draw.text((512, 119), f'ID 2026-10-{suffix}     CONTENITORE VUOTO', font=small, anchor='mm', fill=(45, 47, 48))
        rng = np.random.default_rng(ord(suffix))
        for x in range(280, 745, 7):
            draw.rectangle((x, 163, x+int(rng.integers(1, 5)), 228), fill=(30, 32, 33))
        canvas.save(textures/f'{sample}_label.png')


def render_materials():
    """Texture procedurali locali: microvariazioni, senza nuovi asset remoti."""
    from PIL import Image
    textures = ROOT/'webots_sim/textures'
    rng = np.random.default_rng(20261012)
    for name, base, rough, strength in [
        ('epoxy', (85, 88, 89), 140, 1.8),
        ('steel', (165, 166, 163), 90, 1.0),
        ('floor', (176, 178, 173), 195, 3.2),
    ]:
        noise = rng.normal(0, strength, (1024, 1024))
        if name == 'steel':
            noise += rng.normal(0, 1.6, (1024, 1))
        color = np.clip(np.array(base)[None, None, :]+noise[:, :, None], 0, 255).astype('uint8')
        Image.fromarray(color).save(textures/f'{name}_color.png')
        Image.fromarray(np.clip(rough+noise*2, 0, 255).astype('uint8')).save(textures/f'{name}_rough.png')
        gy, gx = np.gradient(noise)
        normal = np.stack([np.clip(128-gx*2, 0, 255), np.clip(128-gy*2, 0, 255),
                           np.full_like(gx, 254)], axis=-1).astype('uint8')
        Image.fromarray(normal).save(textures/f'{name}_normal.png')


def rounded_housing(width, depth, radius, layers):
    """Guscio raccordato: sezioni rettangolari con angoli arrotondati."""
    vertices, faces = [], []
    contour = []
    for cx, cy, start in [(width/2-radius, depth/2-radius, 0),
                          (-width/2+radius, depth/2-radius, 90),
                          (-width/2+radius, -depth/2+radius, 180),
                          (width/2-radius, -depth/2+radius, 270)]:
        for i in range(9):
            a = math.radians(start+i*90/8)
            contour.append((cx+radius*math.cos(a), cy+radius*math.sin(a)))
    n = len(contour)
    for z, scale in layers:
        vertices.extend(f'{x*scale:.7f} {y*scale:.7f} {z:.7f}' for x, y in contour)
    for row in range(len(layers)-1):
        for i in range(n):
            a, b = row*n+i, row*n+(i+1)%n
            faces.append(f'{a} {b} {b+n} {a+n} -1')
    faces += [' '.join(str(i) for i in reversed(range(n)))+' -1',
              ' '.join(str((len(layers)-1)*n+i) for i in range(n))+' -1']
    return ('IndexedFaceSet { creaseAngle 0.65 coord Coordinate { point [ '
            + ', '.join(vertices)+' ] } coordIndex [ '+', '.join(faces)+' ] }')



def hex_housing(width, depth, layers):
    """Guscio esagonale del modello corrente; le dita restano raccordate."""
    contour = [(width/2, 0), (width/4, depth/2), (-width/4, depth/2),
               (-width/2, 0), (-width/4, -depth/2), (width/4, -depth/2)]
    vertices, faces = [], []
    n = len(contour)
    for z, scale in layers:
        vertices.extend(f'{x*scale:.7f} {y*scale:.7f} {z:.7f}' for x, y in contour)
    for row in range(len(layers)-1):
        for i in range(n):
            a, b = row*n+i, row*n+(i+1)%n
            faces.append(f'{a} {b} {b+n} {a+n} -1')
    faces += [' '.join(str(i) for i in reversed(range(n)))+' -1',
              ' '.join(str((len(layers)-1)*n+i) for i in range(n))+' -1']
    return ('IndexedFaceSet { creaseAngle 0.35 coord Coordinate { point [ '
            + ', '.join(vertices)+' ] } coordIndex [ '+', '.join(faces)+' ] }')


def gripper_source():
    fingers = []
    for i in range(3):
        angle = 2 * math.pi * i / 3
        c, s = math.cos(angle), math.sin(angle)
        fingers.append(f'''SliderJoint {{
          jointParameters JointParameters {{ axis {-c:.9f} {-s:.9f} 0 minStop 0 maxStop 0.033 }}
          device [ LinearMotor {{ name "lab_finger_{i+1}" maxVelocity 0.08 maxForce 20 minPosition 0 maxPosition 0.033 }}
                   PositionSensor {{ name "lab_finger_{i+1}_sensor" }} ]
          endPoint Solid {{ translation {0.054*c:.9f} {0.054*s:.9f} 0.14696
            rotation 0 0 1 {angle:.9f}
            children [
              Shape {{ appearance {appearance('0.018 0.022 0.018', 0.32)}
                geometry {rounded_housing(0.029, 0.035, 0.006, [(-0.030, 0.92), (-0.027, 1), (-0.009, 1), (-0.006, 0.94)])} }}
              Pose {{ translation 0.003 0 0 children [ Shape {{ appearance {appearance('0.018 0.022 0.018', 0.32)}
                geometry {rounded_housing(0.018, 0.032, 0.006, [(-0.013, 0.95), (-0.011, 1), (0.030, 1), (0.033, 0.94), (0.035, 0.78)])} }} ] }}
              {box('-0.0065 0 0.010', '0.001 0.026 0.043', '0.40 0.88 0.012', roughness=0.23, metalness=1)}
              Pose {{ translation -0.008 0 -0.003 rotation 0 1 0 1.570796327
                children [ {cylinder('0 0 0', 0.0026, 0.0015, '0.22 0.23 0.24', 0.24, 1)} ] }}
              Pose {{ translation -0.008 0 0.020 rotation 0 1 0 1.570796327
                children [ {cylinder('0 0 0', 0.0026, 0.0015, '0.22 0.23 0.24', 0.24, 1)} ] }}
              Pose {{ translation -0.006 0 0.030
                children [ Shape {{ appearance PBRAppearance {{ baseColor 0.032 0.035 0.038 roughness 0.87 metalness 0 }}
                  geometry Box {{ size 0.004 0.020 0.022 }} }} ] }}
            ]
            name "lab_finger_{i+1}"
            # Cursore cinematico: questa pinza descrive apertura e chiusura,
            # senza attribuire masse o inerzie non misurate al gripper futuro.
          }}
        }}''')
    bolts = ''.join(cylinder(f'{0.027*math.cos(i*math.pi/3):.6f} {0.027*math.sin(i*math.pi/3):.6f} 0.003',
                            0.0025, 0.0015, '0.55 0.56 0.57', 0.24, 1) for i in range(6))
    source = '''#VRML_SIM R2025a utf8
# Aspetto ispirato all'EBG-30: guscio esagonale, tre dita larghe e inserti.
# Riferimento: https://toolboxrobotics.com/robotic-gripper-ebg30
# Pinza radiale schematica: tre cursori comandati come un solo attuatore.
# Non rappresenta la cinematica o le forze di un EBG-30 reale.
PROTO DocLabGripper [ field SFRotation rotation 1 0 0 -1.5707953071795862 ] {
  Solid {
    rotation IS rotation
    children [
      Shape { appearance PBRAppearance { baseColor 0.40 0.88 0.012 roughness 0.34 metalness 0 }
        geometry __BODY__ }
      Shape { appearance PBRAppearance { baseColor 0.018 0.022 0.018 roughness 0.30 metalness 0 }
        geometry __COVER__ }
      Pose { translation 0 0 0.015
        children [ Shape { appearance PBRAppearance { baseColor 0.018 0.022 0.018 roughness 0.30 metalness 1 }
          geometry Cylinder { radius 0.033 height 0.027 subdivision 64 } } ] }
      Shape { appearance PBRAppearance { baseColor 0.40 0.88 0.012 roughness 0.34 metalness 0 }
        geometry __DECK__ }
''' + bolts + '\n'.join(fingers) + '''
    ]
    name "doc_lab_gripper"
    boundingObject Pose { translation 0 0 0.045 children [ Cylinder { radius 0.035 height 0.09 } ] }
    physics Physics { density -1 mass 0.18 }
  }
}
'''
    return (source.replace('__BODY__', hex_housing(0.104, 0.098,
                [(0.027, 0.83), (0.029, 0.92), (0.032, 0.97), (0.036, 1), (0.076, 1)]))
            .replace('__COVER__', hex_housing(0.104, 0.098,
                [(0.077, 1), (0.115, 1), (0.119, 0.98), (0.121, 0.96)]))
            .replace('__DECK__', hex_housing(0.100, 0.095,
                [(0.120, 1), (0.123, 1), (0.124, 0.98)])))


def station_source(station, x, y):
    details = cylinder('0 0 0.00255', 0.023, 0.0007, '0.075 0.078 0.080', 0.84)
    # Boccola incassata e tre viti sul disco in alluminio, alla stessa quota operativa.
    for i in range(3):
        a = 2*math.pi*i/3
        px, py = 0.031*math.cos(a), 0.031*math.sin(a)
        details += cylinder(f'{px} {py} 0.00255', 0.0025, 0.0008, '0.12 0.13 0.14', 0.3, 1)
        details += box(f'{px} {py} 0.003', '0.0026 0.0004 0.0002', '0.015 0.017 0.018')
    return f'''DEF {station.upper()} Solid {{ translation {x} {y} {BENCH_Z+SUPPORT_HEIGHT/2}
      children [ Shape {{ appearance {appearance('0.58 0.59 0.60', 0.32, 1)}
        geometry Cylinder {{ radius 0.038 height {SUPPORT_HEIGHT} subdivision 64 }} }}
        {details} ] name "{station}"
      boundingObject Cylinder {{ radius 0.038 height {SUPPORT_HEIGHT} }}
    }}\n'''


def bottle_source(sample, station):
    x, y = STATIONS[station]
    color = '0.10 0.16 0.22' if sample.endswith('a') else '0.17 0.17 0.16'
    body = lathe([(-0.04, 0.016), (-0.039, 0.018), (-0.035, 0.019),
                  (0.027, 0.019), (0.030, 0.0185), (0.034, 0.013), (0.040, 0.013)])
    details = cylinder('0 0 0.038', 0.0168, 0.013, color, 0.43)
    details += cylinder('0 0 0.031', 0.0169, 0.0018, '0.045 0.049 0.050', 0.55)
    for i in range(48):
        angle = 2*math.pi*i/48
        px, py = 0.0168*math.cos(angle), 0.0168*math.sin(angle)
        details += f'''Pose {{ translation {px:.7f} {py:.7f} 0.038 rotation 0 0 1 {angle:.7f}
          children [ Shape {{ appearance {appearance(color, 0.45)}
            geometry Box {{ size 0.0009 0.00055 0.009 }} }} ] }}\n'''
    return f'''DEF {sample.upper()} Solid {{ translation {x} {y} {BENCH_Z+SUPPORT_HEIGHT+HEIGHT/2}
      children [
        Shape {{ appearance {appearance('0.88 0.89 0.87', 0.24)} geometry {body} }}
        Pose {{ translation 0 0 -0.003 rotation 0 0 1 -0.85 children [ Shape {{
          appearance PBRAppearance {{ baseColorMap ImageTexture {{ url ["../textures/{sample}_label.png"] }}
            roughness 0.73 metalness 0 }}
          geometry Cylinder {{ radius 0.0191 height 0.026 subdivision 64 }} }} ] }}
        {details}
      ] name "{sample}"
      # Presa vincolata dal supervisore; nessun modello di attrito del flacone.
    }}\n'''


def room_source():
    source = box('0.0 1.25 1.35', '4.5 0.10 2.7', '0.82 0.84 0.83', True, 'parete_fondo', roughness=0.9)
    source += box('-1.15 0.10 1.35', '0.10 2.3 2.7', '0.85 0.86 0.84', True, 'parete_laterale', roughness=0.9)
    source += box('0.10 0.88 0.79', '2.1 0.55 0.04', '0.25 0.26 0.27', texture='epoxy')
    source += box('0.10 0.90 0.395', '2.0 0.46 0.72', '0.64 0.65 0.62', roughness=0.62)
    for x in (-0.65, -0.15, 0.35, 0.85):
        source += box(f'{x} 0.664 0.40', '0.483 0.015 0.68', '0.84 0.85 0.83', roughness=0.48)
        source += box(f'{x+0.18} 0.646 0.54', '0.012 0.015 0.13', '0.37 0.38 0.39', metalness=1, roughness=0.27)
    source += box('0.10 1.188 0.065', '4.2 0.02 0.13', '0.24 0.25 0.24')
    source += box('-1.088 0.10 0.065', '0.02 2.3 0.13', '0.24 0.25 0.24')
    for x in (-0.72, 0.74):
        source += box(f'{x} 1.185 1.08', '0.14 0.018 0.08', '0.83 0.83 0.79')
        for dx in (-0.035, 0.035):
            source += box(f'{x+dx} 1.174 1.08', '0.015 0.002 0.007', '0.10 0.10 0.10')
    # Portaprovette e piccoli strumenti sul banco posteriore, fuori dal volume di lavoro.
    source += box('0.73 0.86 0.828', '0.22 0.11 0.035', '0.21 0.22 0.21', roughness=0.5)
    for i in range(5):
        source += cylinder(f'{0.65+i*0.039:.4f} 0.855 0.867', 0.008, 0.085, '0.60 0.64 0.63', 0.2)
        source += cylinder(f'{0.65+i*0.039:.4f} 0.855 0.910', 0.009, 0.010, '0.13 0.18 0.23', 0.45)
    source += box('-0.62 0.89 0.831', '0.13 0.13 0.04', '0.58 0.60 0.59', roughness=0.35)
    source += box('-0.62 0.92 0.973', '0.012 0.018 0.245', '0.35 0.36 0.37', roughness=0.27, metalness=1)
    for x in (-0.65, -0.62, -0.59):
        source += cylinder(f'{x} 0.895 0.986', 0.008, 0.15, '0.80 0.81 0.77', 0.31)
        source += cylinder(f'{x} 0.895 0.89', 0.003, 0.06, '0.48 0.53 0.53', 0.24)
    return source


def world_source(config):
    source = '''#VRML_SIM R2025a utf8
# Generato da scripts/genera_laboratorio.py. Unita': metri, z verso l'alto.
EXTERNPROTO "../protos/DocArm5.proto"
EXTERNPROTO "../protos/DocLabGripper.proto"
WorldInfo { title "DOC | Laboratorio: trasferimento campioni" basicTimeStep 16 }
DEF CAMERA Viewpoint { orientation __CAMERA__ position 0.94 -1.18 1.42
  fieldOfView 0.92 exposure 1.25 ambientOcclusionRadius 0.12 bloomThreshold -1 }
Background { skyColor [0.52 0.54 0.53] luminosity 1.35
  backIrradianceUrl ["../protos/vendor/webots/projects/default/worlds/textures/cubic/factory_back.hdr"]
  bottomIrradianceUrl ["../protos/vendor/webots/projects/default/worlds/textures/cubic/factory_bottom.hdr"]
  frontIrradianceUrl ["../protos/vendor/webots/projects/default/worlds/textures/cubic/factory_front.hdr"]
  leftIrradianceUrl ["../protos/vendor/webots/projects/default/worlds/textures/cubic/factory_left.hdr"]
  rightIrradianceUrl ["../protos/vendor/webots/projects/default/worlds/textures/cubic/factory_right.hdr"]
  topIrradianceUrl ["../protos/vendor/webots/projects/default/worlds/textures/cubic/factory_top.hdr"]
}
# Luce principale dall'alto; riempimenti senza ombre per schiarire i dettagli.
DEF LAB_KEY DirectionalLight { direction -0.6 0.7 -1.8 intensity 1.7 ambientIntensity 0.025
  color 1 1 1 castShadows TRUE }
DEF LAB_FILL DirectionalLight { direction 0.6 -0.4 -1 intensity 0.50 ambientIntensity 0
  color 0.93 0.97 1 castShadows FALSE }
DEF LAB_FRONT DirectionalLight { direction -0.82 1.41 -0.36 intensity 0.25 ambientIntensity 0
  color 1 1 1 castShadows FALSE }
'''
    source = source.replace('__CAMERA__', camera_pose((0.94, -1.18, 1.42), (0.21, 0.015, 1.00)))
    source += box('0.25 0 0.715', '1.2 0.95 0.07', '0.25 0.26 0.27', True, 'banco', texture='epoxy')
    source += box('0 0 0.76', '0.17 0.17 0.02', '0.30 0.31 0.32', True, 'base_braccio', metalness=1, roughness=0.31)
    for x in (-0.066, 0.066):
        for y in (-0.066, 0.066):
            source += cylinder(f'{x} {y} 0.771', 0.006, 0.002, '0.13 0.14 0.15', 0.27, 1)
    for x in (-0.25, 0.75):
        for y in (-0.36, 0.36):
            source += box(f'{x} {y} 0.35', '0.045 0.045 0.70', '0.50 0.51 0.52', True,
                          f'gamba_{x}_{y}', metalness=1, texture='steel')
            source += cylinder(f'{x} {y} 0.017', 0.033, 0.034, '0.055 0.055 0.052', 0.85)
    for y in (-0.36, 0.36):
        source += box(f'0.25 {y} 0.17', '1.0 0.03 0.03', '0.50 0.51 0.52', metalness=1, texture='steel')
    source += box('0 0 -0.015', '5 5 0.03', '0.57 0.56 0.54', True, 'pavimento', texture='floor')
    source += room_source()
    source += box('0.37 0.045 0.751', '0.28 0.58 0.002', '0.105 0.11 0.105', roughness=0.88)
    source += plaque('titolo', '0.59 -0.40 0.7507', '0.22 0.038')
    for station, (x, y) in STATIONS.items():
        source += station_source(station, x, y)
        source += plaque(station, f'{x+0.092} {y} 0.7522', '0.070 0.018')
    for sample, station in SAMPLES.items():
        source += bottle_source(sample, station)
    source += 'DEF DOC_ARM DocArm5 {\n'
    for i, q in enumerate(config['waypoint']['home']):
        axis = '0 0 1' if i in (0, 4) else '0 1 0'
        angle = q + (1.570796 if i in (1, 3) else 0)
        source += f'  hidden position_{i}_0 {q}\n  hidden rotation_{i+1} {axis} {angle}\n'
    source += '''  translation 0 0 0.77
  name "DOC laboratorio (5 assi)"
  controller "doc_laboratorio" supervisor TRUE
  toolSlot [
    DocLabGripper { }
    DEF LAB_TCP Solid { translation 0 0.17696 0 rotation 1 0 0 -1.5707953071795862 name "lab_tcp" }
  ]
}
'''
    return source


def main():
    config = generate_config()
    generate_arm()
    render_labels()
    render_materials()
    GRIPPER.parent.mkdir(parents=True, exist_ok=True)
    GRIPPER.write_text(gripper_source(), encoding='utf-8')
    WORLD.write_text(world_source(config), encoding='utf-8')
    CONFIG.write_text(json.dumps(config, indent=2) + '\n', encoding='utf-8')
    print(f'Laboratorio generato: {len(config["waypoint"])} pose, {len(config["routes"])} tratte cartesiane.')
    print(WORLD)


if __name__ == '__main__':
    main()
