# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
from glob import glob
from setuptools import find_packages, setup

setup(
    name='doc_lab', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/doc_lab']),
        ('share/doc_lab', ['package.xml']),
        ('share/doc_lab/config', glob('config/*.json')),
        ('share/doc_lab/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Michael', maintainer_email='177836445+MichaelGAglieri@users.noreply.github.com',
    description='DOC: coordinamento della simulazione di laboratorio a cinque assi',
    license='GPL-3.0-only',
    entry_points={'console_scripts': [
        'laboratorio_node = doc_lab.laboratorio_node:main',
        'mission_monitor = doc_lab.mission_monitor:main',
        'stt_node = doc_lab.stt_node:main',
        'tts_node = doc_lab.tts_node:main',
        'llm_brain_node = doc_lab.llm_brain_node:main',
    ]},
)
