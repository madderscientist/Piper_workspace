from glob import glob
import os
from setuptools import find_packages, setup

setup(
    name='piper_friction',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/piper_friction']),
        ('share/piper_friction', ['package.xml']),
        (os.path.join('share', 'piper_friction', 'web'), glob('web/*')),
        (os.path.join('share', 'piper_friction', 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'sweep = piper_friction.cli:sweep_main',
            'fit = piper_friction.cli:fit_main',
            'evaluate = piper_friction.cli:evaluate_main',
            'calibrate_v0 = piper_friction.cli:v0_main',
            'web = piper_friction.web:web_main',
        ],
    },
)
