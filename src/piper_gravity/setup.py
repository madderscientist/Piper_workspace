from glob import glob
from setuptools import find_packages, setup

setup(
    name='piper_gravity',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/piper_gravity']),
        ('share/piper_gravity', ['package.xml']),
        ('share/piper_gravity/config', glob('config/*.yaml')),
        ('share/piper_gravity/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'probe = piper_gravity.cli:probe_main',
            'fit = piper_gravity.cli:fit_main',
            'evaluate = piper_gravity.cli:evaluate_main',
            'web = piper_gravity.web:web_main',
        ],
    },
)
