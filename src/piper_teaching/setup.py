from glob import glob
from setuptools import find_packages, setup

setup(
    name='piper_teaching', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/piper_teaching']),
        ('share/piper_teaching', ['package.xml']),
        ('share/piper_teaching/launch', glob('launch/*.launch.py')),
        ('share/piper_teaching/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'bilateral_teaching = piper_teaching.node:main',
    ]},
)
