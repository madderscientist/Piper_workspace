from glob import glob
from setuptools import find_packages, setup

setup(
    name='piper_x_driver', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/piper_x_driver']),
        ('share/piper_x_driver', ['package.xml']),
        ('share/piper_x_driver/launch', glob('launch/*.launch.py')),
        ('share/piper_x_driver/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'driver = piper_x_driver.node:main',
            'power_off = piper_x_driver.node:power_off_main',
            'damping = piper_x_driver.gravity:damping_main',
        ],
    },
)
