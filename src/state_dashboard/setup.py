from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'state_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'dashboard'), glob('dashboard/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Workspace maintainer',
    maintainer_email='maintainer@example.invalid',
    description='Four-arm Web telemetry, configuration and MIT debugging through ROS.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_dashboard = state_dashboard.dashboard:main',
        ],
    },
)
