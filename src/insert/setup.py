from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'insert'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Workspace maintainer',
    maintainer_email='maintainer@example.invalid',
    description='InsertAnything ONNX policy deployment and Web dashboard for PiPER-X.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web = insert.web:web_main',
            'dashboard = insert.web:web_main',
            'insert = insert.web:web_main',
        ],
    },
)
