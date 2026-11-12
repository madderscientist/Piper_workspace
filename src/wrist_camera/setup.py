import os
from glob import glob
from setuptools import setup

package_name = 'wrist_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={
        package_name: ['static/*.html', 'static/*.css', 'static/*.js'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piper_x',
    maintainer_email='dev@example.com',
    description='PiPER-X 腕部相机纯 Python RTSP 取图、内外参读取、网页预览与单臂手眼标定',
    license='MIT',
    entry_points={
        'console_scripts': [
            'preview = wrist_camera.preview:main',
            'calibrate = wrist_camera.calib:main',
            'make_board = wrist_camera.make_board:main',
        ],
    },
)
