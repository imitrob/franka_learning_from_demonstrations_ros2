#!/usr/bin/env python

from setuptools import find_packages, setup

package_name = 'trajectory_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests']),
    package_data={'trajectory_tools.trajectory_to_video': ['*.stl']},
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='Petr Vanc',
    maintainer_email='petr.vanc@cvut.cz',
    description='Code that reads trajectory_data: skill names, viewers, npz-to-video.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'make_videos = trajectory_tools.make_videos:main',
            'render_skill = trajectory_tools.trajectory_to_video.render_skill:main',
            'check_mesh = trajectory_tools.trajectory_to_video.check_mesh:main',
        ],
    },
)
