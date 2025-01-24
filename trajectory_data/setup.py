#!/usr/bin/env python

from setuptools import setup
from glob import glob
import os 

package_name = 'trajectory_data'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
    ],
    zip_safe=True,
    maintainer='Petr Vanc',
    maintainer_email='petr.vanc@cvut.cz',
    author='Giovanni Franzese',
    author_email='g.franzese@tudelft.nl',
    description='This is a modified version of the original package created by Giovanni Franzese.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'scene_marker_pub = scene_getter.scene_marker_pub:main',
        ],
    },
)

