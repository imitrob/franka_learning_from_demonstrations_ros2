#!/usr/bin/env python

from setuptools import setup
from glob import glob
import os 

package_name = 'quaternion_algebra'

setup(
    name=package_name,
    version='1.0.0',
    description='This package is a collection of function to perform agebraic operation on quaternion',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
    ],
    zip_safe=True,
    maintainer='Giovanni Franzese',
    maintainer_email='g.franzese@tudelft.nl',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'scene_marker_pub = scene_getter.scene_marker_pub:main',
        ],
    },
)

