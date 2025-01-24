#!/usr/bin/env python

from setuptools import setup

package_name = 'panda_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
    ],
    zip_safe=True,
    maintainer='Petr Vanc',
    maintainer_email='petr.vanc@cvut.cz',
    author='Giovanni Franzese',
    author_email='g.franzese@tudelft.nl',
    description='Panda control library. This is a modified version of the original package created by Giovanni Franzese. Modifications include ROS2 support, and Jean Elsner PandaPy backend.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panda_test = panda_control.panda:main',
        ],
    },
)

