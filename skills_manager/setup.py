from setuptools import setup
from glob import glob
import os 

package_name = 'skills_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    zip_safe=True,
    maintainer='Petr Vanc',
    maintainer_email='petr.vanc@cvut.cz',
    author='Giovanni Franzese',
    author_email='g.franzese@tudelft.nl',
    description='This is a modified version of the original package created by Giovanni Franzese. Modifications include ROS2 support, and Jean Elsner PandaPy backend.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "home = skills_manager.home:main",
            "record_skill = skills_manager.record_skill:main",
            "play_skill = skills_manager.play_skill:main",
        ],
    },
)




