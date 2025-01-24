from setuptools import setup
from glob import glob
import os 

package_name = 'object_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # ('share/' + package_name + '/config', glob('config/*.yaml')),
        # ('share/' + package_name + '/cfg', 
        #  [f for f in glob('cfg/**/*', recursive=True) if os.path.isfile(f)])
    ],
    zip_safe=True,
    maintainer='Petr Vanc',
    maintainer_email='petr.vanc@cvut.cz',
    author='Giovanni Franzese',
    author_email='g.franzese@tudelft.nl',
    description='Box localization in python. This is a modified version of the original package created by Giovanni Franzese. Modifications include ROS2 support, and Jean Elsner PandaPy backend.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "static_transform_camera = object_localization.static_transform_camera:main",
            "record_template = object_localization.record_template:main",
            "localizer_service = object_localization.localizer_service:main",
            "active_localizer = object_localization.active_localizer:main",
        ],
    },
)
