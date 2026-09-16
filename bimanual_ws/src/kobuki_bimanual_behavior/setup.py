import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'kobuki_bimanual_behavior'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nishrraj',
    maintainer_email='nishrraj@todo.todo',
    description='Perception, IK and mission logic for the bimanual demo.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_node = kobuki_bimanual_behavior.perception_node:main',
            'mission_node = kobuki_bimanual_behavior.mission_node:main',
        ],
    },
)
