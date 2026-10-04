import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tekbot_pathfinding'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm') + glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='black-nexus06',
    maintainer_email='danekando@gmail.com',
    description='Pathfinding A* et suivi de chemin pour TekBot (Test 3 TRC2K25)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_node = tekbot_pathfinding.planner_node:main',
            'controller_node = tekbot_pathfinding.controller_node:main',
        ],
    },
)
