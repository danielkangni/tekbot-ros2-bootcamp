import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sensor_data_evaluation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='black-nexus06',
    maintainer_email='jean-daniel.kangni@epitech.eu',
    description='Package ROS2 publiant et évaluant des données de capteurs simulées',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher_node = sensor_data_evaluation.publisher_node:main',
            'subscriber_node = sensor_data_evaluation.subscriber_node:main',
        ],
    },
)
