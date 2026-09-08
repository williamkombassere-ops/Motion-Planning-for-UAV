import os
from glob import glob
from setuptools import setup

package_name = 'motion_planning_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ton Nom',
    maintainer_email='you@example.com',
    description='Motion planning 2D (A*) pour le Crazyflie.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = motion_planning_project.planner_node:main',
            'path_follower_node = motion_planning_project.path_follower_node:main',
        ],
    },
)
