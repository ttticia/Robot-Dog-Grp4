from glob import glob

from setuptools import find_packages, setup

package_name = 'maze_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/sim', glob('sim/*.sdf')),
        ('share/' + package_name + '/sim/mazes', glob('sim/mazes/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngtri',
    maintainer_email='ng.tricia2005@gmail.com',
    description='Maze exploration for the Lite3 with slam_toolbox, Nav2 and explore_lite, '
                'plus 360-degree spins, camera-coverage goals and a Gazebo test world.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'explore_supervisor = maze_explorer.explore_supervisor_node:main',
            'cmd_vel_relay = maze_explorer.cmd_vel_relay_node:main',
            'step_height = maze_explorer.step_height_node:main',
        ],
    },
)
