from glob import glob

from setuptools import find_packages, setup

package_name = 'team_perception_external'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngtri',
    maintainer_email='ng.tricia2005@gmail.com',
    description='External-computer build of team_perception: YOLO person detector '
                '(human_detector) for the robot dog.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'human_detector = team_perception_external.human_detector_node:main',
        ],
    },
)
