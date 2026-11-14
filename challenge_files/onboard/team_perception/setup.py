from glob import glob

from setuptools import find_packages, setup

package_name = 'team_perception'

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
    description='Onboard robot build of team_perception: QR scanner, poster detector and '
                'the image throttle feeding the external YOLO computer.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'qr_scanner = team_perception.qr_scanner_node:main',
            'image_throttle = team_perception.image_throttle_node:main',
            'poster_detector = team_perception.poster_detector_node:main',
        ],
    },
)
