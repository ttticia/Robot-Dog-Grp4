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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngtri',
    maintainer_email='ng.tricia2005@gmail.com',
    description='QR code scanner and YOLO person detector for the robot-dog challenge, '
                '+ an offline annotation tool.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_scanner = team_perception.qr_scanner_node:main',
            'human_detector = team_perception.human_detector_node:main',
            'webcam_bridge = team_perception.webcam_bridge_node:main',
            'annotate = team_perception.annotate:main',
        ],
    },
)
