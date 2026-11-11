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
    maintainer_email='ngtri@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = team_perception.yolo_node:main',
            'qr_scanner = team_perception.qr_scanner_node:main',
            'webcam_bridge = team_perception.webcam_bridge_node:main',
        ],
    },
)
