import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rob_project'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miguel',
    maintainer_email='miguel@upc.edu',
    description='Autonomous navigation mission for TurtleBot3 Burger',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planner = rob_project.mission_planner:main',
            'obstacle_detector = rob_project.obstacle_detector:main',
            'station_detector = rob_project.station_detector:main',
            'precision_parking = rob_project.precision_parking:main',
            'csv_logger = rob_project.csv_logger:main',
        ],
    },
)
