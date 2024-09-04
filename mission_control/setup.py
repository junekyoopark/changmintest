import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mission_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz')),
        (os.path.join('share', package_name), ['resource/test_clothoid_result.csv'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joe',
    maintainer_email='jihwanshin@yonsei.ac.kr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_start = mission_control.offboard_start:main',
            'mission1 = mission_control.mission1:main',
            'visualizer = mission_control.visualizer:main',
            'offboard_test = mission_control.offboard_test:main',
            'waypointsmc = mission_control.waypointsmc:main'
        ],
    },
    package_data={
        package_name: ['resource/*.csv']  # Ensure CSV files are included
    },
    include_package_data=True,
)
