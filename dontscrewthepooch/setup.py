from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dontscrewthepooch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='junekyoopark@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_map_to_odom = dontscrewthepooch.tf2_map_to_odom:main',
            'goal_pose_to_traj_setpoint = dontscrewthepooch.goal_pose_to_traj_setpoint:main',
            'command_give = dontscrewthepooch.command_give:main'
        ],
    },
)
