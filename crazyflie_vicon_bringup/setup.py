import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'crazyflie_vicon_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name, ['README_POSITION_EXCHANGE_SETUP.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minhyuk',
    maintainer_email='minhyuk@todo.todo',
    description='Local Crazyflie bringup package for Vicon-based flight tests.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gcbf_experiments = crazyflie_vicon_bringup.gcbf_experiments:main',
            'four_drone_setpoint = crazyflie_vicon_bringup.four_drone_setpoint:main',
            'four_drone_setpoint_cf5_cf8 = '
            'crazyflie_vicon_bringup.four_drone_setpoint_cf5_cf8:main',
            'eight_drone_hover = crazyflie_vicon_bringup.eight_drone_hover:main',
            'single_drone_streamed_hover = '
            'crazyflie_vicon_bringup.single_drone_streamed_hover:main',
            'cf5_position_exchange = '
            'crazyflie_vicon_bringup.cf5_position_exchange:main',
            'eight_drone_position_exchange = '
            'crazyflie_vicon_bringup.eight_drone_position_exchange:main',
            'check_vicon_positions = crazyflie_vicon_bringup.check_vicon_positions:main',
            'identify_crazyradios = '
            'crazyflie_vicon_bringup.identify_crazyradios:main',
            'pose_debug = crazyflie_vicon_bringup.pose_debug:main',
            'radio70_sequential = '
            'crazyflie_vicon_bringup.radio70_sequential:main',
        ],
    },
)
