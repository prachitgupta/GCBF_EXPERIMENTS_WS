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
            'pose_debug = crazyflie_vicon_bringup.pose_debug:main',
        ],
    },
)
