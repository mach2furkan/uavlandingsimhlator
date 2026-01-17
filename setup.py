from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'uav_landing_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/landing_pad'), glob('models/landing_pad/*')),
        (os.path.join('share', package_name, 'models/vtol_uav'), glob('models/vtol_uav/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='UAV Landing Simulation for Gazebo Sim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_processor = uav_landing_sim.vision_processor:main',
            'flight_controller = uav_landing_sim.flight_controller:main',
        ],
    },
)
