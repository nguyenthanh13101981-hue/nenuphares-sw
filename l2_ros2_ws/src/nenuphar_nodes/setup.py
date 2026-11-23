from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'nenuphar_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NENUPHARES Dev',
    maintainer_email='dev@nenuphares.local',
    description='11-node ROS 2 Jazzy skeleton for NENUPHARES BONNEVAL V1 (CdCT Final 2.0).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_driver = nenuphar_nodes.camera_driver_node:main',
            'vio = nenuphar_nodes.vio_node:main',
            'perception_geom = nenuphar_nodes.perception_geom_node:main',
            'clearance = nenuphar_nodes.clearance_node:main',
            'mission_fsm = nenuphar_nodes.mission_fsm_node:main',
            'corridor_monitor = nenuphar_nodes.corridor_monitor_node:main',
            'formation = nenuphar_nodes.formation_node:main',
            'energy = nenuphar_nodes.energy_node:main',
            'mavlink_bridge = nenuphar_nodes.mavlink_bridge_node:main',
            'blackbox = nenuphar_nodes.blackbox_node:main',
            'system_monitor = nenuphar_nodes.system_monitor_node:main',
            'latency_probe = nenuphar_nodes.latency_probe:main',
        ],
    },
)
