from glob import glob
from setuptools import setup
package_name = 'robot_bringup'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('lib/' + package_name, glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emanuel',
    maintainer_email='you@example.com',
    description='Launch files for Emanuel bot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'zone_waypoint_node = robot_bringup.zone_waypoint_node:main',
            'cleaning_controller = robot_bringup.cleaning_controller:main',
            'tag_detector_node = robot_bringup.tag_detector_node:main',
            'go_to_base_node = robot_bringup.go_to_base_node:main',
        ],
    },
)
