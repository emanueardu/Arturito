from glob import glob
from setuptools import setup

package_name = 'robertito'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        package_name + '.eyes',
        package_name + '.orchestrator',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py') + ['launch/robertito_launch.py']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/assets', glob('assets/*.wav')),
        ('lib/' + package_name, ['scripts/wake_word_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Nodo principal de Robertito',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eyes_node = robertito.eyes_node:main',
            'voice_synth_node = robertito.voice_synth_node:main',
            'uart_node = robertito.uart_node:main',
            'face_detector_node = robertito.face_detector_node:main',
            'person_tracker_node = robertito.person_tracker_node:main',
            'teleop_node = robertito.teleop_node:main',
            'wake_word_listener_node = robertito.wake_word_listener_node:main',
            'api_chat_node = robertito.api_chat_node:main',
            'behavior_state_node = robertito.behavior_state_node:main',
            'presence_orchestrator_node = robertito.presence_orchestrator_node:main',
            'wander_avoid_node = robertito.wander_avoid_node:main',
            'clean_quick_node = robertito.clean_quick_node:main',
            'twist_mux_node = robertito.twist_mux_node:main',
            'cmd_vel_bridge_node = robertito.cmd_vel_bridge_node:main',
            'open_loop_odom_node = robertito.open_loop_odom_node:main',
            'map_odom_broadcaster = robertito.map_odom_broadcaster:main',
            'obstacles_node = robertito.obstacles_node:main',
            'wifi_localization_node = robertito.wifi_localization_node:main',
            'docking_calibration_node = robertito.docking_calibration_node:main',
        ],
    },
)
