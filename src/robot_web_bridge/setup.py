from setuptools import setup
from glob import glob

package_name = 'robot_web_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Bridge web para video, audio y rosbridge del robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_bridge = robot_web_bridge.bridge_node:main',
            'video_server = robot_web_bridge.video_stream:main',
            'audio_server = robot_web_bridge.audio_stream:main',
            'control_bridge = robot_web_bridge.control_bridge:main',
        ],
    },
)
