from setuptools import setup
package_name = 'esp32_serial_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/bridge']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='emanuel',
    maintainer_email='you@example.com',
    description='ROS2 <-> ESP32 serial bridge',
    license='MIT',
    entry_points={'console_scripts':['bridge = esp32_serial_bridge.bridge:main']},
)
