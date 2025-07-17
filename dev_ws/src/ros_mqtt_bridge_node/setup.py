from setuptools import setup

package_name = 'ros_mqtt_bridge_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/hello_world_mqtt_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/hello_world_mqtt_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Nolen Hsu',
    maintainer_email='26091004662@qq.com',
    description='ROS2 to MQTT Bridge Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world_mqtt_bridge = ros_mqtt_bridge_node.hello_world_mqtt_bridge:main',
        ],
    },
)
