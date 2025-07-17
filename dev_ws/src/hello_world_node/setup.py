from setuptools import setup

package_name = 'hello_world_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NOLEN HSU',
    maintainer_email='2691004662@qq.com',
    description='Hello World ROS2 节点包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world_publisher = hello_world_node.hello_world_publisher:main',
        ],
    },
)
