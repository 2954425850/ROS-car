from setuptools import find_packages, setup

package_name = 'k230_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cy',
    maintainer_email='cy@todo.todo',
    description='K230 vision result stream to ROS topics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = k230_ros.bridge_node:main',
        ],
    },
)
