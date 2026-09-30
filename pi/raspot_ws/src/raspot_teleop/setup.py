from setuptools import find_packages, setup

package_name = 'raspot_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ps2_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cy',
    maintainer_email='cy@todo.todo',
    description='PS2 gamepad teleop + l150pro serial driver (chassis + arm)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps2_teleop_node = raspot_teleop.ps2_teleop_node:main',
            'l150pro_driver_node = raspot_teleop.l150pro_driver_node:main',
            'arm_recorder_node = raspot_teleop.arm_recorder_node:main',
        ],
    },
)
