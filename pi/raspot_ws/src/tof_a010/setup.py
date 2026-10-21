from glob import glob
import os

from setuptools import setup

package_name = 'tof_a010'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cy',
    maintainer_email='cy@nepu.local',
    description='MetaSense A010 ToF on the Pi',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'tof_node = tof_a010.tof_node:main',
            'tof_probe = tof_a010.tof_probe:main',
        ],
    },
)
