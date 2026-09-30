from setuptools import setup

package_name = 'arm_grasp'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cy',
    maintainer_email='cy@nepu.local',
    description='LeArm vision grasp',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'collect = arm_grasp.collect:main',
            'calib_report = arm_grasp.calib_report:main',
        ],
    },
)
