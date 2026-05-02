from setuptools import find_packages, setup

package_name = 'hybrik_pose_bridge'
description = (
    'Bridge HybrIK custom joint messages to standard ROS pose messages '
    'for visualization.'
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tang',
    maintainer_email='Tang29Zx@outlook.com',
    description=description,
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_array_bridge = hybrik_pose_bridge.pose_array_bridge:main',
        ],
    },
)
