from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'data_acquisicion_charlie'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools', 'tf-transformations'],
    zip_safe=True,
    maintainer='clip2004',
    maintainer_email='felipe.mercado59@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_to_pose_node = data_acquisicion_charlie.odom_to_pose_node:main',
            'pose_broadcaster_node = data_acquisicion_charlie.pose_broadcaster_node:main',
            'cmd_vel_ctrl_node_charlie = data_acquisicion_charlie.cmd_vel_ctrl_node_charlie:main',
            'pose_simulator_node_charlie = data_acquisicion_charlie.pose_simulator_node_charlie:main',
        ],
    },
)
