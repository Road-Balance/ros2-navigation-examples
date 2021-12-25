
from glob import glob

import os

from setuptools import setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_pose_action_client = path_planning.nav_to_pose_action_client:main',
            'clicked_point_sub = path_planning.clicked_point_sub:main',
            'goal_pose_pub = path_planning.goal_pose_pub:main',
        ],
    },
)
