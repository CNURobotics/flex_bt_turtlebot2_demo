import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = 'flex_bt_turtlebot2_demo_bringup'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        (os.path.join('share', PACKAGE_NAME), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='jmz919',
    author_email='joshua.zutell.18@cnu.edu',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='Startup demonstration of flexible_behavior_trees with CHRISLab Turtlebot2.',
    license='Apache 2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
