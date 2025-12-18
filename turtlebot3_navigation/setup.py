from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
        # Add required data files to be installed
        (os.path.join('share', package_name,'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cse4568',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add entry points
            'wall_follower = turtlebot3_navigation.wall_follower:main',
            'bug2 = turtlebot3_navigation.bug2:main',
        ],
    },
)
