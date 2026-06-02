import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'guideBot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.lua')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seed',
    maintainer_email='seed@todo.todo',
    description='Home guide robot simulation',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        'teleop_game = nodes.teleop_game:main',
        'wanderer = nodes.wanderer:main',
        'waypoint_navigator = nodes.waypoint_navigator:main',
        'guide_announcer = nodes.guide_announcer:main',
    ],
    },
)