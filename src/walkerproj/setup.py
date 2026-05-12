from setuptools import setup
import os
from glob import glob

package_name = 'walkerproj'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seed',
    maintainer_email='seed@todo.todo',
    description='Walker project launch file assignment',
    license='MIT',
    entry_points={
        'console_scripts': [
            'my_pose_node = walkerproj.myPose:main',
        ],
    },
)
