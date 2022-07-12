import os
from glob import glob
from setuptools import setup

package_name = 'autonomous_driving_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_driving = autonomous_driving_v2.autonomous_driving:main', 
            'lane_based_steer = autonomous_driving_v2.lane_based_steer:main',
            'overtaker = autonomous_driving_v2.overtaker:main',
            'parker = autonomous_driving_v2.parker:main',
            'velocity_controller = autonomous_driving_v2.velocity_controller:main'
        ],
    },
)
