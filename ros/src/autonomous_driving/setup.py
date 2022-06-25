from setuptools import setup

package_name = 'autonomous_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'autonomous_driving = autonomous_driving.lane_based_steer:main',
            'overtaker = autonomous_driving.overtaker:main',
            'parker = autonomous_driving.parker:main'
        ],
    },
)
