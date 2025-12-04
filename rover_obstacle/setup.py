from setuptools import find_packages, setup

package_name = 'rover_obstacle'

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
    maintainer='bharat',
    maintainer_email='bharat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
            'console_scripts': [
    'obstacle_detector_lidar = rover_obstacle.obstacle_detector_lidar:main',
    'obstacle_avoidance_lidar= rover_obstacle.obstacle_avoidance_lidar:main',
    'ultrasonic_reader = rover_obstacle.ultrasonic_reader:main',
    'obstacle_avoidance_combined= rover_obstacle.obstacle_avoidance_combined:main',
     'slam_explorer = rover_obstacle.slam_explorer:main',
        ],
    },
)
