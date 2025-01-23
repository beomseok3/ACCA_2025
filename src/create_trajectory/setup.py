from setuptools import find_packages, setup

package_name = 'create_trajectory'

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
    maintainer='ps',
    maintainer_email='ps@todo.todo',
    description='ROS 2 rclpy basic package for the ROS 2 seminar',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory=create_trajectory.trajectory:main',
            'save_waypoint=create_trajectory.save_waypoint:main',
            'waypoints=create_trajectory.waypoints:main',
            'waypoint_publisher=create_trajectory.waypoint_publisher:main',
            'find_lane_id=create_trajectory.find_lane_id:main',
            'fake_initialpose=create_trajectory.fake_initialpose:main',
            'dummy=create_trajectory.dummy:main',
            'cone_pub=create_trajectory.cone_pub:main',
            'marker=create_trajectory.marker:main',
            'roi_pls=create_trajectory.roi_pls:main',
        ],
    },
)
