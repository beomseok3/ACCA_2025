import os
from setuptools import setup
from glob import glob

package_name = 'erp42_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/stanley.py']),
        ('lib/' + package_name, [package_name+'/stanley_cone.py']),
        ('lib/' + package_name, [package_name+'/DB.py']),
        ('lib/' + package_name, [package_name+'/state_machine.py']),
        ('lib/' + package_name, [package_name+'/state_machine_ys.py']),
        ('lib/' + package_name, [package_name+'/controller_obstacle.py']),
        ('lib/' + package_name, [package_name+'/controller_obstacle_ys.py']),
        ('lib/' + package_name, [package_name+'/controller_pickup.py']),
        ('lib/' + package_name, [package_name+'/controller_parking.py']),
        ('lib/' + package_name, [package_name+'/controller_delivery.py']),
        ('lib/' + package_name, [package_name+'/controller_traffic_light.py']),
        ('lib/' + package_name, [package_name+'/controller_uturn.py']),
        ('lib/' + package_name, [package_name+'/controller_stop_line.py']),
        ('lib/' + package_name, [package_name+'/Modifier_param.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='gjs',
    maintainer_email='junseonggg2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_tunning = erp42_control.pid_tunning:main',
            'controller_cone = erp42_control.controller_cone:main',
            'state_machine = erp42_control.state_machine:main',
            'state_machine_ys = erp42_control.state_machine_ys:main',
            'state_machine_ys_test = erp42_control.state_machine_ys_test:main',
            'controller_obstacle = erp42_control.controller_obstacle:main',
            'controller_obstacle_ys = erp42_control.controller_obstacle_ys:main',
            'controller_pickup = erp42_control.controller_pickup:main',
            'controller_stop_line = erp42_control.controller_stop_line:main',
            'controller_traffic_light = erp42_control.controller_traffic_light:main',
            'controller_delivery = erp42_control.controller_delivery:main',
            'controller_uturn = erp42_control.controller_uturn:main'
        ],
    },
)
