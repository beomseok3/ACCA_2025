from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'costmap_has'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='libok',
    maintainer_email='libok1117@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "costmap = costmap_has.costmap:main",
            "path_planner = costmap_has.path_planner:main",
            "pure_pursuit = costmap_has.pure_pursuit:main"
        ],
    },
)
