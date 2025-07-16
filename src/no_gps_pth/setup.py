from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'no_gps_pth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['no_gps_pth.stanley',],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py"))
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
            "costmap = no_gps_pth.costmap:main",
            "rrt = no_gps_pth.rrt:main"
        ],
    },
)
