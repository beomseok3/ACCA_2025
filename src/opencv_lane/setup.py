from setuptools import find_packages, setup

package_name = 'opencv_lane'

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
    maintainer='lee',
    maintainer_email='pojdlqudtjr@soongsil.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_sub = opencv_lane.node:main',
            'byeongseok = opencv_lane.byeongseok:main',
        ],
    },
)
