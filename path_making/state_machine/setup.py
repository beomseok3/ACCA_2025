from setuptools import find_packages, setup
package_name = 'state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/DB.py']),
        # (os.path.join('share/', package_name, 'msg'), glob('msg/*.msg')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ps',
    maintainer_email='ps@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_making = state_machine.path_making:main',
            'marker = state_machine.marker:main',            
            'path_making_bs = state_machine.path_making_bs:main',
            'path_making_estop = state_machine.path_making_estop:main',
            "db_read = state_machine.db_read:main",
            "db_write = state_machine.db_write:main",
            "path_collector = state_machine.path_collector:main",

            

        ],
    },
)
