from setuptools import find_packages, setup

package_name = "global_path_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("lib/" + package_name, [package_name + "/DB.py"]),
        # (os.path.join('share/', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ps",
    maintainer_email="bumwhale333c@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_making = global_path_planning.path_making:main",
            "marker = global_path_planning.marker:main",
            "path_making_bs = global_path_planning.path_making_bs:main",
            "path_making_estop = global_path_planning.path_making_estop:main",
            "db_read = global_path_planning.db_read:main",
            "db_read_2 = global_path_planning.db_read_2:main",
            "db_read_3 = global_path_planning.db_read_3:main",
            "db_write = global_path_planning.db_write:main",
            "path_collector = global_path_planning.path_collector:main",
        ],
    },
)
