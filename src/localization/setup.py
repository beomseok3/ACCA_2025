import os
from setuptools import setup
from glob import glob

package_name = "localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.rviz")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="gjs",
    maintainer_email="junseonggg2001@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "erp_twist = localization.erp_twist:main",
            "rotate_yaw = localization.rotate_yaw:main",
            "rotate_yaw_cone = localization.rotate_yaw_cone:main",
            "gps_odom = localization.gps_odom:main",
            "imu_bias_measure = localization.imu_bias_measure:main",
            "create_accel = localization.create_accel:main",
            "kalman_data = localization.kalman_data:main",
            "ndt_rotate = localization.ndt_rotate:main",
            "ndt_kalman_tf = localization.ndt_kalman_tf:main",
            "wheel_odometry = localization.wheel_odometry:main",
            "set_initial = localization.set_initial:main",
            "solve_pnp = localization.solve_pnp:main",
            "test_map = localization.test_map:main",
            "kalman_localization = localization.kalman_localization:main",
            "map_odom_tf_publisher = localization.map_odom_tf_publisher:main",
            "real_time_map_server = localization.real_time_map_server:main",
            "dfilter_yaw = localization.dfilter_yaw:main",
            "imu_test = localization.imu_test:main",
            "erp_twist_world = localization.erp_twist_world:main",
            "position_filter_yaw = localization.position_filter_yaw:main",
            "orientation_source_2 = localization.orientation_source_2:main",
            "orientation_source_3 = localization.orientation_source_3:main",
            "orientation_source_4 = localization.orientation_source_4:main",
            "orientation_source_5 = localization.orientation_source_5:main",
            
        ],
    },
)
