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
            "bicycle_erp_twist_world = localization.bicycle_erp_twist_world:main",
            "imu_odometry = localization.imu_odometry:main",
            "bicycle_steer_estimation = localization.bicycle_steer_estimation:main",
            "imu_scalefactor_measure = localization.imu_scalefactor_measure:main",
            "complementory_filter = localization.complementory_filter:main",
            "imu_temper_measure = localization.imu_temper_measure:main",
            "erp_twist_lpf = localization.erp_twist_lpf:main",
            "imu_test = localization.imu_test:main",
            "erp_twist = localization.erp_twist:main",
            "erp_twist_world_lpf = localization.erp_twist_world_lpf:main",
            "mag = localization.mag:main",
            "erp_twist_world = localization.erp_twist_world:main",
            "position_filter_yaw = localization.position_filter_yaw:main",
            "fake_feedback = localization.fake_feedback:main",
            "gps_odom = localization.gps_odom:main",
            "rotate_yaw_cone = localization.rotate_yaw_cone:main",
            "gps_variance_filter = localization.gps_variance_filter:main",
            "rotate_yaw = localization.rotate_yaw:main",
            "gyro = localization.gyro:main",
            "wheel_odometry = localization.wheel_odometry:main",
            "imu_bias_measure = localization.imu_bias_measure:main",
            "wheel_odometry_steer = localization.wheel_odometry_steer:main",
            "imu_odometry_orientation = localization.imu_odometry_orientation:main",
            " map_odom_tf_publisher = localization.map_odom_tf_publisher:main",
        ],
    },
)
