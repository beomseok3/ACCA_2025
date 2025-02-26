from setuptools import find_packages, setup

package_name = "odometry"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "imu_odometry = localization.imu_odometry:main",
            "imu_test = localization.imu_test:main",
            "imu_odometry_orientation = localization.imu_odometry_orientation:main",
            "imu_encoder_odometry = localization.imu_encoder_odometry:main",
            "imu_bias_measure = localization.imu_bias_measure:main",
            "imu_temper_measure = localization.imu_temper_measure:main",
            "imu_scalefactor_measure = localization.imu_scalefactor_measure:main",
        ],
    },
)
