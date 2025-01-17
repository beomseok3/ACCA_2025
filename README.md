# ACCA_2025
Autonomous Vihicle Project with ERP42 


<!-- sudo apt update
rosdep update
rosdep install darknet_ros_msgs -->

git clone (https://github.com/beomseok3/ACCA_2025)

cd /ACCA_2025

#의존성 파일
rosdep install --from-paths src --ignore-src -r -y

#msg 먼저 빌드해주기!!
colcon build --symlink-install --packages-select adaptive_clustering_msgs
# 소스
. install/setup.bash
#전체 빌드
colcon build --symlink-install

