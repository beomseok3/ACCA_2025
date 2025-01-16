# ACCA_2025
Autonomous Vihicle Project with ERP42 


<!-- sudo apt update
rosdep update
rosdep install darknet_ros_msgs -->

git clone [text](https://github.com/beomseok3/ACCA_2025)

cd /ACCA_2025

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --packages-select adaptive_clustering_msgs

. install/setup.bash

colcon build --symlink-install