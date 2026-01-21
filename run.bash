source /opt/ros/humble/setup.bash 
source ./install/setup.bash 
export VEHICLE_NAME=duckie05
export USER_NAME=duckie05
chmod a+x ./install/blank_package/lib/blank_package/blank_node.py
ls -al ./install/blank_package/lib/blank_package/
ros2 run blank_package blank_node.py