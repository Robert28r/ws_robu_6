source /opt/ros/humble/setup.bash
cd ~/work/ws_robu
. install/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=6
ros2 launch turtlebot3_cartographer cartographer.launch.py
