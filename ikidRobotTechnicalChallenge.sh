#!/bin/bash

# gnome-terminal --tab -- bash -c "roscore; exec bash"

gnome-terminal --tab -- bash -c "\
source /home/nvidia/ikid_ws/devel/setup.bash; \
roslaunch ikid_robot my_launch.launch; \
exec bash"

gnome-terminal --tab -- bash -c "\
sleep 5s; \
source /home/nvidia/ikid_ws/devel/setup.bash; \
rosrun ikid_motion_control technical_challenges_node; \
exec bash"

# 要保证 serial_port节点先于robot_walk_node启动
gnome-terminal --tab -- bash -c "\
sleep 3s; \
source /home/nvidia/ikid_ws/devel/setup.bash; \
rosrun ros_socket serial_port; \
exec bash"
