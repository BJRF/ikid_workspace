#!/bin/bash

# gnome-terminal --tab -- bash -c "roscore; exec bash"

gnome-terminal --tab -- bash -c "\
source /home/nvidia/ikid_ws/devel/setup.bash; \
roslaunch ikid_robot my_launch.launch; \
exec bash"

gnome-terminal --tab -- bash -c "\
sleep 5s; \
source /home/nvidia/ikid_ws/devel/setup.bash; \
rosrun ros_socket robot_walk_node_specialgait; \
exec bash"

gnome-terminal --tab -- bash -c "\
sleep 5s; \
source /home/nvidia/ikid_ws/devel/setup.bash; \
rosrun ros_socket server_special_gait; \
exec bash"

gnome-terminal --tab -- bash -c "\
sleep 3s; \
source /home/nvidia/ikid_ws/devel/setup.bash; \
rosrun ros_socket serial_port; \
exec bash"

gnome-terminal --tab -- bash -c "\
sleep 5s; \
source /home/nvidia/ikid_ws/devel/setup.bash; \
rosrun ros_socket udp_special_gait; \
exec bash"
