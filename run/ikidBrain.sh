#!/bin/bash

gnome-terminal --tab -- bash -c "roscore; exec bash"

gnome-terminal --tab -- bash -c "\
cd /home/hjf/project/ikid_workspace/run; \
source ../devel/setup.bash; \
cd ../; \
rosrun robot_brain_pkg head_control; \
exec bash"

gnome-terminal --tab -- bash -c "\
cd /home/hjf/project/ikid_workspace/run; \
source ../devel/setup.bash; \
cd ../; \
rosrun robot_brain_pkg robot_brain; \
exec bash"

gnome-terminal --tab -- bash -c "\
cd /home/hjf/project/ikid_workspace/run; \
source ../devel/setup.bash; \
cd ../; \
rosrun calculate_position_pkg calculate_position; \
exec bash"

gnome-terminal --tab -- bash -c "\
cd /home/hjf/project/ikid_workspace/run; \
source ../devel/setup.bash; \
cd ../; \
rosrun realtime_detect_pkg realtime_detect.py; \
exec bash"

