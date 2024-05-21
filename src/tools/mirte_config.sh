#!bin/bash
sudo service mirte-ros stop
# export ROS_IP=192.168.42.1
export ROS_IP=192.168.31.83
roslaunch mirte_bringup minimal_master.launch
# sudo service mirte-ros stop/start/restart