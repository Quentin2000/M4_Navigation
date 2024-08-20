#!/bin/bash

# Fetch command line arguments
rosbag_filename="$1"
rl="$2"

# Display the received arguments
echo "Rosbag filename: $rosbag_filename"
echo "RL parameter: $rl"

# Create a directory for the rosbag if it doesn't exist
mkdir -p ~/catkin_ws/src/M4_Local_Planner/rosbags/bagfiles/$rosbag_filename

# Open a new terminal, source the bash configuration, and run the ROS launch with parameter
gnome-terminal -- bash -c "cd ~/catkin_ws/src/M4_Local_Planner/simulation_slam/launch; roslaunch simulation_merged.launch rl:=$rl; exec bash"

# Open a new terminal and start recording the rosbag with specified topics in the new directory
gnome-terminal -- bash -c "cd ~/catkin_ws/src/M4_Local_Planner/rosbags/bagfiles/$rosbag_filename; rosbag record -O ${rosbag_filename}.bag /power_consumption_total /power_consumption_wheel /power_consumption_hip /goal_status /base_link_contacts; exec bash"
