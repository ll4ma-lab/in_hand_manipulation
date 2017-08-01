#!/bin/bash
# Usage: sh bag_record.sh bag_file_name

rosbag record -O $1 /tf /camera/rgb/image_raw /camera/rgb/camera_info /allegro_hand_right/joint_states /allegro_hand_right_joint_cmd 
