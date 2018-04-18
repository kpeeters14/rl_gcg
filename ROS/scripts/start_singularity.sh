#!/bin/bash

# echo "exec $1 in singularity image /gluster/visics/singularity/ros_gazebo_tensorflow.imgs"
# cd /gluster/visics/singularity
# pwd
# ls /gluster/visics/singularity
# sleep 1
# /usr/bin/singularity exec --nv /gluster/visics/singularity/ros_gazebo_tensorflow.imgs /home/kevin/catkin_ws/src/rl_gcg/mainscript.sh
singularity exec --nv /home/kevin/ros_gazebo_tensorflow.img /home/kevin/catkin_ws/src/rl_gcg/scripts/mainscript.sh