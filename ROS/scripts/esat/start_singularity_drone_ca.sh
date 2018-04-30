#!/bin/bash

echo "exec mainscript in singularity image /gluster/visics/singularity/ros_gazebo_tensorflow.imgs"
cd /gluster/visics/singularity
pwd
ls /gluster/visics/singularity
sleep 1

/usr/bin/singularity exec --nv /gluster/visics/singularity/ros_gazebo_tensorflow.imgs /users/start2014/r0453462/catkin_ws/src/rl_gcg/scripts/esat/mainscript_drone_ca.sh
