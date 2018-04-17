#!/bin/bash

echo "exec $1 in singularity image /gluster/visics/singularity/ros_gazebo_tensorflow.imgs"
cd /gluster/visics/singularity
pwd
ls /gluster/visics/singularity
sleep 1
/usr/bin/singularity exec --nv /gluster/visics/singularity/ros_gazebo_tensorflow.imgs $1
