#!/bin/bash
export HOME=/users/start2014/r0453462

source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/catkin_ws/devel/setup.bash --extend

export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:/opt/ros/kinetic/share

export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/simulation_supervised/simulation_supervised_demo/models

echo 'Starting ROS launch file'

log_ros="$HOME/log/collision_avoidance/drone/logros_$(date +%F_%H%M)"
xterm -l -lf $log_ros -e roslaunch rl_gcg collision_avoidance_drone.launch &
pid_ros=$!

echo 'ROS launch file ready'

# Prepare everything for running gcg on esat
export PATH=$HOME/Documents/Thesis/anaconda2/bin:$HOME:$PATH
cd $HOME/Documents/Thesis/gcg/sandbox/gkahn/gcg
source activate gcg
export PYTHONPATH=$HOME/Documents/Thesis/gcg
export LD_LIBRARY_PATH=/users/visics/kkelchte/local/cuda-8.0/lib64:/users/visics/kkelchte/local/lib/cudnn-5.1/cuda/lib64:/.singularity.d/libs:$LD_LIBRARY_PATH

echo 'Starting GCG algorithm'

log_gcg="$HOME/log/collision_avoidance/drone/loggcg_$(date +%F_%H%M)"
# python eval_exp.py folder/with/pkl/files/ num_rollouts
xterm -l -lf $log_gcg -e python eval_exp.py $HOME/Documents/Thesis/gcg/data/sim-rccar/ours/ 14 &
pid_gcg=$!

echo 'GCG algorithm ready'

while [ $(cat $log_ros | wc -l ) -lt 50000 ] ; do 
    sleep 1;
done
kill -9 $pid_ros
kill -9 $pid_gcg
echo 'done'
