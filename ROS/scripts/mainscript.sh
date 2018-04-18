#!/bin/bash
export HOME=/home/kevin

source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/catkin_ws/devel/setup.bash --extend

export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:/opt/ros/kinetic/share

export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/simulation_supervised/simulation_supervised_demo/models

echo 'Starting ROS launch file'

log_ros="logros_$(date +%F_%H%M)"
xterm -l -lf $HOME/logs/$log_ros -e roslaunch rl_gcg simulation_tb.launch &
pid_ros=!$

echo 'ROS launch file ready'

# Prepare everything for running gcg on laptop
export PATH=$HOME/Documents/Thesis/GKAHN/anaconda2/bin:$PATH
# export PATH=$HOME/Documents/Thesis/GKAHN/anaconda2/bin:$HOME:$PATH
cd $HOME/Documents/Thesis/GKAHN/gcg/sandbox/gkahn/gcg
source activate gcg
export PYTHONPATH=$HOME/Documents/Thesis/GKAHN/gcg
export LD_LIBRARY_PATH=$HOME/Documents/Thesis/cudnn/lib64:$HOME/Documents/Thesis/cuda-8.0/lib64:$LD_LIBRARY_PATH

# xvfb-run -s "-screen 0 1400x900x24" /bin/bash &
# export THEANO_FLAGS=floatX=float64
# export NUM_CPUS=1

echo 'Starting GCG algorithm'

log_gcg="loggcg_$(date +%F_%H%M)"
# xterm -l -lf $HOME/logs/$log_gcg -e python run_exp.py --exps ours | xargs -n 1 -P $NUM_CPUS &
xterm -l -lf $HOME/logs/$log_gcg -e python run_exp.py --exps ours &
pid_gcg=!$

echo 'GCG algorithm ready'

while [ $(cat $HOME/logs/$log_ros | wc -l ) -lt 500 ] ; do 
    sleep 1;
done
kill -9 $pid_ros
kill -9 $pid_gcg
echo 'done'