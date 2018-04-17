#!/bin/bash
# export HOME=/esat/qayd/r0453462/home/
export HOME=/home/kevin


# export XAUTHORITY=$HOME/.Xauthority
# export DISPLAY=:$((100 + RANDOM % 154))

# export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
# export LD_LIBRARY_PATH=''

# xpra --xvfb="Xorg -noreset -nolisten tcp \
#     -config /etc/xpra/xorg.conf\
#     -logfile ${HOME}/.xpra/Xorg-${DISPLAY}.log" \
#     start $DISPLAY

# sleep 3

# # test
# if [ $(xdpyinfo | grep GLX | wc -w) -ge 2 ] ; then
#     echo "started xpra with GL successfully"
# else
#     echo "ERROR: failed to start xpra with GLX."
#     echo "------xdpyinfo"
#     xdpyinfo
#     echo "------ps -ef | xpra"
#     ps -ef | grep xpra
#     echo "------printenv"
#     printenv
#     exit
# fi

source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/catkin_ws/devel/setup.bash --extend

export ROS_PACKAGE_PATH=$HOME/catkin_ws/src:/opt/ros/kinetic/share

export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/simulation_supervised/simulation_supervised_demo/models

echo 'Starting ROS launch file'

xterm -l -lf log_ros -e roslaunch rl_gcg simulation_tb.launch &
pid_ros=!$

echo 'ROS launch file ready'

# Prepare everything for running gcg on laptop
export PATH=$HOME/Documents/Thesis/GKAHN/anaconda2/bin:$PATH
cd $HOME/Documents/Thesis/GKAHN/gcg/sandbox/gkahn/gcg
source activate gcg
export PYTHONPATH=$HOME/Documents/Thesis/GKAHN/gcg
export LD_LIBRARY_PATH=$HOME/Documents/Thesis/cudnn/lib64:$HOME/Documents/Thesis/cuda-8.0/lib64:$LD_LIBRARY_PATH

# Prepare everything for running gcg on esat computers
# export PATH=$HOME/Documents/Thesis/anaconda2/bin:$PATH
# export outdir=$HOME/Documents/Thesis/output
# cd $HOME/Documents/Thesis/gcg/sandbox/gkahn/gcg 
# source activate gcg
# export PYTHONPATH=$HOME/Documents/Thesis/gcg:$PYTHONPATH
# export LD_LIBRARY_PATH=/users/visics/kkelchte/local/cuda-8.0/lib64:/users/visics/kkelchte/local/lib/cudnn-5.1/cuda/lib64:$LD_LIBRARY_PATH

echo 'Starting GCG algorithm'

xterm -l -lf log_gcg -e python run_exp.py --exps ours &
pid_gcg=!$

echo 'GCG algorithm ready'

while [ $(cat log_ros | wc -l ) -lt 1000 ] ; do 
    sleep 1;
done
kill -9 $pid_ros
kill -9 $pid_gcg
echo 'done'