cmake_minimum_required(VERSION 2.8.3)
project(rl_gcg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  gazebo_ros
  bebop_msgs
)

find_package(Boost REQUIRED COMPONENTS system signals thread)

find_package(OpenCV REQUIRED)

## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)
include_directories(
  python
)
catkin_package()

catkin_install_python(PROGRAMS
	python/simulate_behaviour.py
	python/simulate_random_behaviour.py
	python/evaluate.py
	python/image_to_np.py
	python/orientation_to_ypr.py
	python/simulate_behaviour_tb.py
	python/evaluate_tb.py
	python/image_to_np_tb.py
	python/orientation_to_ypr_tb.py
	python/simulate_target.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)