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
	python/static_target/behaviour_drone_st.py
	python/static_target/behaviour_tb_st.py
	python/static_target/evaluate_drone_st.py
	python/static_target/evaluate_tb_st.py
	python/static_target/random_behaviour_drone.py
	python/static_target/image_to_np_drone_st.py
	python/static_target/image_to_np_tb_st.py
	python/static_target/orientation_to_ypr_drone_st.py
	python/static_target/orientation_to_ypr_tb_st.py

	python/circle_target/behaviour_drone_ct.py
	python/circle_target/behaviour_target.py
	python/circle_target/behaviour_tb_ct.py
	python/circle_target/evaluate_drone_ct.py
	python/circle_target/evaluate_tb_ct.py
	python/circle_target/image_to_np_drone_ct.py
	python/circle_target/image_to_np_tb_ct.py
	python/circle_target/orientation_to_ypr_drone_ct.py
	python/circle_target/orientation_to_ypr_tb_ct.py

	python/collision_avoidance/behaviour_drone_ca.py
	python/collision_avoidance/behaviour_tb_ca.py
	python/collision_avoidance/evaluate_drone_ca.py
	python/collision_avoidance/evaluate_tb_ca.py
	python/collision_avoidance/image_to_np_drone_ca.py
	python/collision_avoidance/image_to_np_tb_ca.py
	python/collision_avoidance/orientation_to_ypr_drone_ca.py
	python/collision_avoidance/orientation_to_ypr_tb_ca.py

	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)