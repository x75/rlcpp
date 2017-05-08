cmake_minimum_required(VERSION 2.8.3)

project(rlcpp)

# find_package(catkin REQUIRED)

link_directories(/opt/ros/jade/lib)

add_library(rlcpp_util StateActionUtils.cpp cNeuralNetwork.cpp Algorithm.cpp StateActionAlgorithm.cpp Experiment.cpp)

add_library(rlcpp_algorithms Sarsa.cpp Qlearning.cpp QVlearning.cpp Acla.cpp Cacla.cpp)

# this doesn't work as ROS code needs to be included conditionally
# add_executable(CartPole CartPole.cpp CartPoleInterface.cpp)
# target_link_libraries(CartPole rlcpp_util rlcpp_algorithms)

add_executable(CartPoleRos CartPole.cpp CartPoleInterface.cpp)
target_link_libraries(CartPoleRos rlcpp_util rlcpp_algorithms roscpp rosconsole roscpp_serialization roslib rostime)

add_executable(ArmRos Arm.cpp ArmInterface.cpp)
target_link_libraries(ArmRos rlcpp_util rlcpp_algorithms roscpp rosconsole roscpp_serialization roslib rostime)

