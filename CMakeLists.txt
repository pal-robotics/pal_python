cmake_minimum_required(VERSION 2.8.3)
project(pal_python)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
      rospy
      roslib
      rosgraph
      actionlib
      actionlib_msgs)
